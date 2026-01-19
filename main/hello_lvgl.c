#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_check.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "driver/gpio.h"
#include "driver/i2c.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"

#include "esp_lcd_touch.h"
#include "esp_lcd_touch_gt911.h"

#include "freertos/semphr.h"
#include "hello_lvgl.h"
#include "lvgl.h"

static SemaphoreHandle_t s_lvgl_mutex;

static inline void lvgl_lock(void) {
    xSemaphoreTake(s_lvgl_mutex, portMAX_DELAY);
}
static inline void lvgl_unlock(void) { xSemaphoreGive(s_lvgl_mutex); }

static const char *TAG = "hello_lvgl";

/* =========================
 *  ESP32-S3-Touch-LCD-7 V1.2 pinout (from schematic)
 * ========================= */
#define LCD_HRES 800
#define LCD_VRES 480

// LCD RGB pins (directly from schematic)
#define PIN_LCD_VSYNC 3  // IO3
#define PIN_LCD_HSYNC 46 // IO46
#define PIN_LCD_DE 5     // IO5
#define PIN_LCD_PCLK 7   // IO7

#define PIN_LCD_B3 14 // IO14
#define PIN_LCD_B4 38 // IO38
#define PIN_LCD_B5 18 // IO18
#define PIN_LCD_B6 17 // IO17
#define PIN_LCD_B7 10 // IO10

#define PIN_LCD_G2 39 // IO39
#define PIN_LCD_G3 0  // IO0
#define PIN_LCD_G4 45 // IO45
#define PIN_LCD_G5 48 // IO48
#define PIN_LCD_G6 47 // IO47
#define PIN_LCD_G7 21 // IO21

#define PIN_LCD_R3 1  // IO1
#define PIN_LCD_R4 2  // IO2
#define PIN_LCD_R5 42 // IO42
#define PIN_LCD_R6 41 // IO41
#define PIN_LCD_R7 40 // IO40

// I2C bus (shared between CH422G and GT911)
#define I2C_PORT I2C_NUM_0
#define PIN_I2C_SDA 8 // IO8
#define PIN_I2C_SCL 9 // IO9

// Touch interrupt
#define PIN_TOUCH_INT 4 // IO4 = CTP_IRQ

/* =========================
 * CH422G I/O Expander
 *
 * CH422G uses a special I2C protocol where the 7-bit I2C address
 * determines which register is being accessed:
 *
 * 8-bit addr | 7-bit addr | Function
 * -----------|------------|----------
 *    0x48    |    0x24    | WR_SET_IO_DIR (bit0=OE, bit1=OD_EN)
 *    0x46    |    0x23    | WR_SET_OC
 *    0x70    |    0x38    | WR_IO_L (IO0-IO7 output)
 *    0x72    |    0x39    | WR_IO_H (IO8-IO11 output)
 *    0x4C    |    0x26    | RD_IO (input)
 *
 * From schematic:
 * - EXIO1 = IO1 = CTP_RST (touch reset)
 * - EXIO2 = IO2 = DISP (LCD backlight enable)
 * ========================= */
#define CH422G_WR_SET_IO 0x24 // 7-bit addr for SET_IO_DIR
#define CH422G_WR_OC 0x23     // 7-bit addr for SET_OC
#define CH422G_WR_IO_L 0x38   // 7-bit addr for IO0-IO7 output

// IO direction mode bits
#define CH422G_IO_OE 0x01 // Output enable for IO0-IO7
#define CH422G_OD_EN 0x02 // Open-drain enable (0=push-pull)

// Pin mapping from schematic
#define EXIO_CTP_RST 1 // IO1 = touch panel reset
#define EXIO_LCD_BL 2  // IO2 = LCD backlight

static esp_lcd_panel_handle_t g_panel = NULL;
static esp_lcd_touch_handle_t g_touch = NULL;

static lv_display_t *g_disp = NULL;
static lv_indev_t *g_indev = NULL;

static void *g_buf1 = NULL;
static void *g_buf2 = NULL;

static esp_timer_handle_t g_lv_tick_timer = NULL;

static uint8_t g_ch422g_io_state = 0x00;

/* =========================
 * I2C functions
 * ========================= */
static esp_err_t i2c_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_I2C_SDA,
        .scl_io_num = PIN_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000, // 400kHz standard
        .clk_flags = 0,
    };
    ESP_RETURN_ON_ERROR(i2c_param_config(I2C_PORT, &conf), TAG,
                        "i2c config failed");
    ESP_RETURN_ON_ERROR(i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0), TAG,
                        "i2c install failed");
    return ESP_OK;
}

// Write single byte to CH422G register (address = register)
static esp_err_t ch422g_write_reg(uint8_t reg_addr_7bit, uint8_t data) {
    return i2c_master_write_to_device(I2C_PORT, reg_addr_7bit, &data, 1,
                                      pdMS_TO_TICKS(100));
}

// Set CH422G IO pin (0-7)
static esp_err_t ch422g_set_io(uint8_t pin, bool level) {
    if (pin > 7)
        return ESP_ERR_INVALID_ARG;

    if (level)
        g_ch422g_io_state |= (1 << pin);
    else
        g_ch422g_io_state &= ~(1 << pin);

    return ch422g_write_reg(CH422G_WR_IO_L, g_ch422g_io_state);
}

// Initialize CH422G
static esp_err_t ch422g_init(void) {
    esp_err_t ret;

    // Set IO0-IO7 as push-pull outputs
    ret = ch422g_write_reg(CH422G_WR_SET_IO, CH422G_IO_OE); // OE=1, OD_EN=0
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "CH422G SET_IO write returned 0x%x (may be normal)", ret);
    }

    // Initialize all outputs to 0
    g_ch422g_io_state = 0x00;
    ret = ch422g_write_reg(CH422G_WR_IO_L, g_ch422g_io_state);
    ESP_RETURN_ON_ERROR(ret, TAG, "CH422G IO write failed");

    ESP_LOGI(TAG, "CH422G initialized");
    return ESP_OK;
}

/* =========================
 * I2C scan and GT911 detection
 * ========================= */
static void i2c_scan(const char *msg) {
    ESP_LOGI(TAG, "I2C Scan [%s]:", msg);
    int count = 0;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            if (addr == 0x14 || addr == 0x5D) {
                ESP_LOGW(TAG, "  0x%02X <- GT911!", addr);
            } else if (addr >= 0x20 && addr <= 0x3F) {
                // CH422G range - don't spam
                if (addr == 0x20)
                    ESP_LOGI(TAG, "  0x20-0x3F (CH422G)");
            } else {
                ESP_LOGI(TAG, "  0x%02X", addr);
            }
            count++;
        }
    }
    ESP_LOGI(TAG, "  Total: %d devices", count);
}

static bool gt911_check_present(uint8_t addr) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return (ret == ESP_OK);
}

static bool gt911_read_product_id(uint8_t addr) {
    uint8_t reg[2] = {0x81, 0x40}; // Register 0x8140 (big-endian)
    uint8_t pid[4] = {0};

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, reg, 2, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, pid, 3, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &pid[3], I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG,
                 "GT911 @ 0x%02X: Product ID = '%c%c%c%c' (0x%02X%02X%02X%02X)",
                 addr, pid[0], pid[1], pid[2], pid[3], pid[0], pid[1], pid[2],
                 pid[3]);
        return true;
    }
    return false;
}

/* =========================
 * GT911 Reset Sequence (from datasheet)
 *
 * To select I2C address:
 * - INT LOW during RST rising edge -> 0x5D (0xBA/0xBB)
 * - INT HIGH during RST rising edge -> 0x14 (0x28/0x29)
 * ========================= */
static void gt911_set_int_output(bool level) {
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << PIN_TOUCH_INT,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    gpio_set_level(PIN_TOUCH_INT, level ? 1 : 0);
}

static void gt911_set_int_input(void) {
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << PIN_TOUCH_INT,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
}

static bool gt911_reset_and_detect(bool int_high_for_0x14, uint8_t *out_addr) {
    uint8_t expected_addr = int_high_for_0x14 ? 0x14 : 0x5D;

    ESP_LOGI(TAG, "GT911 reset: INT=%s -> expect 0x%02X",
             int_high_for_0x14 ? "HIGH" : "LOW", expected_addr);

    // Step 1: Set INT output to desired level
    gt911_set_int_output(int_high_for_0x14);
    vTaskDelay(pdMS_TO_TICKS(1));

    // Step 2: Assert reset (RST LOW) via CH422G EXIO1
    ch422g_set_io(EXIO_CTP_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20)); // t1 >= 10ms

    // Step 3: Release reset (RST HIGH) - INT must stay at level!
    ch422g_set_io(EXIO_CTP_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(5)); // t2 >= 5ms

    // Step 4: Set INT to input (floating)
    gt911_set_int_input();
    vTaskDelay(pdMS_TO_TICKS(50)); // t3 >= 50ms

    // Step 5: Check if GT911 responds
    if (gt911_check_present(expected_addr)) {
        ESP_LOGI(TAG, "  Found at 0x%02X!", expected_addr);
        if (gt911_read_product_id(expected_addr)) {
            *out_addr = expected_addr;
            return true;
        }
    }

    // Try alternate address
    uint8_t alt_addr = int_high_for_0x14 ? 0x5D : 0x14;
    if (gt911_check_present(alt_addr)) {
        ESP_LOGW(TAG, "  Found at alternate 0x%02X!", alt_addr);
        if (gt911_read_product_id(alt_addr)) {
            *out_addr = alt_addr;
            return true;
        }
    }

    ESP_LOGW(TAG, "  Not detected");
    return false;
}

static bool gt911_init_sequence(uint8_t *out_addr) {
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== GT911 Initialization ===");

    // First check if already present (may have been reset by bootloader)
    ESP_LOGI(TAG, "Checking if GT911 already responds...");
    if (gt911_check_present(0x5D) && gt911_read_product_id(0x5D)) {
        *out_addr = 0x5D;
        return true;
    }
    if (gt911_check_present(0x14) && gt911_read_product_id(0x14)) {
        *out_addr = 0x14;
        return true;
    }

    // Try reset with INT=LOW (address 0x5D)
    if (gt911_reset_and_detect(false, out_addr)) {
        return true;
    }

    // Try reset with INT=HIGH (address 0x14)
    if (gt911_reset_and_detect(true, out_addr)) {
        return true;
    }

    ESP_LOGE(TAG, "GT911 not detected!");
    return false;
}

/* =========================
 * LCD RGB Panel
 * ========================= */
static esp_err_t lcd_init(void) {
    esp_lcd_rgb_timing_t timing = {
        .pclk_hz = 12 * 1000 * 1000,
        .h_res = LCD_HRES,
        .v_res = LCD_VRES,
        .hsync_pulse_width = 4,
        .hsync_back_porch = 8,
        .hsync_front_porch = 8,
        .vsync_pulse_width = 4,
        .vsync_back_porch = 8,
        .vsync_front_porch = 8,
        .flags =
            {
                .pclk_active_neg = 1,
            },
    };

    esp_lcd_rgb_panel_config_t cfg = {
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .timings = timing,
        .data_width = 16,
        .bits_per_pixel = 16,
        .num_fbs = 1,
        .hsync_gpio_num = PIN_LCD_HSYNC,
        .vsync_gpio_num = PIN_LCD_VSYNC,
        .de_gpio_num = PIN_LCD_DE,
        .pclk_gpio_num = PIN_LCD_PCLK,
        .disp_gpio_num = -1,
        .data_gpio_nums =
            {
                PIN_LCD_B3,
                PIN_LCD_B4,
                PIN_LCD_B5,
                PIN_LCD_B6,
                PIN_LCD_B7,
                PIN_LCD_G2,
                PIN_LCD_G3,
                PIN_LCD_G4,
                PIN_LCD_G5,
                PIN_LCD_G6,
                PIN_LCD_G7,
                PIN_LCD_R3,
                PIN_LCD_R4,
                PIN_LCD_R5,
                PIN_LCD_R6,
                PIN_LCD_R7,
            },
        .flags =
            {
                .fb_in_psram = 1,
            },
    };

    ESP_RETURN_ON_ERROR(esp_lcd_new_rgb_panel(&cfg, &g_panel), TAG,
                        "RGB panel create failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(g_panel), TAG,
                        "panel reset failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(g_panel), TAG, "panel init failed");

    return ESP_OK;
}

/* =========================
 * Touch driver initialization
 * ========================= */
static esp_err_t touch_driver_init() {
    esp_lcd_panel_io_handle_t io = NULL;

    const esp_lcd_panel_io_i2c_config_t io_cfg =
        ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();

    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c(I2C_PORT, &io_cfg, &io), TAG,
                        "panel io failed");

    static esp_lcd_touch_io_gt911_config_t gt911_cfg = {
        .dev_addr = io_cfg.dev_addr,
    };

    esp_lcd_touch_config_t tp_cfg = {
        .x_max = LCD_HRES,
        .y_max = LCD_VRES,
        .rst_gpio_num = -1, // reset via CH422G
        .int_gpio_num = -1, // polling
        .levels = {.reset = 0, .interrupt = 0},
        .flags = {.swap_xy = 0, .mirror_x = 0, .mirror_y = 0},
        .driver_data = &gt911_cfg, // <-- ESSENCIAL
    };

    ESP_RETURN_ON_ERROR(esp_lcd_touch_new_i2c_gt911(io, &tp_cfg, &g_touch), TAG,
                        "gt911 driver failed");
    return ESP_OK;
}

/* =========================
 * LVGL
 * ========================= */
static void lv_tick_cb(void *arg) {
    (void)arg;
    lv_tick_inc(2);
}

static void lv_flush_cb(lv_display_t *disp, const lv_area_t *area,
                        uint8_t *buf) {
    esp_lcd_panel_draw_bitmap(g_panel, area->x1, area->y1, area->x2 + 1,
                              area->y2 + 1, buf);
    lv_display_flush_ready(disp);
}

static void lv_touch_cb(lv_indev_t *indev, lv_indev_data_t *data) {
    (void)indev;

    if (!g_touch) {
        data->state = LV_INDEV_STATE_RELEASED;
        return;
    }

    uint16_t x, y, strength;
    uint8_t cnt = 0;

    esp_lcd_touch_read_data(g_touch);
    if (esp_lcd_touch_get_coordinates(g_touch, &x, &y, &strength, &cnt, 1) &&
        cnt > 0) {
        data->point.x = x;
        data->point.y = y;
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

static void lvgl_task(void *arg) {
    (void)arg;
    while (1) {
        lvgl_lock();
        uint32_t ms = lv_timer_handler();
        lvgl_unlock();

        if (ms > 50)
            ms = 50;
        if (ms < 5)
            ms = 5;
        vTaskDelay(pdMS_TO_TICKS(ms));
    }
}

static esp_err_t lvgl_init(void) {
    s_lvgl_mutex = xSemaphoreCreateMutex();
    if (!s_lvgl_mutex)
        return ESP_ERR_NO_MEM;

    lvgl_lock();
    lv_init();

    g_disp = lv_display_create(LCD_HRES, LCD_VRES);
    lv_display_set_default(g_disp);
    lv_display_set_flush_cb(g_disp, lv_flush_cb);

    size_t buf_size = LCD_HRES * 40 * sizeof(uint16_t);
    g_buf1 = heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM);
    g_buf2 = heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM);
    if (!g_buf1 || !g_buf2) {
        lvgl_unlock();
        return ESP_ERR_NO_MEM;
    }
    lv_display_set_buffers(g_disp, g_buf1, g_buf2, buf_size,
                           LV_DISPLAY_RENDER_MODE_PARTIAL);

    if (g_touch) {
        g_indev = lv_indev_create();
        lv_indev_set_type(g_indev, LV_INDEV_TYPE_POINTER);
        lv_indev_set_read_cb(g_indev, lv_touch_cb);
        lv_indev_set_display(g_indev, g_disp);
    }

    lvgl_unlock();

    esp_timer_create_args_t timer_args = {
        .callback = lv_tick_cb,
        .name = "lv_tick",
    };
    esp_timer_handle_t timer;
    ESP_RETURN_ON_ERROR(esp_timer_create(&timer_args, &timer), TAG,
                        "timer create failed");
    ESP_RETURN_ON_ERROR(esp_timer_start_periodic(timer, 2000), TAG,
                        "timer start failed");

    xTaskCreatePinnedToCore(lvgl_task, "lvgl", 16384, NULL, 4, NULL, 1);

    return ESP_OK;
}

static void create_ui(void) {
    lvgl_lock();

    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x003366), LV_PART_MAIN);

    // Main label - explicitly positioned at center
    lv_obj_t *label = lv_label_create(scr);
    lv_label_set_text(label, "Hello World!\n\nESP32-S3-Touch-LCD-7 V1.2");
    lv_obj_set_style_text_color(label, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    // Force position to exact center
    lv_obj_set_pos(label, (LCD_HRES - lv_obj_get_width(label)) / 2,
                   (LCD_VRES - lv_obj_get_height(label)) / 2);
    lv_obj_update_layout(label);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

    // Touch status at bottom
    lv_obj_t *status = lv_label_create(scr);
    if (g_touch) {
        lv_label_set_text(status, "Touch: ENABLED");
        lv_obj_set_style_text_color(status, lv_color_hex(0x00FF00),
                                    LV_PART_MAIN);
    } else {
        lv_label_set_text(status, "Touch: NOT DETECTED");
        lv_obj_set_style_text_color(status, lv_color_hex(0xFF6666),
                                    LV_PART_MAIN);
    }
    lv_obj_align(status, LV_ALIGN_BOTTOM_MID, 0, -20);

    // Debug info at top-left
    lv_obj_t *info = lv_label_create(scr);
    lv_label_set_text_fmt(info, "Screen: %dx%d", LCD_HRES, LCD_VRES);
    lv_obj_set_style_text_color(info, lv_color_hex(0x888888), LV_PART_MAIN);
    lv_obj_align(info, LV_ALIGN_TOP_LEFT, 10, 10);

    lvgl_unlock();
}

/* =========================
 * Main entry point
 * ========================= */
    void hello_lvgl_start(void) {
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "ESP32-S3-Touch-LCD-7 V1.2 Initialization");
    ESP_LOGI(TAG, "========================================");

    // 1. Initialize I2C
    ESP_ERROR_CHECK(i2c_init());
    ESP_LOGI(TAG, "I2C initialized (SDA=%d, SCL=%d)", PIN_I2C_SDA, PIN_I2C_SCL);

    // 2. Scan I2C bus
    i2c_scan("initial");

    // 3. Initialize CH422G I/O expander
    ESP_ERROR_CHECK(ch422g_init());

    // 4. Turn on LCD backlight via CH422G
    ESP_ERROR_CHECK(ch422g_set_io(EXIO_LCD_BL, 1));
    ESP_LOGI(TAG, "LCD backlight ON");

    // 5. Initialize LCD RGB panel
    ESP_ERROR_CHECK(lcd_init());
    ESP_LOGI(TAG, "LCD panel initialized");

    // Give some time for everything to stabilize
    vTaskDelay(pdMS_TO_TICKS(100));

    // 6. Initialize GT911 touch
    if (touch_driver_init() != ESP_OK) {
        ESP_LOGE(TAG, "Touch driver init failed!");
        g_touch = NULL;
    }

    // 7. Initialize LVGL
    ESP_ERROR_CHECK(lvgl_init());
    ESP_LOGI(TAG, "LVGL initialized");

    // 8. Create UI
    create_ui();

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Ready! Touch: %s", g_touch ? "ENABLED" : "DISABLED");
    ESP_LOGI(TAG, "========================================");
}