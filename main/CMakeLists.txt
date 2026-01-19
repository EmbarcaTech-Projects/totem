idf_component_register(
    SRCS 
        "main.c"
        "hello_lvgl.c"
    INCLUDE_DIRS 
        "."
    REQUIRES 
        lvgl 
        esp_lcd_touch 
        esp_lcd_touch_gt911
        esp_timer
)

if(NOT CMAKE_BUILD_EARLY_EXPANSION)
    target_compile_definitions(${COMPONENT_LIB} PUBLIC LV_CONF_INCLUDE_SIMPLE=1)
endif()