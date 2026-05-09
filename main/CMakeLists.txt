idf_component_register(
    SRCS
        "main.cpp"
        "Motor/MecanumRobot.cpp"
        "PWM/MAZPWM.cpp"
    INCLUDE_DIRS
        "."
    REQUIRES
        esp_driver_gpio   # driver/gpio.h
        esp_driver_mcpwm  # driver/mcpwm_*.h — IDF MCPWM driver
        freertos
        log
)
