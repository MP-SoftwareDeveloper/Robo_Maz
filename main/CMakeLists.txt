idf_component_register(
    SRCS
        "main.cpp"
        "Motor/MecanumRobot.cpp"
        "PWM/MAZPWM.cpp"
    INCLUDE_DIRS
        "."          # lets Motor/ include "../PWM/MAZPWM.hpp"
    REQUIRES
        esp_driver_gpio  # driver/gpio.h (gpio_reset_pin, gpio_set_direction, etc.)
        esp_hw_support
        freertos
        log
        soc
        hal
)
