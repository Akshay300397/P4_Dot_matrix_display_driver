idf_component_register(
    SRCS
        "main.c"
        "hub75.c"
        "framebuffer.c"
        "font.c"
        # font_5x7.c and font_3x5.c are #included by font.c — NOT listed here
        # Adding them here would cause duplicate symbol errors
        "eth_uart.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        driver
        freertos
        esp_rom
        log
        spi_flash
)