add_executable(mcp342x_example
    example.c
)

target_link_libraries(mcp342x_example
    pico_stdlib
    mcp342x
)

# Enable USB output for printf
pico_enable_stdio_usb(mcp342x_example 1)
pico_enable_stdio_uart(mcp342x_example 0)

pico_add_extra_outputs(mcp342x_example)
