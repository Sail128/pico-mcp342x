add_library(mcp342x
    src/mcp342x.c
)

target_include_directories(mcp342x PUBLIC include)

target_link_libraries(mcp342x
    pico_stdlib
    hardware_i2c
)
