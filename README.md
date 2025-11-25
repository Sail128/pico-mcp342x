# Pico MCP342x Library

This is a Raspberry Pi Pico library for the MCP342x series of I2C ADCs (MCP3421, MCP3422, MCP3423, MCP3424, etc.).

## Structure

- `mcp342x/`: The library source code.
  - `include/`: Header files.
  - `src/`: Source files.
- `examples/`: Example usage of the library.

## Usage

To use this library in your project:

1. Copy the `mcp342x` folder to your project or add this repository as a submodule.
2. In your `CMakeLists.txt`, add:
   ```cmake
   add_subdirectory(mcp342x)
   target_link_libraries(your_target mcp342x)
   ```
3. Include the header in your code:
   ```c
   #include "mcp342x.h"
   ```

## Building the Example

1. Set up the Pico SDK environment.
2. Create a build directory: `mkdir build && cd build`
3. Run CMake: `cmake ..`
4. Build: `make`
