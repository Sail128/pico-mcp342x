#include "mcp342x.h"
#include <stdio.h>

// Configuration Register Bits
#define MCP342X_CMD_RDY_BIT     7
#define MCP342X_CMD_C1_BIT      6
#define MCP342X_CMD_C0_BIT      5
#define MCP342X_CMD_OC_BIT      4
#define MCP342X_CMD_S1_BIT      3
#define MCP342X_CMD_S0_BIT      2
#define MCP342X_CMD_G1_BIT      1
#define MCP342X_CMD_G0_BIT      0

void mcp342x_init(mcp342x_t *dev, i2c_inst_t *i2c, uint8_t address) {
    dev->i2c = i2c;
    dev->address = address;
    dev->current_channel = 0;
    dev->gain = MCP342X_GAIN_1X;
    dev->resolution = MCP342X_BIT_12;
    dev->mode = MCP342X_MODE_CONTINUOUS;
    dev->config_reg = 0;
}

static uint8_t mcp342x_build_config(mcp342x_t *dev, bool initiate_conversion) {
    uint8_t config = 0;
    
    // RDY bit: 1 to initiate conversion in one-shot mode, 0 otherwise (effect depends on mode)
    // When writing: 1 = Initiate a new conversion (One-Shot). 0 = No effect.
    if (initiate_conversion && dev->mode == MCP342X_MODE_ONE_SHOT) {
        config |= (1 << MCP342X_CMD_RDY_BIT);
    }

    // Channel Selection
    config |= ((dev->current_channel & 0x03) << MCP342X_CMD_C0_BIT);

    // Conversion Mode
    if (dev->mode == MCP342X_MODE_CONTINUOUS) {
        config |= (1 << MCP342X_CMD_OC_BIT);
    }

    // Sample Rate / Resolution
    config |= ((dev->resolution & 0x03) << MCP342X_CMD_S0_BIT);

    // Gain
    config |= ((dev->gain & 0x03) << MCP342X_CMD_G0_BIT);

    return config;
}

int mcp342x_configure(mcp342x_t *dev, uint8_t channel, mcp342x_mode_t mode, mcp342x_resolution_t resolution, mcp342x_gain_t gain) {
    if (channel < 1 || channel > 4) {
        return PICO_ERROR_INVALID_ARG;
    }

    dev->current_channel = channel - 1; // Convert 1-based to 0-based
    dev->mode = mode;
    dev->resolution = resolution;
    dev->gain = gain;

    uint8_t config = mcp342x_build_config(dev, false);
    dev->config_reg = config;

    int ret = i2c_write_blocking(dev->i2c, dev->address, &config, 1, false);
    if (ret == PICO_ERROR_GENERIC || ret != 1) {
        return PICO_ERROR_GENERIC;
    }

    return 0;
}

int mcp342x_start_conversion(mcp342x_t *dev) {
    if (dev->mode != MCP342X_MODE_ONE_SHOT) {
        return 0; // Not needed in continuous mode, but not an error
    }

    uint8_t config = mcp342x_build_config(dev, true);
    dev->config_reg = config;

    int ret = i2c_write_blocking(dev->i2c, dev->address, &config, 1, false);
    if (ret == PICO_ERROR_GENERIC || ret != 1) {
        return PICO_ERROR_GENERIC;
    }
    return 0;
}

int mcp342x_read_raw(mcp342x_t *dev, int32_t *val, uint8_t *config_out) {
    uint8_t buffer[4];
    size_t read_len;

    // 18-bit mode returns 3 data bytes + 1 config byte
    // Others return 2 data bytes + 1 config byte
    if (dev->resolution == MCP342X_BIT_18) {
        read_len = 4;
    } else {
        read_len = 3;
    }

    int ret = i2c_read_blocking(dev->i2c, dev->address, buffer, read_len, false);
    if (ret == PICO_ERROR_GENERIC || ret != read_len) {
        return PICO_ERROR_GENERIC;
    }

    uint8_t config_byte = buffer[read_len - 1];
    if (config_out) {
        *config_out = config_byte;
    }

    // Check RDY bit (Bit 7). 
    // 0 = Data Ready (New data updated)
    // 1 = Output not updated (In One-Shot: Conversion not complete)
    if (config_byte & (1 << MCP342X_CMD_RDY_BIT)) {
        // Data not ready
        return 1; 
    }

    int32_t raw_val = 0;
    if (dev->resolution == MCP342X_BIT_18) {
        // 18-bit: Byte 0 (Upper), Byte 1 (Middle), Byte 2 (Lower)
        // Combine 3 bytes into a 32-bit integer
        raw_val = ((int32_t)buffer[0] << 16) | ((int32_t)buffer[1] << 8) | buffer[2];
        
        // Sign extension for 18-bit
        // The 18th bit (bit 17, 0-indexed) is the sign bit.
        // If bit 17 is set, the number is negative. We need to extend this sign to the full 32-bit integer.
        // 0x20000 is bit 17. 0xFFFC0000 is the mask for the upper bits that need to be set to 1.
        if (raw_val & 0x20000) {
            raw_val |= 0xFFFC0000;
        }
    } else {
        // 12, 14, 16-bit: Byte 0 (Upper), Byte 1 (Lower)
        raw_val = ((int32_t)buffer[0] << 8) | buffer[1];

        // Sign extension
        if (dev->resolution == MCP342X_BIT_16) {
            // 16-bit is naturally handled by int16_t cast or sign extension of 16th bit
            // Since raw_val is int32_t, casting to int16_t and back to int32_t preserves the sign.
            raw_val = (int16_t)raw_val; 
        } else if (dev->resolution == MCP342X_BIT_14) {
            // 14-bit: Bit 13 is sign bit.
            // 0x2000 is bit 13. 0xFFFFC000 is the mask for upper bits.
            if (raw_val & 0x2000) {
                raw_val |= 0xFFFFC000;
            }
        } else { // 12-bit
            // 12-bit: Bit 11 is sign bit.
            // 0x800 is bit 11. 0xFFFFF000 is the mask for upper bits.
            if (raw_val & 0x800) {
                raw_val |= 0xFFFFF000;
            }
        }
    }

    *val = raw_val;
    return 0;
}

int mcp342x_read_voltage(mcp342x_t *dev, double *voltage) {
    int32_t raw;
    uint8_t config;
    int ret = mcp342x_read_raw(dev, &raw, &config);
    
    if (ret < 0) return ret; // Error
    if (ret == 1) return 1;  // Not ready

    double gain_val = 1.0;

    switch (dev->gain) {
        case MCP342X_GAIN_1X: gain_val = 1.0; break;
        case MCP342X_GAIN_2X: gain_val = 2.0; break;
        case MCP342X_GAIN_4X: gain_val = 4.0; break;
        case MCP342X_GAIN_8X: gain_val = 8.0; break;
    }

    // Calculate LSB size and convert raw value to voltage.
    // Formula: Voltage = (Raw Code * LSB) / PGA
    // Where LSB = (2 * Vref) / 2^N
    // Vref = 2.048V
    // So: Voltage = (Raw * 2 * 2.048) / (Gain * 2^N)
    // Simplified: Voltage = (Raw * 2.048) / (Gain * 2^(N-1))
    
    int n_bits = 12;
    switch (dev->resolution) {
        case MCP342X_BIT_12: n_bits = 12; break;
        case MCP342X_BIT_14: n_bits = 14; break;
        case MCP342X_BIT_16: n_bits = 16; break;
        case MCP342X_BIT_18: n_bits = 18; break;
    }

    double divisor = (double)(1UL << (n_bits - 1));
    *voltage = ((double)raw * 2.048) / (gain_val * divisor);

    return 0;
}

int mcp342x_read_channel_voltage(mcp342x_t *dev, uint8_t channel, double *voltage) {
    // If channel is different, we must re-configure
    if ((channel - 1) != dev->current_channel) {
        int ret = mcp342x_configure(dev, channel, dev->mode, dev->resolution, dev->gain);
        if (ret != 0) return ret;
        
        // If in continuous mode, we might need to wait for the first conversion?
        // The datasheet says: "When the configuration register is written... the device aborts the current conversion and starts a new conversion."
        // So we should wait for the conversion time.
        // For simplicity, the user should handle timing or we can poll read_raw until ready.
    }

    // Poll for data ready
    int32_t raw;
    uint8_t config;
    int ret;
    int timeout = 100; // Arbitrary timeout counter
    
    do {
        ret = mcp342x_read_voltage(dev, voltage);
        if (ret == 0) break; // Ready
        if (ret < 0) return ret; // Error
        
        sleep_ms(10); // Wait a bit
        timeout--;
    } while (timeout > 0);

    if (timeout == 0) return PICO_ERROR_TIMEOUT;


    return 0;
}


int mcp342x_expected_sample_time(mcp342x_t *dev){
    int sample_time_ms = 1000/3.75;
    switch (dev->resolution) {
        case MCP342X_BIT_12: sample_time_ms = 1000/240; break;
        case MCP342X_BIT_14: sample_time_ms = 1000/60; break;
        case MCP342X_BIT_16: sample_time_ms = 1000/15; break;
        case MCP342X_BIT_18: sample_time_ms = 1000/3.75; break;
    }
    return sample_time_ms;
}