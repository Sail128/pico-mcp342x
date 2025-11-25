#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "mcp342x.h"

#define I2C_PORT i2c0
#define I2C_SDA 4
#define I2C_SCL 5

// The MCP342x has an internal reference of 2.048V.
// In single-ended mode (CH- connected to GND), the maximum measurable voltage is 2.048V (at 1x Gain).
// To measure a 0-10V signal, you MUST use a voltage divider.
// Example: R1 = 39k (Series), R2 = 10k (To Ground).
// V_in_adc = V_signal * R2 / (R1 + R2)
// V_signal = V_in_adc * (R1 + R2) / R2
// Ratio = (39+10)/10 = 4.9. 
// Max Input = 2.048 * 4.9 = 10.035V.
#define VOLTAGE_DIVIDER_RATIO 4.9 

int main() {
    stdio_init_all();

    // Initialize I2C
    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Wait for serial monitor
    sleep_ms(2000);
    printf("MCP342x 0-10V Measurement Example\n");
    printf("Note: Ensure you are using a voltage divider (Ratio %.1f) for 10V signals!\n", VOLTAGE_DIVIDER_RATIO);

    mcp342x_t adc;
    mcp342x_init(&adc, I2C_PORT, MCP342X_DEFAULT_ADDRESS);

    // ------------------------------------------------------------------------------
    // SCENARIO 1: Continuous Measurement of a Single Signal (e.g., on Channel 1)
    // ------------------------------------------------------------------------------
    printf("\n--- Scenario 1: Continuous Mode (Channel 1) ---\n");
    
    // Configure: Channel 1, Continuous, 12-bit (fastest), 1x Gain
    // We use 1x Gain to maximize the input range (0-2.048V).
    mcp342x_configure(&adc, 1, MCP342X_MODE_CONTINUOUS, MCP342X_BIT_12, MCP342X_GAIN_1X);

    for (int i = 0; i < 10; i++) {
        double adc_voltage;
        int ret = mcp342x_read_voltage(&adc, &adc_voltage);
        
        if (ret == 0) {
            double actual_voltage = adc_voltage * VOLTAGE_DIVIDER_RATIO;
            printf("Ch1 (Continuous): ADC=%.4f V, Actual=%.4f V\n", adc_voltage, actual_voltage);
        } else if (ret == 1) {
            // In continuous mode, this means we read faster than the conversion rate.
            // For 12-bit (240 SPS), this is unlikely with the sleep below.
            printf("Ch1: Data not ready\n");
        } else {
            printf("Ch1: Error reading ADC\n");
        }
        sleep_ms(100); // Sample every 100ms
    }

    // ------------------------------------------------------------------------------
    // SCENARIO 2: One-Shot Measurement (e.g., on Channel 2)
    // ------------------------------------------------------------------------------
    printf("\n--- Scenario 2: One-Shot Mode (Channel 2) ---\n");
    printf("Useful for low power or infrequent sampling.\n");

    // Configure: Channel 2, One-Shot, 16-bit (high precision), 1x Gain
    mcp342x_configure(&adc, 2, MCP342X_MODE_ONE_SHOT, MCP342X_BIT_16, MCP342X_GAIN_1X);

    for (int i = 0; i < 3; i++) {
        printf("Triggering conversion on Ch2...\n");
        mcp342x_start_conversion(&adc);

        // Poll for completion
        // 16-bit conversion takes ~67ms (15 SPS)
        int32_t raw;
        uint8_t config;
        int ret;
        do {
            ret = mcp342x_read_raw(&adc, &raw, &config);
            if (ret == 0) break; // Data Ready
            sleep_ms(10);
        } while (ret == 1); // ret=1 means Not Ready

        if (ret == 0) {
            double adc_voltage;
            mcp342x_read_voltage(&adc, &adc_voltage); // Convert the raw value we just waited for
            double actual_voltage = adc_voltage * VOLTAGE_DIVIDER_RATIO;
            printf("Ch2 (One-Shot): ADC=%.4f V, Actual=%.4f V\n", adc_voltage, actual_voltage);
        } else {
            printf("Ch2: Error\n");
        }
        sleep_ms(1000);
    }

    // ------------------------------------------------------------------------------
    // SCENARIO 3: Scanning Multiple Channels (Simulated Continuous)
    // ------------------------------------------------------------------------------
    printf("\n--- Scenario 3: Scanning Channels 1, 2, 3, 4 ---\n");
    printf("Since the hardware doesn't scan automatically, we loop through them.\n");

    // We can use the helper function `mcp342x_read_channel_voltage` which handles 
    // reconfiguration and waiting.
    // Note: Switching channels resets the internal conversion timer, so the effective 
    // sample rate per channel is lower than the single-channel rate.
    
    // Set common settings (applied when we switch channels)
    adc.resolution = MCP342X_BIT_14; // 60 SPS
    adc.gain = MCP342X_GAIN_1X;
    adc.mode = MCP342X_MODE_CONTINUOUS; // We leave it in continuous, but switching channels restarts it.

    for (int loop = 0; loop < 3; loop++) {
        printf("Scan Loop %d:\n", loop + 1);
        for (int ch = 1; ch <= 4; ch++) {
            double adc_voltage;
            // This function will:
            // 1. Write config to select 'ch'
            // 2. Wait for conversion to complete (poll)
            // 3. Return voltage
            int ret = mcp342x_read_channel_voltage(&adc, ch, &adc_voltage);
            
            if (ret == 0) {
                double actual_voltage = adc_voltage * VOLTAGE_DIVIDER_RATIO;
                printf("  Ch%d: %.4f V (Actual: %.4f V)\n", ch, adc_voltage, actual_voltage);
            } else {
                printf("  Ch%d: Error %d\n", ch, ret);
            }
        }
        sleep_ms(1000);
    }

    return 0;
}
