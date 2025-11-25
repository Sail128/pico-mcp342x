#ifndef MCP342X_H
#define MCP342X_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Default I2C address for MCP342x.
 * The address depends on the floating pins (Adr0, Adr1).
 * 0x68 is common (Adr0=Float, Adr1=Float or similar depending on specific chip variant).
 * Check datasheet for your specific hardware configuration.
 */
#define MCP342X_DEFAULT_ADDRESS 0x68

/**
 * @brief PGA Gain settings.
 */
typedef enum {
    MCP342X_GAIN_1X = 0, /**< 1x Gain */
    MCP342X_GAIN_2X,     /**< 2x Gain */
    MCP342X_GAIN_4X,     /**< 4x Gain */
    MCP342X_GAIN_8X      /**< 8x Gain */
} mcp342x_gain_t;

/**
 * @brief Resolution and Sample Rate settings.
 */
typedef enum {
    MCP342X_BIT_12 = 0, /**< 12-bit resolution, 240 SPS */
    MCP342X_BIT_14,     /**< 14-bit resolution, 60 SPS */
    MCP342X_BIT_16,     /**< 16-bit resolution, 15 SPS */
    MCP342X_BIT_18      /**< 18-bit resolution, 3.75 SPS */
} mcp342x_resolution_t;

/**
 * @brief Operating modes.
 */
typedef enum {
    MCP342X_MODE_ONE_SHOT = 0, /**< One-shot conversion mode */
    MCP342X_MODE_CONTINUOUS    /**< Continuous conversion mode */
} mcp342x_mode_t;

/**
 * @brief MCP342x Device Structure.
 * Holds the configuration and state of the device.
 */
typedef struct {
    i2c_inst_t *i2c;                /**< I2C instance (e.g., i2c0, i2c1) */
    uint8_t address;                /**< I2C address of the device */
    uint8_t current_channel;        /**< Currently selected channel (0-3) */
    mcp342x_gain_t gain;            /**< Current gain setting */
    mcp342x_resolution_t resolution;/**< Current resolution setting */
    mcp342x_mode_t mode;            /**< Current operating mode */
    uint8_t config_reg;             /**< Last written configuration register value */
} mcp342x_t;

/**
 * @brief Initialize the MCP342x device structure.
 * 
 * @param dev Pointer to mcp342x_t structure.
 * @param i2c I2C instance (i2c0 or i2c1).
 * @param address I2C address of the device.
 */
void mcp342x_init(mcp342x_t *dev, i2c_inst_t *i2c, uint8_t address);

/**
 * @brief Configure the device parameters.
 * This function updates the internal structure and writes the configuration to the device.
 * 
 * @param dev Pointer to mcp342x_t structure.
 * @param channel Channel to read (1-4). Note: 1-based index for user convenience, converted to 0-3 internally.
 * @param mode Operating mode (One-Shot or Continuous).
 * @param resolution Resolution (12, 14, 16, or 18 bits).
 * @param gain Gain (1x, 2x, 4x, 8x).
 * @return int 0 on success, negative on error (PICO_ERROR_GENERIC or I2C error).
 */
int mcp342x_configure(mcp342x_t *dev, uint8_t channel, mcp342x_mode_t mode, mcp342x_resolution_t resolution, mcp342x_gain_t gain);

/**
 * @brief Start a one-shot conversion.
 * Only useful if the device is in One-Shot mode.
 * 
 * @param dev Pointer to mcp342x_t structure.
 * @return int 0 on success, negative on error.
 */
int mcp342x_start_conversion(mcp342x_t *dev);

/**
 * @brief Read raw value from the ADC.
 * This function reads the data registers. In One-Shot mode, it checks the RDY bit.
 * If the conversion is not ready, it may return a specific status or wait depending on implementation.
 * Here, it attempts to read. If the RDY bit indicates old data (in One-Shot), it might be up to the user to poll.
 * However, for simplicity, this function reads the current register value.
 * 
 * @param dev Pointer to mcp342x_t structure.
 * @param val Pointer to store the raw signed integer value.
 * @param config Pointer to store the read configuration byte (optional, can be NULL).
 * @return int 0 on success (new data), 1 if data is not ready (in One-Shot mode), negative on I2C error.
 */
int mcp342x_read_raw(mcp342x_t *dev, int32_t *val, uint8_t *config);

/**
 * @brief Read voltage from the ADC.
 * Converts the raw value to voltage based on current gain and resolution settings.
 * 
 * @param dev Pointer to mcp342x_t structure.
 * @param voltage Pointer to store the voltage (in Volts).
 * @return int 0 on success, negative on error.
 */
int mcp342x_read_voltage(mcp342x_t *dev, double *voltage);

/**
 * @brief General purpose read function that handles configuration and reading in one go.
 * Useful for simple polling.
 * 
 * @param dev Pointer to mcp342x_t structure.
 * @param channel Channel to read.
 * @param voltage Pointer to store the result.
 * @return int 0 on success, negative on error.
 */
int mcp342x_read_channel_voltage(mcp342x_t *dev, uint8_t channel, double *voltage);

/**
 * @brief   Get the expected sample time in milliseconds
 *
 * This function returns the expected sample time based on the configured
 * resolution of the device.
 *
 * @param[in] dev       Device descriptor of the MCP342X device
 *
 * @return              Expected sample time in milliseconds
 */
int mcp342x_expected_sample_time(mcp342x_t *dev);

#ifdef __cplusplus
}
#endif

#endif // MCP342X_H
