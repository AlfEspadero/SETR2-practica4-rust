//! Peripheral Configuration Module
//!
//! This module contains initialization code for additional peripherals,
//! ported from the original C project's MX_*_Init functions.
//!
//! Peripherals included:
//! - I2C2: For sensor communication
//! - SPI3: For external devices (WiFi, BLE, etc.)
//! - QSPI: For external flash (not fully supported in embassy-stm32 yet)

use defmt::*;
use embassy_stm32::i2c::Config as I2cConfig;
use embassy_stm32::spi::Config as SpiConfig;
use embassy_stm32::time::Hertz;

/// I2C2 Configuration
///
/// Equivalent to `MX_I2C2_Init()` in the original C code.
/// I2C2 pins on B-L475E-IOT01A:
/// - SCL: PB10
/// - SDA: PB11
///
/// Used for:
/// - HTS221 (Temperature/Humidity sensor)
/// - LPS22HB (Pressure sensor)
/// - LSM6DSL (Accelerometer/Gyroscope)
/// - LIS3MDL (Magnetometer)
/// - VL53L0X (Time-of-Flight sensor)
pub struct I2c2Peripheral;

impl I2c2Peripheral {
    /// Get default I2C configuration
    ///
    /// Original C code:
    /// ```c
    /// hi2c2.Init.Timing = 0x00000E14;  // ~400kHz
    /// hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    /// ```
    pub fn config() -> I2cConfig {
        I2cConfig::default()
    }

    /// Get the I2C frequency (400kHz Fast mode)
    pub fn frequency() -> Hertz {
        Hertz(400_000)
    }
}

/// SPI3 Configuration
///
/// Equivalent to `MX_SPI3_Init()` in the original C code.
/// SPI3 pins on B-L475E-IOT01A:
/// - SCK:  PC10
/// - MISO: PC11
/// - MOSI: PC12
///
/// Used for:
/// - ISM43362 (WiFi module)
/// - SPBTLE-RF (Bluetooth LE module)
/// - SPSGRF-915 (Sub-GHz RF module)
pub struct Spi3Peripheral;

impl Spi3Peripheral {
    /// Initialize SPI3 with default configuration
    ///
    /// Original C code:
    /// ```c
    /// hspi3.Init.Mode = SPI_MODE_MASTER;
    /// hspi3.Init.Direction = SPI_DIRECTION_2LINES;
    /// hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
    /// hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
    /// hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
    /// hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    /// ```
    pub fn config() -> SpiConfig {
        // Mode 0: CPOL=0, CPHA=0 (matches SPI_POLARITY_LOW, SPI_PHASE_1EDGE)
        // Default in Embassy is Mode 0, so no changes needed
        SpiConfig::default()
    }
}

/// Sensor I2C addresses on the B-L475E-IOT01A Discovery board
pub mod sensor_addresses {
    /// HTS221 Temperature and Humidity sensor
    pub const HTS221: u8 = 0x5F;

    /// LPS22HB Pressure sensor
    pub const LPS22HB: u8 = 0x5D;

    /// LSM6DSL Accelerometer and Gyroscope
    pub const LSM6DSL: u8 = 0x6A;

    /// LIS3MDL Magnetometer
    pub const LIS3MDL: u8 = 0x1E;

    /// VL53L0X Time-of-Flight sensor
    pub const VL53L0X: u8 = 0x29;

    /// M24SR64-Y NFC/RFID tag
    pub const M24SR64: u8 = 0x56;
}

/// Example: Read WHO_AM_I from HTS221 sensor
///
/// This demonstrates how to use the I2C peripheral to verify sensor communication.
/// Returns true if the sensor is detected.
///
/// Usage example in main:
/// ```rust,ignore
/// use embassy_stm32::i2c::I2c;
/// 
/// bind_interrupts!(struct I2cIrqs {
///     I2C2_EV => i2c::EventInterruptHandler<peripherals::I2C2>;
///     I2C2_ER => i2c::ErrorInterruptHandler<peripherals::I2C2>;
/// });
///
/// let i2c = I2c::new(
///     p.I2C2,
///     p.PB10,  // SCL
///     p.PB11,  // SDA
///     I2cIrqs,
///     p.DMA1_CH6,
///     p.DMA1_CH7,
///     Hertz(400_000),
///     Default::default(),
/// );
/// ```
pub async fn check_hts221<T: embassy_stm32::i2c::Instance, TXDMA, RXDMA>(
    i2c: &mut embassy_stm32::i2c::I2c<'_, T, TXDMA, RXDMA>,
) -> Result<bool, embassy_stm32::i2c::Error>
where
    TXDMA: embassy_stm32::i2c::TxDma<T>,
    RXDMA: embassy_stm32::i2c::RxDma<T>,
{
    use sensor_addresses::HTS221;

    // Read WHO_AM_I register (0x0F) to verify communication
    let mut who_am_i = [0u8; 1];
    i2c.write_read(HTS221, &[0x0F], &mut who_am_i).await?;

    let detected = who_am_i[0] == 0xBC;
    if detected {
        info!("HTS221 detected! WHO_AM_I = {:#x}", who_am_i[0]);
    } else {
        warn!("HTS221 not found! WHO_AM_I = {:#x}", who_am_i[0]);
    }

    Ok(detected)
}

/// Example: Read temperature from HTS221 sensor
///
/// This demonstrates how to use the I2C peripheral to read sensor data.
/// Note: This is a simplified conversion. For accurate readings, you need
/// to read the calibration coefficients from the sensor.
pub async fn read_hts221_temperature<T: embassy_stm32::i2c::Instance, TXDMA, RXDMA>(
    i2c: &mut embassy_stm32::i2c::I2c<'_, T, TXDMA, RXDMA>,
) -> Result<f32, embassy_stm32::i2c::Error>
where
    TXDMA: embassy_stm32::i2c::TxDma<T>,
    RXDMA: embassy_stm32::i2c::RxDma<T>,
{
    use sensor_addresses::HTS221;

    // First verify the sensor is present
    let mut who_am_i = [0u8; 1];
    i2c.write_read(HTS221, &[0x0F], &mut who_am_i).await?;

    if who_am_i[0] != 0xBC {
        error!("HTS221 not found! WHO_AM_I = {:#x}", who_am_i[0]);
        return Ok(0.0);
    }

    // Enable the sensor (set PD bit and BDU bit)
    i2c.write(HTS221, &[0x20, 0x84]).await?; // CTRL_REG1: PD=1, BDU=1

    // Wait for data to be ready
    embassy_time::Timer::after_millis(20).await;

    // Read temperature registers (0x2A, 0x2B)
    let mut temp_raw = [0u8; 2];
    i2c.write_read(HTS221, &[0x2A | 0x80], &mut temp_raw).await?; // 0x80 for auto-increment

    let temp_raw_value = i16::from_le_bytes(temp_raw);

    // Read calibration values for accurate conversion
    // (Simplified - actual implementation needs full calibration from registers 0x30-0x3F)
    let temperature = temp_raw_value as f32 / 64.0; // Simplified conversion

    info!("Temperature: {} °C (raw: {})", temperature, temp_raw_value);
    Ok(temperature)
}
