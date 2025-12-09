//! Practica4 - STM32L475 FreeRTOS project ported to Rust with Embassy
//!
//! This is a port of the original C/FreeRTOS project to Rust using the Embassy
//! async embedded framework. The project demonstrates:
//! - Async UART communication (TX and RX)
//! - Task-based concurrency (replacing FreeRTOS tasks)
//! - Channel-based communication (replacing FreeRTOS queues)
//! - I2C, SPI peripheral initialization
//!
//! Target: STM32L475VGT (B-L475E-IOT01A Discovery board)

#![no_std]
#![no_main]

mod serial_task;
mod peripherals_config;

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::usart::{Config as UartConfig, Uart};
use embassy_stm32::{bind_interrupts, peripherals, usart, Config};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

use crate::serial_task::serial_rx_task;

// Bind USART1 interrupts to the Embassy interrupt handler
bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
});

/// Configure the system clocks to match the original C project
/// Original config: MSI @ 4MHz -> PLL -> 80MHz SYSCLK
fn configure_clocks() -> Config {
    let mut config = Config::default();

    // Configure RCC for 80MHz operation (matching original C code)
    // MSI Range 6 = 4MHz, PLL multiplier = 40, divider = 2 -> 80MHz
    {
        use embassy_stm32::rcc::*;

        config.rcc.hsi = false;
        config.rcc.msi = Some(MSIRange::RANGE4M); // MSI @ 4MHz (MSIRANGE_6)
        config.rcc.hse = None;
        config.rcc.pll = Some(Pll {
            source: PllSource::MSI,
            prediv: PllPreDiv::DIV1,   // PLLM = 1
            mul: PllMul::MUL40,        // PLLN = 40
            divp: None,
            divq: Some(PllQDiv::DIV2), // PLLQ = 2
            divr: Some(PllRDiv::DIV2), // PLLR = 2 -> 80MHz
        });
        // System clock source is set automatically when PLL is configured
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV1;
        config.rcc.apb2_pre = APBPrescaler::DIV1;

        // Configure LSE for RTC if needed
        config.rcc.ls = LsConfig::default_lse();
    }

    config
}

/// Main entry point - equivalent to main() in the C project
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Practica4 - Embassy Rust");
    info!("Initializing STM32L475...");

    // Initialize with configured clocks (equivalent to SystemClock_Config)
    let config = configure_clocks();
    let p = embassy_stm32::init(config);

    info!("System clock configured @ 80MHz");

    // =========================================================================
    // GPIO Initialization (equivalent to MX_GPIO_Init)
    // =========================================================================

    // LED2 on PB14 (user LED on Discovery board)
    let mut led2 = Output::new(p.PB14, Level::Low, Speed::Low);
    info!("GPIO initialized");

    // =========================================================================
    // USART1 Initialization (equivalent to MX_USART1_UART_Init)
    // Pins: PB6 (TX), PB7 (RX) - connected to ST-Link VCP on B-L475E-IOT01A
    // =========================================================================
    let mut uart_config = UartConfig::default();
    uart_config.baudrate = 115200;
    // 8 data bits, no parity, 1 stop bit (8N1) - default in Embassy

    let uart = Uart::new(
        p.USART1,
        p.PB7,  // RX - ST-Link VCP
        p.PB6,  // TX - ST-Link VCP
        Irqs,
        p.DMA1_CH4, // TX DMA
        p.DMA1_CH5, // RX DMA
        uart_config,
    )
    .unwrap();

    // Split UART into TX and RX halves
    let (_tx, rx) = uart.split();

    info!("USART1 initialized @ 115200 baud");

    // =========================================================================
    // Create Serial Objects (equivalent to CreateSerialObjects)
    // =========================================================================

    // Spawn the serial RX task (equivalent to xTaskCreate for SerialRxTask)
    // This matches the original C implementation exactly - single character processing
    // Output is visible via RTT (probe-rs), not UART
    spawner.spawn(serial_rx_task(rx)).unwrap();

    info!("Serial tasks created");
    info!("Entering main loop...");

    // =========================================================================
    // Main loop - blink LED to show system is running
    // (In the original C code, this was an infinite empty loop since
    //  FreeRTOS scheduler took over)
    // =========================================================================
    loop {
        led2.set_high();
        Timer::after_millis(500).await;
        led2.set_low();
        Timer::after_millis(500).await;
    }
}
