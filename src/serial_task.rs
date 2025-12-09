//! Serial Task Module
//!
//! This module is a Rust port of SerialTask.c from the original FreeRTOS project.
//! It provides async UART communication for receiving bytes.
//!
//! ## Original C Code Mapping:
//! - SerialRxTask -> serial_rx_task
//! - HAL_UART_RxCpltCallback -> Handled automatically by Embassy async read
//! - xQueue -> Not needed (Embassy handles buffering internally)

use defmt::*;
use embassy_stm32::mode::Async;
use embassy_stm32::usart::UartRx;

/// Type alias for USART1 RX with DMA (Async mode)
pub type Usart1Rx = UartRx<'static, Async>;

/// Serial RX Task - Async task for receiving UART data
///
/// This is a direct port of SerialRxTask from the original C code.
/// In Embassy, we do not need an explicit queue because the async read
/// already handles the interrupt-driven reception internally.
#[embassy_executor::task]
pub async fn serial_rx_task(mut rx: Usart1Rx) {
    info!("SerialRxTask started");

    let mut buf = [0u8; 1];

    loop {
        // Read one byte (equivalent to xQueueReceive with portMAX_DELAY)
        match rx.read(&mut buf).await {
            Ok(_) => {
                let c = buf[0];

                if c == b'\r' {
                    // Original: printf("Terminal Limpiada\r\n");
                    info!("Terminal Limpiada");
                } else {
                    // Original: printf("Nuevo dato recibido: %c\r\n", c);
                    info!("Nuevo dato recibido: {}", c as char);
                }
            }
            Err(_e) => {
                error!("UART read error");
            }
        }
    }
}
