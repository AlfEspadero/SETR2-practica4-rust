//! Serial Task Module
//!
//! This module is a Rust port of SerialTask.c from the original FreeRTOS project.
//! It provides async UART communication with:
//! - SerialWriter: Thread-safe UART transmitter (replaces xSemaphore + HAL_UART_Transmit_IT)
//! - serial_rx_task: Async task for receiving bytes (replaces SerialRxTask + xQueue)
//!
//! ## Original C Code Mapping:
//! - `xSemaphore` -> `Mutex<UartTx>`
//! - `xQueue` -> `Channel<u8, 16>`
//! - `SerialRxTask` -> `serial_rx_task`
//! - `SerialSendByte` -> `SerialWriter::write_byte`
//! - `HAL_UART_TxCpltCallback` -> Handled automatically by Embassy DMA
//! - `HAL_UART_RxCpltCallback` -> Handled automatically by Embassy async read

use defmt::*;
use embassy_stm32::usart::{UartRx, UartTx};
use embassy_stm32::peripherals::{DMA1_CH4, DMA1_CH5, USART1};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embedded_io_async::Write;

/// Type alias for USART1 TX with DMA
pub type Usart1Tx = UartTx<'static, USART1, DMA1_CH4>;

/// Type alias for USART1 RX with DMA
pub type Usart1Rx = UartRx<'static, USART1, DMA1_CH5>;

/// Serial Writer - Thread-safe UART transmitter
///
/// This replaces the combination of:
/// - `xSemaphore = xSemaphoreCreateBinary()`
/// - `HAL_UART_Transmit_IT(&huart1, ...)`
/// - `HAL_UART_TxCpltCallback`
///
/// The Mutex ensures only one task can transmit at a time (like the binary semaphore).
/// Embassy's async UART automatically handles the interrupt-driven transmission.
pub struct SerialWriter {
    tx: Mutex<CriticalSectionRawMutex, Usart1Tx>,
}

impl SerialWriter {
    /// Create a new SerialWriter from a UART transmitter
    pub fn new(tx: Usart1Tx) -> Self {
        Self {
            tx: Mutex::new(tx),
        }
    }

    /// Send a single byte over UART
    ///
    /// Equivalent to `SerialSendByte` in the original C code:
    /// ```c
    /// void SerialSendByte(char data) {
    ///     BaseType_t status = xSemaphoreTake(xSemaphore, portMAX_DELAY);
    ///     HAL_UART_Transmit_IT(&huart1, (uint8_t*)&data, 1);
    /// }
    /// ```
    pub async fn write_byte(&self, byte: u8) {
        let mut tx = self.tx.lock().await;
        let _ = tx.write(&[byte]).await;
    }

    /// Send a string over UART
    pub async fn write_str(&self, s: &str) {
        let mut tx = self.tx.lock().await;
        let _ = tx.write_all(s.as_bytes()).await;
    }

    /// Send a buffer over UART
    pub async fn write_buf(&self, buf: &[u8]) {
        let mut tx = self.tx.lock().await;
        let _ = tx.write_all(buf).await;
    }
}

/// Serial RX Task - Async task for receiving UART data
///
/// This replaces `SerialRxTask` from the original C code:
/// ```c
/// void SerialRxTask(void *argument) {
///     char c;
///     for (;;) {
///         if (xQueueReceive(xQueue, &c, portMAX_DELAY) == pdTRUE) {
///             if (c == '\r') {
///                 printf("\e[1;1H\e[2J");
///                 printf("Terminal Limpiada\r\n");
///             }
///             else printf("Nuevo dato recibido: %c\r\n", c);
///         }
///     }
/// }
/// ```
///
/// In Embassy, we don't need an explicit queue because the async read
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
                    // Clear terminal (ANSI escape sequence)
                    // Original: printf("\e[1;1H\e[2J");
                    info!("Terminal Limpiada");
                    // Note: To actually send the escape sequence, we'd need
                    // access to the TX. For now, just log it.
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

/// Alternative RX task with echo functionality
///
/// This version echoes received characters back, which is useful for
/// interactive terminal applications.
#[embassy_executor::task]
pub async fn serial_echo_task(
    mut rx: Usart1Rx,
    writer: &'static SerialWriter,
) {
    info!("SerialEchoTask started");

    let mut buf = [0u8; 1];
    let mut line_buf = [0u8; 128];
    let mut line_pos = 0usize;

    loop {
        match rx.read(&mut buf).await {
            Ok(_) => {
                let c = buf[0];

                match c {
                    b'\r' | b'\n' => {
                        // Send newline
                        writer.write_str("\r\n").await;

                        if line_pos > 0 {
                            // Process the received line
                            let line = &line_buf[..line_pos];
                            if let Ok(s) = core::str::from_utf8(line) {
                                info!("Received line: {}", s);
                            }

                            // Echo back
                            writer.write_str("Recibido: ").await;
                            writer.write_buf(line).await;
                            writer.write_str("\r\n").await;

                            line_pos = 0;
                        }
                    }
                    0x7F | 0x08 => {
                        // Backspace
                        if line_pos > 0 {
                            line_pos -= 1;
                            writer.write_str("\x08 \x08").await; // Erase character
                        }
                    }
                    0x03 => {
                        // Ctrl+C - clear terminal
                        writer.write_str("\x1b[1;1H\x1b[2J").await;
                        writer.write_str("Terminal Limpiada\r\n").await;
                        line_pos = 0;
                    }
                    _ => {
                        // Regular character
                        if line_pos < line_buf.len() - 1 {
                            line_buf[line_pos] = c;
                            line_pos += 1;
                            // Echo character
                            writer.write_byte(c).await;
                        }
                    }
                }
            }
            Err(_e) => {
                error!("UART read error");
            }
        }
    }
}
