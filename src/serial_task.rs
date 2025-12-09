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
use embassy_stm32::usart::{UartRx, UartTx};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;

/// Type alias for USART1 RX with DMA (Async mode)
pub type Usart1Rx = UartRx<'static, Async>;

/// Type alias for USART1 TX with DMA (Async mode)
pub type Usart1Tx = UartTx<'static, Async>;

/// Shared TX handle protected by a Mutex (equivalent to xSemaphore in C)
pub static TX: Mutex<ThreadModeRawMutex, Option<Usart1Tx>> = Mutex::new(None);

/// Write a string to UART (equivalent to printf in C)
async fn uart_print(msg: &[u8]) {
	let mut tx_guard = TX.lock().await;
	if let Some(ref mut tx) = *tx_guard {
		let _ = tx.write(msg).await;
	}
}

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
					// Original: printf("\e[1;1H\e[2J"); printf("Terminal Limpiada\r\n");
					uart_print(b"\x1b[1;1H\x1b[2JTerminal Limpiada\r\n").await;
				} else {
					// Original: printf("Nuevo dato recibido: %c\r\n", c);
					// Build the message manually since we can't use format! in no_std
					let mut msg: [u8; 32] = [0; 32];
					let prefix = b"Nuevo dato recibido: ";
					msg[..prefix.len()].copy_from_slice(prefix);
					msg[prefix.len()] = c;
					msg[prefix.len() + 1] = b'\r';
					msg[prefix.len() + 2] = b'\n';
					uart_print(&msg[..prefix.len() + 3]).await;
				}
			}
			Err(_e) => {
				error!("UART read error");
			}
		}
	}
}
