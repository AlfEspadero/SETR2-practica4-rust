# Practica4 - Rust Embassy Port

This project is a complete port of the original C/FreeRTOS STM32L475 project to Rust using the Embassy async embedded framework.

For a detailed comparison between the original C/FreeRTOS implementation and this Rust/Embassy port, please refer to the [COMPARISON.md](./COMPARISON.md) file.

## Target Hardware

- **Board**: B-L475E-IOT01A Discovery Kit
- **MCU**: STM32L475VGT6 (Cortex-M4F, 80MHz, 1MB Flash, 128KB SRAM)

## Project Structure

```
practica4-rust/
├── Cargo.toml              # Project dependencies
├── .cargo/config.toml      # Build configuration
├── rust-toolchain.toml     # Rust version pinning
├── build.rs                # Build script
└── src/
    ├── main.rs             # Main entry point (replaces main.c)
    ├── serial_task.rs      # UART handling (replaces SerialTask.c)
    └── peripherals_config.rs # I2C, SPI config (replaces MX_*_Init)
```

## Mapping from C/FreeRTOS to Rust/Embassy

| C/FreeRTOS | Rust/Embassy |
|------------|--------------|
| `main()` | `#[embassy_executor::main] async fn main()` |
| `xTaskCreate()` | `#[embassy_executor::task]` + `spawner.spawn()` |
| `xSemaphoreCreateBinary()` | `Mutex<T>` or `Signal` |
| `xQueueCreate()` | `Channel<T, N>` |
| `HAL_UART_Transmit_IT()` | `uart_tx.write().await` |
| `HAL_UART_Receive_IT()` | `uart_rx.read().await` |
| `vTaskDelay()` | `Timer::after_millis().await` |
| Interrupt callbacks | Handled automatically by Embassy |

## Prerequisites

### 1. Install Rust

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

### 2. Add the ARM target

```bash
rustup target add thumbv7em-none-eabihf
```

### 3. Install probe-rs (for flashing and debugging)

```bash
# Using cargo
cargo install probe-rs-tools

# Or using the install script
curl --proto '=https' --tlsv1.2 -LsSf https://github.com/probe-rs/probe-rs/releases/latest/download/probe-rs-tools-installer.sh | sh
```

### 4. Install flip-link (optional, for stack overflow protection)

```bash
cargo install flip-link
```

## Building

```bash
cd practica4-rust

# Debug build
cargo build

# Release build (optimized)
cargo build --release
```

## Flashing

Make sure your B-L475E-IOT01A board is connected via USB (ST-Link).

```bash
# Build and flash (debug)
cargo run

# Build and flash (release)
cargo run --release

# Flash only (without rebuilding)
probe-rs run --chip STM32L475VGTx target/thumbv7em-none-eabihf/debug/practica4-rust
```

## Debugging

### Using probe-rs

```bash
# Start GDB server
probe-rs gdb --chip STM32L475VGTx target/thumbv7em-none-eabihf/debug/practica4-rust

# In another terminal, connect with GDB
arm-none-eabi-gdb -x .gdbinit target/thumbv7em-none-eabihf/debug/practica4-rust
```

### Using VS Code

Install the `probe-rs` extension and use the included launch configuration.

### Viewing Debug Output (defmt)

The project uses `defmt` for logging, which outputs via RTT (Real-Time Transfer):

```bash
# View logs in real-time
probe-rs run --chip STM32L475VGTx target/thumbv7em-none-eabihf/debug/practica4-rust
```

You should see output like:
```
INFO  Practica4 - Embassy Rust
INFO  Initializing STM32L475...
INFO  System clock configured @ 80MHz
INFO  GPIO initialized
INFO  USART1 initialized @ 115200 baud
INFO  Serial tasks created
INFO  SerialRxTask started
```

## Serial Terminal

Connect to the serial port to interact with the UART:

```bash
# Linux
screen /dev/ttyACM0 115200

# Or using minicom
minicom -D /dev/ttyACM0 -b 115200

# Or using picocom
picocom -b 115200 /dev/ttyACM0
```

On the Discovery board, the USART1 is connected to the ST-Link Virtual COM Port.

## Features Ported

- [x] System clock configuration (80MHz from MSI + PLL)
- [x] GPIO initialization (LED2 on PB14)
- [x] USART1 async TX/RX (115200 baud, 8N1)
- [x] FreeRTOS task → Embassy async task
- [x] Semaphore → Mutex
- [x] Queue → Channel
- [x] I2C2 configuration (for sensors)
- [x] SPI3 configuration (for WiFi/BLE modules)

## Differences from C Version

1. **Memory Safety**: Rust's ownership system prevents data races at compile time
2. **No Dynamic Allocation**: All tasks and buffers are statically allocated
3. **Async/Await**: Natural syntax for concurrent operations
4. **No Manual ISR Management**: Embassy handles interrupt routing automatically
5. **Compile-Time Task Validation**: Invalid task configurations fail at compile time

## Troubleshooting

### "No probe found"
- Ensure the board is connected via USB
- Check if the ST-Link firmware is up to date
- On Linux, you may need udev rules:
  ```bash
  sudo cp 99-probe-rs.rules /etc/udev/rules.d/
  sudo udevadm control --reload-rules
  ```

### "Target not halted"
- Press the reset button on the board
- Try: `probe-rs reset --chip STM32L475VGTx`

### Build errors with embassy-stm32
- Make sure you're using the correct feature flags in Cargo.toml
- Run `cargo update` to get the latest compatible versions

## License

MIT License - Same as the original project.

## Resources

- [Embassy Documentation](https://embassy.dev/)
- [embassy-stm32 Examples](https://github.com/embassy-rs/embassy/tree/main/examples/stm32l4)
- [B-L475E-IOT01A User Manual](https://www.st.com/resource/en/user_manual/um2153-discovery-kit-for-iot-node-multichannel-communication-with-stm32l4-stmicroelectronics.pdf)
- [Rust Embedded Book](https://docs.rust-embedded.org/book/)
