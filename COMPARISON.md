# STM32 Embedded Development: C/FreeRTOS vs Rust/Embassy

A side-by-side comparison of the same UART echo application implemented in both paradigms.

## 📊 Metrics Comparison

| Metric | C/FreeRTOS (STM32CubeIDE) | Rust/Embassy | Winner |
|--------|---------------------------|--------------|--------|
| **User Code (Lines)** | 2,479 lines | 530 lines | 🦀 Rust (78% less) |
| **Total Code (with headers)** | 3,456 lines | 530 lines | 🦀 Rust (85% less) |
| **Flash Usage (text)** | 52,100 bytes | 22,620 bytes | 🦀 Rust (57% smaller) |
| **RAM Usage (bss)** | 37,432 bytes | 1,980 bytes | 🦀 Rust (95% less!) |
| **Data Section** | 472 bytes | 80 bytes | 🦀 Rust (83% less) |
| **Total Binary Size** | 90,004 bytes | 24,680 bytes | 🦀 Rust (73% smaller) |
| **Driver Files** | 26 HAL + 10 FreeRTOS | 0 (managed by cargo) | 🦀 Rust |
| **Config Files** | .ioc, .cproject, etc. | Cargo.toml only | 🦀 Rust |

## 🔍 Code Comparison

### Serial Send Function

**C/FreeRTOS:**
```c
SemaphoreHandle_t xSemaphore;
extern UART_HandleTypeDef huart1;

void SerialSendByte(char data) {
    BaseType_t status = xSemaphoreTake(xSemaphore, portMAX_DELAY);
    HAL_UART_Transmit_IT(&huart1, (uint8_t*)&data, 1);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart1) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(xSemaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
```

**Rust/Embassy:**
```rust
pub async fn write_byte(&self, byte: u8) {
    let mut tx = self.tx.lock().await;
    let _ = tx.write(&[byte]).await;
}
```

### RX Task

**C/FreeRTOS:**
```c
QueueHandle_t xQueue;
uint8_t rxByte;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart1) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(xQueue, &rxByte, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        HAL_UART_Receive_IT(&huart1, &rxByte, 1);
    }
}

void SerialRxTask(void *argument) {
    char c;
    for (;;) {
        if (xQueueReceive(xQueue, &c, portMAX_DELAY) == pdTRUE) {
            if (c == '\r') {
                printf("\e[1;1H\e[2J");
                printf("Terminal Limpiada\r\n");
            }
            else printf("Nuevo dato recibido: %c\r\n", c);
        }
    }
}
```

**Rust/Embassy:**
```rust
#[embassy_executor::task]
pub async fn serial_rx_task(mut rx: Usart1Rx) {
    let mut buf = [0u8; 1];
    loop {
        match rx.read(&mut buf).await {
            Ok(_) => {
                let c = buf[0];
                if c == b'\r' {
                    info!("Terminal Limpiada");
                } else {
                    info!("Nuevo dato recibido: {}", c as char);
                }
            }
            Err(_) => error!("UART read error"),
        }
    }
}
```

### Initialization

**C/FreeRTOS (777 lines in main.c):**
- `HAL_Init()`
- `SystemClock_Config()` - 50+ lines of register configuration
- `MX_GPIO_Init()`, `MX_USART1_UART_Init()`, etc. - 500+ lines of boilerplate
- `osKernelInitialize()`, `osKernelStart()`
- Global variables for handles everywhere

**Rust/Embassy (148 lines in main.rs):**
```rust
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let config = configure_clocks();
    let p = embassy_stm32::init(config);
    
    let uart = Uart::new(p.USART1, p.PB7, p.PB6, Irqs, 
                         p.DMA1_CH4, p.DMA1_CH5, uart_config).unwrap();
    let (tx, rx) = uart.split();
    
    spawner.spawn(serial_echo_task(rx, writer)).unwrap();
}
```

## ✅ Advantages of Rust/Embassy

### 1. **Memory Safety at Compile Time**
- No null pointer dereferences
- No buffer overflows
- No data races between tasks
- No use-after-free bugs

### 2. **No RTOS Kernel Overhead**
- Embassy uses async/await (stackless coroutines)
- All tasks share a single stack
- No context switching overhead
- No per-task stack size tuning needed

### 3. **Expressive Type System**
- Peripherals are **moved** (ownership), preventing double-use
- DMA channels are type-checked at compile time
- Pin assignments verified by compiler

### 4. **Modern Dependency Management**
- `cargo` handles all dependencies
- No manually copying HAL/driver files
- Semantic versioning with `Cargo.toml`
- Easy updates with `cargo update`

### 5. **Better Tooling**
- `probe-rs` for flashing and debugging
- `defmt` for efficient logging (better than printf)
- `rust-analyzer` for IDE support
- Built-in formatting with `rustfmt`

### 6. **Smaller Binary Size**
- 73% smaller total binary
- 95% less RAM usage
- No RTOS task stacks needed

## ⚠️ Challenges of Rust/Embassy

### 1. **Steeper Learning Curve**
- Rust ownership/borrowing concepts
- Async/await patterns
- Lifetime annotations

### 2. **Smaller Ecosystem (for now)**
- Less documentation than STM32 HAL
- Fewer example projects
- Still rapidly evolving

### 3. **Debugging Differences**
- No traditional RTOS-aware debuggers
- Different mental model for async

### 4. **Vendor Support**
- STM32CubeMX generates C code, not Rust
- No official ST support for Rust

## 🏗️ Project Structure Comparison

**C/FreeRTOS:**
```
Practica4/
├── .cproject           # Eclipse project config
├── .mxproject          # STM32CubeMX project
├── Practica4.ioc       # STM32CubeMX config (18KB XML)
├── Core/
│   ├── Inc/            # Header files (5 files)
│   └── Src/            # Source files (9 files, 2479 lines)
├── Drivers/
│   ├── CMSIS/          # ARM CMSIS headers
│   └── STM32L4xx_HAL/  # ST HAL library (26 files)
├── Middlewares/
│   └── FreeRTOS/       # FreeRTOS kernel (10 files)
└── Debug/              # Build output
```

**Rust/Embassy:**
```
practica4-rust/
├── Cargo.toml          # Dependencies & config (50 lines)
├── .cargo/config.toml  # Build target config
├── src/
│   ├── main.rs         # Entry point (148 lines)
│   ├── serial_task.rs  # UART handling (196 lines)
│   └── peripherals_config.rs  # Sensor configs (186 lines)
└── target/             # Build output (managed by cargo)
```

## 📈 Why This Matters for Embedded Systems Education

1. **Modern Programming Paradigms**: Async/await is the future of concurrent programming
2. **Memory Safety**: Critical for safety-critical embedded systems (automotive, medical, aerospace)
3. **Industry Trend**: Companies like Google, Microsoft, and Volvo are adopting Rust for embedded
4. **Portable Skills**: Rust skills transfer between embedded, systems, and web development
5. **Open Source**: No vendor lock-in, community-driven development

## 🔗 Resources

- [Embassy Documentation](https://embassy.dev/)
- [Embassy GitHub](https://github.com/embassy-rs/embassy)
- [The Embedded Rust Book](https://docs.rust-embedded.org/book/)
- [Rust Async Book](https://rust-lang.github.io/async-book/)
- [defmt Logging Framework](https://defmt.ferrous-systems.com/)

---

*Generated for SETR2 - Practica 4: Comparison between traditional C/FreeRTOS and modern Rust/Embassy embedded development*
