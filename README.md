# wch-cumbus

Simple CumBus library for WCH MCUs

You must have `modbus_config.h` with contents like [example_modbus_config.h](example_modbus_config.h) file in `include/` directory.

Example usage code:

```c
#include <ch32v00x.h>

#include "modbus.h"

static void Init() {
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  ModbusInit();
}

void ModbusWriteCoil(uint16_t address, bool value) {
  switch (address) {
    // your handlers
  }
}

bool ModbusReadDiscreteInput(uint16_t address) {
  switch (address) { 
    // your handlers
  }
  return false;
}

uint16_t ModbusReadInputRegister(uint16_t address) {
  switch (address) {
    // your handlers
  }
  return 0;
}

void ModbusWriteRegister(uint16_t address, uint16_t value) {
  switch (address) {
    // your handlers
  }
  return 0;
}

int main(void) {
  Init();

  while (1) {
    ModbusProcess();
  }
}
```

Example's `platformio.ini`:
```ini
[env:ch32v003f4p6]
platform = https://github.com/Community-PIO-CH32V/platform-ch32v.git
board = genericCH32V003F4P6
framework = noneos-sdk
upload_protocol = wch-link
lib_deps = https://github.com/radioegor146/wch-cumbus.git@0.0.4
build_flags = -Iinclude
```
