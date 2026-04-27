#ifndef MODBUS_CONFIG_H_
#define MODBUS_CONFIG_H_

#include <ch32v00x.h>

// USART configuration
#define MODBUS_UART USART1
#define MODBUS_SPEED 115200
#define MODBUS_SLAVE_ID 231

// RS485 pins configuration
#define TX_PORT GPIOD
#define TX_PIN GPIO_Pin_5

#define RX_PORT GPIOD
#define RX_PIN GPIO_Pin_6

#define DIR_PORT GPIOA
#define DIR_PIN GPIO_Pin_1

// Debug LED pin configuration
#define DBG_LED_PORT GPIOC
#define DBG_LED_PIN GPIO_Pin_4

// Default addresses limits
#define MAX_READ_DISCRETE_INPUTS 2
#define MAX_READ_INPUT_REGISTERS 0
#define MAX_WRITE_COILS 1
#define MAX_WRITE_REGISTERS 0

// Limits for response buffers (max number of consecutive outputs)
#define MAX(a, b) ((a) < (b) ? (b) : (a))

#define READ_DISCRETE_INPUTS_SIZE MAX(0, MAX(1, MAX_READ_DISCRETE_INPUTS))
#define READ_INPUT_REGISTERS_SIZE MAX(0, MAX(1, MAX_READ_INPUT_REGISTERS))

#endif  // MCU_CONFIG_H

