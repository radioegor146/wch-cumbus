#include "modbus.h"

#include <ch32v00x.h>
#include <string.h>

#include "modbus_config.h"

#define FRAME_DELAY (SystemCoreClock / 8000000 * 1750)
#define CHARACTER_DELAY (SystemCoreClock / 8000000 * 750)

static void ModbusStartSysTimer(uint32_t cmp_value) {
  SysTick->CTLR &= ~1;
  SysTick->SR &= ~1;
  SysTick->CMP = cmp_value;
  SysTick->CNT = 0;
  SysTick->CTLR |= 1;
}

static uint8_t ModbusIsSysTimerTriggered() {
  return SysTick->SR & 1;
}

static uint16_t ModbusReadRawPacket(uint8_t* buffer, uint16_t length) {
  uint16_t read_length = 0;

  while (USART_GetFlagStatus(MODBUS_UART, USART_FLAG_RXNE) == RESET) {}

  while (read_length < length) {
    buffer[read_length++] = USART_ReceiveData(MODBUS_UART);
    ModbusStartSysTimer(CHARACTER_DELAY);
    while (USART_GetFlagStatus(MODBUS_UART, USART_FLAG_RXNE) == RESET &&
           !ModbusIsSysTimerTriggered()) {}
    if (ModbusIsSysTimerTriggered()) {
      break;
    }
  }

  return read_length;
}

static uint16_t ModbusCalcCRC(const uint8_t* data, uint16_t length) {
  uint16_t crc = 0xFFFF;

  while (length--) {
    crc ^= *(data++);
    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 1) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }

  return crc;
}

static uint8_t ModbusReadPacket(ModbusPacket* packet) {
  uint8_t buffer[256 + 16];
  uint16_t length = ModbusReadRawPacket(buffer, sizeof(buffer));

  if (length < 1 + 1 + 2) {
    return MODBUS_READ_ERROR_WRONG_PACKET_SIZE;
  }

  uint16_t crc = (uint16_t)(buffer[length - 1] << 8) | buffer[length - 2];
  uint16_t calculated_crc = ModbusCalcCRC(buffer, length - 2);

  if (calculated_crc != crc) {
    return MODBUS_READ_ERROR_WRONG_CRC;
  }

  packet->slave_id = buffer[0];
  packet->function = buffer[1];

  switch (packet->function) {
    case MODBUS_PACKET_FUNCTION_CODE_READ_DISCRETE_INPUTS:
      if (length < 1 + 1 + 2 + 2 + 2) {
        return MODBUS_READ_ERROR_WRONG_PACKET_SIZE;
      }
      packet->read_discrete_inputs_data.address =
          (uint16_t)(buffer[2] << 8) | buffer[3];
      packet->read_discrete_inputs_data.number =
          (uint16_t)(buffer[4] << 8) | buffer[5];
      break;
    case MODBUS_PACKET_FUNCTION_CODE_READ_INPUT_REGISTERS:
      if (length < 1 + 1 + 2 + 2 + 2) {
        return MODBUS_READ_ERROR_WRONG_PACKET_SIZE;
      }
      packet->read_input_registers_data.address =
          (uint16_t)(buffer[2] << 8) | buffer[3];
      packet->read_input_registers_data.number =
          (uint16_t)(buffer[4] << 8) | buffer[5];
      break;
    case MODBUS_PACKET_FUNCTION_CODE_WRITE_SINGLE_COIL:
      if (length < 1 + 1 + 2 + 2 + 2) {
        return MODBUS_READ_ERROR_WRONG_PACKET_SIZE;
      }
      packet->write_single_coil_data.address =
          (uint16_t)(buffer[2] << 8) | buffer[3];
      packet->write_single_coil_data.value =
          (uint16_t)(buffer[4] << 8) | buffer[5];
      break;
    case MODBUS_PACKET_FUNCTION_CODE_WRITE_SINGLE_REGISTER:
      if (length < 1 + 1 + 2 + 2 + 2) {
        return MODBUS_READ_ERROR_WRONG_PACKET_SIZE;
      }
      packet->write_single_register_data.address =
          (uint16_t)(buffer[2] << 8) | buffer[3];
      packet->write_single_register_data.value =
          (uint16_t)(buffer[4] << 8) | buffer[5];
      break;
    case MODBUS_PACKET_FUNCTION_CODE_WRITE_MULTIPLE_COILS:
      if (length < 1 + 1 + 2 + 2 + 1 + 2) {
        return MODBUS_READ_ERROR_WRONG_PACKET_SIZE;
      }
      if (length < 1 + 1 + 2 + 2 + 1 + buffer[6] + 2) {
        return MODBUS_READ_ERROR_WRONG_PACKET_SIZE;
      }
      packet->write_multiple_coils_data.address =
          (uint16_t)(buffer[2] << 8) | buffer[3];
      packet->write_multiple_coils_data.number =
          (uint16_t)(buffer[4] << 8) | buffer[5];
      packet->write_multiple_coils_data.bytes = buffer[6];
      for (uint16_t i = 0; i < packet->write_multiple_coils_data.bytes; i++) {
        packet->write_multiple_coils_data.data[i] = buffer[7 + i];
      }
      break;
    case MODBUS_PACKET_FUNCTION_CODE_WRITE_MULTIPLE_REGISTERS:
      if (length < 1 + 1 + 2 + 2 + 1 + 2) {
        return MODBUS_READ_ERROR_WRONG_PACKET_SIZE;
      }
      if (length < 1 + 1 + 2 + 2 + 1 + buffer[6] + 2) {
        return MODBUS_READ_ERROR_WRONG_PACKET_SIZE;
      }
      if (buffer[6] % 2 != 0) {
        return MODBUS_READ_ERROR_WRONG_PACKET_SIZE;
      }
      packet->write_multiple_registers_data.address =
          (uint16_t)(buffer[2] << 8) | buffer[3];
      packet->write_multiple_registers_data.number =
          (uint16_t)(buffer[4] << 8) | buffer[5];
      packet->write_multiple_registers_data.bytes = buffer[6];
      for (uint16_t i = 0; i < packet->write_multiple_registers_data.bytes;
           i += 2) {
        packet->write_multiple_registers_data.data[i] =
            (buffer[7 + i] << 8) | buffer[7 + i + 1];
      }
      break;
    default:
      return MODBUS_READ_ERROR_UNSUPPORTED_PACKET;
  }

  return MODBUS_READ_ERROR_SUCCESS;
}

static void ModbusSendPacket(const uint8_t* data, uint8_t length) {
  GPIO_WriteBit(DIR_PORT, DIR_PIN, Bit_SET);
  ModbusStartSysTimer(FRAME_DELAY);
  while (!ModbusIsSysTimerTriggered()) {}
  for (uint16_t i = 0; i < length; i++) {
    USART_SendData(MODBUS_UART, data[i]);
    while (USART_GetFlagStatus(MODBUS_UART, USART_FLAG_TXE) == RESET) {}
  }
  uint16_t crc = ModbusCalcCRC(data, length);
  USART_SendData(MODBUS_UART, crc & 0xFF);
  while (USART_GetFlagStatus(MODBUS_UART, USART_FLAG_TXE) == RESET) {}
  USART_SendData(MODBUS_UART, crc >> 8);
  while (USART_GetFlagStatus(MODBUS_UART, USART_FLAG_TXE) == RESET) {}
  ModbusStartSysTimer(FRAME_DELAY);
  while (!ModbusIsSysTimerTriggered()) {}
  GPIO_WriteBit(DIR_PORT, DIR_PIN, Bit_RESET);
}

static void ModbusSendException(uint8_t function, uint8_t exception_code) {
  uint8_t response[3] = {MODBUS_SLAVE_ID, function | 0x80, exception_code};
  ModbusSendPacket(response, sizeof(response));
}

static void ModbusSendReadDiscreteInputsResponse(uint8_t* data, uint8_t bytes) {
  uint8_t response[1 + 1 + 1 + (READ_DISCRETE_INPUTS_SIZE + 7) / 8] = {
      MODBUS_SLAVE_ID, MODBUS_PACKET_FUNCTION_CODE_READ_DISCRETE_INPUTS, bytes};
  memcpy(response + 3, data, bytes);
  ModbusSendPacket(response, 1 + 1 + 1 + bytes);
}

static void ModbusSendReadInputRegistersResponse(uint8_t count, uint16_t* registers) {
  uint8_t response[1 + 1 + 1 + READ_INPUT_REGISTERS_SIZE * 2] = {
      MODBUS_SLAVE_ID, MODBUS_PACKET_FUNCTION_CODE_READ_INPUT_REGISTERS,
      count * 2};
  for (uint8_t i = 0; i < count; i++) {
    response[1 + 1 + 1 + i * 2] = registers[i] >> 8;
    response[1 + 1 + 1 + i * 2 + 1] = registers[i] & 0xFF;
  }
  ModbusSendPacket(response, sizeof(response));
}

static void ModbusSendWriteSingleCoilResponse(uint16_t address, uint16_t value) {
  uint8_t response[6] = {
      MODBUS_SLAVE_ID, MODBUS_PACKET_FUNCTION_CODE_WRITE_SINGLE_COIL,
      address >> 8,    address & 0xFF,
      value >> 8,      value & 0xFF};
  ModbusSendPacket(response, sizeof(response));
}

static void ModbusSendWriteMultipleCoilsResponse(uint16_t address, uint16_t number) {
  uint8_t response[6] = {
      MODBUS_SLAVE_ID, MODBUS_PACKET_FUNCTION_CODE_WRITE_MULTIPLE_COILS,
      address >> 8,    address & 0xFF,
      number >> 8,     number & 0xFF};
  ModbusSendPacket(response, sizeof(response));
}

static void ModbusSendWriteSingleRegisterResponse(uint16_t address, uint16_t value) {
  uint8_t response[6] = {
      MODBUS_SLAVE_ID, MODBUS_PACKET_FUNCTION_CODE_WRITE_SINGLE_REGISTER,
      address >> 8,    address & 0xFF,
      value >> 8,      value & 0xFF};
  ModbusSendPacket(response, sizeof(response));
}

static void ModbusSendWriteMultipleRegistersResponse(uint16_t address,
                                              uint16_t number) {
  uint8_t response[6] = {
      MODBUS_SLAVE_ID, MODBUS_PACKET_FUNCTION_CODE_WRITE_MULTIPLE_REGISTERS,
      address >> 8,    address & 0xFF,
      number >> 8,     number & 0xFF};
  ModbusSendPacket(response, sizeof(response));
}

static void GPIOInitPin(GPIO_TypeDef* gpio, uint16_t pin, GPIOSpeed_TypeDef speed,
                 GPIOMode_TypeDef mode) {
  GPIO_InitTypeDef init = {0};
  init.GPIO_Pin = pin;
  init.GPIO_Speed = speed;
  init.GPIO_Mode = mode;
  GPIO_Init(gpio, &init);
}

void ModbusInit() {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
                             RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,
                         ENABLE);

  GPIOInitPin(TX_PORT, TX_PIN, GPIO_Speed_50MHz, GPIO_Mode_AF_PP);
  GPIOInitPin(RX_PORT, RX_PIN, GPIO_Speed_50MHz, GPIO_Mode_IPU);
  GPIOInitPin(DIR_PORT, DIR_PIN, GPIO_Speed_50MHz, GPIO_Mode_Out_PP);
  GPIO_WriteBit(DIR_PORT, DIR_PIN, Bit_RESET);

  GPIOInitPin(DBG_LED_PORT, DBG_LED_PIN, GPIO_Speed_50MHz, GPIO_Mode_Out_PP);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

  USART_InitTypeDef uart;
  uart.USART_BaudRate = MODBUS_SPEED;
  uart.USART_WordLength = USART_WordLength_8b;
  uart.USART_StopBits = USART_StopBits_1;
  uart.USART_Parity = USART_Parity_No;
  uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(MODBUS_UART, &uart);
  USART_Cmd(MODBUS_UART, ENABLE);
}

static bool ModbusIsReadDiscreteInputValid(uint16_t address) {
  return ModbusIsReadDiscreteInputValidExt(address) ||
         address < MAX_READ_DISCRETE_INPUTS;
}

static bool ModbusIsReadInputRegisterValid(uint16_t address) {
  return ModbusIsReadInputRegisterValidExt(address) ||
         address < MAX_READ_INPUT_REGISTERS;
}

static bool ModbusIsWriteCoilValid(uint16_t address) {
  return ModbusIsWriteCoilValidExt(address) || address < MAX_WRITE_COILS;
}

static bool ModbusIsWriteRegisterValid(uint16_t address) {
  return ModbusIsWriteRegisterValidExt(address) ||
         address < MAX_WRITE_REGISTERS;
}

void ModbusProcess() {
  GPIO_WriteBit(DBG_LED_PORT, DBG_LED_PIN, Bit_RESET);
  ModbusPacket packet;
  enum ModbusReadError error = ModbusReadPacket(&packet);
  switch (error) {
    case MODBUS_READ_ERROR_WRONG_PACKET_SIZE:
    case MODBUS_READ_ERROR_WRONG_CRC:
      return;
    case MODBUS_READ_ERROR_UNSUPPORTED_PACKET:
      if (packet.slave_id == MODBUS_SLAVE_ID) {
        ModbusSendException(packet.function,
                            MODBUS_EXCEPTION_CODE_ILLEGAL_FUNCTION);
      }
      return;
    case MODBUS_READ_ERROR_SUCCESS:
      break;
  }

  if (packet.slave_id != MODBUS_SLAVE_ID) {
    return;
  }

  GPIO_WriteBit(DBG_LED_PORT, DBG_LED_PIN, Bit_SET);

  switch (packet.function) {
    case MODBUS_PACKET_FUNCTION_CODE_READ_DISCRETE_INPUTS: {
      for (uint32_t i = 0; i < packet.read_discrete_inputs_data.number; i++) {
        if (i + packet.read_discrete_inputs_data.address > 65535) {
          ModbusSendException(packet.function,
                              MODBUS_EXCEPTION_CODE_ILLEGAL_DATA_ADDRESS);
          return;
        }
        if (!ModbusIsReadDiscreteInputValid(
                i + packet.read_discrete_inputs_data.address)) {
          ModbusSendException(packet.function,
                              MODBUS_EXCEPTION_CODE_ILLEGAL_DATA_ADDRESS);
          return;
        }
      }

      uint8_t response_data[(READ_DISCRETE_INPUTS_SIZE + 7) / 8] = {};
      for (uint32_t i = 0; i < packet.read_discrete_inputs_data.number; i++) {
        response_data[i / 8] |=
            ModbusReadDiscreteInput(i +
                                    packet.read_discrete_inputs_data.address)
            << (i % 8);
      }
      ModbusSendReadDiscreteInputsResponse(
          response_data, (packet.read_discrete_inputs_data.number + 7) / 8);
      break;
    }
    case MODBUS_PACKET_FUNCTION_CODE_READ_INPUT_REGISTERS: {
      for (uint32_t i = 0; i < packet.read_input_registers_data.number; i++) {
        if (i + packet.read_input_registers_data.address > 65535) {
          ModbusSendException(packet.function,
                              MODBUS_EXCEPTION_CODE_ILLEGAL_DATA_ADDRESS);
          return;
        }
        if (!ModbusIsReadInputRegisterValid(
                i + packet.read_input_registers_data.address)) {
          ModbusSendException(packet.function,
                              MODBUS_EXCEPTION_CODE_ILLEGAL_DATA_ADDRESS);
          return;
        }
      }

      uint16_t registers[READ_INPUT_REGISTERS_SIZE] = {};
      for (uint32_t i = 0; i < packet.read_input_registers_data.number; i++) {
        registers[i] = ModbusReadInputRegister(
            i + packet.read_input_registers_data.address);
      }

      ModbusSendReadInputRegistersResponse(
          packet.read_input_registers_data.number, registers);
      break;
    }
    case MODBUS_PACKET_FUNCTION_CODE_WRITE_SINGLE_COIL: {
      if (!ModbusIsWriteCoilValid(packet.write_single_coil_data.address)) {

        ModbusSendException(packet.function,
                            MODBUS_EXCEPTION_CODE_ILLEGAL_DATA_ADDRESS);
        return;
      }

      ModbusWriteCoil(packet.write_single_coil_data.address,
                      packet.write_single_coil_data.value == 0xFF00);
      ModbusSendWriteSingleCoilResponse(packet.write_single_coil_data.address,
                                        packet.write_single_coil_data.value);
      break;
    }
    case MODBUS_PACKET_FUNCTION_CODE_WRITE_MULTIPLE_COILS: {
      for (uint32_t i = 0; i < packet.write_multiple_coils_data.number; i++) {
        if (i + packet.write_multiple_coils_data.address > 65535) {
          ModbusSendException(packet.function,
                              MODBUS_EXCEPTION_CODE_ILLEGAL_DATA_ADDRESS);
        }
        if (!ModbusIsWriteCoilValid(i +
                                    packet.write_multiple_coils_data.address)) {
          ModbusSendException(packet.function,
                              MODBUS_EXCEPTION_CODE_ILLEGAL_DATA_ADDRESS);
        }
      }

      for (uint32_t i = 0; i < packet.write_multiple_coils_data.number; i++) {
        ModbusWriteCoil(
            packet.write_multiple_coils_data.address + i,
            (packet.write_multiple_coils_data.data[i / 8] >> (i % 8)) & 1);
      }

      ModbusSendWriteMultipleCoilsResponse(
          packet.write_multiple_coils_data.address,
          packet.write_multiple_coils_data.number);
      break;
    }
    case MODBUS_PACKET_FUNCTION_CODE_WRITE_SINGLE_REGISTER: {
      if (!ModbusIsWriteRegisterValid(
              packet.write_single_register_data.address)) {

        ModbusSendException(packet.function,
                            MODBUS_EXCEPTION_CODE_ILLEGAL_DATA_ADDRESS);
        return;
      }

      ModbusWriteRegister(packet.write_single_register_data.address,
                          packet.write_single_register_data.value);

      ModbusSendWriteSingleRegisterResponse(
          packet.write_single_register_data.address,
          packet.write_single_register_data.value);
      break;
    }
    case MODBUS_PACKET_FUNCTION_CODE_WRITE_MULTIPLE_REGISTERS: {
      for (uint32_t i = 0; i < packet.write_multiple_registers_data.number;
           i++) {
        if (i + packet.write_multiple_registers_data.address > 65535) {
          ModbusSendException(packet.function,
                              MODBUS_EXCEPTION_CODE_ILLEGAL_DATA_ADDRESS);
        }
        if (!ModbusIsWriteRegisterValid(
                i + packet.write_multiple_registers_data.address)) {
          ModbusSendException(packet.function,
                              MODBUS_EXCEPTION_CODE_ILLEGAL_DATA_ADDRESS);
        }
      }

      for (uint32_t i = 0; i < packet.write_multiple_registers_data.number;
           i++) {
        ModbusWriteRegister(i + packet.write_multiple_registers_data.address,
                            packet.write_multiple_registers_data.data[i]);
      }

      ModbusSendWriteMultipleRegistersResponse(
          packet.write_multiple_registers_data.address,
          packet.write_multiple_registers_data.number);
      break;
    }
  }
}

bool __attribute__((weak)) ModbusIsReadDiscreteInputValidExt(uint16_t address) {
  return false;
}
bool __attribute__((weak)) ModbusIsReadInputRegisterValidExt(uint16_t address) {
  return false;
}
bool __attribute__((weak)) ModbusIsWriteCoilValidExt(uint16_t address) {
  return false;
}
bool __attribute__((weak)) ModbusIsWriteRegisterValidExt(uint16_t address) {
  return false;
}

bool __attribute__((weak)) ModbusReadDiscreteInput(uint16_t address) {
  return false;
}
uint16_t __attribute__((weak)) ModbusReadInputRegister(uint16_t address) {
  return 0;
}
void __attribute__((weak)) ModbusWriteCoil(uint16_t address, bool value) {}
void __attribute__((weak)) ModbusWriteRegister(uint16_t address,
                                               uint16_t value) {}