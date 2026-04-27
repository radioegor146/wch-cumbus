#ifndef MODBUS_H_
#define MODBUS_H_

#include <stdint.h>
#include <stdbool.h>

enum ModbusReadError {
  MODBUS_READ_ERROR_SUCCESS,
  MODBUS_READ_ERROR_WRONG_CRC,
  MODBUS_READ_ERROR_WRONG_PACKET_SIZE,
  MODBUS_READ_ERROR_UNSUPPORTED_PACKET
};

enum ModbusPacketFunctionCode {
  MODBUS_PACKET_FUNCTION_CODE_READ_DISCRETE_INPUTS = 0x02,
  MODBUS_PACKET_FUNCTION_CODE_READ_INPUT_REGISTERS = 0x04,
  MODBUS_PACKET_FUNCTION_CODE_WRITE_SINGLE_COIL = 0x05,
  MODBUS_PACKET_FUNCTION_CODE_WRITE_SINGLE_REGISTER = 0x06,
  MODBUS_PACKET_FUNCTION_CODE_WRITE_MULTIPLE_COILS = 0x0F,
  MODBUS_PACKET_FUNCTION_CODE_WRITE_MULTIPLE_REGISTERS = 0x10
};

enum ModbusExceptionCode {
  MODBUS_EXCEPTION_CODE_ILLEGAL_FUNCTION = 0x01,
  MODBUS_EXCEPTION_CODE_ILLEGAL_DATA_ADDRESS = 0x02
};

typedef struct {
  uint16_t address;
  uint16_t number;
} ModbusPacketReadDiscreteInputsData;

typedef struct {
  uint16_t address;
  uint16_t value;
} ModbusPacketWriteSingleCoilData;

typedef struct {
  uint16_t address;
  uint16_t value;
} ModbusPacketWriteSingleRegisterData;

typedef struct {
  uint16_t address;
  uint16_t number;
} ModbusPacketReadInputRegistersData;

typedef struct {
  uint16_t address;
  uint16_t number;
  uint8_t bytes;
  uint8_t data[256];
} ModbusPacketWriteMultipleCoilsData;

typedef struct {
  uint16_t address;
  uint16_t number;
  uint8_t bytes;
  uint16_t data[128];
} ModbusPacketWriteMultipleRegistersData;

typedef struct {
  uint8_t slave_id;
  enum ModbusPacketFunctionCode function;
  union {
    ModbusPacketReadDiscreteInputsData read_discrete_inputs_data;
    ModbusPacketReadInputRegistersData read_input_registers_data;
    ModbusPacketWriteSingleCoilData write_single_coil_data;
    ModbusPacketWriteSingleRegisterData write_single_register_data;
    ModbusPacketWriteMultipleCoilsData write_multiple_coils_data;
    ModbusPacketWriteMultipleRegistersData write_multiple_registers_data;
  };
} ModbusPacket;

void ModbusInit();
void ModbusProcess();

bool ModbusIsReadDiscreteInputValidExt(uint16_t address);
bool ModbusIsReadInputRegisterValidExt(uint16_t address);
bool ModbusIsWriteCoilValidExt(uint16_t address);
bool ModbusIsWriteRegisterValidExt(uint16_t address);

bool ModbusReadDiscreteInput(uint16_t address);
uint16_t ModbusReadInputRegister(uint16_t address);
void ModbusWriteCoil(uint16_t address, bool value);
void ModbusWriteRegister(uint16_t address, uint16_t value);

#endif  // MODBUS_H_
