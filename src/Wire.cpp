#include "Wire.h"

TwoWire Wire;

TwoWire::TwoWire(void) {
}

TwoWire::~TwoWire(void) {
  end();
}

void TwoWire::begin() {
  begin(100000);
}

void TwoWire::begin(uint32_t bound) {
  i2c_Init(bound);
}

void TwoWire::end() {
  I2C_DeInit(i2c_periph);
  rxBufferIndex = 0;
}

void TwoWire::setClock(uint32_t bound) {
  I2C_DeInit(i2c_periph);
  i2c_Init(bound);
}

uint8_t TwoWire::beginTransmission(uint8_t address) {
  if (transmitting) {
    return I2C_BUSY;
  }
  txAddress = address;
  txBufferIndex = 0;
  txBufferLength = 0;
  transmitting = true;
  return I2C_OK;
}

size_t TwoWire::write(uint8_t data) {
  if (!transmitting) {
    return 0;
  }
  if (txBufferLength >= WIRE_TX_BUFFER_LENGTH) {
    return 0;
  }
  txBuffer[txBufferLength++] = data;
  return 1;
}

uint8_t TwoWire::endTransmission(uint8_t sendStop) {
  volatile uint32_t timeout = TIMEOUT_MAX;

  if (!transmitting) {
    return I2C_BUSY;
  }
  transmitting = false;

  // Ждем свободной шины, i2c master sends start signal only when the bus is idle
  while ((I2C_GetFlagStatus(i2c_periph, I2C_FLAG_BUSY) != RESET) && --timeout)
    ;
  if (timeout == 0) {
    I2C_GenerateSTOP(i2c_periph, ENABLE);
    return I2C_BUSY;
  }

  // Start the transmission
  I2C_GenerateSTART(i2c_periph, ENABLE); // send the start signal
  while (!I2C_CheckEvent(i2c_periph, I2C_EVENT_MASTER_MODE_SELECT) && --timeout)
    ;
  if (timeout == 0) {
    I2C_GenerateSTOP(i2c_periph, ENABLE);
    return I2C_BUSY;
  }

  // Адрес на передачу
  I2C_Send7bitAddress(i2c_periph, txAddress << 1, I2C_Direction_Transmitter);
  // address flag set means i2c slave sends ACK
  while ((!I2C_CheckEvent(i2c_periph, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) && --timeout)
    ;
  if (timeout == 0) {
    I2C_GenerateSTOP(i2c_periph, ENABLE);
    return I2C_ADDR_NACK;
  }

  // Отправка данных
  for (uint16_t i = 0; i < txBufferLength; i++) {
    I2C_SendData(i2c_periph, txBuffer[i]);
    while ((!I2C_CheckEvent(i2c_periph, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) && --timeout)
      ;
    if (timeout == 0) {
      I2C_GenerateSTOP(i2c_periph, ENABLE);
      return I2C_TIMEOUT;
    }
  }

  // STOP если нужно
  if (sendStop) {
    I2C_GenerateSTOP(i2c_periph, ENABLE);
  }

  return I2C_OK;
}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity) {
  return requestFrom(address, quantity, 0, 0, true);
}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop) {
  return requestFrom(address, quantity, 0, 0, sendStop);
}

//==============================================================================
// uint8_t address,     // I²C-адрес устройства
// uint8_t quantity,    // сколько байт прочитать
// uint32_t iaddress,   // внутренний адрес регистра/памяти в устройстве (например, 0x00 в EEPROM)
// uint8_t isize,       // размер этого внутреннего адреса (1, 2 или 3 байта)
// uint8_t sendStop     // отправлять ли STOP в конце
//-----------------------------------------------------------------------------
uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity, uint32_t iaddress, uint8_t isize, uint8_t sendStop) {
  uint32_t timeout = TIMEOUT_MAX;

  if (isize > 0) {
    // Записываем внутренний адрес
    if (beginTransmission(address) != I2C_OK) {
      return 0;
    }
    if (isize > 3) { // the maximum size of internal address is 3 bytes
      isize = 3;
    }
    // Пишем байты внутреннего адреса
    while (isize-- > 0) {
      uint8_t byteToSend = (iaddress >> (isize * 8)) & 0xFF;
      if (write(byteToSend) != 1) {
        return 0;
      }
    }
    // END без STOP (отправляем данные)
    if (endTransmission(0) != I2C_OK) {
      return 0;
    }
  }

  // Повторный START для чтения
  timeout = TIMEOUT_MAX;
  I2C_GenerateSTART(i2c_periph, ENABLE);
  while (!I2C_CheckEvent(i2c_periph, I2C_EVENT_MASTER_MODE_SELECT) && --timeout)
    ;
  if (timeout == 0) {
    I2C_GenerateSTOP(i2c_periph, ENABLE);
    return 0;
  }
  // Адрес на прием
  I2C_Send7bitAddress(i2c_periph, address << 1, I2C_Direction_Receiver);
  while ((!I2C_CheckEvent(i2c_periph, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) && --timeout)
    ;
  if (timeout == 0) {
    I2C_GenerateSTOP(i2c_periph, ENABLE);
    return 0;
  }
  // Чтение данных
  uint8_t read = 0;
  I2C_AcknowledgeConfig(i2c_periph, ENABLE);
  for (read = 0; read < quantity && read < WIRE_RX_BUFFER_LENGTH; read++) {
    if (read == quantity - 1) {
      // Последний байт - отключаем ACK
      I2C_AcknowledgeConfig(i2c_periph, DISABLE);
    }
    timeout = TIMEOUT_MAX;
    while ((I2C_GetFlagStatus(i2c_periph, I2C_FLAG_RXNE) == RESET) && --timeout)
      ;
    if (timeout == 0) {
      break;
    }

    rxBuffer[read] = I2C_ReceiveData(i2c_periph);
  }

  if (sendStop) {
    I2C_GenerateSTOP(i2c_periph, ENABLE);
  }

  I2C_AcknowledgeConfig(i2c_periph, ENABLE); // Восстанавливаем ACK

  rxBufferIndex = 0;
  rxBufferLength = read;
  return read;
}

size_t TwoWire::write(const uint8_t *pData, size_t Size) {
  int count;
  for (count = 0; count < (int)Size; count++) {
    size_t res;
    res = write(*pData);
    if (res == 0) {
      return 0; // error
    } else {
      pData++; // point to the next byte to be written
    }
  }
  return Size;
}

int TwoWire::available(void) const {
  return rxBufferLength - rxBufferIndex;
}

int TwoWire::read(void) {
  int value = -1;

  if (rxBufferIndex < rxBufferLength) {
    value = rxBuffer[rxBufferIndex];
    ++rxBufferIndex;
  }
  return value;
}

void TwoWire::flush(void) {
  rxBufferIndex = 0;
  rxBufferLength = 0;
  txBufferIndex = 0;
  txBufferLength = 0;
}

//==============================================================================
// @fn      IIC_Init
// @brief   Настройка аппаратного интерфейса IIC в CH32V003.
// @params
// - bound - скорость работы шины i2c
// - address - любой адрес, отсутствующий на шине (кроме зарезервированных), например 0xEE
//
// @return  none
//
// Режимы GPIO:
// - GPIO_Mode_IN_FLOATING   → вход без подтяжки
// - GPIO_Mode_IPU           → вход с подтяжкой к VCC
// - GPIO_Mode_IPD           → вход с подтяжкой к GND
// - GPIO_Mode_AIN           → аналоговый вход
// - GPIO_Mode_Out_OD        → выход с открытым стоком
// - GPIO_Mode_Out_PP        → выход push-pull
// - GPIO_Mode_AF_OD         → альтернативная функция OD ← I2C!
// - GPIO_Mode_AF_PP         → альтернативная функция PP
//------------------------------------------------------------------------------
void i2c_Init(u32 bound, u16 address) {
  GPIO_InitTypeDef GPIO_InitStructure = {0};
  I2C_InitTypeDef I2C_InitTStructure = {0};

  // ШАГ 1: Включаем тактирование необходимой периферии

  // GPIO порта C и AFIO на APB2 (высокоскоростная шина)
  // Без этого нельзя настраивать пины PC1/PC2
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);

  // I2C1 на APB1 (низкоскоростная шина)
  // Без этого модуль I2C не будет работать
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

  // ШАГ 2: Настраиваем пины GPIO

  // PC1 (SDA) и PC2 (SCL) как альтернативная функция с открытым стоком
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;   // Для I2C обязательно OD!
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // Достаточно для 400 кГц I2C
  GPIO_Init(GPIOC, &GPIO_InitStructure);            // Порт C

  // ШАГ 3: Настраиваем модуль I2C1

  I2C_InitTStructure.I2C_ClockSpeed = bound;  // 100000 или 400000
  I2C_InitTStructure.I2C_Mode = I2C_Mode_I2C; // Режим I2C (не SMBus)
  I2C_InitTStructure.I2C_DutyCycle = (bound >= 400000) ? I2C_DutyCycle_16_9 : I2C_DutyCycle_2;
  I2C_InitTStructure.I2C_OwnAddress1 = address; // Адрес контроллера на шине, если он будет slave.
  I2C_InitTStructure.I2C_Ack = I2C_Ack_Enable;  // Включить ACK
  I2C_InitTStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_Init(I2C1, &I2C_InitTStructure);

  // ШАГ 4: Включаем модуль I2C1
  I2C_Cmd(I2C1, ENABLE);
}
