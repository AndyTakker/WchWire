#ifndef WCH_WIRE_H
#define WCH_WIRE_H

#include <debug.h>

// I2C таймаут в количестве попыток. Чтобы не тащить следом библиотеку
// для работы с микросекундами, а следовательно, таймеры и прочее.
// Методом научного тыка было выяснено, что 1500 попыток достаточно.
// Для функции записи буфера таймаут увеличивается вдвое.
#ifndef TIMEOUT_MAX
#define TIMEOUT_MAX 1500 // Тайм аут ожидания ответа, задается в количестве попыток
#endif

#if !defined(WIRE_RX_BUFFER_LENGTH)
#define WIRE_RX_BUFFER_LENGTH 32 // Размер статического буфера для чтения
#endif
#if !defined(WIRE_TX_BUFFER_LENGTH)
#define WIRE_TX_BUFFER_LENGTH 128 // Размер статического буфера для записи
#endif

void i2c_Init(u32 bound, u16 address = 0xEE);

typedef enum {
  I2C_OK = 0,
  I2C_ADDR_NACK,
  I2C_TIMEOUT,
  I2C_BUSY,
} i2c_result;

class TwoWire {
  private:
  I2C_TypeDef *i2c_periph = I2C1;
  uint8_t rxBuffer[WIRE_RX_BUFFER_LENGTH];
  uint16_t rxBufferIndex = 0;
  uint16_t rxBufferLength = 0;

  uint8_t txAddress;               
  bool transmitting = false;               
  uint8_t txBuffer[WIRE_TX_BUFFER_LENGTH]; 
  uint16_t txBufferIndex = 0;
  uint16_t txBufferLength = 0;

  public:
  TwoWire(void);
  ~TwoWire(void);

  void begin(void);
  void begin(uint32_t bound);
  void end();
  void setClock(uint32_t bound);
  uint8_t beginTransmission(uint8_t);
  uint8_t endTransmission(uint8_t sendStop = 1);
  uint8_t requestFrom(uint8_t, uint8_t);
  uint8_t requestFrom(uint8_t, uint8_t, uint8_t);
  uint8_t requestFrom(uint8_t address, uint8_t quantity, uint32_t iaddress, uint8_t isize, uint8_t sendStop);
  size_t write(uint8_t);
  size_t write(const uint8_t *, size_t);
  int available(void) const;
  int read(void);
  void flush(void);
};

#if !defined(NO_GLOBAL_INSTANCES) && !defined(NO_GLOBAL_WIRE)
extern TwoWire Wire;
#endif

#endif // WCH_WIRE_H