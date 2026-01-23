//=============================================================== (c) A.Kolesov ==
// I2C Scanner
// За основу сканера взята идея by Nick Gammon
// Сканер позволяет обнаружить доступные устройства на шине I2C и убедиться в
// работоспособности устройств.
// Сканер реализован для CH32V003 в двух вариантах:
// 1. Использование Arduino framework. В этом случае могла бы использоваться "родная"
//    библиотека Wire, но она не умещается в памяти. Поэтому используется моя библиотека.
// 2. Без использования Arduino framework, на фреймворке Noneos-sdk, т.е. максимально
//    железячные функции. Но в этом случае используется моя библиотека Wire,
//    написаная как обертка над методами I2C ch32. Библиотека устанавливается через
//    lib_deps в platformio.ini ссылкой на git: https://github.com/AndyTakker/WchWire.git
// Переключение вариантов выполняется выбором соответстующего environment в platformio.ini
//
// Поскольку сканер использует аппаратный I2C, то должен работать и на Arduino Nano.
//
// Ниже комментарии из исходного текста сканера.
// CH32 Support I2C scanning using Wire.endTransmission() without sending data - Да, работает.
// CH32 changes required in libraries/Wire/src/utility/twi.c
//  - timeout on an addresses should release the bus - да, реализовано
//  - allow only sending the address (without actual data) - да, реализовано
//  - smaller timeout: I2C_TIMEOUT_TICK 25 (was 100ms) - таймаут реализован не через оценку времени,
//    а через счетчик попыток. Поэтому есть зависимость от частоты процессора.
//
// Note: Currently there's no support for Wire.setWireTimeout(timeout, reset_on_timeout)
// https://www.arduino.cc/reference/en/language/functions/communication/wire/setwiretimeout/
// Inspiration from https://github.com/mockthebear/easy-ch32v003/tree/main/examples/i2c_scanner
//------------------------------------------------------------------------------
#ifdef LOG_ENABLE // Логирование включается директивой -DLOG_ENABLE в platformio.ini
#ifdef ARDUINO
#define logs(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
#else
#define logs(fmt, ...) printf(fmt, ##__VA_ARGS__)
#endif // ARDUINO
#else
#define logs(fmt, ...) ((void)0)
#endif // LOG_ENABLE

#include <Wire.h> // Подключится нужная версия библиотеки Wire в зависимости от выбранного environment

#ifdef ARDUINO
#include <Arduino.h>
#define Delay_Ms(x) delay(x)
void setup() {
#else
int main(void) {
  SystemCoreClockUpdate();
  Delay_Init();
#endif

#ifdef LOG_ENABLE // Логирование включается директивой -DLOG_ENABLE в platformio.ini
  Delay_Ms(4000); // Чтобы дождаться готовности UART-монитора
#ifdef ARDUINO
  Serial.begin(115200);
#else
  USART_Printf_Init(115200);
#endif
#endif
  logs("SystemClk: %ldHz\r\n", SystemCoreClock);
#ifndef ARDUINO
  logs("   ChipID: 0x%08lx\r\n", DBGMCU_GetCHIPID());
#endif

  logs("I2C scanner. Scanning ...\r\n");
  bool fast = false; // Через раз запускаем тест на 100 или на 400 kHz
  while (true) {
    Wire.begin();                            // Настраиваем I2C перед каждым циклом сканирования
    Wire.setClock((fast) ? 400000 : 100000); // 400/100 kHz
    Delay_Ms(100);

    fast = !fast;
    logs("== I2C Scanning... %s\r\n", (fast) ? "FAST mode" : "SLOW mode");
    uint8_t count = 0;
    for (uint8_t i = 8; i < 120; i++) { // Перебираем доступные адреса на шине
      Wire.beginTransmission(i);
      if (Wire.endTransmission() == 0) {
        logs("Found address: %d (0x%02X)\r\n", i, i);
        count++;
      }
    }
    logs("== Done. Found %d device(s).\r\n\r\n", count);
    Delay_Ms(2000);
  }
}

#ifdef ARDUINO
void loop() {}
#endif