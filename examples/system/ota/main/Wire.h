/*
 * Wire.h
 *
 *  Created on: Sep 5, 2018
 *      Author: Xensr
 */

#ifndef MAIN_WIRE_H_
#define MAIN_WIRE_H_

#include <stdbool.h>
#include <driver/esp32-hal-i2c.h>

void wireEnd(void);
void begin(int sda, int scl, uint32_t frequency);

     //defaults bus:0 sda=SDA, scl=SCL, frequency =100khz via variant pins_arduino.h
    // bus:1 unspecified, emits Log_E()
    void setClock(uint32_t frequency); // change bus clock without initing hardware
    void beginTransmission(uint16_t address);
    uint8_t endTransmission(bool sendStop);
	uint8_t	requestFrom(uint16_t address, uint8_t size, bool sendStop);
	uint16_t requestFromBuff(uint16_t address, uint8_t* buf, uint16_t size, bool sendStop);
	//@stickBreaker for big blocks and ISR model
    i2c_err_t writeTransmission(uint16_t address, uint8_t* buff, uint16_t size, bool sendStop);
    i2c_err_t readTransmission(uint16_t address, uint8_t* buff, uint16_t size, bool sendStop);

	uint8_t	transact(uint8_t readLen);
    uint16_t transactBuff(uint8_t* readBuff, uint16_t readLen);
	uint8_t	lastError();
    char * getErrorText(uint8_t err);

    size_t getClock(); // current bus clock rate in hz
    void setTimeOut(uint16_t timeOutMillis);
    uint16_t getTimeOut();

    void onReceive( void (*)(int) );
    void onRequest( void (*)(void) );

    size_t iwrite(uint8_t data);
    //size_t write_qty(const uint8_t *data,size_t quantity);
    int available(void);
    int iread(void);
    int peek(void);
    void flush(void);



#endif /* MAIN_WIRE_H_ */
