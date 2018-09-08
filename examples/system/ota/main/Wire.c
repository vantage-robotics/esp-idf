/*
  TwoWire.cpp - TWI/I2C library for Arduino & Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Modified 2012 by Todd Krein (todd@krein.org) to implement repeated starts
  Modified December 2014 by Ivan Grokhotkov (ivan@esp8266.com) - esp8266 support
  Modified April 2015 by Hrsto Gochkov (ficeto@ficeto.com) - alternative esp8266 support
  Modified Nov 2017 by Chuck Todd (ctodd@cableone.net) - ESP32 ISR Support
*/

#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "driver/esp32-hal-i2c.h"
#include "driver/esp32-hal.h"
#include "Wire.h"
#include "esp_log.h"

static const char *TAG = "wire";

#define I2C_BUFFER_LENGTH 128

    uint8_t num = 0;
    int8_t sda = -1;
    int8_t scl = -1;
    i2c_t * i2c = NULL;

    uint8_t rxBuffer[I2C_BUFFER_LENGTH];
    uint16_t rxIndex = 0;
    uint16_t rxLength = 0;
    uint16_t rxQueued = 0; //@stickBreaker

    uint8_t txBuffer[I2C_BUFFER_LENGTH];
    uint16_t txIndex = 0;
    uint16_t txLength = 0;
    uint16_t txAddress = 0;
    uint16_t txQueued = 0; //@stickbreaker

    uint8_t transmitting = 0;
/* slave Mode, not yet Stickbreaker
		static user_onRequest uReq[2];
		static user_onReceive uRcv[2];
    void onRequestService(void);
    void onReceiveService(uint8_t*, int);
*/
    i2c_err_t last_error = I2C_ERROR_OK; // @stickBreaker from esp32-hal-i2c.h
    i2c_err_t processQueue(uint32_t *readCount);
    uint16_t _timeOutMillis = 50;

    bool initHardware(int sdaPin, int sclPin, uint32_t frequency);


void wireEnd(void)
{
	flush();
	ESP_LOGI(TAG,"wire end");
	i2cDetachSCL(i2c,scl); // detach pins before resetting I2C perpherial
	i2cDetachSDA(i2c,sda); // else a glitch will appear on the i2c bus
	if(i2c){
		ESP_LOGI(TAG,"i2c release all");
	  i2cReleaseAll(i2c);
	  i2c=NULL;
	  }
}

void begin(int sdaPin, int sclPin, uint32_t frequency)
{

  if(!initHardware(sdaPin, sclPin, frequency)) return;

  flush();
  ESP_LOGI(TAG,"init hardware SUCCESS");

}

void setTimeOut(uint16_t timeOutMillis){
  _timeOutMillis = timeOutMillis;
}
  
uint16_t getTimeOut(){
  return _timeOutMillis;
}

void setClock(uint32_t frequency)
{
    i2cSetFrequency(i2c, frequency);
}

bool initHardware(int sdaPin, int sclPin, uint32_t frequency){

    i2cDetachSCL(i2c,scl); // detach pins before resetting I2C perpherial 
    i2cDetachSDA(i2c,sda); // else a glitch will appear on the i2c bus
    i2c = i2cInit(num);// i2cInit() now performs a hardware reset
    if(i2c == NULL) {
      return false;
      }
  
    if(frequency==0) {// don't change existing frequency
      frequency = i2cGetFrequency(i2c);
      }
    if(frequency==0) frequency = 100000L; // default to 100khz
	
    i2cSetFrequency(i2c, frequency);

    sda = sdaPin;
    scl = sclPin;

// 03/15/2018 What about MultiMaster? How can I be polite and still catch glitches?    

// 03/10/2018 test I2C bus before attach. 
// if the bus is not 'clear' try the recommended recovery sequence, START, 9 Clocks, STOP
    digitalWrite(sda,HIGH);
    digitalWrite(scl,HIGH);
    pinMode(sda,PULLUP|OPEN_DRAIN|OUTPUT|INPUT);
    pinMode(scl,PULLUP|OPEN_DRAIN|OUTPUT|INPUT);
    
    if(!digitalRead(sda)||!digitalRead(scl)){ // bus in busy state
      log_e("invalid state sda=%d, scl=%d\n",digitalRead(sda),digitalRead(scl));
      digitalWrite(sda,HIGH);
      digitalWrite(scl,HIGH);
      delayMicroseconds(5);
      digitalWrite(sda,LOW);
      for(uint8_t a=0; a<9;a++){
        delayMicroseconds(5);
        digitalWrite(scl,LOW);
        delayMicroseconds(5);
        digitalWrite(scl,HIGH);
        }
      delayMicroseconds(5);
      digitalWrite(sda,HIGH);
      }
    i2cAttachSDA(i2c, sda);
    i2cAttachSCL(i2c, scl);
  
    if(!digitalRead(sda)||!digitalRead(scl)){ // bus in busy state
      log_e("Bus Invalid State, TwoWire() Can't init");
      return false; // bus is busy
      }

    return true;
}	

/*@StickBreaker common handler for processing the queued commands
*/
i2c_err_t processQueue(uint32_t * readCount){
  last_error=i2cProcQueue(i2c,readCount,_timeOutMillis);
  if(last_error==I2C_ERROR_BUSY){ // try to clear the bus
    if(initHardware(sda,scl,getClock())){
      last_error=i2cProcQueue(i2c,readCount,_timeOutMillis);
      }
    }
  
  rxIndex = 0;
  rxLength = rxQueued;
  rxQueued = 0;
  txQueued = 0; // the SendStop=true will restart all Queueing 

  i2cFreeQueue(i2c);
  return last_error;
}
 
/* @stickBreaker 11/2017 fix for ReSTART timeout, ISR
*/
uint8_t requestFrom(uint16_t address, uint8_t size, bool sendStop){
//use internal Wire rxBuffer, multiple requestFrom()'s may be pending, try to share rxBuffer

    uint16_t cnt = rxQueued; // currently queued reads, next available position in rxBuffer 
    if(cnt<(I2C_BUFFER_LENGTH-1)){ // any room left in rxBuffer 
      if((size+cnt)>I2C_BUFFER_LENGTH)
        size = (I2C_BUFFER_LENGTH-cnt);
      rxQueued += size;
      }
    else { // no room to receive more!
      log_e("rxBuff overflow %d",cnt+size);
      cnt = 0;
      last_error = I2C_ERROR_MEMORY;
      flush();
      return cnt;
      }
    
    return requestFromBuff(address, &rxBuffer[cnt],size,sendStop);
 }

uint16_t requestFromBuff(uint16_t address, uint8_t * readBuff, uint16_t size, bool sendStop){
    uint32_t cnt=0;
    last_error =i2cAddQueueRead(i2c,address,readBuff,size,sendStop,NULL);
    if(last_error==I2C_ERROR_OK){ // successfully queued the read
      if(sendStop){ //now actually process the queued commands
        last_error = processQueue(&cnt);
        }
      else { // stop not received, so wait for I2C stop,
        last_error=I2C_ERROR_CONTINUE;
        cnt = 0;
        }
      }
    else {// only possible error is I2C_ERROR_MEMORY
      cnt = 0;
      }
    return cnt;
}

/* stickBreaker Nov 2017 ISR, and bigblock 64k-1
*/
i2c_err_t writeTransmission(uint16_t address, uint8_t *buff, uint16_t size, bool sendStop){
// will destroy any partially created beginTransaction()
log_i("i2c=%p",i2c);
last_error=i2cAddQueueWrite(i2c,address,buff,size,sendStop,NULL);

if(last_error==I2C_ERROR_OK){ //queued
  if(sendStop){ //now actually process the queued commands, including READs
    uint32_t dummy;
    last_error=processQueue(&dummy);
    }
  else { // stop not received, so wait for I2C stop,
    last_error=I2C_ERROR_CONTINUE;
    }
  }
txIndex=0;
txLength=0;
transmitting = 0;
return last_error;
}

i2c_err_t readTransmission(uint16_t address, uint8_t *buff, uint16_t size, bool sendStop){

last_error=i2cAddQueueRead(i2c,address,buff,size,sendStop,NULL);

if(last_error==I2C_ERROR_OK){ //queued
  if(sendStop){ //now actually process the queued commands, including READs
    uint32_t dummy;
    last_error=processQueue(&dummy);
    }
  else { // stop not received, so wait for I2C stop,
    last_error=I2C_ERROR_CONTINUE;
    }
  }
return last_error;
}

/*stickbreaker i2c isr Debugging
*/
size_t getClock(){
  return i2cGetFrequency(i2c); 
}

/*stickbreaker simple ReSTART handling using internal Wire data buffers
*/
uint8_t transact(uint8_t readLen){ // Assumes Wire.beginTransaction(),Wire.write()
// this command replaces Wire.endTransmission(false) and Wire.requestFrom(readLen,true);
if(transmitting){
  last_error = (i2c_err_t)(endTransmission(false));
  }
  
if(last_error==I2C_ERROR_CONTINUE){ // must have queued the Write
  uint8_t cnt = requestFrom(txAddress,readLen,true);
  return cnt;
  }
else {
  last_error = I2C_ERROR_NO_BEGIN;
  return 0;
  }
}

/*stickbreaker isr ReSTART with external read Buffer
*/
uint16_t transactBuff(uint8_t * readBuff, uint16_t readLen){ // Assumes Wire.beginTransaction(),Wire.write()
// this command replaces Wire.endTransmission(false) and Wire.requestFrom(readLen,true);
if(transmitting){
  last_error = (i2c_err_t)(endTransmission(false));
  }

if(last_error==I2C_ERROR_CONTINUE){ // must have queued the write
  size_t cnt = requestFromBuff(txAddress,readBuff,readLen,true);
  return cnt;
  }
else {
  last_error = I2C_ERROR_NO_BEGIN;
  return 0;
  }
}

/*stickbreaker isr
*/
uint8_t endTransmission(bool sendStop){ // Assumes Wire.beginTransaction(), Wire.write()
// this command replaces Wire.endTransmission(true)

if(transmitting==1){
//  log_e("txQueued=%d txLength=%d stop %d",txQueued,txLength,sendStop);
  last_error =i2cAddQueueWrite(i2c,txAddress,&txBuffer[txQueued],txLength-txQueued,sendStop,NULL);  //queue tx element

  if(last_error == I2C_ERROR_OK){
    if(sendStop){
      uint32_t dummy;
      last_error = processQueue(&dummy);
      }
    else { // queued because it had sendStop==false
      // txlength is howmany bytes in txbufferhave been use
      txQueued = txLength;
      last_error = I2C_ERROR_CONTINUE;
      }
    }
  }
else {
  last_error= I2C_ERROR_NO_BEGIN;
  flush();
  }
txIndex = 0;
txLength =0;
transmitting = 0;
return last_error;
}

/* stickbreaker Nov2017 better error reporting
*/
uint8_t lastError(){
	return (uint8_t)last_error;
}

const char ERRORTEXT[] =
  "OK\0"
  "DEVICE\0"
  "ACK\0"
  "TIMEOUT\0"
  "BUS\0"
  "BUSY\0"
  "MEMORY\0"
  "CONTINUE\0"
  "NO_BEGIN\0"
  "\0";
  

char * getErrorText(uint8_t err){
uint8_t t = 0;
bool found=false;
char * message=(char*)&ERRORTEXT;

while((!found)&&(message[0])){
  found = t==err;
  if(!found) {
    message = message +strlen(message)+1;
    t++;
    }
  }
if(!found) return NULL;
else return message;
}
/*
uint8_t requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop)
{
    return requestFrom((uint16_t)(address), (size_t)(quantity), (bool)(sendStop));
}

uint8_t requestFrom(uint16_t address, uint8_t quantity, uint8_t sendStop)
{
    return requestFrom(address, (size_t)(quantity), (bool)(sendStop));
}

uint8_t requestFrom(uint8_t address, uint8_t quantity)
{
    return requestFrom((uint16_t)(address), (size_t)(quantity), true);
}

uint8_t requestFrom(uint16_t address, uint8_t quantity)
{
    return requestFrom(address, (size_t)(quantity), true);
}

uint8_t requestFrom(int address, int quantity)
{
    return requestFrom((uint16_t)(address), (size_t)(quantity), true);
}

uint8_t requestFrom(int address, int quantity, int sendStop)
{
    return (uint8_t)(requestFrom((uint16_t)(address), (size_t)(quantity), (bool)(sendStop)));
}
*/
void beginTransmission(uint16_t address)
{
    transmitting = 1;
    txAddress = address;
    txIndex = txQueued; // allow multiple beginTransmission(),write(),endTransmission(false) until endTransmission(true)
    txLength = txQueued;
}
/*
void beginTransmission(int address)
{
    beginTransmission((uint16_t)(address));
}

void beginTransmission(uint8_t address)
{
    beginTransmission((uint16_t)(address));
}

uint8_t endTransmission(void)
{
    return endTransmission(true);
}

uint8_t endTransmission(uint8_t sendStop)
{
    return endTransmission((bool)(sendStop));
}
*/
size_t iwrite(uint8_t data)
{
    if(transmitting) {
        if(txLength >= I2C_BUFFER_LENGTH) {
            return 0;
        }
        txBuffer[txIndex] = data;
        ++txIndex;
        txLength = txIndex;
    }
    return 1;
}
/*
size_t write_qty(const uint8_t *data, size_t quantity)
{
    if(transmitting) {
        for(size_t i = 0; i < quantity; ++i) {
            if(!write(data[i])) {
                return i;
            }
        }
    }
    return quantity;
}
*/
int available(void)
{
    int result = rxLength - rxIndex;
    return result;
}

int iread(void)
{
    int value = -1;
    if(rxIndex < rxLength) {
        value = rxBuffer[rxIndex];
        ++rxIndex;
    }
    return value;
}

int peek(void)
{
    int value = -1;
    if(rxIndex < rxLength) {
        value = rxBuffer[rxIndex];
    }
    return value;
}

void flush(void)
{
    rxIndex = 0;
    rxLength = 0;
    txIndex = 0;
    txLength = 0;
    rxQueued = 0;
    txQueued = 0;
    i2cFreeQueue(i2c); // cleanup
}



