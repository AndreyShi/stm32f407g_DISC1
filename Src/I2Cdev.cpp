// I2Cdev library collection - Main I2C device class header file
// Abstracts bit and byte I2C R/W functions into a convenient class
// 6/9/2012 by Jeff Rowberg <jeff@rowberg.net>
// 6/6/2015 by Andrey Voloshin <voloshin@think.in.ua>
//
// Changelog:
//      2015-06-06 - ported to STM32 HAL library from Arduino code

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2013 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include "I2Cdev.h"

// Hold pointer to inited HAL I2C device
static I2C_HandleTypeDef * I2Cdev_hi2c;

/** Default timeout value for read operations.
 * Set this to 0 to disable timeout detection.
 */
uint16_t I2Cdev::readTimeout = I2CDEV_DEFAULT_READ_TIMEOUT;

/** Sets device handle to use for communications
 * You can call this function and set any other device at any moment
 */
void I2Cdev::init(I2C_HandleTypeDef * hi2c){
	I2Cdev_hi2c = hi2c;
}

/** Read a single bit from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev_readTimeout)
 * @return Status of read operation (true = success)
 */
uint8_t I2Cdev::readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data, uint16_t timeout,void* wireObj)
{
    uint8_t b;
    uint8_t count = I2Cdev::readByte(devAddr, regAddr, &b, timeout,wireObj);
    *data = b & (1 << bitNum);
    return count;
}

/** Read a single bit from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-15)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev_readTimeout)
 * @return Status of read operation (true = success)
 */
uint8_t I2Cdev::readBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t *data, uint16_t timeout,void* wireObj)
{
    uint16_t b;
    uint8_t count = I2Cdev::readWord(devAddr, regAddr, &b, timeout,wireObj);
    *data = b & (1 << bitNum);
    return count;
}

/** Read multiple bits from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev_readTimeout)
 * @return Status of read operation (true = success)
 */
uint8_t I2Cdev::readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, uint16_t timeout,void* wireObj)
{
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t count, b;
    if ((count = I2Cdev::readByte(devAddr, regAddr, &b, timeout,wireObj)) != 0)
    {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        b &= mask;
        b >>= (bitStart - length + 1);
        *data = b;
    }
    return count;
}

/** Read multiple bits from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-15)
 * @param length Number of bits to read (not more than 16)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev_readTimeout)
 * @return Status of read operation (1 = success, 0 = failure, -1 = timeout)
 */
uint8_t I2Cdev::readBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *data, uint16_t timeout,void* wireObj)
{
    // 1101011001101001 read byte
    // fedcba9876543210 bit numbers
    //    xxx           args: bitStart=12, length=3
    //    010           masked
    //           -> 010 shifted
    uint8_t count;
    uint16_t w;
    if ((count = I2Cdev::readWord(devAddr, regAddr, &w, timeout,wireObj)) != 0)
    {
        uint16_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        w &= mask;
        w >>= (bitStart - length + 1);
        *data = w;
    }
    return count;
}

/** Read single byte from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for byte value read from device
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev_readTimeout)
 * @return Status of read operation (true = success)
 */
uint8_t I2Cdev::readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t timeout,void* wireObj)
{
    return I2Cdev::readBytes(devAddr, regAddr, 1, data, timeout,wireObj);
}

/** Read single word from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for word value read from device
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev_readTimeout)
 * @return Status of read operation (true = success)
 */
uint8_t I2Cdev::readWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data, uint16_t timeout,void* wireObj)
{
    return I2Cdev::readWords(devAddr, regAddr, 1, data, timeout,wireObj);
}

/** Read multiple bytes from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of bytes to read
 * @param data Buffer to store read data in
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev_readTimeout)
 * @return Number of bytes read (-1 indicates failure)
 */
uint8_t I2Cdev::readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout,void* wireObj)
{
    uint16_t tout = timeout > 0 ? timeout : I2CDEV_DEFAULT_READ_TIMEOUT;

    HAL_I2C_Master_Transmit(I2Cdev_hi2c, devAddr << 1, &regAddr, 1, tout);
    if (HAL_I2C_Master_Receive(I2Cdev_hi2c, devAddr << 1, data, length, tout) == HAL_OK) return length;
    return -1;
}

/** Read multiple words from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of words to read
 * @param data Buffer to store read data in
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev_readTimeout)
 * @return Number of words read (-1 indicates failure)
 */
uint8_t I2Cdev::readWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data, uint16_t timeout,void* wireObj)
{
    uint16_t tout = timeout > 0 ? timeout : I2CDEV_DEFAULT_READ_TIMEOUT;

    HAL_I2C_Master_Transmit(I2Cdev_hi2c, devAddr << 1, &regAddr, 1, tout);
    if (HAL_I2C_Master_Receive(I2Cdev_hi2c, devAddr << 1, (uint8_t *)data, length*2, tout) == HAL_OK)
        return length;
    else
        return -1;
}

/** write a single bit in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
uint16_t I2Cdev::writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data,void* wireObj)
{
    uint8_t b;
    I2Cdev::readByte(devAddr, regAddr, &b, I2Cdev::readTimeout,wireObj);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return I2Cdev::writeByte(devAddr, regAddr, b,wireObj);
}

/** write a single bit in a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-15)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
uint16_t I2Cdev::writeBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t data,void* wireObj)
{
    uint16_t w;
    I2Cdev::readWord(devAddr, regAddr, &w, 100,wireObj);
    w = (data != 0) ? (w | (1 << bitNum)) : (w & ~(1 << bitNum));
    return I2Cdev::writeWord(devAddr, regAddr, w,wireObj);
}

/** Write multiple bits in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
uint16_t I2Cdev::writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data,void* wireObj)
{
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t b;
    if (I2Cdev::readByte(devAddr, regAddr, &b, 100,wireObj) != 0)
    {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        b &= ~(mask); // zero all important bits in existing byte
        b |= data; // combine data with existing byte
        return I2Cdev::writeByte(devAddr, regAddr, b,wireObj);
    }
    else
    {
        return 0;
    }
}

/** Write multiple bits in a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-15)
 * @param length Number of bits to write (not more than 16)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
uint16_t I2Cdev::writeBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data,void* wireObj)
{
    //              010 value to write
    // fedcba9876543210 bit numbers
    //    xxx           args: bitStart=12, length=3
    // 0001110000000000 mask word
    // 1010111110010110 original value (sample)
    // 1010001110010110 original & ~mask
    // 1010101110010110 masked | value
    uint16_t w;
    if (I2Cdev::readWord(devAddr, regAddr, &w, 100,wireObj) != 0)
    {
        uint16_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        w &= ~(mask); // zero all important bits in existing word
        w |= data; // combine data with existing word
        return I2Cdev::writeWord(devAddr, regAddr, w,wireObj);
    }
    else
    {
        return 0;
    }
}

/** Write single byte to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New byte value to write
 * @return Status of operation (true = success)
 */
uint16_t I2Cdev::writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data,void* wireObj)
{
    return I2Cdev::writeBytes(devAddr, regAddr, 1, &data,wireObj);
}

/** Write single word to a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New word value to write
 * @return Status of operation (true = success)
 */
uint16_t I2Cdev::writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data,void* wireObj)
{
    return I2Cdev::writeWords(devAddr, regAddr, 1, &data,wireObj);
}

/** Write multiple bytes to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register address to write to
 * @param length Number of bytes to write
 * @param data Buffer to copy new data from
 * @return Status of operation (true = success)
 */
uint16_t I2Cdev::writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* pData,void* wireObj)
{
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(I2Cdev_hi2c, devAddr << 1, regAddr, I2C_MEMADD_SIZE_8BIT, pData, length, 1000);
    return status == HAL_OK;
}

/** Write multiple words to a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register address to write to
 * @param length Number of words to write
 * @param data Buffer to copy new data from
 * @return Status of operation (true = success)
 */
uint16_t I2Cdev::writeWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t* pData,void* wireObj)
{
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(I2Cdev_hi2c, devAddr << 1, regAddr, I2C_MEMADD_SIZE_8BIT, (uint8_t *)pData, sizeof(uint16_t) * length, 1000);
    return status == HAL_OK;
}


Arduino_Serial Serial;
uint32_t ulHighFrequencyTimerTicks;

void Arduino_Serial::write(char ch)
{
   printf("%c",ch);
}

void Arduino_Serial::print(const char* s)
{
   printf("%s",s);
}

void Arduino_Serial::println(const char* s)
{
    if(s == 0)
    {printf("\n");}
    else
    {printf("%s\n",s);}
}

void Arduino_Serial::println(char s)
{
    printf("%c\n",s);
}

void Arduino_Serial::print(float dt,int sz)
{
    printf("%.5f",dt);
}

void Arduino_Serial::println(float dt,int sz)
{
    printf("%.3f\n",dt);
}

void Arduino_Serial::println(double dt,int sz)
{
    printf("%.3f\n",dt);
}

void Arduino_Serial::print(double dt,int sz)
{
    printf("%.3f",dt);
}
void Arduino_Serial::print(int t)
{
    printf("%d",t);
}
void Arduino_Serial::print(char t)
{
    printf("%c",t);
}
/*
for arduino compability
in uint32_t 4294 sec max.
*/
uint32_t micros(void)
{
    extern TIM_HandleTypeDef htim7;
    return __HAL_TIM_GET_COUNTER(&htim7);//ulHighFrequencyTimerTicks;
}

void delayMicroseconds(int t)
{
    uint32_t tmp = micros();
    while(micros() - tmp <= (uint32_t)t)
        {;}
}
/*
for arduino compability
*/
void delay(int n)
{
    HAL_Delay(n);
    return;
    uint32_t tmp = micros();
    uint32_t timer = n * 1000;

    while(micros() - tmp <= timer)
        {__asm("nop");}
}
