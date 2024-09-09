/*
    MIT License

    Copyright (c) 2018-2019, Alexey Dynda

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

/*
 * @file hal/pico/pico_i2c.h LCDGFX Raspberry Pi Pico Dummy !!! Interface
 */

#pragma once


#if defined(PICO_BOARD)
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// WIRE_HAS_END means Wire has end()
#define WIRE_HAS_END 1

#ifndef WIRE_BUFFER_SIZE
#define WIRE_BUFFER_SIZE 256
#endif

#ifndef __WIRE0_DEVICE
#define __WIRE0_DEVICE i2c0
#endif
#ifndef __WIRE1_DEVICE
#define __WIRE1_DEVICE i2c1
#endif

template <size_t N>
constexpr uint32_t __bitset(const int (&a)[N], size_t i = 0U) {
    return i < N ? (1L << a[i]) | __bitset(a, i + 1) : 0;
}


class TwoWire {
public:
    TwoWire(i2c_inst_t *i2c, int8_t sda, int8_t scl);
    ~TwoWire();

    // Start as Master
    void begin();
    // Start as Slave
    void begin(uint8_t address);
    // Shut down the I2C interface
    void end();

    // Select IO pins to use.  Call before ::begin()
    bool setSDA(int8_t sda);
    bool setSCL(int8_t scl);

    void setClock(uint32_t freqHz);

    void beginTransmission(uint8_t);
    uint8_t endTransmission(bool stopBit);
    uint8_t endTransmission(void);

    size_t requestFrom(uint8_t address, size_t quantity, bool stopBit);
    size_t requestFrom(uint8_t address, size_t quantity);

    size_t write(uint8_t data);
    size_t write(const uint8_t * data, size_t quantity);

    virtual int available(void);
    virtual int read(void);
    virtual int peek(void);
    virtual void flush(void);
    void onReceive(void(*)(int));
    void onRequest(void(*)(void));

    inline size_t write(unsigned long n) {
        return write((uint8_t)n);
    }
    inline size_t write(long n) {
        return write((uint8_t)n);
    }
    inline size_t write(unsigned int n) {
        return write((uint8_t)n);
    }
    inline size_t write(int n) {
        return write((uint8_t)n);
    }

    void setTimeout(uint32_t timeout = 25, bool reset_with_timeout = false);     // sets the maximum number of milliseconds to wait
    bool getTimeoutFlag(void);
    void clearTimeoutFlag(void);

    // IRQ callback
    void onIRQ();

private:
    i2c_inst_t *_i2c;
    uint8_t _sda;
    uint8_t _scl;
    int _clkHz;

    bool _running;
    bool _slave;
    uint8_t _addr;
    bool _txBegun;

    unsigned long _timeout = 1000; 
    bool _timeoutFlag;
    bool _reset_with_timeout;
    void _handleTimeout(bool reset);

    uint8_t _buff[WIRE_BUFFER_SIZE];
    int _buffLen;
    int _buffOff;

    // Callback user functions
    void (*_onRequestCallback)(void);
    void (*_onReceiveCallback)(int);

    bool _slaveStartDet = false;

    // TWI clock frequency
    static const uint32_t TWI_CLOCK = 100000;
};

extern TwoWire *Wire;
extern TwoWire *Wire1;

/**
 * Class implements i2c support via Wire library for Arduino platforms
 */
class PicoI2c
{
public:
    /**
     * Creates i2c implementation instance for Arduino platform.
     * @param scl clock pin to use for i2c
     * @param sda data pin to use for i2c
     * @param sa i2c address of the device to control over i2c
     */
    PicoI2c(int8_t i2c_bus, int8_t scl = -1, int8_t sda = -1, uint8_t sa = 0x00, uint32_t freq = 0);
    ~PicoI2c();

    /**
     * Initializes i2c interface
     */
    void begin();

    /**
     * Closes i2c interface
     */
    void end();

    /**
     * Starts communication with SSD1306 display.
     */
    void start();

    /**
     * Ends communication with SSD1306 display.
     */
    void stop();

    /**
     * Sends byte to SSD1306 device
     * @param data - byte to send
     */
    void send(uint8_t data);

    /**
     * @brief Sends bytes to SSD1306 device
     *
     * Sends bytes to SSD1306 device. This functions gives
     * ~ 30% performance increase than ssd1306_intf.send.
     *
     * @param buffer - bytes to send
     * @param size - number of bytes to send
     */
    void sendBuffer(const uint8_t *buffer, uint16_t size);

    /**
     * Sets i2c address for communication
     * This API is required for some led displays having multiple
     * i2c addresses for different types of data.
     *
     * @param addr i2c address to set (7 bits)
     */
    void setAddr(uint8_t addr)
    {
        m_sa = addr;
    }

    i2c_inst_t* m_i2c;
    uint8_t m_sa;
private:
    int8_t m_i2c_bus;
    int8_t m_scl;
    int8_t m_sda;
    uint8_t m_mode;
    uint32_t m_freq;
};

#endif
