/*
    MIT License

    Copyright (c) 2016-2020, Alexey Dynda

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

#include "../io.h"

#if defined(PICO_BOARD)
#include "pico_i2c.h"

//////////////////////////////////////////////////////////////////////////////////
//                        PICO I2C IMPLEMENTATION
//////////////////////////////////////////////////////////////////////////////////


#include "pico/stdlib.h"
#include "hardware/i2c.h"

TwoWire *Wire = nullptr;
TwoWire *Wire1 = nullptr;

TwoWire::TwoWire(i2c_inst_t *i2c, int8_t sda, int8_t scl) {
    _sda = sda;
    _scl = scl;
    _i2c = i2c;
    _clkHz = TWI_CLOCK;
    _running = false;
    _txBegun = false;
    _buffLen = 0;
}

bool TwoWire::setSDA(int8_t pin) {
    constexpr uint32_t valid[2] = { __bitset({0, 4, 8, 12, 16, 20, 24, 28}) /* I2C0 */,
                                    __bitset({2, 6, 10, 14, 18, 22, 26})  /* I2C1 */
                                  };
    if ((!_running) && ((1 << pin) & valid[i2c_hw_index(_i2c)])) {
        _sda = pin;
        return true;
    }

    if (_sda == pin) {
        return true;
    }

    if (_running) {
        panic("FATAL: Attempting to set Wire%s.SDA while running", i2c_hw_index(_i2c) ? "1" : "");
    } else {
        panic("FATAL: Attempting to set Wire%s.SDA to illegal pin %d", i2c_hw_index(_i2c) ? "1" : "", pin);
    }
    return false;
}

bool TwoWire::setSCL(int8_t pin) {
    constexpr uint32_t valid[2] = { __bitset({1, 5, 9, 13, 17, 21, 25, 29}) /* I2C0 */,
                                    __bitset({3, 7, 11, 15, 19, 23, 27})  /* I2C1 */
                                  };
    if ((!_running) && ((1 << pin) & valid[i2c_hw_index(_i2c)])) {
        _scl = pin;
        return true;
    }

    if (_scl == pin) {
        return true;
    }

    if (_running) {
        panic("FATAL: Attempting to set Wire%s.SCL while running", i2c_hw_index(_i2c) ? "1" : "");
    } else {
        panic("FATAL: Attempting to set Wire%s.SCL to illegal pin %d", i2c_hw_index(_i2c) ? "1" : "", pin);
    }
    return false;
}

void TwoWire::setClock(uint32_t hz) {
    _clkHz = hz;
    if (_running) {
        i2c_set_baudrate(_i2c, hz);
    }
}

// Master mode
void TwoWire::begin() {
    if (_running) {
        // ERROR
        return;
    }
    _slave = false;
    i2c_init(_i2c, _clkHz);
    i2c_set_slave_mode(_i2c, false, 0);
    gpio_set_function(_sda, GPIO_FUNC_I2C);
    gpio_pull_up(_sda);
    gpio_set_function(_scl, GPIO_FUNC_I2C);
    gpio_pull_up(_scl);

    _running = true;
    _txBegun = false;
    _buffLen = 0;
}

static void _handler0() {
#if defined(__WIRE0_DEVICE)
    if (__WIRE0_DEVICE == i2c0) {
        Wire->onIRQ();
    } else {
        Wire1->onIRQ();
    }
#else
    Wire->onIRQ();
#endif
}

static void _handler1() {
#if defined(__WIRE1_DEVICE)
    if (__WIRE1_DEVICE == i2c0) {
        Wire->onIRQ();
    } else {
        Wire1->onIRQ();
    }
#else
    Wire1->onIRQ();
#endif
}

// Slave mode
void TwoWire::begin(uint8_t addr) {
    if (_running) {
        // ERROR
        return;
    }
    _slave = true;
    i2c_init(_i2c, _clkHz);
    i2c_set_slave_mode(_i2c, true, addr);

    // Our callback IRQ
    _i2c->hw->intr_mask = (1 << 12) | (1 << 10) | (1 << 9) | (1 << 6) | (1 << 5) | (1 << 2);

    int irqNo = I2C0_IRQ + i2c_hw_index(_i2c);
    irq_set_exclusive_handler(irqNo, i2c_hw_index(_i2c) == 0 ? _handler0 : _handler1);
    irq_set_enabled(irqNo, true);

    gpio_set_function(_sda, GPIO_FUNC_I2C);
    gpio_pull_up(_sda);
    gpio_set_function(_scl, GPIO_FUNC_I2C);
    gpio_pull_up(_scl);

    _running = true;
}

// See: https://github.com/earlephilhower/arduino-pico/issues/979#issuecomment-1328237128
#pragma GCC push_options
#pragma GCC optimize ("O0")
void TwoWire::onIRQ() {
    // Make a local copy of the IRQ status up front.  If it changes while we're
    // running the IRQ callback will fire again after returning.  Avoids potential
    // race conditions
    uint32_t irqstat = _i2c->hw->intr_stat;
    if (irqstat == 0) {
        return;
    }

    // First, pull off any data available
    if (irqstat & (1 << 2)) {
        // RX_FULL
        if (_buffLen < (int)sizeof(_buff)) {
            _buff[_buffLen++] = _i2c->hw->data_cmd & 0xff;
        } else {
            _i2c->hw->data_cmd;
        }
    }
    // RD_REQ
    if (irqstat & (1 << 5)) {
        if (_onRequestCallback) {
            _onRequestCallback();
        }
        _i2c->hw->clr_rd_req;
    }
    // TX_ABRT
    if (irqstat & (1 << 6)) {
        _i2c->hw->clr_tx_abrt;
    }
    // START_DET
    if (irqstat & (1 << 10)) {
        _slaveStartDet = true;
        _i2c->hw->clr_start_det;
    }
    // RESTART_DET
    if (irqstat & (1 << 12)) {
        if (_onReceiveCallback && _buffLen) {
            _onReceiveCallback(_buffLen);
        }
        _buffLen = 0;
        _buffOff = 0;
        _slaveStartDet = false;
        _i2c->hw->clr_restart_det;
    }
    // STOP_DET
    if (irqstat & (1 << 9)) {
        if (_onReceiveCallback && _buffLen) {
            _onReceiveCallback(_buffLen);
        }
        _buffLen = 0;
        _buffOff = 0;
        _slaveStartDet = false;
        _i2c->hw->clr_stop_det;
    }
}
#pragma GCC pop_options

void TwoWire::end() {
    if (!_running) {
        // ERROR
        return;
    }

    if (_slave) {
        int irqNo = I2C0_IRQ + i2c_hw_index(_i2c);
        irq_remove_handler(irqNo, i2c_hw_index(_i2c) == 0 ? _handler0 : _handler1);
        irq_set_enabled(irqNo, false);
    }

    i2c_deinit(_i2c);

    lcd_gpioMode(_sda, LCD_GPIO_INPUT);
    lcd_gpioMode(_scl, LCD_GPIO_INPUT);
    _running = false;
    _txBegun = false;
}

void TwoWire::beginTransmission(uint8_t addr) {
    if (!_running || _txBegun) {
        // ERROR
        return;
    }
    _addr = addr;
    _buffLen = 0;
    _buffOff = 0;
    _txBegun = true;
}

size_t TwoWire::requestFrom(uint8_t address, size_t quantity, bool stopBit) {
    if (!_running || _txBegun || !quantity || (quantity > sizeof(_buff))) {
        return 0;
    }

    _buffLen = i2c_read_blocking_until(_i2c, address, _buff, quantity, !stopBit, make_timeout_time_ms(_timeout));
    if ((_buffLen == PICO_ERROR_GENERIC) || (_buffLen == PICO_ERROR_TIMEOUT)) {
        if (_buffLen == PICO_ERROR_TIMEOUT) {
            _handleTimeout(_reset_with_timeout);
        }
        _buffLen = 0;
    }
    _buffOff = 0;
    return _buffLen;
}

size_t TwoWire::requestFrom(uint8_t address, size_t quantity) {
    return requestFrom(address, quantity, true);
}

// Equivalent to clock stretching using Pico SDK
static bool _clockStretch(uint gpio) {
    uint64_t end = time_us_64() + 100;
    while ((time_us_64() < end) && (gpio_get(gpio) == 0)) {
        // No-op, waiting for clock to be released
    }
    return gpio_get(gpio) != 0;
}

bool _probe(int addr, int8_t sda, int8_t scl, int freq) {
    int delay = (1000000 / freq) / 2;
    bool ack = false;

    lcd_gpioMode(sda, LCD_GPIO_INPUT_PULLUP);
    lcd_gpioMode(scl, LCD_GPIO_INPUT_PULLUP);
    gpio_set_function(scl, GPIO_FUNC_SIO);
    gpio_set_function(sda, GPIO_FUNC_SIO);

    lcd_gpioWrite(sda, LCD_HIGH);
    sleep_us(delay);
    lcd_gpioWrite(scl, LCD_HIGH);
    if (!_clockStretch(scl)) {
        goto stop;
    }
    lcd_gpioWrite(sda, LCD_LOW);
    sleep_us(delay);
    lcd_gpioWrite(scl, LCD_LOW);
    sleep_us(delay);
    for (int i = 0; i < 8; i++) {
        addr <<= 1;
        lcd_gpioWrite(sda, (addr & (1 << 7)) ? LCD_HIGH : LCD_LOW);
        sleep_us(delay);
        lcd_gpioWrite(scl, LCD_HIGH);
        sleep_us(delay);
        if (!_clockStretch(scl)) {
            goto stop;
        }
        lcd_gpioWrite(scl, LCD_LOW);
        sleep_us(5); // Ensure we don't change too close to clock edge
    }

    lcd_gpioWrite(sda, LCD_HIGH);
    sleep_us(delay);
    lcd_gpioWrite(scl, LCD_HIGH);
    if (!_clockStretch(scl)) {
        goto stop;
    }

    ack = gpio_get(sda) == LCD_LOW;
    sleep_us(delay);
    lcd_gpioWrite(scl, LCD_LOW);

stop:
    sleep_us(delay);
    lcd_gpioWrite(sda, LCD_LOW);
    sleep_us(delay);
    lcd_gpioWrite(scl, LCD_HIGH);
    sleep_us(delay);
    lcd_gpioWrite(sda, LCD_HIGH);
    sleep_us(delay);
    gpio_set_function(scl, GPIO_FUNC_I2C);
    gpio_set_function(sda, GPIO_FUNC_I2C);

    return ack;
}

void TwoWire::_handleTimeout(bool reset) {
    _timeoutFlag = true;

    if (reset) {
        if (_slave) {
            uint8_t prev_addr = _addr;
            int prev_clkHz = _clkHz;
            end();
            setClock(prev_clkHz);
            begin(prev_addr);
        } else {
            int prev_clkHz = _clkHz;
            end();
            setClock(prev_clkHz);
            begin();
        }
    }
}

// Errors:
//  0 : Success
//  1 : Data too long
//  2 : NACK on transmit of address
//  3 : NACK on transmit of data
//  4 : Other error
//  5 : Timeout
uint8_t TwoWire::endTransmission(bool stopBit) {
    if (!_running || !_txBegun) {
        return 4;
    }
    _txBegun = false;
    if (!_buffLen) {
        // Special-case 0-len writes which are used for I2C probing
        return _probe(_addr, _sda, _scl, _clkHz) ? 0 : 2;
    } else {
        auto len = _buffLen;
        auto ret = i2c_write_blocking_until(_i2c, _addr, _buff, _buffLen, !stopBit, make_timeout_time_ms(_timeout));
        if (ret == PICO_ERROR_TIMEOUT) {
            _handleTimeout(_reset_with_timeout);
            return 5;
        }
        _buffLen = 0;
        return (ret == len) ? 0 : 4;
    }
}

uint8_t TwoWire::endTransmission() {
    return endTransmission(true);
}

size_t TwoWire::write(uint8_t ucData) {
    if (!_running) {
        return 0;
    }

    if (_slave) {
        // Wait for a spot in the TX FIFO
        while (0 == (_i2c->hw->status & (1 << 1))) { /* noop wait */ }
        _i2c->hw->data_cmd = ucData;
        return 1;
    } else {
        if (!_txBegun || (_buffLen == sizeof(_buff))) {
            return 0;
        }
        _buff[_buffLen++] = ucData;
        return 1 ;
    }
}

size_t TwoWire::write(const uint8_t *data, size_t quantity) {
    for (size_t i = 0; i < quantity; ++i) {
        if (!write(data[i])) {
            return i;
        }
    }

    return quantity;
}

int TwoWire::available(void) {
    return _running  ? _buffLen - _buffOff : 0;
}

int TwoWire::read(void) {
    if (available()) {
        return _buff[_buffOff++];
    }
    return -1; // EOF
}

int TwoWire::peek(void) {
    if (available()) {
        return _buff[_buffOff];
    }
    return -1; // EOF
}

void TwoWire::flush(void) {
    // Do nothing, use endTransmission(..) to force
    // data transfer.
}

void TwoWire::onReceive(void(*function)(int)) {
    _onReceiveCallback = function;
}

void TwoWire::onRequest(void(*function)(void)) {
    _onRequestCallback = function;
}

void TwoWire::setTimeout(uint32_t timeout, bool reset_with_timeout) {
    _timeoutFlag = false;
    _timeout = timeout;
    _reset_with_timeout = reset_with_timeout;
}

bool TwoWire::getTimeoutFlag() {
    return _timeoutFlag;
}

void TwoWire::clearTimeoutFlag() {
    _timeoutFlag = false;
}

static uint8_t s_bytesWritten = 0;

PicoI2c::PicoI2c(int8_t i2c_bus, int8_t scl, int8_t sda, uint8_t sa, uint32_t freq)
    : m_i2c_bus(i2c_bus), m_scl(scl), m_sda(sda), m_sa(sa), m_freq(freq), m_mode(0)
{
}

PicoI2c::~PicoI2c()
{
}

void PicoI2c::begin()
{
    m_i2c = m_i2c_bus ? i2c1 : i2c0;
    Wire = new TwoWire(m_i2c, m_sda, m_scl);
    Wire->begin();
    Wire->setClock(m_freq ?: 400000);
}

void PicoI2c::end()
{

}

void PicoI2c::start()
{
    Wire->beginTransmission(m_sa);
    s_bytesWritten = 0;
}

void PicoI2c::stop()
{
    Wire->endTransmission();
}

void PicoI2c::send(uint8_t data)
{
    if ( s_bytesWritten == 0 )
        m_mode = data;
        // Do not write too many bytes for standard Wire->h. It may become broken
/*
    if ( Wire->write(data) != 0 )
    {
        s_bytesWritten++;
        return;
    }
*/
    if ( s_bytesWritten >= 64 )
    {
        stop();
        start();
        send(m_mode);
        /* Commands never require many bytes. Thus assume that user tries to send data */
    }
    Wire->write(data);
    s_bytesWritten++;
}

void PicoI2c::sendBuffer(const uint8_t *buffer, uint16_t size)
{
    while ( size-- )
    {
        send(*buffer);
        buffer++;
    }
}


#endif // PICO_BOARD
