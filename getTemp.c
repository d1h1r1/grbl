#include "grbl.h"
#include "getTemp.h"

// DS18B20连接在 PE0（数字0）
#define DS18B20_DDR  DDRE
#define DS18B20_PORT PORTE
#define DS18B20_PIN  PINE
#define DS18B20_BIT  3

volatile bool tempConversionDone = false;
volatile uint16_t tempConversionCounter = 0;
static bool conversionStarted = false;


// 设置为输出
void onewire_output() {
    DS18B20_DDR |= (1 << DS18B20_BIT);
}

// 设置为输入（释放总线）
void onewire_input() {
    DS18B20_DDR &= ~(1 << DS18B20_BIT);
}

// 写0或1
void onewire_write_bit(uint8_t bit) {
    onewire_output();
    if (bit) {
        DS18B20_PORT &= ~(1 << DS18B20_BIT);
        _delay_us(5);
        onewire_input();
        _delay_us(55);
    } else {
        DS18B20_PORT &= ~(1 << DS18B20_BIT);
        _delay_us(65);
        onewire_input();
        _delay_us(5);
    }
}

// 读1位
uint8_t onewire_read_bit() {
    uint8_t bit = 0;
    onewire_output();
    DS18B20_PORT &= ~(1 << DS18B20_BIT);
    _delay_us(3);
    onewire_input();
    _delay_us(10);
    bit = (DS18B20_PIN & (1 << DS18B20_BIT)) ? 1 : 0;
    _delay_us(50);
    return bit;
}

// 写1字节
void onewire_write_byte(uint8_t byte) {
    for (int i = 0; i < 8; i++) {
        onewire_write_bit(byte & 0x01);
        byte >>= 1;
    }
}

// 读1字节
uint8_t onewire_read_byte() {
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        byte >>= 1;
        if (onewire_read_bit()) {
            byte |= 0x80;
        }
    }
    return byte;
}

// 复位并检测设备存在
uint8_t onewire_reset() {
    uint8_t presence = 0;
    onewire_output();
    DS18B20_PORT &= ~(1 << DS18B20_BIT);
    _delay_us(480);
    onewire_input();
    _delay_us(70);
    presence = (DS18B20_PIN & (1 << DS18B20_BIT)) ? 0 : 1;
    _delay_us(410);
    return presence;
}

// 读取温度
float ds18b20_read_temp() {
    uint8_t temp_l, temp_h;
    int16_t temp;
    
    if (!onewire_reset()) return -1;  // 无响应

    onewire_write_byte(0xCC);  // Skip ROM
    onewire_write_byte(0x44);  // Convert T
    _delay_ms(500);            // 等待转换

    onewire_reset();
    onewire_write_byte(0xCC);  // Skip ROM
    onewire_write_byte(0xBE);  // Read Scratchpad

    temp_l = onewire_read_byte();
    temp_h = onewire_read_byte();
    temp = (temp_h << 8) | temp_l;
    return temp / 16.0;  // 每位代表0.0625℃
}


void time2_init() {
    // 配置 Timer2（8位定时器，CTC模式，64分频）
    TCCR2A = (1 << WGM21);  // CTC模式
    TCCR2B = (1 << CS22);   // 64分频（16MHz / 64 = 250kHz）
    OCR2A = 249;            // 1ms = (250kHz / 250) - 1
    TIMSK2 = (1 << OCIE2A); // 启用比较匹配中断
    sei();                  // 启用全局中断
}


ISR(TIMER2_COMPA_vect) {
    if (tempConversionCounter < 5000) {  // 500ms = 500 * 1ms
        tempConversionCounter++;
    } else {
        tempConversionDone = true;
        conversionStarted = false;
        tempConversionCounter = 0;
    }
}

float ds18b20_read_temp_timer2() {
    if (!conversionStarted) {
        if (!onewire_reset()) return 0;
        onewire_write_byte(0xCC);  // Skip ROM
        onewire_write_byte(0x44);  // Convert T
        conversionStarted = true;
        return 0;
    }

    if (tempConversionDone) {
        onewire_reset();
        onewire_write_byte(0xCC);  // Skip ROM
        onewire_write_byte(0xBE);  // Read Scratchpad
        uint8_t temp_l = onewire_read_byte();
        uint8_t temp_h = onewire_read_byte();
        int16_t temp = (temp_h << 8) | temp_l;
        conversionStarted = false;
        tempConversionDone = false;
        return temp / 16.0;
    }

    return 0;  // 等待中
}
