#include "grbl.h"

volatile bool tempConversionDone = false;
volatile uint16_t tempConversionCounter = 0;
static bool conversionStarted = false;
volatile uint16_t readSpindleTempNum = 0;

// 0主轴，1左风扇，2右风扇

all_temp temp_obj;

// 设置为输出
void onewire_output(uint8_t flag) {
    if(flag==0){
        SPINDLE_TEMP_DDR |= (1 << SPINDLE_TEMP_BIT);
    }else if (flag==1)
    {
        L_FAN_TEMP_DDR |= (1 << L_FAN_TEMP_BIT);
    }else if (flag==2)
    {
        R_FAN_TEMP_DDR |= (1 << R_FAN_TEMP_BIT);
    }
}

// 设置为输入（释放总线）
void onewire_input(uint8_t flag) {
    if(flag==0){
        SPINDLE_TEMP_DDR &= ~(1 << SPINDLE_TEMP_BIT);
        SPINDLE_TEMP_PORT |= (1 << SPINDLE_TEMP_BIT); // 启用内部上拉
    }else if (flag==1)
    {
        L_FAN_TEMP_DDR &= ~(1 << L_FAN_TEMP_BIT);
        L_FAN_TEMP_PORT |= (1 << L_FAN_TEMP_BIT); // 启用内部上拉
    }else if (flag==2)
    {
        R_FAN_TEMP_DDR &= ~(1 << R_FAN_TEMP_BIT);
        R_FAN_TEMP_PORT |= (1 << R_FAN_TEMP_BIT); // 启用内部上拉
    }

}

// 写0或1
void onewire_write_bit(uint8_t flag, uint8_t bit) {
    onewire_output(flag);
    if (bit) {
        if(flag==0){
            SPINDLE_TEMP_PORT &= ~(1 << SPINDLE_TEMP_BIT);
        }else if (flag==1)
        {
            L_FAN_TEMP_PORT &= ~(1 << L_FAN_TEMP_BIT);
        }else if (flag==2)
        {
            R_FAN_TEMP_PORT &= ~(1 << R_FAN_TEMP_BIT);
        }
        _delay_us(5);
        onewire_input(flag);
        _delay_us(55);
    } else {
        if(flag==0){
            SPINDLE_TEMP_PORT &= ~(1 << SPINDLE_TEMP_BIT);
        }else if (flag==1)
        {
            L_FAN_TEMP_PORT &= ~(1 << L_FAN_TEMP_BIT);
        }else if (flag==2)
        {
            R_FAN_TEMP_PORT &= ~(1 << R_FAN_TEMP_BIT);
        }
        _delay_us(65);
        onewire_input(flag);
        _delay_us(5);
    }
}

// 读1位
uint8_t onewire_read_bit(uint8_t flag) {
    uint8_t bit = 0;
    onewire_output(flag);
    if(flag==0){
        SPINDLE_TEMP_PORT &= ~(1 << SPINDLE_TEMP_BIT);
    }else if (flag==1)
    {
        L_FAN_TEMP_PORT &= ~(1 << L_FAN_TEMP_BIT);
    }else if (flag==2)
    {
        R_FAN_TEMP_PORT &= ~(1 << R_FAN_TEMP_BIT);
    }
    _delay_us(3);
    onewire_input(flag);
    _delay_us(10);
    if(flag==0){
        bit = (SPINDLE_TEMP_PIN & (1 << SPINDLE_TEMP_BIT)) ? 1 : 0;
    }else if (flag==1)
    {
        bit = (L_FAN_TEMP_PIN & (1 << L_FAN_TEMP_BIT)) ? 1 : 0;
    }else if (flag==2)
    {
        bit = (R_FAN_TEMP_PIN & (1 << R_FAN_TEMP_BIT)) ? 1 : 0;
    }
    _delay_us(50);
    return bit;
}

// 写1字节
void onewire_write_byte(uint8_t flag, uint8_t byte) {
    for (int i = 0; i < 8; i++) {
        onewire_write_bit(flag, byte & 0x01);
        byte >>= 1;
    }
}

// 读1字节
uint8_t onewire_read_byte(uint8_t flag) {
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        byte >>= 1;
        if (onewire_read_bit(flag)) {
            byte |= 0x80;
        }
    }
    return byte;
}

// 复位并检测设备存在
uint8_t onewire_reset(uint8_t flag) {
    uint8_t presence = 0;
    onewire_output(flag);
    if(flag==0){
        SPINDLE_TEMP_PORT &= ~(1 << SPINDLE_TEMP_BIT);
    }else if (flag==1)
    {
        L_FAN_TEMP_PORT &= ~(1 << L_FAN_TEMP_BIT);
    }else if (flag==2)
    {
        R_FAN_TEMP_PORT &= ~(1 << R_FAN_TEMP_BIT);
    }
    _delay_us(480);
    onewire_input(flag);
    _delay_us(70);
    if(flag==0){
        presence = (SPINDLE_TEMP_PIN & (1 << SPINDLE_TEMP_BIT)) ? 0 : 1;
    }else if (flag==1)
    {
        presence = (L_FAN_TEMP_PIN & (1 << L_FAN_TEMP_BIT)) ? 0 : 1;
    }else if (flag==2)
    {
        presence = (R_FAN_TEMP_PIN & (1 << R_FAN_TEMP_BIT)) ? 0 : 1;
    }
    _delay_us(410);
    return presence;
}

// // 读取温度
// float ds18b20_read_temp(uint8_t flag) {
//     uint8_t temp_l, temp_h;
//     int16_t temp;
    
//     if (!onewire_reset(flag)) return -1;  // 无响应

//     onewire_write_byte(flag, 0xCC);  // Skip ROM
//     onewire_write_byte(flag, 0x44);  // Convert T
//     _delay_ms(750);            // 等待转换

//     onewire_reset();
//     onewire_write_byte(flag, 0xCC);  // Skip ROM
//     onewire_write_byte(flag, 0xBE);  // Read Scratchpad

//     temp_l = onewire_read_byte(flag);
//     temp_h = onewire_read_byte(flag);
//     temp = (temp_h << 8) | temp_l;
//     // printFloat(temp * 0.0625, 3);
//     return temp * 0.0625;  // 每位代表0.0625℃
// }


// void time2_init() {
//     // 配置 Timer2（8位定时器，CTC模式，64分频）
//     TCCR2A = (1 << WGM21);  // CTC模式
//     TCCR2B = (1 << CS22);   // 64分频（16MHz / 64 = 250kHz）
//     OCR2A = 249;            // 1ms = (250kHz / 250) - 1
//     TIMSK2 = (1 << OCIE2A); // 启用比较匹配中断
//     sei();                  // 启用全局中断
// }

void time2_init() {
    // 配置 Timer2（8位定时器，CTC模式，1024分频）
    TCCR2A = (1 << WGM21);   // CTC模式
    TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);  // 1024分频（16MHz / 1024 = 15.625kHz）
    OCR2A = 1561;            // 100ms = (15.625kHz / 1562) - 1
    TIMSK2 = (1 << OCIE2A);  // 启用比较匹配中断
    sei();                   // 启用全局中断
}

ISR(TIMER2_COMPA_vect) {
    // 500ms延时等待
    if (tempConversionCounter < 50) {  // 5000ms = 50 * 100ms
        tempConversionCounter++;
        if (tempConversionCounter % 5 == 0) {  // 仅在 5, 10, 15... 时触发
            switch (tempConversionCounter / 5)
            {
            case 1:
                ds18b20_read_temp_timer2(0);
                break;
            case 2:
                tempConversionDone = true;
                conversionStarted = false;
                break;
            case 3:
                ds18b20_read_temp_timer2(0);
                break;
            case 4:
                ds18b20_read_temp_timer2(1);
                break;
            case 5:
                tempConversionDone = true;
                conversionStarted = false;
                break;
            case 6:
                ds18b20_read_temp_timer2(1);
                break;
            case 7:
                ds18b20_read_temp_timer2(2);
                break;
            case 8:
                tempConversionDone = true;
                conversionStarted = false;
                break;
            case 9:
                ds18b20_read_temp_timer2(2);
                break;
            default:
                break;
            }
        }
        // if(tempConversionCounter == 500){
        //     ds18b20_read_temp_timer2(0);
        // }
        // if (tempConversionCounter == 1000)
        // {
        //     tempConversionDone = true;
        //     conversionStarted = false;
        // }
        // if(tempConversionCounter == 1500){
        //     ds18b20_read_temp_timer2(0);
        // }
    } else {
        tempConversionCounter = 0;
    }
}

float ds18b20_read_temp_timer2(uint8_t flag) {
    if (!conversionStarted) {
        if (!onewire_reset(flag)) return 0;
        onewire_write_byte(flag, 0xCC);  // Skip ROM
        onewire_write_byte(flag, 0x44);  // Convert T
        conversionStarted = true;
        return 0;
    }

    if (tempConversionDone) {
        onewire_reset(flag);
        onewire_write_byte(flag, 0xCC);  // Skip ROM
        onewire_write_byte(flag, 0xBE);  // Read Scratchpad
        uint8_t temp_l = onewire_read_byte(flag);
        uint8_t temp_h = onewire_read_byte(flag);
        int16_t temp = (temp_h << 8) | temp_l;
        conversionStarted = false;
        tempConversionDone = false;
        if(flag==0){
            temp_obj.spindle_temp = temp * 0.0625;
        }else if (flag==1)
        {
            temp_obj.l_fan_temp = temp * 0.0625;
        }else if (flag==2)
        {
            temp_obj.r_fan_temp = temp * 0.0625;
        }
        
        // printFloat(temp * 0.0625, 3);
        return temp / 16.0;
    }
    return 0;  // 等待中
}
