#ifndef getTemp_h
#define getTemp_h

#define SPINDLE_TEMP_DDR  DDRF
#define SPINDLE_TEMP_PORT PORTF
#define SPINDLE_TEMP_PIN  PINF
#define SPINDLE_TEMP_BIT  0

#define R_FAN_TEMP_DDR  DDRE
#define R_FAN_TEMP_PORT PORTE
#define R_FAN_TEMP_PIN  PINE
#define R_FAN_TEMP_BIT  5

#define L_FAN_TEMP_DDR  DDRC
#define L_FAN_TEMP_PORT PORTC
#define L_FAN_TEMP_PIN  PINC
#define L_FAN_TEMP_BIT  0

typedef struct
{
    float spindle_temp;
} all_temp;
// 声明温度
extern all_temp temp_obj;

float ds18b20_read_temp();
float ds18b20_read_temp_timer2();
void time2_init();
#endif
