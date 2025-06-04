#ifndef getTemp_h
#define getTemp_h


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
