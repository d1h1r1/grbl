#include "grbl.h"

void fan_init()
{
    AIR_FAN_DDR |= (1 << AIR_FAN_BIT); // 将其配置为输出引脚。
}

// 1开0关
void air_fan_control(uint8_t flag)
{
    if(flag-48){
        AIR_FAN_PORT |= (1 << AIR_FAN_BIT);
    }else{
        AIR_FAN_PORT &= ~(1 << AIR_FAN_BIT);
    }
}
