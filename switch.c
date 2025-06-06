#include "grbl.h"

void switch_init()
{
    AIR_FAN_DDR |= (1 << AIR_FAN_BIT);
    SPINDLE_L_FAN_DDR |= (1 << SPINDLE_L_FAN_BIT);
    SPINDLE_R_FAN_DDR |= (1 << SPINDLE_R_FAN_BIT);
    BLOW_FAN_DDR |= (1 << BLOW_FAN_BIT);
    SUCTION_CUP_DDR |= (1 << SUCTION_CUP_BIT);
    LIGHT_DDR |= (1 << LIGHT_BIT);
    SPRAY_DDR |= (1 << SPRAY_BIT);
    L_WATER_DDR |= (1 << L_WATER_BIT);
    R_WATER_DDR |= (1 << R_WATER_BIT);
    OUTLINE_DDR |= (1 << OUTLINE_BIT);
    CAMERA_DDR |= (1 << CAMERA_BIT);
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

// 1开0关
void spindle_l_fan_control(uint8_t flag)
{
    if(flag-48){
        SPINDLE_L_FAN_PORT |= (1 << SPINDLE_L_FAN_BIT);
    }else{
        SPINDLE_L_FAN_PORT &= ~(1 << SPINDLE_L_FAN_BIT);
    }
}

// 1开0关
void spindle_r_fan_control(uint8_t flag)
{
    if(flag-48){
        SPINDLE_R_FAN_PORT |= (1 << SPINDLE_R_FAN_BIT);
    }else{
        SPINDLE_R_FAN_PORT &= ~(1 << SPINDLE_R_FAN_BIT);
    }
}

// 1开0关
void blow_fan_control(uint8_t flag)
{
    if(flag-48){
        BLOW_FAN_PORT |= (1 << BLOW_FAN_BIT);
    }else{
        BLOW_FAN_PORT &= ~(1 << BLOW_FAN_BIT);
    }
}

// 1开0关
void suction_cup_control(uint8_t flag)
{
    if(flag-48){
        SUCTION_CUP_PORT |= (1 << SUCTION_CUP_BIT);
    }else{
        SUCTION_CUP_PORT &= ~(1 << SUCTION_CUP_BIT);
    }
}

// 1开0关
void light_control(uint8_t flag)
{
    if(flag-48){
        LIGHT_PORT |= (1 << LIGHT_BIT);
    }else{
        LIGHT_PORT &= ~(1 << LIGHT_BIT);
    }
}

// 1开0关
void spray_control(uint8_t flag)
{
    if(flag-48){
        SPRAY_PORT |= (1 << SPRAY_BIT);
    }else{
        SPRAY_PORT &= ~(1 << SPRAY_BIT);
    }
}

// 1开0关
void l_water_control(uint8_t flag)
{
    if(flag-48){
        L_WATER_PORT |= (1 << L_WATER_BIT);
    }else{
        L_WATER_PORT &= ~(1 << L_WATER_BIT);
    }
}

// 1开0关
void r_water_control(uint8_t flag)
{
    if(flag-48){
        R_WATER_PORT |= (1 << R_WATER_BIT);
    }else{
        R_WATER_PORT &= ~(1 << R_WATER_BIT);
    }
}

// 1开0关
void outline_control(uint8_t flag)
{
    if(flag-48){
        OUTLINE_PORT |= (1 << OUTLINE_BIT);
    }else{
        OUTLINE_PORT &= ~(1 << OUTLINE_BIT);
    }
}
// 1开0关
void camera_control(uint8_t flag)
{
    if(flag-48){
        CAMERA_PORT |= (1 << CAMERA_BIT);
    }else{
        CAMERA_PORT &= ~(1 << CAMERA_BIT);
    }
}