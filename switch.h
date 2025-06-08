#ifndef switch_h
#define switch_h

#define AIR_FAN_DDR DDRG
#define AIR_FAN_PORT PORTG
#define AIR_FAN_BIT 2 // MEGA2560 Digital Pin 39

#define SPINDLE_L_FAN_DDR DDRF
#define SPINDLE_L_FAN_PORT PORTF
#define SPINDLE_L_FAN_BIT 1 // MEGA2560 Digital Pin A1

#define SPINDLE_R_FAN_DDR DDRF
#define SPINDLE_R_FAN_PORT PORTF
#define SPINDLE_R_FAN_BIT 2 // MEGA2560 Digital Pin A2

#define BLOW_FAN_DDR DDRF
#define BLOW_FAN_PORT PORTF
#define BLOW_FAN_BIT 7 // MEGA2560 Digital Pin A7

#define SUCTION_CUP_DDR DDRG
#define SUCTION_CUP_PORT PORTG
#define SUCTION_CUP_BIT 5 // MEGA2560 Digital Pin 4

#define LIGHT_DDR DDRE
#define LIGHT_PORT PORTE
#define LIGHT_BIT 3 // MEGA2560 Digital Pin 5

#define SPRAY_DDR DDRH
#define SPRAY_PORT PORTH
#define SPRAY_BIT 3 // MEGA2560 Digital Pin 6

#define L_WATER_DDR DDRH
#define L_WATER_PORT PORTH
#define L_WATER_BIT 5 // MEGA2560 Digital Pin 8

#define R_WATER_DDR DDRH
#define R_WATER_PORT PORTH
#define R_WATER_BIT 6 // MEGA2560 Digital Pin 9

#define OUTLINE_DDR DDRA
#define OUTLINE_PORT PORTA
#define OUTLINE_BIT 0 // MEGA2560 Digital Pin 22

#define CAMERA_DDR DDRF
#define CAMERA_PORT PORTF
#define CAMERA_BIT 3 // MEGA2560 Digital Pin 22

#define RFID_ELE_DDR DDRD
#define RFID_ELE_PORT PORTD
#define RFID_ELE_BIT 7 // MEGA2560 Digital Pin 38

void switch_init();
void air_fan_control(uint8_t flag);
void spindle_l_fan_control(uint8_t flag);
void spindle_r_fan_control(uint8_t flag);
void blow_fan_control(uint8_t flag);
void suction_cup_control(uint8_t flag);
void light_control(uint8_t flag);
void spray_control(uint8_t flag);
void l_water_control(uint8_t flag);
void r_water_control(uint8_t flag);
void outline_control(uint8_t flag);
void camera_control(uint8_t flag);
void rfid_ele_control(uint8_t flag);


#endif
