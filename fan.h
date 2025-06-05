#ifndef fan_h
#define fan_h

#define AIR_FAN_DDR DDRG
#define AIR_FAN_PORT PORTG
#define AIR_FAN_BIT 2 // MEGA2560 Digital Pin 39

void fan_init();
void air_fan_control(uint8_t flag);

#endif
