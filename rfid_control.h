#ifndef rfid_control_h
#define rfid_control_h

#define RFID_LIMIT_DDR DDRK
#define RFID_LIMIT_PORTD PORTK
#define RFID_LIMIT_PIN PINK
#define RFID_LIMIT_BIT 1

void rfid_control_init();
void set_rfid(uint8_t flag);
void rfid_read(uint8_t* return_data);

#endif
