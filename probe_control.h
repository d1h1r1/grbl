#ifndef probe_control_h
#define probe_control_h

#include "grbl.h"

void probe_control_init();
void close_all_relay();
void open_all_relay();
void up_relay(uint8_t flag);
void down_relay(uint8_t flag);
void set_probe(uint8_t flag);

#endif
