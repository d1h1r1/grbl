#ifndef tool_change_h
#define tool_change_h

#include "grbl.h"

void tool_loose();        // 松刀
void tool_tight();        // 紧刀
void tool_home(uint8_t flag);
void change_tool(uint8_t tool_number);
void return_tool();
void get_tool(uint8_t tool_number);

#endif
