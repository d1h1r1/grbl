#ifndef tool_change_h
#define tool_change_h

// void tool_loose();        // 松刀
// void tool_tight();        // 紧刀
void change_tool(uint8_t tool_number);
void return_tool();
void get_tool(uint8_t tool_number);
void tool_control_init();
void tool_length_zero();
void getToolStatus();
#endif
