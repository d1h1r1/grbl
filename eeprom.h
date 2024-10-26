/*
  eeprom.h - EEPROM方法
  Grbl的一部分

  版权所有 (c) 2009-2011 Simen Svale Skogsrud

  Grbl是一个自由软件：您可以根据自由软件基金会发布的GNU通用公共许可证的条款，重新发布和/或修改
  该软件，版本3或（根据您的选择）任何更新的版本。

  Grbl的发布是希望它能有用，
  但没有任何保证；甚至不含有
  适销性或特定用途的隐含保证。有关更多详细信息，请参见GNU通用公共许可证。

  您应该已随Grbl一起收到GNU通用公共许可证的副本。如果没有，请参见 <http://www.gnu.org/licenses/>。
*/


#ifndef eeprom_h
#define eeprom_h

unsigned char eeprom_get_char(unsigned int addr);
void eeprom_put_char(unsigned int addr, unsigned char new_value);
void memcpy_to_eeprom_with_checksum(unsigned int destination, char *source, unsigned int size);
int memcpy_from_eeprom_with_checksum(char *destination, unsigned int source, unsigned int size);

#endif
