/* 
  该文件已为Doxygen自动文档生成做好准备。
  /*! \file ********************************************************************
  *
  * Atmel Corporation
  *
  * \li 文件:               eeprom.c
  * \li 编译器:           IAR EWAAVR 3.10c
  * \li 支持邮件:       avr@atmel.com
  *
  * \li 支持设备:  所有具有分离EEPROM擦除/写入功能的设备均可使用。
  *                 示例是为ATmega48编写的。
  *
  * \li 应用说明:            AVR103 - 使用EEPROM编程模式。
  *
  * \li 描述:        示例如何使用例如ATmega48中的分离EEPROM擦除/写入功能。
  *                 所有EEPROM编程模式都已测试，即擦除+写入、
  *                 仅擦除和仅写入。
  *
  *                 $修订版: 1.6 $
  *                 $日期: 2005年2月11日星期五07:16:44 UTC $
  ****************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>

/* 这些EEPROM位在不同设备上的名称不同。 */
#ifndef EEPE
    #define EEPE  EEWE  //!< EEPROM编程/写入使能。
    #define EEMPE EEMWE //!< EEPROM主程序/写入使能。
#endif

/* 这两个在设备包含文件中不幸未定义。 */
#define EEPM1 5 //!< EEPROM编程模式位1。
#define EEPM0 4 //!< EEPROM编程模式位0。

/* 定义以减少代码大小。 */
#define EEPROM_IGNORE_SELFPROG //!< 移除SPM标志轮询。

/*! \brief  从EEPROM中读取字节。
 *
 *  此函数从给定的EEPROM地址读取一个字节。
 *
 *  \note  在EEPROM读取期间，CPU暂停4个时钟周期。
 *
 *  \param  addr  要读取的EEPROM地址。
 *  \return  从EEPROM地址读取的字节。
 */
unsigned char eeprom_get_char(unsigned int addr)
{
    do {} while(EECR & (1<<EEPE)); // 等待先前写入完成。
    EEAR = addr; // 设置EEPROM地址寄存器。
    EECR = (1<<EERE); // 启动EEPROM读取操作。
    return EEDR; // 返回从EEPROM读取的字节。
}

/*! \brief  写入字节到EEPROM。
 *
 *  此函数将一个字节写入给定的EEPROM地址。
 *  通过比较现有字节与新值之间的差异选择最有效的EEPROM编程模式。
 *
 *  \note  在EEPROM编程期间，CPU暂停2个时钟周期。
 *
 *  \note  当此函数返回时，新的EEPROM值在EEPROM编程时间过去之前不可用。
 *         应轮询EECR中的EEPE位以检查编程是否完成。
 *
 *  \note  EEPROM_GetChar()函数会自动检查EEPE位。
 *
 *  \param  addr  要写入的EEPROM地址。
 *  \param  new_value  新的EEPROM值。
 */
void eeprom_put_char(unsigned int addr, unsigned char new_value)
{
    char old_value; // 旧的EEPROM值。
    char diff_mask; // 差异掩码，即旧值与新值的异或。

    cli(); // 确保写入操作的原子性。

    do {} while(EECR & (1<<EEPE)); // 等待先前写入完成。
    #ifndef EEPROM_IGNORE_SELFPROG
    do {} while(SPMCSR & (1<<SELFPRGEN)); // 等待SPM完成。
    #endif

    EEAR = addr; // 设置EEPROM地址寄存器。
    EECR = (1<<EERE); // 启动EEPROM读取操作。
    old_value = EEDR; // 获取旧的EEPROM值。
    diff_mask = old_value ^ new_value; // 获取位差异。

    // 检查新值中是否有任何位被更改为'1'。
    if(diff_mask & new_value) {
        // 现在我们知道需要擦除一些位为'1'。

        // 检查新值中是否有位为'0'。
        if(new_value != 0xff) {
            // 现在我们知道也需要将一些位编程为'0'。

            EEDR = new_value; // 设置EEPROM数据寄存器。
            EECR = (1<<EEMPE) | // 设置主写入使能位...
                   (0<<EEPM1) | (0<<EEPM0); // ...和擦除+写入模式。
            EECR |= (1<<EEPE);  // 启动擦除+写入操作。
        } else {
            // 现在我们知道所有位都应被擦除。

            EECR = (1<<EEMPE) | // 设置主写入使能位...
                   (1<<EEPM0);  // ...和仅擦除模式。
            EECR |= (1<<EEPE);  // 启动仅擦除操作。
        }
    } else {
        // 现在我们知道没有位需要擦除为'1'。

        // 检查旧值中是否有任何位从'1'更改。
        if(diff_mask) {
            // 现在我们知道需要将一些位编程为'0'。

            EEDR = new_value;   // 设置EEPROM数据寄存器。
            EECR = (1<<EEMPE) | // 设置主写入使能位...
                   (1<<EEPM1);  // ...和仅写入模式。
            EECR |= (1<<EEPE);  // 启动仅写入操作。
        }
    }

    sei(); // 恢复中断标志状态。
}

// Grbl扩展添加的部分

void memcpy_to_eeprom_with_checksum(unsigned int destination, char *source, unsigned int size) {
    unsigned char checksum = 0;
    for(; size > 0; size--) { 
        checksum = (checksum << 1) || (checksum >> 7);
        checksum += *source;
        eeprom_put_char(destination++, *(source++)); 
    }
    eeprom_put_char(destination, checksum);
}

int memcpy_from_eeprom_with_checksum(char *destination, unsigned int source, unsigned int size) {
    unsigned char data, checksum = 0;
    for(; size > 0; size--) { 
        data = eeprom_get_char(source++);
        checksum = (checksum << 1) || (checksum >> 7);
        checksum += data;    
        *(destination++) = data; 
    }
    return(checksum == eeprom_get_char(source));
}

// 文件结束
