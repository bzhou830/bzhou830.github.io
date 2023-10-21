#ifndef _SPI_FLASH_H_
#define _SPI_FLASH_H_

#include "stm32f10x.h"

#define     SST_GPIO_RCC_CONFIG       RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC       //flash芯片时钟

#define     SST_HOLD_GPIOx      GPIOA              //HOLD引脚
#define     SST_HOLD_PIN        GPIO_Pin_7

#define     SST_WP_GPIOx        GPIOA              //WP引脚
#define     SST_WP_PIN          GPIO_Pin_4

#define     SST_CE_GPIOx        GPIOA              //片选引脚
#define     SST_CE_PIN          GPIO_Pin_6

#define     SST_SCLK_GPIOx      GPIOC              //时钟
#define     SST_SCLK_PIN        GPIO_Pin_4

#define     SST_MOSI_GPIOx      GPIOC              //MOSI
#define     SST_MOSI_PIN        GPIO_Pin_5

#define     SST_MISO_GPIOx      GPIOA			   //配置成输入模式 MISO   
#define     SST_MISO_PIN        GPIO_Pin_5

/* 指令表 */
#define SFLASH_WRITE_ENABLE       0x06                     //写使能
#define SFLASH_WRITE_DISABLE      0x04                     //写失能
#define SFLASH_READ_STATUS_REG    0x05                     //读状态寄存器
#define SFLASH_WRITE_STATUS_REG   0x01                     //写状态寄存器

#define SFLASH_READ_DATA          0x03                     //读数据
#define SFLASH_FAST_READ          0x0B                     //快读数据
#define SFLASH_FAST_READ_DUAL     0x3B                     //快读数据(双数据线输出)
#define SFLASH_WRITE_PAGE         0x02                     //页编程
#define SFLASH_ERASE_BLOCK        0xD8                     //擦除块
#define SFLASH_ERASE_SECTOR       0x20                     //擦除扇区
#define SFLASH_ERASE_CHIP         0xC7                     //擦除芯片
#define SFLASH_POWER_DOWN         0xB9                     //掉电
#define SFLASH_RELEASE_POWER_DOWN 0xAB                     //释放掉电
#define SFLASH_DEVICE_ID          0x90                     //设备ID
#define SFLASH_JEDEC_ID           0x9F                     //Jedec ID

extern void Sensor_StateCheck(void);  //传感器状态轮询
extern void BJ_Action(void);          //报警动作

void SPI_FLASH_config(void);    //spi的GPIO引脚初始化
void SPI_Initconfig(void);   //SPI初始化
uint32_t SFLASH_ReadJEDEC_ID(void);     //读取JEDEC-ID
void SPI_WriteByte(uint8_t txdata);
uint8_t SPI_ReadByte(void);
uint8_t SFLASH_ReadSR(void);     //读取SFLASH状态寄存器
void SFLASH_WriteSR(uint8_t SR,uint8_t CR);    //写SFLASH状态寄存器
void SFLASH_ReadBlockProtectSR(uint8_t *pBuff);   //读取块保护
void SFLASH_DeleteBlockProtectSR(void);    //取消块保护状态寄存器
void SFLASH_ClearAllBlockProtect(void);    //清除所有的块保护状态寄存器
uint8_t SFLASH_ReadConfigRegSR(void);      //读取SFLASH配置状态寄存器
/****从ReadAddr地址开始连续读取SFLASH的nByte*********************************/
void SFLASH_ReadNByte(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t nByte);

/*****从ReadAddr地址开始连续快速读取SFLASH的nByte
******最多读取65535B***************************/
void SFLASH_FastReadNByte(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t nByte);

void SFLASH_WaitForNoBusy(void);    // 等待不忙 

/*****在SFLASH内写入少于1页(256个字节)的数据************************************/
void SFLASH_WritePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t nByte);

/*********无检验写SFLASH
*必须确保所写的地址范围内的数据全部为0xFF,否则在非0xFF处写入的数据将失败!
*具有自动换页功能**********/
void SFLASH_WriteNoCheck(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t nByte);

void SFLASH_EraseBlock(uint32_t BlockAddr);   //擦除块
void SFLASH_EraseSector(uint32_t SectorAddr);  //擦除扇区
void SFLASH_EraseChip(void);  //擦除整个芯片(整片擦除时间较长)

/**从ReadAddr地址开始连续写入nByte到SFLASH中
***pBuffer ----- 写入数据区首地址
*** WriteAddr --- 要写入Flash的地址
**** nByte ------- 要写入的字节数(最大65535B = 64K 块)**********************/
void SFLASH_WriteNByte(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t nByte);

uint32_t SFLASH_ReadDevice_ID(void);   //读取设备ID
void SFLASH_WriteoneBytetest(void);     //写一个字节，用于测试
uint8_t SFLASH_ReadoneBytetest(void);	   //读一个字节，用于测试
void SFLASH_ReleasePowerDown(void);     //释放掉电
#endif     //_SPI_FLASH_H
