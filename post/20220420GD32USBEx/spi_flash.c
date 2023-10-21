#include "spi_flash.h"
#include "delay.h"
#include "usart1.h"
#include "can.h"

#define SST_HOLD_HIGH GPIO_SetBits(SST_HOLD_GPIOx, SST_HOLD_PIN)
#define SST_HOLD_LOW GPIO_ResetBits(SST_HOLD_GPIOx, SST_HOLD_PIN)
#define SST_WP_HIGH GPIO_SetBits(SST_WP_GPIOx, SST_WP_PIN)
#define SST_WP_LOW GPIO_ResetBits(SST_WP_GPIOx, SST_WP_PIN)

#define SST_CE_DISABLE GPIO_SetBits(SST_CE_GPIOx, SST_CE_PIN)  //失能
#define SST_CE_ENABLE GPIO_ResetBits(SST_CE_GPIOx, SST_CE_PIN) //使能

#define SST_SCLK_HIGH GPIO_SetBits(SST_SCLK_GPIOx, SST_SCLK_PIN)
#define SST_SCLK_LOW GPIO_ResetBits(SST_SCLK_GPIOx, SST_SCLK_PIN)

#define SST_MOSI_HIGH GPIO_SetBits(SST_MOSI_GPIOx, SST_MOSI_PIN)
#define SST_MOSI_LOW GPIO_ResetBits(SST_MOSI_GPIOx, SST_MOSI_PIN)

#define SST_MISO_READ GPIO_ReadInputDataBit(SST_MISO_GPIOx, SST_MISO_PIN)

/************************************************
函数名称 ： SPI_FLASH_config
功    能 ： Flash引脚配置
参    数 ： 无
返 回 值 ： 无
*************************************************/
void SPI_FLASH_config(void)
{
  GPIO_InitTypeDef GPIO_initStructure;
  RCC_APB2PeriphClockCmd(SST_GPIO_RCC_CONFIG, ENABLE);

  GPIO_initStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_initStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_initStructure.GPIO_Pin = SST_HOLD_PIN;
  GPIO_Init(SST_HOLD_GPIOx, &GPIO_initStructure);

  GPIO_initStructure.GPIO_Pin = SST_WP_PIN;
  GPIO_Init(SST_WP_GPIOx, &GPIO_initStructure);

  GPIO_initStructure.GPIO_Pin = SST_CE_PIN;
  GPIO_Init(SST_CE_GPIOx, &GPIO_initStructure);

  GPIO_initStructure.GPIO_Pin = SST_SCLK_PIN;
  GPIO_Init(SST_SCLK_GPIOx, &GPIO_initStructure);

  GPIO_initStructure.GPIO_Pin = SST_MOSI_PIN;
  GPIO_Init(SST_MOSI_GPIOx, &GPIO_initStructure);

  GPIO_initStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_initStructure.GPIO_Pin = SST_MISO_PIN;
  GPIO_Init(SST_MISO_GPIOx, &GPIO_initStructure);
}
/************************************************
函数名称 ： SPI_Initconfig
功    能 ： SPI_Flash初始化
参    数 ： 无
返 回 值 ： 无
*************************************************/
void SPI_Initconfig(void)
{
  SPI_FLASH_config(); // GPIO引脚配置
  SST_CE_DISABLE;     //失能
  SST_SCLK_HIGH;      //初始拉高
  SST_MOSI_HIGH;
  SST_HOLD_HIGH;
  SST_WP_HIGH;
}

/************************************************
函数名称 ： SPI_WriteByte
功    能 ： 写一个字节
参    数 ： 无
返 回 值 ： 无
*************************************************/
#if 1
void SPI_WriteByte(uint8_t txdata)
{
  uint8_t cnt;
  //	uint8_t testdata=0;
  for (cnt = 0; cnt < 8; cnt++)
  {
    SST_SCLK_LOW;
    // delay_us (1);
    if ((txdata & 0x80) == 0x80)
      SST_MOSI_HIGH;
    else
      SST_MOSI_LOW;
    txdata <<= 1;
    /*********************************/
    //		testdata<<=1;
    //		if(SST_MISO_READ)
    //		{
    //			testdata |=0x01;
    //		}
    /*********************************/
    delay_us(1); //-------------
    SST_SCLK_HIGH;
    delay_us(1);
  }
  //	return testdata ;
}
#else
void SPI_WriteByte(uint8_t txdata)
{
  uint8_t cnt;
  //	uint8_t testdata=0;
  for (cnt = 0; cnt < 8; cnt++)
  {

    // delay_us (1);
    if ((txdata & 0x80) == 0x80)
      SST_MOSI_HIGH;
    else
      SST_MOSI_LOW;
    txdata <<= 1;
    SST_SCLK_LOW;
    /*********************************/
    //		testdata<<=1;
    //		if(SST_MISO_READ)
    //		{
    //			testdata |=0x01;
    //		}
    /*********************************/
    delay_us(1); //-------------
    SST_SCLK_HIGH;
    delay_us(1);
  }
  //	return testdata ;
}
#endif
/************************************************
函数名称 ： SPI_ReadByte
功    能 ： 读取一个字节
参    数 ： 无
返 回 值 ： 读取的字节
*************************************************/
#if 1
uint8_t SPI_ReadByte(void)
{
  uint8_t cnt;
  uint8_t rxdata = 0;
  for (cnt = 0; cnt < 8; cnt++)
  {
    SST_SCLK_LOW;
    delay_us(1);
    rxdata <<= 1;
    if (SST_MISO_READ)
    {
      rxdata |= 0x01;
    }
    SST_SCLK_HIGH;
    delay_us(1);
  }
  return rxdata;
}
#else
uint8_t SPI_ReadByte(void)
{
  uint8_t cnt;
  uint8_t rxdata = 0;
  SST_SCLK_HIGH;
  delay_us(1);
  for (cnt = 0; cnt < 8; cnt++)
  {
    SST_SCLK_LOW;
    delay_us(1);
    rxdata <<= 1;
    if (SST_MISO_READ)
    {
      rxdata |= 0x01;
    }
    SST_SCLK_HIGH;
    delay_us(1);
  }
  return rxdata;
}
#endif
/************************************************
函数名称 ： SFLASH_ReadDevice_ID
功    能 ： 读取芯片Device_ID
参    数 ： 无
返 回 值 ： ID --- 24位ID号
*************************************************/
uint32_t SFLASH_ReadDevice_ID(void)
{
  uint32_t ID1 = 0, ID2 = 0, ID;
  SST_CE_ENABLE; //使能器件

  SPI_WriteByte(SFLASH_DEVICE_ID); //《SFLASH_DEVICE_ID》指令

  //  SPI_WriteByte(0);                      //读取ID
  //  SPI_WriteByte(0);
  //  SPI_WriteByte(0x01);
  ID1 |= SPI_ReadByte();
  ID2 |= SPI_ReadByte();
  SST_CE_DISABLE; //失能器件
  ID = (ID1 << 8) | ID2;
  return ID;
}
/************************************************
函数名称 ： SFLASH_ReadJEDEC_ID
功    能 ： 读取芯片JEDEC_ID
参    数 ： 无
返 回 值 ： ID --- 24位ID号
*************************************************/
uint32_t SFLASH_ReadJEDEC_ID(void)
{
  uint32_t ID = 0;
  SST_CE_ENABLE; //使能器件

  SPI_WriteByte(SFLASH_JEDEC_ID); //《JEDEC_ID》指令

  ID |= SPI_ReadByte() << 16; //读取ID
  ID |= SPI_ReadByte() << 8;
  ID |= SPI_ReadByte();
  SST_CE_DISABLE; //失能器件
  return ID;
}
/************************************************
函数名称 ： SFLASH_WrStatusRegEnable
功    能 ： SPI_FLASH写状态寄存器使能
参    数 ： 无
返 回 值 ： 无
*************************************************/
// static void SFLASH_WrStatusRegEnable(void)
//{
//   SST_CE_ENABLE;                                 //使能器件
//   SPI_WriteByte(0x50);            //写状态寄存器使能
//   SST_CE_DISABLE; 	//失能器件
delay_us(2); //----------------
//}
/************************************************
函数名称 ： SFLASH_WriteEnable
功    能 ： SPI_FLASH写使能，将WEL置位
参    数 ： 无
返 回 值 ： 无
*************************************************/
static void SFLASH_WriteEnable(void)
{
  SST_CE_ENABLE;                      //使能器件
  SPI_WriteByte(SFLASH_WRITE_ENABLE); //《写使能》指令06
  SST_CE_DISABLE;                     //失能器件
  //	delay_us(2);   //----------------
}
/************************************************
函数名称 ： SFLASH_WriteDisable
功    能 ： SPI_FLASH写禁止,将WEL清零
参    数 ： 无
返 回 值 ： 无
*************************************************/
static void SFLASH_WriteDisable(void)
{
  SST_CE_ENABLE;                       //使能器件
  SPI_WriteByte(SFLASH_WRITE_DISABLE); //《写失能》指令
  SST_CE_DISABLE;                      //失能器件
}
/************************************************
函数名称 ： SFLASH_ReadBlockProtectSR
功    能 ： 读取块保护状态寄存器
参    数 ： 无
返 回 值 ： Byte --- 读取字节
*************************************************/
void SFLASH_ReadBlockProtectSR(uint8_t *pBuff)
{
  uint8_t data_tmp;
  SST_CE_ENABLE;       //使能器件
  SPI_WriteByte(0x72); //《读状态寄存器》指令
  for (data_tmp = 0; data_tmp < 18; data_tmp++)
  {
    *pBuff = SPI_ReadByte(); //读取一个字节
    pBuff++;
  }
  SST_CE_DISABLE; //失能器件
}
/************************************************
函数名称 ： SFLASH_ReadSR
功    能 ： 读取SFLASH状态寄存器
参    数 ： 无
返 回 值 ： Byte --- 读取字节
*************************************************/
uint8_t SFLASH_ReadSR(void)
{
  uint8_t data_tmp;
  SST_CE_ENABLE;                         //使能器件
  SPI_WriteByte(SFLASH_READ_STATUS_REG); //《读状态寄存器》指令
  data_tmp = SPI_ReadByte();             //读取一个字节
  SST_CE_DISABLE;                        //失能器件
  return data_tmp;
}

/************************************************
函数名称 ： SFLASH_ReadSR
功    能 ： 读取SFLASH配置状态寄存器
参    数 ： 无
返 回 值 ： Byte --- 读取字节
*************************************************/
uint8_t SFLASH_ReadConfigRegSR(void)
{
  uint8_t data_tmp;
  SST_CE_ENABLE;             //使能器件
  SPI_WriteByte(0x35);       //《读状态寄存器》指令
  data_tmp = SPI_ReadByte(); //读取一个字节
  SST_CE_DISABLE;            //失能器件
  return data_tmp;
}
/************************************************
函数名称 ： SFLASH_DeleteBlockProtectSR
功    能 ： 取消块保护状态寄存器
参    数 ： 无
返 回 值 ： 无
*************************************************/
void SFLASH_DeleteBlockProtectSR(void)
{
  uint8_t i;
  SFLASH_WriteEnable(); //写状态寄存器使能
  delay_us(2);          //-------------------------
  SST_CE_ENABLE;
  SPI_WriteByte(0x42); //《写状态寄存器》指令
  for (i = 0; i < 18; i++)
  {
    SPI_WriteByte(0x00);
  }               //写入一个字节
                  //	SPI_WriteByte(SR);          //---------------------------
  SST_CE_DISABLE; //失能器件
}
/************************************************
函数名称 ： SFLASH_DeleteAllBlockProtect
功    能 ： 清除所有的块保护状态寄存器
参    数 ： 无
返 回 值 ： 无
*************************************************/
void SFLASH_ClearAllBlockProtect(void) //执行uLBPR(98)
{

  SFLASH_WriteEnable(); //写状态寄存器使能
  delay_us(2);          //-------------------------
  SST_CE_ENABLE;
  SPI_WriteByte(0x98); //《写状态寄存器》指令
                       //	for(i=0;i<18;i++)
                       //  SPI_WriteByte(0);                             //写入一个字节
                       //	SPI_WriteByte(SR);          //---------------------------
  SST_CE_DISABLE;      //失能器件
}
/************************************************
函数名称 ： SFLASH_WriteSR
功    能 ： 写SFLASH状态寄存器
参    数 ： SR --- 写状态寄存器命令
返 回 值 ： 无
*************************************************/
void SFLASH_WriteSR(uint8_t SR, uint8_t CR)
{
  SFLASH_WriteEnable(); //写使能
  delay_us(2);          //-------------------------
  SST_CE_ENABLE;
  SPI_WriteByte(SFLASH_WRITE_STATUS_REG); //《写状态寄存器》指令
  SPI_WriteByte(SR);                      //写入一个字节
  SPI_WriteByte(CR);                      //---------------------------
  SST_CE_DISABLE;                         //失能器件
}

/************************************************
函数名称 ： SFLASH_ReadNByte
功    能 ： 从ReadAddr地址开始连续读取SFLASH的nByte
参    数 ： pBuffer ---- 数据存储区首地址
            ReadAddr --- 要读取SFLASH Flash的首地址地址
            nByte ------ 要读取的字节数(最大65535B = 64K 块)
返 回 值 ： 无
*************************************************/
void SFLASH_ReadNByte(uint8_t *pBuffer, uint32_t ReadAddr, uint16_t nByte)
{
  //	SST_CE_DISABLE;
  //	delay_us (2);
  SST_CE_ENABLE; //使能器件

  SPI_WriteByte(SFLASH_READ_DATA);                   //《读数据》指令
  SPI_WriteByte((uint8_t)((ReadAddr >> 16) & 0xff)); //发送24bit地址
  SPI_WriteByte((uint8_t)((ReadAddr >> 8) & 0xff));
  SPI_WriteByte((uint8_t)(ReadAddr & 0xff));
  while (nByte--) //循环读数
  {
    *pBuffer = SPI_ReadByte();
    pBuffer++;
  }

  SST_CE_DISABLE; //失能器件
  //	delay_us (2);//-------------
}

/************************************************
函数名称 ： SFLASH_FastReadNByte
功    能 ： 从ReadAddr地址开始连续快速读取SFLASH的nByte
参    数 ： pBuffer ---- 数据存储区首地址
            ReadAddr --- 要读取SFLASH Flash的首地址地址
            nByte ------ 要读取的字节数(最大65535B = 64K 块)
返 回 值 ： 无
*************************************************/
void SFLASH_FastReadNByte(uint8_t *pBuffer, uint32_t ReadAddr, uint16_t nByte)
{
  //	SST_CE_DISABLE;
  //	delay_us (2);
  SST_CE_ENABLE; //使能器件

  SPI_WriteByte(SFLASH_FAST_READ);                     //《快读数据》指令
  SPI_WriteByte((uint8_t)(((ReadAddr) >> 16) & 0xff)); //发送24bit地址
  SPI_WriteByte((uint8_t)(((ReadAddr) >> 8) & 0xff));
  SPI_WriteByte((uint8_t)(ReadAddr & 0xff));
  SPI_WriteByte(0xFF); //等待8个时钟

  while (nByte--) //循环读数
  {
    *pBuffer = SPI_ReadByte();
    pBuffer++;
  }

  SST_CE_DISABLE; //失能器件
}
/************************************************
函数名称 ： SFLASH_WaitForNoBusy
功    能 ： 等待不忙
参    数 ： 无
返 回 值 ： 无
*************************************************/
void SFLASH_WaitForNoBusy(void)
{
  u8 FLASH_Status = 0;

  /* 选择 FLASH: CS 低 */
  SST_CE_ENABLE;

  /* 发送 读状态寄存器 命令 */
  SPI_WriteByte(SFLASH_READ_STATUS_REG);

  /* 若FLASH忙碌，则等待 */
  do
  {
    /* 读取FLASH芯片的状态寄存器 */
    FLASH_Status = SPI_ReadByte();
    Sensor_StateCheck();                   //传感器状态轮询
    BJ_Action();                           //报警动作
                                           // printf ("忙状态FLASH_Status=%d\n",FLASH_Status);
  } while ((FLASH_Status & 0x01) == 0x01); /* 正在写入标志 */

  /* 停止信号  FLASH: CS 高 */
  SST_CE_DISABLE;
}

/************************************************
函数名称 ： SFLASH_WritePage
功    能 ： 在SFLASH内写入少于1页(256个字节)的数据
参    数 ： pBuffer ----- 写入数据区首地址
            WriteAddr --- 要写入Flash的地址
            nByte ------- 要写入的字节数(最大1页)
返 回 值 ： 无
*************************************************/
void SFLASH_WritePage(uint8_t *pBuffer, uint32_t WriteAddr, uint16_t nByte)
{
  SFLASH_WriteEnable(); //写使能
  delay_us(1);
  // SFLASH_WaitForNoBusy();   	//等待空闲（等待写入结束）----------------------
  SST_CE_ENABLE; //使能器件

  SPI_WriteByte(SFLASH_WRITE_PAGE);            //《页编程》指令02
  SPI_WriteByte((WriteAddr & 0xFF0000) >> 16); //写地址高位  //((uint8_t)(((WriteAddr)>>16)&0xff));     //发送24bit地址
  SPI_WriteByte((WriteAddr & 0xFF00) >> 8);    //写地址中位
  SPI_WriteByte(WriteAddr & 0xFF);             //写地址低位
                                               //  SST_MOSI_HIGH;     //--------------------------
  while (nByte--)
  {
    SPI_WriteByte(*pBuffer); //写入数据
                             // delay_us(1);
    pBuffer++;
  }
  SST_CE_DISABLE;
  //	delay_us (2);     //--------------------------
  //  SST_MOSI_HIGH;     //--------------------------
  SFLASH_WaitForNoBusy(); //等待空闲（等待写入结束）
}
void SFLASH_WriteoneBytetest(void)
{
  //	uint8_t testdat;
  SFLASH_WriteEnable();   //写使能
  SFLASH_WaitForNoBusy(); //等待空闲（等待写入结束）--------------
  delay_us(1);
  SST_CE_ENABLE;
  SPI_WriteByte(SFLASH_WRITE_PAGE);
  //  printf ("这是SFLASH_WriteoneBytetest函数中的testdat=0x%x\n",testdat);
  SPI_WriteByte(0x00);
  //	printf ("这是SFLASH_WriteoneBytetest函数中的testdat=0x%x\n",testdat);
  SPI_WriteByte(0x20);
  //	printf ("这是SFLASH_WriteoneBytetest函数中的testdat=0x%x\n",testdat);
  SPI_WriteByte(0x00);
  //	printf ("这是SFLASH_WriteoneBytetest函数中的testdat=0x%x\n",testdat);
  SPI_WriteByte(0x3b);
  //	printf ("这是SFLASH_WriteoneBytetest函数中的testdat=0x%x\n",testdat);
  SST_CE_DISABLE;
  SFLASH_WaitForNoBusy(); //等待空闲（等待写入结束）
}
uint8_t SFLASH_ReadoneBytetest(void)
{
  //	uint8_t testdat;
  uint8_t ID = 0;
  SST_CE_ENABLE; //使能器件

  SPI_WriteByte(0x03); //《读字节》指令
                       //	printf ("这是SFLASH_ReadoneBytetest函数中的testdat=0x%x\n",testdat);
  SPI_WriteByte(0x00);
  //	printf ("这是SFLASH_ReadoneBytetest函数中的testdat=0x%x\n",testdat);
  SPI_WriteByte(0x20);
  //	printf ("这是SFLASH_ReadoneBytetest函数中的testdat=0x%x\n",testdat);
  SPI_WriteByte(0x00);
  //	printf ("这是SFLASH_ReadoneBytetest函数中的testdat=0x%x\n",testdat);

  ID |= SPI_ReadByte();
  printf("读出的数据为ID=0X%x\n", ID);
  SST_CE_DISABLE; //失能器件

  return ID;
}
/************************************************
函数名称 ： SFLASH_WriteNoCheck
功    能 ： 无检验写SFLASH
            必须确保所写的地址范围内的数据全部为0xFF,否则在非0xFF处写入的数据将失败!
            具有自动换页功能
            在指定地址开始写入指定长度的数据,但是要确保地址不越界!
参    数 ： pBuffer ----- 写入数据区首地址
            WriteAddr --- 要写入Flash的地址
            nByte ------- 要写入的字节数
返 回 值 ： 无
*************************************************/
void SFLASH_WriteNoCheck(uint8_t *pBuffer, uint32_t WriteAddr, uint16_t nByte)
{
  uint16_t PageRemain = 256 - WriteAddr % 256; //单页剩余可写的字节数

  if (nByte <= PageRemain)
    PageRemain = nByte; //不大于256个字节

  while (1)
  {
    SFLASH_WritePage(pBuffer, WriteAddr, PageRemain);
    if (nByte == PageRemain) //写入结束
      break;
    else //写入未结束
    {
      pBuffer += PageRemain;   //下一页写入数据
      WriteAddr += PageRemain; //下一页写入数据地址
      nByte -= PageRemain;     //待写入字节数递减

      if (nByte > 256)
        PageRemain = 256; //待写入1页(256字节)的数据
      else
        PageRemain = nByte; //待写入少于1页(256字节)的数据
    }
  }
  SFLASH_WaitForNoBusy(); //等待空闲（等待写入结束）
}
/************************************************
函数名称 ： SFLASH_EraseBlock
功    能 ： 擦除块
            擦除块需要一定时间
参    数 ： BlockAddr --- 块地址 0~135
返 回 值 ： 无
*************************************************/
void SFLASH_EraseBlock(uint32_t BlockAddr)
{
  BlockAddr *= 65536;   //块首地址
  SFLASH_WriteEnable(); //写使能
  SFLASH_WaitForNoBusy();
  SST_CE_ENABLE;                                        //使能器件
  SPI_WriteByte(SFLASH_ERASE_BLOCK);                    //《擦除块》指令
  SPI_WriteByte((uint8_t)(((BlockAddr) >> 16) & 0xff)); //擦除地址
  SPI_WriteByte((uint8_t)(((BlockAddr) >> 8) & 0xff));
  SPI_WriteByte((uint8_t)(BlockAddr & 0xff));
  SST_CE_DISABLE;

  SFLASH_WaitForNoBusy(); //等待擦除完成
}

/************************************************
函数名称 ： SFLASH_EraseSector
功    能 ： 擦除扇区
参    数 ： SectorAddr --- 扇区地址 0~511
返 回 值 ： 无
*************************************************/
void SFLASH_EraseSector(uint32_t SectorAddr)
{
  SectorAddr *= 4096;   //扇区首地址
  SFLASH_WriteEnable(); //写使能
  SFLASH_WaitForNoBusy();
  SST_CE_ENABLE;                                         //使能器件
  SPI_WriteByte(SFLASH_ERASE_SECTOR);                    //《擦除扇区》指令
  SPI_WriteByte((uint8_t)(((SectorAddr) >> 16) & 0xff)); //擦除地址
  SPI_WriteByte((uint8_t)(((SectorAddr) >> 8) & 0xff));
  SPI_WriteByte((uint8_t)(SectorAddr & 0xff));
  SST_CE_DISABLE;
  // delay_us (2);//-------------------------
  SFLASH_WaitForNoBusy(); //等待擦除完成
}

/************************************************
函数名称 ： SFLASH_EraseChip
功    能 ： 擦除整个芯片(整片擦除时间较长)
参    数 ： 无
返 回 值 ： 无
*************************************************/
void SFLASH_EraseChip(void)
{
  SFLASH_WriteEnable(); //写使能
  SFLASH_WaitForNoBusy();
  SST_CE_ENABLE;                    //使能器件
  SPI_WriteByte(SFLASH_ERASE_CHIP); //《擦除芯片》指令
  SST_CE_DISABLE;

  SFLASH_WaitForNoBusy(); //等待芯片擦除结束
}

/************************************************
函数名称 ： SFLASH_WriteNByte
功    能 ： 从ReadAddr地址开始连续写入nByte到SFLASH中
参    数 ： pBuffer ----- 写入数据区首地址
            WriteAddr --- 要写入Flash的地址
            nByte ------- 要写入的字节数(最大65535B = 64K 块)
返 回 值 ： 无
*************************************************/
void SFLASH_WriteNByte(uint8_t *pBuffer, uint32_t WriteAddr, uint16_t nByte)
{
  static uint8_t SectorBuf[4096]; //扇区buf
  uint32_t SecPos;                //扇区位置
  uint16_t SecOff;                //扇区偏移
  uint16_t SecRemain;             //剩余扇区
  uint16_t i;
  SFLASH_WriteEnable();      //写使能
  SecPos = WriteAddr / 4096; //地址所在扇区(0~2048)
  SecOff = WriteAddr % 4096; //地址所在扇区的偏移
  SecRemain = 4096 - SecOff; //地址所在扇区剩余字节数(扇区大小4096B=4KB)

  if (nByte <= SecRemain)
    SecRemain = nByte; //写入数据大小 < 剩余空间大小 (即剩余空间够保存这些数据)

  while (1)
  {
    /* 第1步·校验 */
    SFLASH_ReadNByte(SectorBuf, SecPos * 4096, 4096); //读出整个扇区的内容
    for (i = 0; i < SecRemain; i++)                   //校验数据,是否需要擦除
    {
      if (SectorBuf[SecOff + i] != 0xFF) //存储数据不为0xFF 则需要擦除
        break;
    }
    if (i < SecRemain) //需要擦除
    {
      SFLASH_EraseSector(SecPos);     //擦除该扇区
      for (i = 0; i < SecRemain; i++) //保存写入的数据(第1次时，是写入那扇区后面剩余的空间)
      {
        SectorBuf[SecOff + i] = pBuffer[i]; //将要写入的新数据存入SectorBuf
      }
      SFLASH_WriteNoCheck(SectorBuf, SecPos * 4096, 4096); //写入整个扇区（扇区 = 老数据 + 新写入数据）
    }
    else
      SFLASH_WriteNoCheck(pBuffer, WriteAddr, SecRemain); //不需要擦除,直接写入扇区剩余空间

    if (nByte == SecRemain) //写入结束
    {
      SFLASH_WriteDisable(); //写失能, 退出写
      break;
    }
    else //写入未结束
    {
      SecPos++;               //扇区地址增1
      SecOff = 0;             //偏移位置归零
      pBuffer += SecRemain;   //指针偏移
      WriteAddr += SecRemain; //写地址偏移
      nByte -= SecRemain;     //待写入字节数递减
      if (nByte > 4096)
        SecRemain = 4096; //待写入1扇区(4096字节)的数据
      else
        SecRemain = nByte; //待写入少于1扇区(4096字节)的数据
    }
  }
  SFLASH_WaitForNoBusy(); //等待芯片擦除结束
}
/************************************************
函数名称 ： SFLASH_ReleasePowerDown
功    能 ： 释放掉电
参    数 ： 无
返 回 值 ： 无
*************************************************/
void SFLASH_ReleasePowerDown(void)
{
  SST_CE_ENABLE;                            //使能器件
  SPI_WriteByte(SFLASH_RELEASE_POWER_DOWN); //《释放掉电》指令
  SST_CE_DISABLE;                           //失能器件
}
