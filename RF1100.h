#ifndef __RF1100_H
#define __RF1100_H
#include "include.h"
/*******************************************************************************
							SPI接口定义
MOSI------PD5(输出)
SCK-------PD6(输出)
MISO------PA4(输入)

GDO2------PC1(空)
GDO0------PC0(输入)
CS------PD7(输出)
*******************************************************************************/


#define CSN_L  GPIOD->ODR &= (uint8_t)(~GPIO_Pin_7);
#define CSN_H  GPIOD->ODR |= GPIO_Pin_7;

#define GDO0_L     !GDO0_H                                                          //低位指示 
#define GDO0_H    (( GPIOC->IDR &GPIO_Pin_0)==0)?0:1              //高位指示

#define MISO GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_4)


#define FIFO_THRESHOLD          0x03
#define RXFIFO_THRESHOLD_BYTES 16
#define TXFIFO_THRESHOLD_BYTES 49
#define FIFO_SIZE               64

enum MODEFLAGESELECT  {TXMODE ,RXMODE};
extern enum MODEFLAGESELECT modeflage;
extern u8 startraansfer;
void SPI_Config(void);

void Init_cc1100(void)  ;
void WriteRfSettings(void);
unsigned char SpiTxRx_Byte(unsigned char data) ;
void POWER_UP_RESET_CC1100(void)  ;
void RESET_CC1100(void)  ;
unsigned char Spi_Read_Byte(unsigned char addr) ;
void Spi_Read_Burst (unsigned char addr,unsigned char *buffer,unsigned char count)  ;
void Spi_Write_Burst(unsigned char addr,unsigned char *buffer,unsigned char count)  ;
u8 Spi_Write_Packet(unsigned char *Tx_buffer,unsigned char size,unsigned char  id)   ;
//unsigned char Spi_Read_Packet(unsigned char *Rx_buffer,unsigned char length) ;
//unsigned char Spi_Read_Packet(struct commstruct * thiscomm) ;
unsigned char Spi_Read_Packet(void);
void Spi_Write_Strobe(unsigned char strobe)  ;
void Rx_cc1100(void) ;
extern unsigned char PaTabel[];
extern unsigned char Rx_data[];
extern u8 QIEHUAN;

/*模块操作指令*****************************************************************/
#define    CRC_OK                 0x80      //校验标志*************************/ 

#define    Read_Byte              0x80      //读单字节*************************/ 
#define    Read_Burst             0xC0      //连续读取*************************/ 

#define    Write_Byte             0x00      //写单字节*************************/ 
#define    Write_Burst            0x40      //连续写入*************************/ 

/*配置寄存器定义***************************************************************/
#define    CC_IOCFG2              0x00      // GDO2输出脚配置******************/ 
#define    CC_IOCFG1              0x01      // GDO1输出脚配置******************/ 
#define    CC_IOCFG0              0x02      // GDO0输出脚配置******************/ 
#define    CC_FIFOTHR             0x03      // RX FIFO和TX FIFO门限************/ 
#define    CC_SYNC1               0x04      // 同步词汇，高字节****************/ 
#define    CC_SYNC0               0x05      // 同步词汇，低字节****************/ 
#define    CC_PKTLEN              0x06      // 数据包长度**********************/ 
#define    CC_PKTCTRL1            0x07      // 数据包自动控制******************/ //Address check and 0 (0x00) and 255 (0xFF)broadcast
#define    CC_PKTCTRL0            0x08      // 数据包自动控制******************/ 
#define    CC_ADDR                0x09      // 设备地址/***********************/ 
#define    CC_CHANNR              0x0A      // 信道数**************************/ 
#define    CC_FSCTRL1             0x0B      // 频率合成器控制******************/ 
#define    CC_FSCTRL0             0x0C      // 频率合成器控制******************/ 
#define    CC_FREQ2               0x0D      // 频率控制词汇，高字节************/ 
#define    CC_FREQ1               0x0E      // 频率控制词汇，中间字节**********/ 
#define    CC_FREQ0               0x0F      // 频率控制词汇，低字节************/ 
#define    CC_MDMCFG4             0x10      // 调制器配置**********************/ 
#define    CC_MDMCFG3             0x11      // 调制器配置**********************/ 
#define    CC_MDMCFG2             0x12      // 调制器配置**********************/ 
#define    CC_MDMCFG1             0x13      // 调制器配置**********************/ 
#define    CC_MDMCFG0             0x14      // 调制器配置**********************/ 
#define    CC_DEVIATN             0x15      // 调制器背离设置 *****************/ 
#define    CC_MCSM2               0x16      // 主通信控制状态机配置************/ 
#define    CC_MCSM1               0x17      // 主通信控制状态机配置************/ 
#define    CC_MCSM0               0x18      // 主通信控制状态机配置************/ 
#define    CC_FOCCFG              0x19      // 频率偏移补偿配置****************/ 
#define    CC_BSCFG               0x1A      // 位同步配置**********************/ 
#define    CC_AGCCTRL2            0x1B      // AGC控制*************************/ 
#define    CC_AGCCTRL1            0x1C      // AGC控制*************************/ 
#define    CC_AGCCTRL0            0x1D      // AGC控制*************************/ 
#define    CC_WOREVT1             0x1E      // 高字节时间0暂停*****************/ 
#define    CC_WOREVT0             0x1F      // 低字节时间0暂停*****************/ 
#define    CC_WORCTRL             0x20      // 电磁波激活控制******************/ 
#define    CC_FREND1              0x21      // 前末端RX配置********************/ 
#define    CC_FREND0              0x22      // 前末端RX配置********************/ 
#define    CC_FSCAL3              0x23      // 频率合成器校准******************/ 
#define    CC_FSCAL2              0x24      // 频率合成器校准******************/ 
#define    CC_FSCAL1              0x25      // 频率合成器校准******************/ 
#define    CC_FSCAL0              0x26      // 频率合成器校准******************/ 
#define    CC_RCCTRL1             0x27      // RC振荡器配置********************/ 
#define    CC_RCCTRL0             0x28      // RC振荡器配置********************/ 
#define    CC_FSTEST              0x29      // 频率合成器标度 *****************/ 
#define    CC_PTEST               0x2A      // 产品测试************************/ 
#define    CC_AGCTEST             0x2B      // AGC测试*************************/ 
#define    CC_TEST2               0x2C      // 不同的测试设置******************/ 
#define    CC_TEST1               0x2D      // 不同的测试设置******************/ 
#define    CC_TEST0               0x2E      // 不同的测试设置******************/ 

/*命令滤波定义*****************************************************************/
#define    CC_SRES                0x30      // 重启芯片************************/ 
#define    CC_SFSTXON             0x31      // 开启和校准频率合成器************/ 
#define    CC_SXOFF               0x32      // 关闭晶体振荡器******************/ 
#define    CC_SCAL                0x33      // 校准频率合成器并关断************/ 
#define    CC_SRX                 0x34      // 启用RX。************************/ 
#define    CC_STX                 0x35      // 空闲状态：启用TX。**************/ 
#define    CC_SIDLE               0x36      // 离开RX/TX***********************/ 
#define    CC_SAFC                0x37      // 频率合成器的AFC调节*************/ 
#define    CC_SWOR                0x38      // 自动RX选举序列（电磁波激活）****/ 
#define    CC_SPWD                0x39      // 当CSn为高时进入功率降低模式*****/ 
#define    CC_SFRX                0x3A      // 冲洗RX FIFO缓冲*****************/ 
#define    CC_SFTX                0x3B      // 冲洗TX FIFO缓冲*****************/ 
#define    CC_SWORRST             0x3C      // 重新设置真实时间时钟************/ 
#define    CC_SNOP                0x3D      // 无操作**************************/ 

/*状态寄存定义*****************************************************************/
#define    CC_PARTNUM             0x30      // CC2550的组成部分数目************/ 
#define    CC_VERSION             0x31      // 当前版本数**********************/ 
#define    CC_FREQEST             0x32      // 率偏移估计**********************/ 
#define    CC_LQI                 0x33      // 接质量的解调器估计**************/ 
#define    CC_RSSI                0x34      // 接收信号强度指示****************/ 
#define    CC_MARCSTATE           0x35      // 控制状态机状态******************/ 
#define    CC_WORTIME1            0x36      // WOR计时器高字节*****************/ 
#define    CC_WORTIME0            0x37      // WOR计时器低字节*****************/ 
#define    CC_PKTSTATUS           0x38      // 当前GDOx状态和数据包状态********/ 
#define    CC_VCO_VC_DAC          0x39      // PLL校准模块的当前设定***********/ 
#define    CC_TXBYTES             0x3A      // TX FIFO中的下溢和比特数*********/ 
#define    CC_RXBYTES             0x3B      // RX FIFO中的下溢和比特数*********/ 
#define    CC_PATABLE             0x3E      // ********************************/ 
#define    CC_TXFIFO              0x3F      // ********************************/ 
#define    CC_RXFIFO              0x3F      // ********************************/ 

/*寄存器配器表 仿真软件给出****************************************************/
#define    IOCFG2                 0x06      // GDO2输出脚配置******************/ 
#define    IOCFG0                 0x06      // GDO0输出脚配置******************/ 
#define    FIFOTHR                0x07      // RX FIFO和TX FIFO门限************/ 
#define    PKTLEN                 0xFF     // 数据包长度**********************/ 
#define    PKTCTRL1               0x05      // 数据包自动控制******************/ 
#define    PKTCTRL0               0x05      // 数据包自动控制******************/ 
#define    DEVICEADDR           0x03      // 设备地址************************/ 
#define    CHANNR                 0x00      // 频道数**************************/  
#define    FSCTRL1                0x06  //08  //08  //0B      // 频率合成器控制******************/ 
#define    FSCTRL0                0x00      // 频率合成器控制******************/ 
#define    FREQ2                  0x10      // 频率控制词汇，高字节************/ 
#define    FREQ1                  0xA7      // 频率控制词汇，中间字节**********/ 
#define    FREQ0                  0x62      // 频率控制词汇，低字节************/ 
#define    MDMCFG4             0xca //0xCC//  0xC7  //CA  //7B  //5B      // 调制器配置**********************/ 
#define    MDMCFG3              0x83 //0x22 // 0x83  //83  //83  //F8      // 调制器配置**********************/ 
#define    MDMCFG2                0x03  //83  //03  //0A      // 调制器配置**********************/ 
#define    MDMCFG1                0x22      // 调制器配置**********************/ 
#define    MDMCFG0                0xF8      // 调制器配置**********************/ 
#define    DEVIATN                0x40  //34  //42  //47      // 调制器背离设置******************/ 
#define    MCSM1                  0x3F      // 主通信控制状态机配置************/ 
#define    MCSM0                  0x18  //38     // 主通信控制状态机配置************/ 
#define    FOCCFG                 0x16  //1D     // 频率偏移补偿配置****************/ 
#define    BSCFG                  0x6C  //1C     // 位同步配置**********************/ 
#define    AGCCTRL2               0x43  //C7     // AGC控制*************************/ 
#define    AGCCTRL1               0x40  //00     // AGC控制*************************/ 
#define    AGCCTRL0               0x91  //B2     // AGC控制*************************/ 
#define    FREND1                 0x56      // 前末端RX配置********************/ 
#define    FREND0                 0x10      // 前末端TX配置********************/ 
#define    FSCAL3                 0xE9      // 频率合成器校准******************/ 
#define    FSCAL2                 0x2A      // 频率合成器校准******************/ 
#define    FSCAL1                 0x00      // 频率合成器校准******************/ 
#define    FSCAL0                 0x1F      // 频率合成器校准******************/ 
#define    FSTEST                 0x59      // 频率合成器标度******************/   
#define    TEST2                  0x88      // 不同的测试设置******************/ 
#define    TEST1                  0x31      // 不同的测试设置******************/ 
#define    TEST0                  0x0B      // 不同的测试设置******************/ 
/******************************************************************************/
#endif
