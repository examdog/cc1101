#ifndef __RF1100_H
#define __RF1100_H
#include "include.h"
/*******************************************************************************
							SPI�ӿڶ���
MOSI------PD5(���)
SCK-------PD6(���)
MISO------PA4(����)

GDO2------PC1(��)
GDO0------PC0(����)
CS------PD7(���)
*******************************************************************************/


#define CSN_L  GPIOD->ODR &= (uint8_t)(~GPIO_Pin_7);
#define CSN_H  GPIOD->ODR |= GPIO_Pin_7;

#define GDO0_L     !GDO0_H                                                          //��λָʾ 
#define GDO0_H    (( GPIOC->IDR &GPIO_Pin_0)==0)?0:1              //��λָʾ

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

/*ģ�����ָ��*****************************************************************/
#define    CRC_OK                 0x80      //У���־*************************/ 

#define    Read_Byte              0x80      //�����ֽ�*************************/ 
#define    Read_Burst             0xC0      //������ȡ*************************/ 

#define    Write_Byte             0x00      //д���ֽ�*************************/ 
#define    Write_Burst            0x40      //����д��*************************/ 

/*���üĴ�������***************************************************************/
#define    CC_IOCFG2              0x00      // GDO2���������******************/ 
#define    CC_IOCFG1              0x01      // GDO1���������******************/ 
#define    CC_IOCFG0              0x02      // GDO0���������******************/ 
#define    CC_FIFOTHR             0x03      // RX FIFO��TX FIFO����************/ 
#define    CC_SYNC1               0x04      // ͬ���ʻ㣬���ֽ�****************/ 
#define    CC_SYNC0               0x05      // ͬ���ʻ㣬���ֽ�****************/ 
#define    CC_PKTLEN              0x06      // ���ݰ�����**********************/ 
#define    CC_PKTCTRL1            0x07      // ���ݰ��Զ�����******************/ //Address check and 0 (0x00) and 255 (0xFF)broadcast
#define    CC_PKTCTRL0            0x08      // ���ݰ��Զ�����******************/ 
#define    CC_ADDR                0x09      // �豸��ַ/***********************/ 
#define    CC_CHANNR              0x0A      // �ŵ���**************************/ 
#define    CC_FSCTRL1             0x0B      // Ƶ�ʺϳ�������******************/ 
#define    CC_FSCTRL0             0x0C      // Ƶ�ʺϳ�������******************/ 
#define    CC_FREQ2               0x0D      // Ƶ�ʿ��ƴʻ㣬���ֽ�************/ 
#define    CC_FREQ1               0x0E      // Ƶ�ʿ��ƴʻ㣬�м��ֽ�**********/ 
#define    CC_FREQ0               0x0F      // Ƶ�ʿ��ƴʻ㣬���ֽ�************/ 
#define    CC_MDMCFG4             0x10      // ����������**********************/ 
#define    CC_MDMCFG3             0x11      // ����������**********************/ 
#define    CC_MDMCFG2             0x12      // ����������**********************/ 
#define    CC_MDMCFG1             0x13      // ����������**********************/ 
#define    CC_MDMCFG0             0x14      // ����������**********************/ 
#define    CC_DEVIATN             0x15      // �������������� *****************/ 
#define    CC_MCSM2               0x16      // ��ͨ�ſ���״̬������************/ 
#define    CC_MCSM1               0x17      // ��ͨ�ſ���״̬������************/ 
#define    CC_MCSM0               0x18      // ��ͨ�ſ���״̬������************/ 
#define    CC_FOCCFG              0x19      // Ƶ��ƫ�Ʋ�������****************/ 
#define    CC_BSCFG               0x1A      // λͬ������**********************/ 
#define    CC_AGCCTRL2            0x1B      // AGC����*************************/ 
#define    CC_AGCCTRL1            0x1C      // AGC����*************************/ 
#define    CC_AGCCTRL0            0x1D      // AGC����*************************/ 
#define    CC_WOREVT1             0x1E      // ���ֽ�ʱ��0��ͣ*****************/ 
#define    CC_WOREVT0             0x1F      // ���ֽ�ʱ��0��ͣ*****************/ 
#define    CC_WORCTRL             0x20      // ��Ų��������******************/ 
#define    CC_FREND1              0x21      // ǰĩ��RX����********************/ 
#define    CC_FREND0              0x22      // ǰĩ��RX����********************/ 
#define    CC_FSCAL3              0x23      // Ƶ�ʺϳ���У׼******************/ 
#define    CC_FSCAL2              0x24      // Ƶ�ʺϳ���У׼******************/ 
#define    CC_FSCAL1              0x25      // Ƶ�ʺϳ���У׼******************/ 
#define    CC_FSCAL0              0x26      // Ƶ�ʺϳ���У׼******************/ 
#define    CC_RCCTRL1             0x27      // RC��������********************/ 
#define    CC_RCCTRL0             0x28      // RC��������********************/ 
#define    CC_FSTEST              0x29      // Ƶ�ʺϳ������ *****************/ 
#define    CC_PTEST               0x2A      // ��Ʒ����************************/ 
#define    CC_AGCTEST             0x2B      // AGC����*************************/ 
#define    CC_TEST2               0x2C      // ��ͬ�Ĳ�������******************/ 
#define    CC_TEST1               0x2D      // ��ͬ�Ĳ�������******************/ 
#define    CC_TEST0               0x2E      // ��ͬ�Ĳ�������******************/ 

/*�����˲�����*****************************************************************/
#define    CC_SRES                0x30      // ����оƬ************************/ 
#define    CC_SFSTXON             0x31      // ������У׼Ƶ�ʺϳ���************/ 
#define    CC_SXOFF               0x32      // �رվ�������******************/ 
#define    CC_SCAL                0x33      // У׼Ƶ�ʺϳ������ض�************/ 
#define    CC_SRX                 0x34      // ����RX��************************/ 
#define    CC_STX                 0x35      // ����״̬������TX��**************/ 
#define    CC_SIDLE               0x36      // �뿪RX/TX***********************/ 
#define    CC_SAFC                0x37      // Ƶ�ʺϳ�����AFC����*************/ 
#define    CC_SWOR                0x38      // �Զ�RXѡ�����У���Ų����****/ 
#define    CC_SPWD                0x39      // ��CSnΪ��ʱ���빦�ʽ���ģʽ*****/ 
#define    CC_SFRX                0x3A      // ��ϴRX FIFO����*****************/ 
#define    CC_SFTX                0x3B      // ��ϴTX FIFO����*****************/ 
#define    CC_SWORRST             0x3C      // ����������ʵʱ��ʱ��************/ 
#define    CC_SNOP                0x3D      // �޲���**************************/ 

/*״̬�Ĵ涨��*****************************************************************/
#define    CC_PARTNUM             0x30      // CC2550����ɲ�����Ŀ************/ 
#define    CC_VERSION             0x31      // ��ǰ�汾��**********************/ 
#define    CC_FREQEST             0x32      // ��ƫ�ƹ���**********************/ 
#define    CC_LQI                 0x33      // �������Ľ��������**************/ 
#define    CC_RSSI                0x34      // �����ź�ǿ��ָʾ****************/ 
#define    CC_MARCSTATE           0x35      // ����״̬��״̬******************/ 
#define    CC_WORTIME1            0x36      // WOR��ʱ�����ֽ�*****************/ 
#define    CC_WORTIME0            0x37      // WOR��ʱ�����ֽ�*****************/ 
#define    CC_PKTSTATUS           0x38      // ��ǰGDOx״̬�����ݰ�״̬********/ 
#define    CC_VCO_VC_DAC          0x39      // PLLУ׼ģ��ĵ�ǰ�趨***********/ 
#define    CC_TXBYTES             0x3A      // TX FIFO�е�����ͱ�����*********/ 
#define    CC_RXBYTES             0x3B      // RX FIFO�е�����ͱ�����*********/ 
#define    CC_PATABLE             0x3E      // ********************************/ 
#define    CC_TXFIFO              0x3F      // ********************************/ 
#define    CC_RXFIFO              0x3F      // ********************************/ 

/*�Ĵ��������� �����������****************************************************/
#define    IOCFG2                 0x06      // GDO2���������******************/ 
#define    IOCFG0                 0x06      // GDO0���������******************/ 
#define    FIFOTHR                0x07      // RX FIFO��TX FIFO����************/ 
#define    PKTLEN                 0xFF     // ���ݰ�����**********************/ 
#define    PKTCTRL1               0x05      // ���ݰ��Զ�����******************/ 
#define    PKTCTRL0               0x05      // ���ݰ��Զ�����******************/ 
#define    DEVICEADDR           0x03      // �豸��ַ************************/ 
#define    CHANNR                 0x00      // Ƶ����**************************/  
#define    FSCTRL1                0x06  //08  //08  //0B      // Ƶ�ʺϳ�������******************/ 
#define    FSCTRL0                0x00      // Ƶ�ʺϳ�������******************/ 
#define    FREQ2                  0x10      // Ƶ�ʿ��ƴʻ㣬���ֽ�************/ 
#define    FREQ1                  0xA7      // Ƶ�ʿ��ƴʻ㣬�м��ֽ�**********/ 
#define    FREQ0                  0x62      // Ƶ�ʿ��ƴʻ㣬���ֽ�************/ 
#define    MDMCFG4             0xca //0xCC//  0xC7  //CA  //7B  //5B      // ����������**********************/ 
#define    MDMCFG3              0x83 //0x22 // 0x83  //83  //83  //F8      // ����������**********************/ 
#define    MDMCFG2                0x03  //83  //03  //0A      // ����������**********************/ 
#define    MDMCFG1                0x22      // ����������**********************/ 
#define    MDMCFG0                0xF8      // ����������**********************/ 
#define    DEVIATN                0x40  //34  //42  //47      // ��������������******************/ 
#define    MCSM1                  0x3F      // ��ͨ�ſ���״̬������************/ 
#define    MCSM0                  0x18  //38     // ��ͨ�ſ���״̬������************/ 
#define    FOCCFG                 0x16  //1D     // Ƶ��ƫ�Ʋ�������****************/ 
#define    BSCFG                  0x6C  //1C     // λͬ������**********************/ 
#define    AGCCTRL2               0x43  //C7     // AGC����*************************/ 
#define    AGCCTRL1               0x40  //00     // AGC����*************************/ 
#define    AGCCTRL0               0x91  //B2     // AGC����*************************/ 
#define    FREND1                 0x56      // ǰĩ��RX����********************/ 
#define    FREND0                 0x10      // ǰĩ��TX����********************/ 
#define    FSCAL3                 0xE9      // Ƶ�ʺϳ���У׼******************/ 
#define    FSCAL2                 0x2A      // Ƶ�ʺϳ���У׼******************/ 
#define    FSCAL1                 0x00      // Ƶ�ʺϳ���У׼******************/ 
#define    FSCAL0                 0x1F      // Ƶ�ʺϳ���У׼******************/ 
#define    FSTEST                 0x59      // Ƶ�ʺϳ������******************/   
#define    TEST2                  0x88      // ��ͬ�Ĳ�������******************/ 
#define    TEST1                  0x31      // ��ͬ�Ĳ�������******************/ 
#define    TEST0                  0x0B      // ��ͬ�Ĳ�������******************/ 
/******************************************************************************/
#endif
