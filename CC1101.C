#include "CC1101.h"
#include "stm8l15x.h"
#include "Global.h"
#include "base_conf.H"
#define CSN_L  GPIOD->ODR &= (uint8_t)(~GPIO_Pin_7);
#define CSN_H  GPIOD->ODR |= GPIO_Pin_7;
#define MISO GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_4)
typedef struct CC_STRUCT{
  unsigned char  add;
  unsigned char  length;
  unsigned char* data_buf;
}CC_DATA;
CC_DATA CC_DATA1;
unsigned char PaTabel[]= {0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0};
unsigned char SpiTxRx_Byte(unsigned char data)
{
    u16 tmpdata;

    while(SPI_GetFlagStatus(SPI2, SPI_FLAG_TXE) == RESET);
    SPI_SendData(SPI2, data);
    while (SPI_GetFlagStatus(SPI2, SPI_FLAG_RXNE) == RESET);
    tmpdata = SPI_ReceiveData(SPI2);

    return tmpdata;
}

void Spi_Write_Byte(unsigned char addr,unsigned char value)
{
    CSN_L;                                      //片选使能
    while(MISO);
    SpiTxRx_Byte(addr);                         //送出地址
    SpiTxRx_Byte(value);                        //写入配置
    CSN_H;                                      //结束使能

}

void RESET_CC1100(void)
{
    CSN_L;                                      //片选使能
   // while(MISO);

//SpiTxRx_Byte(0x30);                              //复位命令
    //temp=SpiTxRx_Byte(0X03);
    //while(MISO);
    //CSN_H;                                      //结束使能

}
void POWER_UP_RESET_CC1100(void)
{
    CSN_H;                                      //上电拉高

  //  delayus(1);                               //
    CSN_L;                                      //片选使能

  //  delayus(1);                               //
    CSN_H;                                      //选择拉高

 //   delayms(1);                              //最少延时40us,没有最大时间限制

    RESET_CC1100();                                        //复位命令
}

/*********************************************************/
/*函数名字：Spi_Write_Strobe                             */
/*输入参数：滤波命令                                     */
/*输出参数：无                                           */
/*功能描述：写入滤波命令                                 */
/*建造日期；2008年03月30日                               */
/*********************************************************/
void Spi_Write_Strobe(unsigned char strobe)
{
    CSN_L;                                      //片选使能

    while(MISO);
    SpiTxRx_Byte(strobe);                                 //写入命令
    CSN_H;                                      //结束使能

}
//
unsigned char halSpiWrite(unsigned char addr, unsigned char* data, unsigned char length)
{

    unsigned char i;
    unsigned char rc;
    CSN_L;                                      //片选使能
    while(MISO);
    rc=SpiTxRx_Byte(addr);           //连续写入
    for (i = 0; i < length; i++)                 //
    {
        SpiTxRx_Byte(data[i]);                  //送出数据
    }
    CSN_H;                                      //结束使能
    return rc;

}

unsigned char halRfWriteReg(unsigned char addr, unsigned char data)
{
    unsigned char rc;
    rc = halSpiWrite(addr, &data, 1);
    return(rc);
}

//----------------------------------------------------------------------------------
//  HAL_RF_STATUS halRfWriteFifo(uint8* data, uint8 length)
//----------------------------------------------------------------------------------
unsigned char halRfWriteFifo(unsigned char* data, unsigned char length)
{
    return(halSpiWrite(CC_TXFIFO |Write_Burst, data, length));
}

void Spi_Write_Burst(unsigned char addr,unsigned char *buffer,unsigned char count)
{
    unsigned char i;

    CSN_L;                                      //片选使能

    while(MISO);

    SpiTxRx_Byte((addr|Write_Burst));           //连续写入

    for (i = 0; i < count; i++)                 //
    {
        SpiTxRx_Byte(buffer[i]);                  //送出数据
    }
    CSN_H;                                      //结束使能

}

unsigned char Spi_Read_Byte(unsigned char addr)
{
    unsigned char value;

    CSN_L;                                      //片选使能

    while(MISO);
    SpiTxRx_Byte((addr|Read_Byte));             //送出地址
    value = SpiTxRx_Byte(0);                    //读寄存器
    CSN_H;                                      //结束使能


    return value;                               //返回数据
}

void Spi_Read_Burst (unsigned char addr,unsigned char *buffer,unsigned char count)
{
    unsigned char i;

    CSN_L;                                      //片选使能

    while(MISO);
    SpiTxRx_Byte((addr|Read_Burst));            //连续读取

    for (i = 0; i < count; i++)                 //
    {
        buffer[i] = SpiTxRx_Byte(0);              //存储数据
    }
    CSN_H;                                      //结束使能

}




void WriteRfSettings(void)
{

    Spi_Write_Byte (CC_IOCFG0,IOCFG0); 
    Spi_Write_Byte (CC_IOCFG2,IOCFG2);  
          //


    
    Spi_Write_Byte (CC_FIFOTHR,FIFOTHR);        //
    Spi_Write_Byte (CC_PKTLEN,PKTLEN);          //
    Spi_Write_Byte (CC_PKTCTRL1,PKTCTRL1);      //
    Spi_Write_Byte (CC_PKTCTRL0,PKTCTRL0);      //可变数据包长度模式。通过同步字后首字节配置的数据包长度

    Spi_Write_Byte (CC_ADDR,DEVICEADDR);

    Spi_Write_Byte (CC_CHANNR,CHANNR);          //
    Spi_Write_Byte (CC_FSCTRL1,FSCTRL1);        //
    Spi_Write_Byte (CC_FSCTRL0,FSCTRL0);        //
    Spi_Write_Byte (CC_FREQ2,FREQ2);            //
    Spi_Write_Byte (CC_FREQ1,FREQ1);            //
    Spi_Write_Byte (CC_FREQ0,FREQ0);            //
    Spi_Write_Byte (CC_MDMCFG4,MDMCFG4);        //
    Spi_Write_Byte (CC_MDMCFG3,MDMCFG3);        //
    Spi_Write_Byte (CC_MDMCFG2,MDMCFG2);        //
    Spi_Write_Byte (CC_MDMCFG1,MDMCFG1);        //
    Spi_Write_Byte (CC_MDMCFG0,MDMCFG0);        //
    Spi_Write_Byte (CC_DEVIATN,DEVIATN);        //
    Spi_Write_Byte (CC_MCSM0,MCSM0);            //
    Spi_Write_Byte (CC_FOCCFG,FOCCFG);          //
    Spi_Write_Byte (CC_BSCFG,BSCFG);            //
    Spi_Write_Byte (CC_AGCCTRL2,AGCCTRL2);      //
    Spi_Write_Byte (CC_AGCCTRL1,AGCCTRL1);      //
    Spi_Write_Byte (CC_AGCCTRL0,AGCCTRL0);      //
    Spi_Write_Byte (CC_FREND1,FREND1);          //
    Spi_Write_Byte (CC_FREND0,FREND0);          //
    Spi_Write_Byte (CC_FSCAL3,FSCAL3);          //
    Spi_Write_Byte (CC_FSCAL2,FSCAL2);          //
    Spi_Write_Byte (CC_FSCAL1,FSCAL1);          //
    Spi_Write_Byte (CC_FSCAL0,FSCAL0);          //
    Spi_Write_Byte (CC_FSTEST,FSTEST);          //
    Spi_Write_Byte (CC_TEST2,TEST2);            //
    Spi_Write_Byte (CC_TEST1,TEST1);            //
    Spi_Write_Byte (CC_TEST0,TEST0);            //

}

void Init_cc1100(void)
{
    int i;
    //ENCC1101;

    POWER_UP_RESET_CC1100();                    //上电复位
    Delay(2);

    WriteRfSettings();                          //写入配置

    // Set RX FIFO threshold
    halRfWriteReg(CC_FIFOTHR, FIFO_THRESHOLD);
    halRfWriteReg(CC_MCSM1, 0x33);      /* TX完成后保持RX状态,(RESET:0x30) */


    // Set GDO0 to be RX FIFO threshold signal
    halRfWriteReg(CC_IOCFG0, 0x06);//00修改为06
    // Set up interrupt on GDO0
    //halDigioIntSetEdge(&pinGDO0, HAL_DIGIO_INT_RISING_EDGE);
    //halDigioIntConnect(&pinGDO0, &Rx_fifo_half_full);
    //halDigioIntEnable(&pinGDO0);
    // Set GDO2 to be packet received signal
    halRfWriteReg(CC_IOCFG2, 0x06);
    // Set up interrupt on GDO2 
    //halDigioIntSetEdge(&pinGDO2, HAL_DIGIO_INT_FALLING_EDGE);
    //halDigioIntConnect(&pinGDO2, &Rx_packet_recvd);
    //halDigioIntEnable(&pinGDO2);
    Delay(1);
    Spi_Write_Burst(CC_PATABLE,PaTabel,8);      //功率配置
    Spi_Write_Strobe(CC_SCAL);
    for(i=0; i<100; i++)  Spi_Write_Strobe(CC_SNOP);
    Spi_Write_Strobe(CC_SIDLE);                 //进入空闲
    Spi_Write_Strobe(CC_SFRX);                 //清空接受区

    Spi_Write_Strobe(CC_SRX);    //进入接收
    G_A=Spi_Read_Byte(CC_IOCFG2);

    //rf_data.rf_state = RX_STATE_RX;
    //_timer_rx_timeout = TIME_OUT;
    //_flag_rx_timeout = FALSE;

    //  Spi_Write_Strobe(CC_SPWD);    //进入接收


    //CSN_H;	
    //上电拉高
    

   


}




u8 Spi_Write_Packet(unsigned char *Tx_buffer,unsigned char size,unsigned char  id)
{
  //int i=0;
    u16 send_ok=1;//i,
    //添加关GDO0中断函数
    //  RFC1100AON();                                  //功率开

    Spi_Write_Strobe(CC_SFTX);                  //清缓冲区
    Spi_Write_Strobe(CC_SIDLE);                 //进入空闲
    CSN_L;
    G_A=Spi_Read_Byte(CC_IOCFG2);
   // Spi_Write_Strobe(CC_STX);                   //进入接收模式
    Spi_Write_Byte(CC_TXFIFO,size+1);             //先送长度
    Spi_Write_Byte(CC_TXFIFO,id);
    Spi_Write_Burst(CC_TXFIFO,Tx_buffer,10);  //发送数据
    Spi_Write_Strobe(CC_STX);                   //发送模式


    // i = 0;
    while (GDO0_L)     ;                         //等待送出
/*
    {
        if (i > 50000)
        {
            send_ok=0;
            break;                        //限时等待
        }
        Delay(1);
        i++;                                      //
    }
    i = 0;
*/
    while (GDO0_H) ;                             //送出完毕
/*
    {
        if (i > 50000)
        {
            send_ok=0;
            break;                        //限时等待
        }
        Delay(1);
        i++;                                      //
    }
*/

    Spi_Write_Strobe(CC_SFTX);                  //清缓冲区
    Spi_Write_Strobe(CC_SIDLE);                 //进入空闲
    Spi_Write_Strobe(CC_SRX);                   //进入接收模式
    //  RFC1100AOFF();
    return send_ok;
}


/*********************************************************/
/*函数名字：Spi_Read_Packet                              */
/*输入参数：接收缓冲区首址,最大长度                      */
/*输出参数：校验标志                                     */
/*功能描述：数据存入缓冲区                               */
/*建造日期；2008年03月30日                               */
/*********************************************************/

unsigned char Spi_Read_Packet( u8 *receivebuf)//程序有问题，需要重新书写
{
    // unsigned char size = 0,addr = 0;
    unsigned char status[2];
    u8 length;
    // memset(thiscomm->receivebuf,0,MAX_RECEIVE_LEN);
    Spi_Read_Byte(CC_SIDLE);
    if ((Spi_Read_Byte(CC_RXBYTES) & 0x7F))
    {
        length= Spi_Read_Byte(CC_RXFIFO)-1;            //数据长度
        receivebuf[0]=Spi_Read_Byte(CC_RXFIFO);
        if(receivebuf[0]<10&&receivebuf[0]>0)
        {
            Spi_Read_Burst(CC_RXFIFO,&(receivebuf[0]),length); //接收数据
            Spi_Read_Burst(CC_RXFIFO,status,2);//校验数据
        }
    }
    Spi_Write_Strobe(CC_SIDLE);                 //进入空闲  L
    Spi_Write_Strobe(CC_SFRX);                  //清缓冲区
    Spi_Write_Strobe(CC_SRX);                   //进入接收   L
    return (status[1]&CRC_OK);                  //校验标志
}


 unsigned char  buffer[6]={0xaa,0xaa,0xaa,0xaa,0xaa,0xaa};
void Send_Data(unsigned char* Tx_buffer,unsigned char data_length,int id)

{
unsigned char length=0;

unsigned char send_num=0;
length=data_length;
halRfWriteReg(CC_IOCFG0, 0x06);
Spi_Write_Strobe(CC_SIDLE);                 //进入空闲
    
//Spi_Read_Byte();
 Spi_Write_Byte(CC_TXFIFO,11);    
    Spi_Write_Burst(CC_TXFIFO,&Tx_buffer[4],10);  //发送数据
      Spi_Write_Strobe(CC_STX); 
      while (GDO0_L); 
   while (GDO0_H);
    Spi_Write_Strobe(CC_SIDLE);
    Spi_Write_Strobe(CC_SFTX);    
    Spi_Write_Byte(CC_TXFIFO,11); 
    Spi_Write_Burst(CC_TXFIFO,&Tx_buffer[6],10);  //发送数据
      Spi_Write_Strobe(CC_STX); 
      while (GDO0_L); 
   while (GDO0_H);
    Spi_Write_Strobe(CC_SIDLE);
    Spi_Write_Strobe(CC_SFTX);    
                 //进入空闲

  
  
}

//缓冲区中断
void    GDO0_INT()
{
    Spi_Write_Strobe(CC_STX); 
    Spi_Write_Strobe(CC_SIDLE);
    Spi_Write_Strobe(CC_SFTX); 
}
//判断发送是否完成
void GDO1_INT()
{
  

}