
#include "hal_rf.h"

/*功率配置************************************************/

unsigned char PaTabel[]= {0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0};
//u1t PaTabel[8] = {0x60 ,0x60 ,0x60 ,0x60 ,0x60 ,0x60 ,0x60 ,0x60};  //0dBm

unsigned char Rf_step=0,RxCounter=0,U_newdata='N',U_Data_length=0,Rf_Data_length=0;
u8 QIEHUAN;

enum MODEFLAGESELECT modeflage;
typedef struct
{
    u8 lqtest;

} TEST_;
u8 startraansfer=0;
u8 cc1101hander;


/*
void mydelay(u32 n)                   //微秒延时
 {
     while (n--);
 }
*/

/*********************************************************/
/*函数名字：SpiTxRx_Byte                                 */
/*输入参数：写入寄存器的数据                             */
/*输出参数：读取寄存器的数据                             */
/*功能描述：通过SPI 串口读写一字节数据                   */
/*建造日期；2008年03月30日                               */
/*********************************************************/
unsigned char SpiTxRx_Byte(unsigned char data)
{
#if 1
    u16 tmpdata;

    while(SPI_GetFlagStatus(SPI2, SPI_FLAG_TXE) == RESET);
    SPI_SendData(SPI2, data);

    while (SPI_GetFlagStatus(SPI2, SPI_FLAG_RXNE) == RESET);
    tmpdata = SPI_ReceiveData(SPI2);

    return tmpdata;
#endif
    
#if 0
//	INT8U i,temp;
//	temp = 0;

    /*等待发送寄存器空*/
    while((SPI2->SR & (uint8_t)SPI_FLAG_TXE)==RESET);
    /*发送一个字节*/
    SPI1->DR = data;
    /* 等待接收寄存器有效*/
    while((SPI2->SR & (uint8_t) SPI_FLAG_RXNE)==RESET);
	
    return(SPI2->DR);
#endif

}


/**********************************************************/
/*函数名字：Spi_Write_Byte                                */
/*输入参数：寄存器地址，配置                              */
/*输出参数：无                                            */
/*功能描述：单字节写入寄存器                              */
/*建造日期；2008年03月30日                                */
/**********************************************************/
void Spi_Write_Byte(unsigned char addr,unsigned char value)
{
    CSN_L;                                      //片选使能
    while(MISO);
    SpiTxRx_Byte(addr);                         //送出地址
    SpiTxRx_Byte(value);                        //写入配置
    CSN_H;                                      //结束使能

}

/**********************************************************/
/*函数名字：Spi_Write_Burst                               */
/*输入参数：寄存器地址，发射缓冲区首址，发送字节数        */
/*输出参数：无                                            */
/*功能描述：连续写入寄存器数据                            */
/*建造日期；2008年03月30日                                */
/**********************************************************/
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


/*********************************************************/
/*函数名字：Spi_Read_Byte                                */
/*输入参数：寄存器地址                                   */
/*输出参数：寄存器配置                                   */
/*功能描述：单字节读取寄存器                             */
/*建造日期；2008年03月30日                               */
/*********************************************************/
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

/**********************************************************/
/*函数名字：Spi_Read_Burst                                */
/*输入参数：寄存器地址，接收缓冲区首址，接收字节数        */
/*输出参数：结果存储在缓冲区地址                          */
/*功能描述：连续读取寄存器数据                            */
/*建造日期；2008年03月30日                                */
/**********************************************************/
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





/*********************************************************/
/*函数名字：WriteRfSettings                              */
/*输入参数：无                                           */
/*输出参数：无                                           */
/*功能描述：模块寄存器配置                               */
/*建造日期；2008年03月30日                               */
/*********************************************************/
void WriteRfSettings(void)
{
    Spi_Write_Byte (CC_IOCFG2,IOCFG2);          //
    Spi_Write_Byte (CC_IOCFG0,IOCFG0);        //

    //测试用
    //  Spi_Write_Byte (CC_IOCFG0,0X3E);        //
//   Spi_Write_Byte (CC_IOCFG2,0X3E);        //

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


/*********************************************************/
/*函数名字：POWER_UP_RESET_CC1100                        */
/*输入参数：无                                           */
/*输出参数：无                                           */
/*功能描述：模块上电初始化                               */
/*建造日期；2008年03月30日                               */
/*********************************************************/
void POWER_UP_RESET_CC1100(void)
{
    CSN_H;                                      //上电拉高

    delayus(1);                               //
    CSN_L;                                      //片选使能

    delayus(1);                               //
    CSN_H;                                      //选择拉高

    delayms(1);                              //最少延时40us,没有最大时间限制

    RESET_CC1100();                                        //复位命令
}


/*********************************************************/
/*函数名字：RESET_CC1100                                 */
/*输入参数：无                                           */
/*输出参数：无                                           */
/*功能描述：写入复位滤波命                               */
/*建造日期；2008年03月30日                               */
/*********************************************************/
void RESET_CC1100(void)
{
    CSN_L;                                      //片选使能
    while(MISO);

    SpiTxRx_Byte(CC_SRES);                              //复位命令
    while(MISO);
    CSN_H;                                      //结束使能

}









/*********************************************************/
/*函数名字：Init_cc1100                                  */
/*输入参数：无                                           */
/*输出参数：无                                           */
/*功能描述：初始化模块                                   */
/*建造日期；2008年03月30日                               */
/*********************************************************/
void Init_cc1100(void)
{
    int i;
    //ENCC1101;

    POWER_UP_RESET_CC1100();                    //上电复位
    delayms(20);

    WriteRfSettings();                          //写入配置

    // Set RX FIFO threshold
    halRfWriteReg(CC_FIFOTHR, FIFO_THRESHOLD);
    halRfWriteReg(CC_MCSM1, 0x33);      /* TX完成后保持RX状态,(RESET:0x30) */


    // Set GDO0 to be RX FIFO threshold signal
    halRfWriteReg(CC_IOCFG0, 0x00);
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
    delayms(10);
    Spi_Write_Burst(CC_PATABLE,PaTabel,8);      //功率配置
    Spi_Write_Strobe(CC_SCAL);
    for(i=0; i<100; i++) Spi_Write_Strobe(CC_SNOP);
    Spi_Write_Strobe(CC_SIDLE);                 //进入空闲
    Spi_Write_Strobe(CC_SFRX);                 //清空接受区

    Spi_Write_Strobe(CC_SRX);    //进入接收
    rf_data.rf_state = RX_STATE_RX;
    _timer_rx_timeout = TIME_OUT;
    _flag_rx_timeout = FALSE;

    //  Spi_Write_Strobe(CC_SPWD);    //进入接收


    //CSN_H;									  //上电拉高
}



/*********************************************************/
/*函数名字：Spi_Write_Packet                             */
/*输入参数：发送缓冲区首址,数组长度                      */
/*输出参数：无                                           */
/*功能描述：发送缓冲区数据                               */
/*建造日期；2008年03月30日                               */
/*********************************************************/
u8 Spi_Write_Packet(unsigned char *Tx_buffer,unsigned char size,unsigned char  id)
{
    u16 send_ok=1;//i,
    //添加关GDO0中断函数
    //  RFC1100AON();                                  //功率开
    Spi_Write_Strobe(CC_SFTX);                  //清缓冲区
    Spi_Write_Strobe(CC_SIDLE);                 //进入空闲
    //Spi_Write_Strobe(CC_STX);                   //进入接收模式
    Spi_Write_Byte(CC_TXFIFO,size+1);             //先送长度
    Spi_Write_Byte(CC_TXFIFO,0x02);
    Spi_Write_Burst(CC_TXFIFO,Tx_buffer,size);  //发送数据
    Spi_Write_Strobe(CC_STX);                   //发送模式
    // i = 0;
    while (GDO0_L)    ;                          //等待送出
#if 0
    {
        if (i > 50000)
        {
            send_ok=0;
            break;                        //限时等待
        }
        mydelay(1);
        i++;                                      //
    }
    i = 0;
#endif
    while (GDO0_H)        ;                      //送出完毕
#if 0
    {
        if (i > 50000)
        {
            send_ok=0;
            break;                        //限时等待
        }
        mydelay(1);
        i++;                                      //
    }
#endif
    Spi_Write_Strobe(CC_SFTX);                  //清缓冲区
    Spi_Write_Strobe(CC_SIDLE);                 //进入空闲
    Spi_Write_Strobe(CC_SRX);                   //进入接收模式
    //  RFC1100AOFF();
    return send_ok;
}

#if 0
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
#endif





/*********************************************************/
/*函数名字：Rx_cc1100                                    */
/*输入参数：无                                           */
/*输出参数：无                                           */
/*功能描述：接收数据,成功发送应答                        */
/*建造日期；2008年03月30日                               */
/*********************************************************/
/*
void Rx_cc1100(void)
 {
         u8 size;
         //size=sizeof(Rx_data);  //最大长度
         size=12;
         if (Spi_Read_Packet(Rx_data,size))          //接收数据
         {
		   Spi_Read_Packet(Rx_data,size);

		//   for(i=0;i<12;i++) Rx_data[i]=0;

         }
        mydelay(110);
        Spi_Write_Strobe(CC_SFRX);                  //清缓冲区
        mydelay(110);
         Spi_Write_Strobe(CC_SIDLE);                 //进入空闲
         Spi_Write_Strobe(CC_SRX);                   //进入接收
 }
*/

#if 0
/* length: playload,不包括长度,地址,sum */
void Rf_master_send_packet(u1t addr, u1t length)
{
    u16 packet_size;
    u8 write_count;
    //if (rf_data.rf_state==RX_STATE_IDLE)
    {
        packet_size = length+4;       /* length+addr+playload+sum */
        rf_data.rf_buf[0] = addr;
        rf_data.length = length+3;    /* addr+playload+sum */
        Make_sum(&rf_data.length, rf_data.length-1);
        if (packet_size>FIFO_SIZE)
        {
            write_count = FIFO_SIZE;
        }
        else
        {
            write_count = packet_size;
        }
        //INTERRUPT_PROTECT_SAVE;
        Spi_Write_Strobe(CC_SIDLE);
        Spi_Write_Byte(CC_IOCFG0, 0x02);
        Spi_Write_Strobe(CC_SFRX);
        Spi_Write_Strobe(CC_SFTX);
        halRfWriteFifo(&rf_data.length, write_count);
        rf_data.index = write_count-1;  /* -length */
        Spi_Write_Strobe(CC_STX);
        //INTERRUPT_PROTECT_RESTORE;
        rf_data.rf_state = TX_STATE_TXING;
        // _timer_rx_timeout = TIME_RESET;
        //_flag_rx_timeout = FALSE;
    }
}
#endif

