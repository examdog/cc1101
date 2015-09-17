
#include "hal_rf.h"

/*��������************************************************/

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
void mydelay(u32 n)                   //΢����ʱ
 {
     while (n--);
 }
*/

/*********************************************************/
/*�������֣�SpiTxRx_Byte                                 */
/*���������д��Ĵ���������                             */
/*�����������ȡ�Ĵ���������                             */
/*����������ͨ��SPI ���ڶ�дһ�ֽ�����                   */
/*�������ڣ�2008��03��30��                               */
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

    /*�ȴ����ͼĴ�����*/
    while((SPI2->SR & (uint8_t)SPI_FLAG_TXE)==RESET);
    /*����һ���ֽ�*/
    SPI1->DR = data;
    /* �ȴ����ռĴ�����Ч*/
    while((SPI2->SR & (uint8_t) SPI_FLAG_RXNE)==RESET);
	
    return(SPI2->DR);
#endif

}


/**********************************************************/
/*�������֣�Spi_Write_Byte                                */
/*����������Ĵ�����ַ������                              */
/*�����������                                            */
/*�������������ֽ�д��Ĵ���                              */
/*�������ڣ�2008��03��30��                                */
/**********************************************************/
void Spi_Write_Byte(unsigned char addr,unsigned char value)
{
    CSN_L;                                      //Ƭѡʹ��
    while(MISO);
    SpiTxRx_Byte(addr);                         //�ͳ���ַ
    SpiTxRx_Byte(value);                        //д������
    CSN_H;                                      //����ʹ��

}

/**********************************************************/
/*�������֣�Spi_Write_Burst                               */
/*����������Ĵ�����ַ�����仺������ַ�������ֽ���        */
/*�����������                                            */
/*��������������д��Ĵ�������                            */
/*�������ڣ�2008��03��30��                                */
/**********************************************************/
void Spi_Write_Burst(unsigned char addr,unsigned char *buffer,unsigned char count)
{
    unsigned char i;

    CSN_L;                                      //Ƭѡʹ��

    while(MISO);

    SpiTxRx_Byte((addr|Write_Burst));           //����д��

    for (i = 0; i < count; i++)                 //
    {
        SpiTxRx_Byte(buffer[i]);                  //�ͳ�����
    }
    CSN_H;                                      //����ʹ��

}

/*********************************************************/
/*�������֣�Spi_Write_Strobe                             */
/*����������˲�����                                     */
/*�����������                                           */
/*����������д���˲�����                                 */
/*�������ڣ�2008��03��30��                               */
/*********************************************************/
void Spi_Write_Strobe(unsigned char strobe)
{
    CSN_L;                                      //Ƭѡʹ��

    while(MISO);
    SpiTxRx_Byte(strobe);                                 //д������
    CSN_H;                                      //����ʹ��

}


/*********************************************************/
/*�������֣�Spi_Read_Byte                                */
/*����������Ĵ�����ַ                                   */
/*����������Ĵ�������                                   */
/*�������������ֽڶ�ȡ�Ĵ���                             */
/*�������ڣ�2008��03��30��                               */
/*********************************************************/
unsigned char Spi_Read_Byte(unsigned char addr)
{
    unsigned char value;

    CSN_L;                                      //Ƭѡʹ��

    while(MISO);
    SpiTxRx_Byte((addr|Read_Byte));             //�ͳ���ַ
    value = SpiTxRx_Byte(0);                    //���Ĵ���
    CSN_H;                                      //����ʹ��


    return value;                               //��������
}

/**********************************************************/
/*�������֣�Spi_Read_Burst                                */
/*����������Ĵ�����ַ�����ջ�������ַ�������ֽ���        */
/*�������������洢�ڻ�������ַ                          */
/*����������������ȡ�Ĵ�������                            */
/*�������ڣ�2008��03��30��                                */
/**********************************************************/
void Spi_Read_Burst (unsigned char addr,unsigned char *buffer,unsigned char count)
{
    unsigned char i;

    CSN_L;                                      //Ƭѡʹ��

    while(MISO);
    SpiTxRx_Byte((addr|Read_Burst));            //������ȡ

    for (i = 0; i < count; i++)                 //
    {
        buffer[i] = SpiTxRx_Byte(0);              //�洢����
    }
    CSN_H;                                      //����ʹ��

}





/*********************************************************/
/*�������֣�WriteRfSettings                              */
/*�����������                                           */
/*�����������                                           */
/*����������ģ��Ĵ�������                               */
/*�������ڣ�2008��03��30��                               */
/*********************************************************/
void WriteRfSettings(void)
{
    Spi_Write_Byte (CC_IOCFG2,IOCFG2);          //
    Spi_Write_Byte (CC_IOCFG0,IOCFG0);        //

    //������
    //  Spi_Write_Byte (CC_IOCFG0,0X3E);        //
//   Spi_Write_Byte (CC_IOCFG2,0X3E);        //

    Spi_Write_Byte (CC_FIFOTHR,FIFOTHR);        //
    Spi_Write_Byte (CC_PKTLEN,PKTLEN);          //
    Spi_Write_Byte (CC_PKTCTRL1,PKTCTRL1);      //
    Spi_Write_Byte (CC_PKTCTRL0,PKTCTRL0);      //�ɱ����ݰ�����ģʽ��ͨ��ͬ���ֺ����ֽ����õ����ݰ�����

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
/*�������֣�POWER_UP_RESET_CC1100                        */
/*�����������                                           */
/*�����������                                           */
/*����������ģ���ϵ��ʼ��                               */
/*�������ڣ�2008��03��30��                               */
/*********************************************************/
void POWER_UP_RESET_CC1100(void)
{
    CSN_H;                                      //�ϵ�����

    delayus(1);                               //
    CSN_L;                                      //Ƭѡʹ��

    delayus(1);                               //
    CSN_H;                                      //ѡ������

    delayms(1);                              //������ʱ40us,û�����ʱ������

    RESET_CC1100();                                        //��λ����
}


/*********************************************************/
/*�������֣�RESET_CC1100                                 */
/*�����������                                           */
/*�����������                                           */
/*����������д�븴λ�˲���                               */
/*�������ڣ�2008��03��30��                               */
/*********************************************************/
void RESET_CC1100(void)
{
    CSN_L;                                      //Ƭѡʹ��
    while(MISO);

    SpiTxRx_Byte(CC_SRES);                              //��λ����
    while(MISO);
    CSN_H;                                      //����ʹ��

}









/*********************************************************/
/*�������֣�Init_cc1100                                  */
/*�����������                                           */
/*�����������                                           */
/*������������ʼ��ģ��                                   */
/*�������ڣ�2008��03��30��                               */
/*********************************************************/
void Init_cc1100(void)
{
    int i;
    //ENCC1101;

    POWER_UP_RESET_CC1100();                    //�ϵ縴λ
    delayms(20);

    WriteRfSettings();                          //д������

    // Set RX FIFO threshold
    halRfWriteReg(CC_FIFOTHR, FIFO_THRESHOLD);
    halRfWriteReg(CC_MCSM1, 0x33);      /* TX��ɺ󱣳�RX״̬,(RESET:0x30) */


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
    Spi_Write_Burst(CC_PATABLE,PaTabel,8);      //��������
    Spi_Write_Strobe(CC_SCAL);
    for(i=0; i<100; i++) Spi_Write_Strobe(CC_SNOP);
    Spi_Write_Strobe(CC_SIDLE);                 //�������
    Spi_Write_Strobe(CC_SFRX);                 //��ս�����

    Spi_Write_Strobe(CC_SRX);    //�������
    rf_data.rf_state = RX_STATE_RX;
    _timer_rx_timeout = TIME_OUT;
    _flag_rx_timeout = FALSE;

    //  Spi_Write_Strobe(CC_SPWD);    //�������


    //CSN_H;									  //�ϵ�����
}



/*********************************************************/
/*�������֣�Spi_Write_Packet                             */
/*������������ͻ�������ַ,���鳤��                      */
/*�����������                                           */
/*�������������ͻ���������                               */
/*�������ڣ�2008��03��30��                               */
/*********************************************************/
u8 Spi_Write_Packet(unsigned char *Tx_buffer,unsigned char size,unsigned char  id)
{
    u16 send_ok=1;//i,
    //��ӹ�GDO0�жϺ���
    //  RFC1100AON();                                  //���ʿ�
    Spi_Write_Strobe(CC_SFTX);                  //�建����
    Spi_Write_Strobe(CC_SIDLE);                 //�������
    //Spi_Write_Strobe(CC_STX);                   //�������ģʽ
    Spi_Write_Byte(CC_TXFIFO,size+1);             //���ͳ���
    Spi_Write_Byte(CC_TXFIFO,0x02);
    Spi_Write_Burst(CC_TXFIFO,Tx_buffer,size);  //��������
    Spi_Write_Strobe(CC_STX);                   //����ģʽ
    // i = 0;
    while (GDO0_L)    ;                          //�ȴ��ͳ�
#if 0
    {
        if (i > 50000)
        {
            send_ok=0;
            break;                        //��ʱ�ȴ�
        }
        mydelay(1);
        i++;                                      //
    }
    i = 0;
#endif
    while (GDO0_H)        ;                      //�ͳ����
#if 0
    {
        if (i > 50000)
        {
            send_ok=0;
            break;                        //��ʱ�ȴ�
        }
        mydelay(1);
        i++;                                      //
    }
#endif
    Spi_Write_Strobe(CC_SFTX);                  //�建����
    Spi_Write_Strobe(CC_SIDLE);                 //�������
    Spi_Write_Strobe(CC_SRX);                   //�������ģʽ
    //  RFC1100AOFF();
    return send_ok;
}

#if 0
/*********************************************************/
/*�������֣�Spi_Read_Packet                              */
/*������������ջ�������ַ,��󳤶�                      */
/*���������У���־                                     */
/*�������������ݴ��뻺����                               */
/*�������ڣ�2008��03��30��                               */
/*********************************************************/

unsigned char Spi_Read_Packet( u8 *receivebuf)//���������⣬��Ҫ������д
{
    // unsigned char size = 0,addr = 0;
    unsigned char status[2];
    u8 length;
    // memset(thiscomm->receivebuf,0,MAX_RECEIVE_LEN);
    Spi_Read_Byte(CC_SIDLE);
    if ((Spi_Read_Byte(CC_RXBYTES) & 0x7F))
    {
        length= Spi_Read_Byte(CC_RXFIFO)-1;            //���ݳ���
        receivebuf[0]=Spi_Read_Byte(CC_RXFIFO);
        if(receivebuf[0]<10&&receivebuf[0]>0)
        {
            Spi_Read_Burst(CC_RXFIFO,&(receivebuf[0]),length); //��������
            Spi_Read_Burst(CC_RXFIFO,status,2);//У������
        }
    }
    Spi_Write_Strobe(CC_SIDLE);                 //�������  L
    Spi_Write_Strobe(CC_SFRX);                  //�建����
    Spi_Write_Strobe(CC_SRX);                   //�������   L
    return (status[1]&CRC_OK);                  //У���־
}
#endif





/*********************************************************/
/*�������֣�Rx_cc1100                                    */
/*�����������                                           */
/*�����������                                           */
/*������������������,�ɹ�����Ӧ��                        */
/*�������ڣ�2008��03��30��                               */
/*********************************************************/
/*
void Rx_cc1100(void)
 {
         u8 size;
         //size=sizeof(Rx_data);  //��󳤶�
         size=12;
         if (Spi_Read_Packet(Rx_data,size))          //��������
         {
		   Spi_Read_Packet(Rx_data,size);

		//   for(i=0;i<12;i++) Rx_data[i]=0;

         }
        mydelay(110);
        Spi_Write_Strobe(CC_SFRX);                  //�建����
        mydelay(110);
         Spi_Write_Strobe(CC_SIDLE);                 //�������
         Spi_Write_Strobe(CC_SRX);                   //�������
 }
*/

#if 0
/* length: playload,����������,��ַ,sum */
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

