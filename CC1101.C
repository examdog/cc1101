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
    CSN_L;                                      //Ƭѡʹ��
    while(MISO);
    SpiTxRx_Byte(addr);                         //�ͳ���ַ
    SpiTxRx_Byte(value);                        //д������
    CSN_H;                                      //����ʹ��

}

void RESET_CC1100(void)
{
    CSN_L;                                      //Ƭѡʹ��
   // while(MISO);

//SpiTxRx_Byte(0x30);                              //��λ����
    //temp=SpiTxRx_Byte(0X03);
    //while(MISO);
    //CSN_H;                                      //����ʹ��

}
void POWER_UP_RESET_CC1100(void)
{
    CSN_H;                                      //�ϵ�����

  //  delayus(1);                               //
    CSN_L;                                      //Ƭѡʹ��

  //  delayus(1);                               //
    CSN_H;                                      //ѡ������

 //   delayms(1);                              //������ʱ40us,û�����ʱ������

    RESET_CC1100();                                        //��λ����
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
//
unsigned char halSpiWrite(unsigned char addr, unsigned char* data, unsigned char length)
{

    unsigned char i;
    unsigned char rc;
    CSN_L;                                      //Ƭѡʹ��
    while(MISO);
    rc=SpiTxRx_Byte(addr);           //����д��
    for (i = 0; i < length; i++)                 //
    {
        SpiTxRx_Byte(data[i]);                  //�ͳ�����
    }
    CSN_H;                                      //����ʹ��
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

    CSN_L;                                      //Ƭѡʹ��

    while(MISO);

    SpiTxRx_Byte((addr|Write_Burst));           //����д��

    for (i = 0; i < count; i++)                 //
    {
        SpiTxRx_Byte(buffer[i]);                  //�ͳ�����
    }
    CSN_H;                                      //����ʹ��

}

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




void WriteRfSettings(void)
{

    Spi_Write_Byte (CC_IOCFG0,IOCFG0); 
    Spi_Write_Byte (CC_IOCFG2,IOCFG2);  
          //


    
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

void Init_cc1100(void)
{
    int i;
    //ENCC1101;

    POWER_UP_RESET_CC1100();                    //�ϵ縴λ
    Delay(2);

    WriteRfSettings();                          //д������

    // Set RX FIFO threshold
    halRfWriteReg(CC_FIFOTHR, FIFO_THRESHOLD);
    halRfWriteReg(CC_MCSM1, 0x33);      /* TX��ɺ󱣳�RX״̬,(RESET:0x30) */


    // Set GDO0 to be RX FIFO threshold signal
    halRfWriteReg(CC_IOCFG0, 0x06);//00�޸�Ϊ06
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
    Spi_Write_Burst(CC_PATABLE,PaTabel,8);      //��������
    Spi_Write_Strobe(CC_SCAL);
    for(i=0; i<100; i++)  Spi_Write_Strobe(CC_SNOP);
    Spi_Write_Strobe(CC_SIDLE);                 //�������
    Spi_Write_Strobe(CC_SFRX);                 //��ս�����

    Spi_Write_Strobe(CC_SRX);    //�������
    G_A=Spi_Read_Byte(CC_IOCFG2);

    //rf_data.rf_state = RX_STATE_RX;
    //_timer_rx_timeout = TIME_OUT;
    //_flag_rx_timeout = FALSE;

    //  Spi_Write_Strobe(CC_SPWD);    //�������


    //CSN_H;	
    //�ϵ�����
    

   


}




u8 Spi_Write_Packet(unsigned char *Tx_buffer,unsigned char size,unsigned char  id)
{
  //int i=0;
    u16 send_ok=1;//i,
    //���ӹ�GDO0�жϺ���
    //  RFC1100AON();                                  //���ʿ�

    Spi_Write_Strobe(CC_SFTX);                  //�建����
    Spi_Write_Strobe(CC_SIDLE);                 //�������
    CSN_L;
    G_A=Spi_Read_Byte(CC_IOCFG2);
   // Spi_Write_Strobe(CC_STX);                   //�������ģʽ
    Spi_Write_Byte(CC_TXFIFO,size+1);             //���ͳ���
    Spi_Write_Byte(CC_TXFIFO,id);
    Spi_Write_Burst(CC_TXFIFO,Tx_buffer,10);  //��������
    Spi_Write_Strobe(CC_STX);                   //����ģʽ


    // i = 0;
    while (GDO0_L)     ;                         //�ȴ��ͳ�
/*
    {
        if (i > 50000)
        {
            send_ok=0;
            break;                        //��ʱ�ȴ�
        }
        Delay(1);
        i++;                                      //
    }
    i = 0;
*/
    while (GDO0_H) ;                             //�ͳ����
/*
    {
        if (i > 50000)
        {
            send_ok=0;
            break;                        //��ʱ�ȴ�
        }
        Delay(1);
        i++;                                      //
    }
*/

    Spi_Write_Strobe(CC_SFTX);                  //�建����
    Spi_Write_Strobe(CC_SIDLE);                 //�������
    Spi_Write_Strobe(CC_SRX);                   //�������ģʽ
    //  RFC1100AOFF();
    return send_ok;
}


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


 unsigned char  buffer[6]={0xaa,0xaa,0xaa,0xaa,0xaa,0xaa};
void Send_Data(unsigned char* Tx_buffer,unsigned char data_length,int id)

{
unsigned char length=0;

unsigned char send_num=0;
length=data_length;
halRfWriteReg(CC_IOCFG0, 0x06);
Spi_Write_Strobe(CC_SIDLE);                 //�������
    
//Spi_Read_Byte();
 Spi_Write_Byte(CC_TXFIFO,11);    
    Spi_Write_Burst(CC_TXFIFO,&Tx_buffer[4],10);  //��������
      Spi_Write_Strobe(CC_STX); 
      while (GDO0_L); 
   while (GDO0_H);
    Spi_Write_Strobe(CC_SIDLE);
    Spi_Write_Strobe(CC_SFTX);    
    Spi_Write_Byte(CC_TXFIFO,11); 
    Spi_Write_Burst(CC_TXFIFO,&Tx_buffer[6],10);  //��������
      Spi_Write_Strobe(CC_STX); 
      while (GDO0_L); 
   while (GDO0_H);
    Spi_Write_Strobe(CC_SIDLE);
    Spi_Write_Strobe(CC_SFTX);    
                 //�������

  
  
}

//�������ж�
void    GDO0_INT()
{
    Spi_Write_Strobe(CC_STX); 
    Spi_Write_Strobe(CC_SIDLE);
    Spi_Write_Strobe(CC_SFTX); 
}
//�жϷ����Ƿ����
void GDO1_INT()
{
  

}