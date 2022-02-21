//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//����Ӳ������Ƭ��STM32H743IIT6,����ԭ��Apollo STM32F4/F7������,��Ƶ400MHZ������12MHZ
//QDtech-TFTҺ������ for STM32 FSMC
//xiao��@ShenZhen QDtech co.,LTD
//��˾��վ:www.qdtft.com
//�Ա���վ��http://qdtech.taobao.com
//wiki������վ��http://www.lcdwiki.com
//��˾�ṩ����֧�֣��κμ������⻶ӭ��ʱ����ѧϰ
//�̻�(����) :+86 0755-23594567 
//�ֻ�:15989313508���빤�� 
//����:lcdwiki01@gmail.com    support@lcdwiki.com    goodtft@163.com 
//����֧��QQ:3002773612  3002778157
//��������QQȺ:324828016
//��������:2018/08/09
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������ȫ�����Ӽ������޹�˾ 2018-2028
//All rights reserved
/****************************************************************************************************
//��ģ�����ֱ�Ӳ�������ԭ��Apollo STM32F4/F7������TFTLCD��ۣ������ֶ�����
//STM32����������ָTFTLCD��������ڲ����ӵ�STM32����
//=========================================��Դ����================================================//
//     LCDģ��             TFTLCD�������        STM32��������
//      VDD       --->         5V/3.3              DC5V/3.3V          //��Դ
//      GND       --->          GND                  GND              //��Դ��
//=======================================Һ���������߽���==========================================//
//��ģ��Ĭ��������������Ϊ16λ��������
//     LCDģ��             TFTLCD�������        STM32��������
//      DB0       --->          D0                   PD14        -|   
//      DB1       --->          D1                   PD15         |  
//      DB2       --->          D2                   PD0          | 
//      DB3       --->          D3                   PD1          | 
//      DB4       --->          D4                   PE7          |
//      DB5       --->          D5                   PE8          |
//      DB6       --->          D6                   PE9          |
//      DB7       --->          D7                   PE10         |
//���ʹ��8λģʽ����ʹ�������8λ������������                    |===>Һ����16λ���������ź�
//      DB8       --->          D8                   PE11         |
//      DB9       --->          D9                   PE12         |
//      DB10      --->          D10                  PE13         |
//      DB11      --->          D11                  PE14         |
//      DB12      --->          D12                  PE15         |
//      DB13      --->          D13                  PD8          |
//      DB14      --->          D14                  PD9          |
//      DB15      --->          D15                  PD10        -|
//=======================================Һ���������߽���==========================================//
//     LCDģ�� 				     TFTLCD�������        STM32�������� 
//       WR       --->          WR                   PD5             //Һ����д���ݿ����ź�
//       RD       --->          RD                   PD4             //Һ���������ݿ����ź�
//       RS       --->          RS                   PD13            //Һ��������/��������ź�
//       RST      --->          RST                ��λ����          //Һ������λ�����ź�
//       CS       --->          CS                   PD7             //Һ����Ƭѡ�����ź�
//       BL       --->          BL                   PB5             //Һ������������ź�
//=========================================������������=========================================//
//���ģ�鲻���������ܻ��ߴ��д������ܣ����ǲ���Ҫ�������ܣ�����Ҫ���д���������
//	   LCDģ��             TFTLCD�������        STM32�������� 
//      PEN       --->          PEN                  PH7             //�����������ж��ź�
//      MISO      --->          MISO                 PG3             //������SPI���߶��ź�
//      MOSI      --->          MOSI                 PI3             //������SPI����д�ź�
//      T_CS      --->          TCS                  PI8             //������Ƭѡ�����ź�
//      CLK       --->          CLK                  PH6             //������SPI����ʱ���ź�
**************************************************************************************************/		
 /* @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, QD electronic SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
**************************************************************************************************/		
#include "lcd.h"
#include "stdlib.h"
#include "delay.h"	 

SRAM_HandleTypeDef SRAM_Handler;    //SRAM���(���ڿ���LCD)
	   
//����LCD��Ҫ����
//Ĭ��Ϊ����
_lcd_dev lcddev;

//������ɫ,������ɫ
u16 POINT_COLOR = 0x0000,BACK_COLOR = 0xFFFF;  
u16 DeviceCode;	 

u16 LCD_read(void)
{
	vu16 data;			//��ֹ���Ż�
	data=LCD->LCD_RAM;	
#if LCD_USE8BIT_MODEL
	return (data>>8);  
#else
	return data;
#endif
}

/*****************************************************************************
 * @name       :void LCD_WR_REG(u16 data)
 * @date       :2018-08-09 
 * @function   :Write an 16-bit command to the LCD screen
 * @parameters :data:Command value to be written
 * @retvalue   :None
******************************************************************************/
void LCD_WR_REG(u16 data)
{ 
	data=data;		//ʹ��-O2�Ż���ʱ��,����������ʱ
#if LCD_USE8BIT_MODEL
	LCD->LCD_REG=(data<<8); //д��Ҫд�ļĴ������
#else
	LCD->LCD_REG=data; //д��Ҫд�ļĴ������
#endif	 
}

/*****************************************************************************
 * @name       :void LCD_WR_DATA(u16 data)
 * @date       :2018-08-09 
 * @function   :Write an 16-bit data to the LCD screen
 * @parameters :data:data value to be written
 * @retvalue   :None
******************************************************************************/
void LCD_WR_DATA(u16 data)
{
	data=data;			//ʹ��-O2�Ż���ʱ��,����������ʱ
#if LCD_USE8BIT_MODEL
	LCD->LCD_RAM=(data<<8);//д��Ҫд������
#else
	LCD->LCD_RAM=data; //д��Ҫд������
#endif	
}

/*****************************************************************************
 * @name       :u16 LCD_RD_DATA(void)
 * @date       :2018-11-13 
 * @function   :Read an 16-bit value from the LCD screen
 * @parameters :None
 * @retvalue   :read value
******************************************************************************/
u16 LCD_RD_DATA(void)
{
	return LCD_read();
}

/*****************************************************************************
 * @name       :void LCD_WriteReg(u16 LCD_Reg, u16 LCD_RegValue)
 * @date       :2018-08-09 
 * @function   :Write data into registers
 * @parameters :LCD_Reg:Register address
                LCD_RegValue:Data to be written
 * @retvalue   :None
******************************************************************************/
void LCD_WriteReg(u16 LCD_Reg, u16 LCD_RegValue)
{
#if LCD_USE8BIT_MODEL	
	LCD->LCD_REG = (LCD_Reg<<8);		//д��Ҫд�ļĴ������	 
	LCD->LCD_RAM = (LCD_RegValue<<8);//д������
#else
	LCD->LCD_REG = LCD_Reg;		//д��Ҫд�ļĴ������	 
	LCD->LCD_RAM = LCD_RegValue;//д������	    		 
#endif
}	   

/*****************************************************************************
 * @name       :u16 LCD_ReadReg(u16 LCD_Reg)
 * @date       :2018-11-13 
 * @function   :read value from specially registers
 * @parameters :LCD_Reg:Register address
 * @retvalue   :read value
******************************************************************************/
void LCD_ReadReg(u16 LCD_Reg,u8 *Rval,int n)
{
	LCD_WR_REG(LCD_Reg); 
	while(n--)
	{		
		*(Rval++) = LCD_RD_DATA();
		delay_us(5);	
	}
}

/*****************************************************************************
 * @name       :void LCD_WriteRAM_Prepare(void)
 * @date       :2018-08-09 
 * @function   :Write GRAM
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	 
void LCD_WriteRAM_Prepare(void)
{
	LCD_WR_REG(lcddev.wramcmd);
}	 

/*****************************************************************************
 * @name       :void LCD_ReadRAM_Prepare(void)
 * @date       :2018-11-13 
 * @function   :Read GRAM
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	 
void LCD_ReadRAM_Prepare(void)
{
	LCD_WR_REG(lcddev.rramcmd);
}

/*****************************************************************************
 * @name       :void Lcd_WriteData_16Bit(u16 Data)
 * @date       :2018-08-09 
 * @function   :Write an 16-bit command to the LCD screen
 * @parameters :Data:Data to be written
 * @retvalue   :None
******************************************************************************/	 
void Lcd_WriteData_16Bit(u16 Data)
{
#if LCD_USE8BIT_MODEL	
	 LCD->LCD_RAM = Data;
	 LCD->LCD_RAM = Data<<8;  //дʮ��λ��ɫֵ
#else
	 LCD->LCD_RAM = Data;  //дʮ��λ��ɫֵ
#endif
}

u16 Color_To_565(u8 r, u8 g, u8 b)
{
	return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3);
}

/*****************************************************************************
 * @name       :u16 Lcd_ReadData_16Bit(void)
 * @date       :2018-11-13 
 * @function   :Read an 16-bit value from the LCD screen
 * @parameters :None
 * @retvalue   :read value
******************************************************************************/	
u16 Lcd_ReadData_16Bit(void)
{
	u16 r,g,b;
	//dummy data
	r = LCD_RD_DATA();
	delay_us(1);//��ʱ1us
	//8bit:red data	
	//16bit:red and green data
	r = LCD_RD_DATA();
	delay_us(1);//��ʱ1us	
	//8bit:green data
	//16bit:blue data
	g = LCD_RD_DATA();
#if LCD_USE8BIT_MODEL
	delay_us(1);//��ʱ1us	
	//8bit:blue data
	b = LCD_RD_DATA();
#else
	b = g>>8;
	g = r&0xFF; 
	r = r>>8;
#endif
	return Color_To_565(r, g, b);
}

/*****************************************************************************
 * @name       :void LCD_DrawPoint(u16 x,u16 y)
 * @date       :2018-08-09 
 * @function   :Write a pixel data at a specified location
 * @parameters :x:the x coordinate of the pixel
                y:the y coordinate of the pixel
 * @retvalue   :None
******************************************************************************/	
void LCD_DrawPoint(u16 x,u16 y)
{
	LCD_SetCursor(x,y);//���ù��λ�� 
	Lcd_WriteData_16Bit(POINT_COLOR); 
}

/*****************************************************************************
 * @name       :u16 LCD_ReadPoint(u16 x,u16 y)
 * @date       :2018-11-13 
 * @function   :Read a pixel color value at a specified location
 * @parameters :x:the x coordinate of the pixel
                y:the y coordinate of the pixel
 * @retvalue   :the read color value
******************************************************************************/	
u16 LCD_ReadPoint(u16 x,u16 y)
{
	u16 color;
	if(x>=lcddev.width||y>=lcddev.height)
	{
		return 0;	//�����˷�Χ,ֱ�ӷ���	
	}
	LCD_SetCursor(x,y);//���ù��λ�� 
	LCD_ReadRAM_Prepare();	
	color = Lcd_ReadData_16Bit();
	return color;
}

/*****************************************************************************
 * @name       :void LCD_Clear(u16 Color)
 * @date       :2018-08-09 
 * @function   :Full screen filled LCD screen
 * @parameters :color:Filled color
 * @retvalue   :None
******************************************************************************/	
void LCD_Clear(u16 Color)
{
  unsigned int i;//,m;  
	u32 total_point=lcddev.width*lcddev.height;
	LCD_SetWindows(0,0,lcddev.width-1,lcddev.height-1);   
	for(i=0;i<total_point;i++)
	{
#if LCD_USE8BIT_MODEL	
		LCD->LCD_RAM = Color;
		LCD->LCD_RAM = Color<<8;//дʮ��λ��ɫֵ
#else
		LCD->LCD_RAM = Color; //дʮ��λ��ɫֵ
#endif
	}
} 

/*****************************************************************************
 * @name       :void LCD_MPU_Config(void)
 * @date       :2018-12-18 
 * @function   :Configure the region of the MPU, and configure 
								the external SRAM area to be in write-through mode.
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	
void LCD_MPU_Config(void)
{	
	MPU_Region_InitTypeDef MPU_Initure;

	HAL_MPU_Disable();							//����MPU֮ǰ�ȹر�MPU,��������Ժ���ʹ��MPU	
	//�ⲿSRAMΪregion0����СΪ2MB��������ɶ�д
	MPU_Initure.Enable=MPU_REGION_ENABLE;	    //ʹ��region
	MPU_Initure.Number=LCD_REGION_NUMBER;		//����region���ⲿSRAMʹ�õ�region0
	MPU_Initure.BaseAddress=LCD_ADDRESS_START;	//region����ַ
	MPU_Initure.Size=LCD_REGION_SIZE;			//region��С
	MPU_Initure.SubRegionDisable=0X00;
	MPU_Initure.TypeExtField=MPU_TEX_LEVEL0;
	MPU_Initure.AccessPermission=MPU_REGION_FULL_ACCESS;	//��region�ɶ�д
	MPU_Initure.DisableExec=MPU_INSTRUCTION_ACCESS_ENABLE;	//�����ȡ�������е�ָ��
	MPU_Initure.IsShareable=MPU_ACCESS_NOT_SHAREABLE;
	MPU_Initure.IsCacheable=MPU_ACCESS_NOT_CACHEABLE;
	MPU_Initure.IsBufferable=MPU_ACCESS_BUFFERABLE;
	HAL_MPU_ConfigRegion(&MPU_Initure);
	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);     //����MPU
}

/*****************************************************************************
 * @name       :void HAL_SRAM_MspInit(SRAM_HandleTypeDef *hsram)
 * @date       :2018-12-18 
 * @function   :SRAM underlying driver, clock enable, pin assignment
								This function will be called by HAL_SRAM_Init ()
 * @parameters :hsram:SRAM handle
 * @retvalue   :None
******************************************************************************/
void HAL_SRAM_MspInit(SRAM_HandleTypeDef *hsram)
{
    GPIO_InitTypeDef GPIO_Initure;
	
	__HAL_RCC_FMC_CLK_ENABLE();				//ʹ��FMCʱ��
	__HAL_RCC_GPIOD_CLK_ENABLE();			//ʹ��GPIODʱ��
	__HAL_RCC_GPIOE_CLK_ENABLE();			//ʹ��GPIOEʱ��
	
	//��ʼ��PD0,1,4,5,7,8,9,10,13,14,15
	GPIO_Initure.Pin=GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_8|\
                     GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
	GPIO_Initure.Mode=GPIO_MODE_AF_PP; 		//����
	GPIO_Initure.Pull=GPIO_PULLUP;			//����
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;	//����
	GPIO_Initure.Alternate=GPIO_AF12_FMC;	//����ΪFMC
	HAL_GPIO_Init(GPIOD,&GPIO_Initure);
	
	//��ʼ��PE7,8,9,10,11,12,13,14,15
	GPIO_Initure.Pin=GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|\
                     GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
	HAL_GPIO_Init(GPIOE,&GPIO_Initure);
}

/*****************************************************************************
 * @name       :void LCD_GPIOInit(void)
 * @date       :2018-08-09 
 * @function   :Initialization LCD screen GPIO
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	
void LCD_GPIOInit(void)
{
		GPIO_InitTypeDef GPIO_Initure;
		FMC_NORSRAM_TimingTypeDef FSMC_ReadWriteTim;
    FMC_NORSRAM_TimingTypeDef FSMC_WriteTim;
		__HAL_RCC_GPIOB_CLK_ENABLE();			//����GPIOBʱ��
    GPIO_Initure.Pin=GPIO_PIN_5;           //PB5,�������
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //�������
    GPIO_Initure.Pull=GPIO_PULLUP;          //����
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;    //����
    HAL_GPIO_Init(GPIOB,&GPIO_Initure); 
    LCD_MPU_Config();                       //ʹ��MPU����LCD����
    SRAM_Handler.Instance=FMC_NORSRAM_DEVICE; //BANK1    
    SRAM_Handler.Extended=FMC_NORSRAM_EXTENDED_DEVICE;         
              
        
    SRAM_Handler.Init.NSBank=FMC_NORSRAM_BANK1;     //ʹ��NE1
    SRAM_Handler.Init.DataAddressMux=FMC_DATA_ADDRESS_MUX_DISABLE;  //����     ��������
    SRAM_Handler.Init.MemoryType=FMC_MEMORY_TYPE_SRAM;              //SRAM
		SRAM_Handler.Init.MemoryDataWidth=FMC_NORSRAM_MEM_BUS_WIDTH_16; //16λ���ݿ��
    SRAM_Handler.Init.BurstAccessMode=FMC_BURST_ACCESS_MODE_DISABLE; //�Ƿ�ʹ��ͻ������,����ͬ��ͻ���洢����Ч,�˴�δ�õ�
    SRAM_Handler.Init.WaitSignalPolarity=FMC_WAIT_SIGNAL_POLARITY_LOW;//�ȴ��źŵļ���,����ͻ��ģʽ����������
    SRAM_Handler.Init.WaitSignalActive=FMC_WAIT_TIMING_BEFORE_WS;   //�洢�����ڵȴ�����֮ǰ��һ��ʱ�����ڻ��ǵȴ������ڼ�ʹ��NWAIT
    SRAM_Handler.Init.WriteOperation=FMC_WRITE_OPERATION_ENABLE;    //�洢��дʹ��
		SRAM_Handler.Init.WaitSignal=FMC_WAIT_SIGNAL_DISABLE;           //�ȴ�ʹ��λ,�˴�δ�õ�
    SRAM_Handler.Init.ExtendedMode=FMC_EXTENDED_MODE_ENABLE;        //��дʹ�ò�ͬ��ʱ��
    SRAM_Handler.Init.AsynchronousWait=FMC_ASYNCHRONOUS_WAIT_DISABLE;//�Ƿ�ʹ��ͬ������ģʽ�µĵȴ��ź�,�˴�δ�õ�
    SRAM_Handler.Init.WriteBurst=FMC_WRITE_BURST_DISABLE;           //��ֹͻ��д
    SRAM_Handler.Init.ContinuousClock=FMC_CONTINUOUS_CLOCK_SYNC_ASYNC;
        
    //FSMC��ʱ����ƼĴ���
    FSMC_ReadWriteTim.AddressSetupTime=0x11;    //��ַ����ʱ��(ADDSET)Ϊ17��HCLK 1/216M=4.6ns*17=78ns
    FSMC_ReadWriteTim.AddressHoldTime=0x00;
    FSMC_ReadWriteTim.DataSetupTime=0x55;       //���ݱ���ʱ��(DATAST)Ϊ85��HCLK	=4.6*85=391ns
    FSMC_ReadWriteTim.AccessMode=FMC_ACCESS_MODE_A; //ģʽA
    //FSMCдʱ����ƼĴ���
    FSMC_WriteTim.AddressSetupTime=0x15;        //��ַ����ʱ��(ADDSET)Ϊ21��HCLK=96ns
    FSMC_WriteTim.AddressHoldTime=0x00;
    FSMC_WriteTim.DataSetupTime=0x15;           //���ݱ���ʱ��(DATAST)Ϊ4.6ns*21��HCLK=96ns
    FSMC_WriteTim.AccessMode=FMC_ACCESS_MODE_A;     //ģʽA
		
    HAL_SRAM_Init(&SRAM_Handler,&FSMC_ReadWriteTim,&FSMC_WriteTim);	
		//��������дʱ����ƼĴ�����ʱ��   	 							    
    FSMC_WriteTim.AddressSetupTime=4;      
    FSMC_WriteTim.DataSetupTime=3;         
    FMC_NORSRAM_Extended_Timing_Init(SRAM_Handler.Extended,&FSMC_WriteTim,SRAM_Handler.Init.NSBank,SRAM_Handler.Init.ExtendedMode);
}

/*****************************************************************************
 * @name       :void LCD_Init(void)
 * @date       :2018-08-09 
 * @function   :Initialization LCD screen
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	 	 
void LCD_Init(void)
{  
	LCD_GPIOInit();//LCD GPIO��ʼ��	
//************* ILI9341��ʼ��**********//	
//*************3.2inch ILI9341��ʼ��**********//	
	LCD_WR_REG(0xCF);  
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0xD9); //C1 
	LCD_WR_DATA(0X30); 
	LCD_WR_REG(0xED);  
	LCD_WR_DATA(0x64); 
	LCD_WR_DATA(0x03); 
	LCD_WR_DATA(0X12); 
	LCD_WR_DATA(0X81); 
	LCD_WR_REG(0xE8);  
	LCD_WR_DATA(0x85); 
	LCD_WR_DATA(0x10); 
	LCD_WR_DATA(0x7A); 
	LCD_WR_REG(0xCB);  
	LCD_WR_DATA(0x39); 
	LCD_WR_DATA(0x2C); 
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0x34); 
	LCD_WR_DATA(0x02); 
	LCD_WR_REG(0xF7);  
	LCD_WR_DATA(0x20); 
	LCD_WR_REG(0xEA);  
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0x00); 
	LCD_WR_REG(0xC0);    //Power control 
	LCD_WR_DATA(0x1B);   //VRH[5:0] 
	LCD_WR_REG(0xC1);    //Power control 
	LCD_WR_DATA(0x12);   //SAP[2:0];BT[3:0] //0x01
	LCD_WR_REG(0xC5);    //VCM control 
	LCD_WR_DATA(0x26); 	 //3F
	LCD_WR_DATA(0x26); 	 //3C
	LCD_WR_REG(0xC7);    //VCM control2 
	LCD_WR_DATA(0XB0); 
	LCD_WR_REG(0x36);    // Memory Access Control 
	LCD_WR_DATA(0x08); 
	LCD_WR_REG(0x3A);   
	LCD_WR_DATA(0x55); 
	LCD_WR_REG(0xB1);   
	LCD_WR_DATA(0x00);   
	LCD_WR_DATA(0x1A); 
	LCD_WR_REG(0xB6);    // Display Function Control 
	LCD_WR_DATA(0x0A); 
	LCD_WR_DATA(0xA2); 
	LCD_WR_REG(0xF2);    // 3Gamma Function Disable 
	LCD_WR_DATA(0x00); 
	LCD_WR_REG(0x26);    //Gamma curve selected 
	LCD_WR_DATA(0x01); 
	LCD_WR_REG(0xE0); //Set Gamma
	LCD_WR_DATA(0x1F);
	LCD_WR_DATA(0x24);
	LCD_WR_DATA(0x24);
	LCD_WR_DATA(0x0D);
	LCD_WR_DATA(0x12);
	LCD_WR_DATA(0x09);
	LCD_WR_DATA(0x52);
	LCD_WR_DATA(0xB7);
	LCD_WR_DATA(0x3F);
	LCD_WR_DATA(0x0C);
	LCD_WR_DATA(0x15);
	LCD_WR_DATA(0x06);
	LCD_WR_DATA(0x0E);
	LCD_WR_DATA(0x08);
	LCD_WR_DATA(0x00);
	LCD_WR_REG(0XE1); //Set Gamma
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x1B);
	LCD_WR_DATA(0x1B);
	LCD_WR_DATA(0x02);
	LCD_WR_DATA(0x0E);
	LCD_WR_DATA(0x06);
	LCD_WR_DATA(0x2E);
	LCD_WR_DATA(0x48);
	LCD_WR_DATA(0x3F);
	LCD_WR_DATA(0x03);
	LCD_WR_DATA(0x0A);
	LCD_WR_DATA(0x09);
	LCD_WR_DATA(0x31);
	LCD_WR_DATA(0x37);
	LCD_WR_DATA(0x1F);
	LCD_WR_REG(0x2B); 
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x01);
	LCD_WR_DATA(0x3f);
	LCD_WR_REG(0x2A); 
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0xef);	 
	LCD_WR_REG(0x11); //Exit Sleep
	delay_ms(120);
	LCD_WR_REG(0x29); //display on		

  LCD_direction(USE_HORIZONTAL);//����LCD��ʾ����
	LCD_LED_SET();//��������	 
	LCD_Clear(WHITE);//��ȫ����ɫ
}
 
/*****************************************************************************
 * @name       :void LCD_SetWindows(u16 xStar, u16 yStar,u16 xEnd,u16 yEnd)
 * @date       :2018-08-09 
 * @function   :Setting LCD display window
 * @parameters :xStar:the bebinning x coordinate of the LCD display window
								yStar:the bebinning y coordinate of the LCD display window
								xEnd:the endning x coordinate of the LCD display window
								yEnd:the endning y coordinate of the LCD display window
 * @retvalue   :None
******************************************************************************/ 
void LCD_SetWindows(u16 xStar, u16 yStar,u16 xEnd,u16 yEnd)
{	
	LCD_WR_REG(lcddev.setxcmd);	
	LCD_WR_DATA(xStar>>8);
	LCD_WR_DATA(0x00FF&xStar);		
	LCD_WR_DATA(xEnd>>8);
	LCD_WR_DATA(0x00FF&xEnd);

	LCD_WR_REG(lcddev.setycmd);	
	LCD_WR_DATA(yStar>>8);
	LCD_WR_DATA(0x00FF&yStar);		
	LCD_WR_DATA(yEnd>>8);
	LCD_WR_DATA(0x00FF&yEnd);

	LCD_WriteRAM_Prepare();	//��ʼд��GRAM			
}   

/*****************************************************************************
 * @name       :void LCD_SetCursor(u16 Xpos, u16 Ypos)
 * @date       :2018-08-09 
 * @function   :Set coordinate value
 * @parameters :Xpos:the  x coordinate of the pixel
								Ypos:the  y coordinate of the pixel
 * @retvalue   :None
******************************************************************************/ 
void LCD_SetCursor(u16 Xpos, u16 Ypos)
{	  	    			
	LCD_SetWindows(Xpos,Ypos,Xpos,Ypos);	
} 

/*****************************************************************************
 * @name       :void LCD_direction(u8 direction)
 * @date       :2018-08-09 
 * @function   :Setting the display direction of LCD screen
 * @parameters :direction:0-0 degree
                          1-90 degree
													2-180 degree
													3-270 degree
 * @retvalue   :None
******************************************************************************/ 
void LCD_direction(u8 direction)
{ 
			lcddev.setxcmd=0x2A;
			lcddev.setycmd=0x2B;
			lcddev.wramcmd=0x2C;
			lcddev.rramcmd=0x2E;
	switch(direction){		  
		case 0:						 	 		
			lcddev.width=LCD_W;
			lcddev.height=LCD_H;		
			LCD_WriteReg(0x36,(1<<3));
		break;
		case 1:
			lcddev.width=LCD_H;
			lcddev.height=LCD_W;
			LCD_WriteReg(0x36,(1<<3)|(1<<5)|(1<<6));
		break;
		case 2:						 	 		
			lcddev.width=LCD_W;
			lcddev.height=LCD_H;	
			LCD_WriteReg(0x36,(1<<3)|(1<<7)|(1<<4)|(1<<6));
		break;
		case 3:
			lcddev.width=LCD_H;
			lcddev.height=LCD_W;
			LCD_WriteReg(0x36,(1<<3)|(1<<7)|(1<<5)|(1<<4));
		break;	
		default:break;
	}		
}	 

/*****************************************************************************
 * @name       :u16 LCD_Read_ID(void)
 * @date       :2018-11-13 
 * @function   :Read ID
 * @parameters :None
 * @retvalue   :ID value
******************************************************************************/ 
u16 LCD_Read_ID(void)
{
	u8 val[4] = {0};
	LCD_ReadReg(0xD3,val,4);
	return (val[2]<<8)|val[3];
}
