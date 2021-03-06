//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//测试硬件：单片机STM32H743IIT6,正点原子Apollo STM32F4/F7开发板,主频400MHZ，晶振12MHZ
//QDtech-TFT液晶驱动 for STM32 FSMC
//xiao冯@ShenZhen QDtech co.,LTD
//公司网站:www.qdtft.com
//淘宝网站：http://qdtech.taobao.com
//wiki技术网站：http://www.lcdwiki.com
//我司提供技术支持，任何技术问题欢迎随时交流学习
//固话(传真) :+86 0755-23594567 
//手机:15989313508（冯工） 
//邮箱:lcdwiki01@gmail.com    support@lcdwiki.com    goodtft@163.com 
//技术支持QQ:3002773612  3002778157
//技术交流QQ群:324828016
//创建日期:2018/08/09
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 深圳市全动电子技术有限公司 2018-2028
//All rights reserved
/****************************************************************************************************
//此模块可以直接插入正点原子Apollo STM32F4/F7开发板TFTLCD插槽，无需手动接线
//STM32连接引脚是指TFTLCD插槽引脚内部连接的STM32引脚
//=========================================电源接线================================================//
//     LCD模块             TFTLCD插槽引脚        STM32连接引脚
//      VDD       --->         5V/3.3              DC5V/3.3V          //电源
//      GND       --->          GND                  GND              //电源地
//=======================================液晶屏数据线接线==========================================//
//本模块默认数据总线类型为16位并口总线
//     LCD模块             TFTLCD插槽引脚        STM32连接引脚
//      DB0       --->          D0                   PD14        -|   
//      DB1       --->          D1                   PD15         |  
//      DB2       --->          D2                   PD0          | 
//      DB3       --->          D3                   PD1          | 
//      DB4       --->          D4                   PE7          |
//      DB5       --->          D5                   PE8          |
//      DB6       --->          D6                   PE9          |
//      DB7       --->          D7                   PE10         |
//如果使用8位模式，请使用下面高8位并口数据引脚                    |===>液晶屏16位并口数据信号
//      DB8       --->          D8                   PE11         |
//      DB9       --->          D9                   PE12         |
//      DB10      --->          D10                  PE13         |
//      DB11      --->          D11                  PE14         |
//      DB12      --->          D12                  PE15         |
//      DB13      --->          D13                  PD8          |
//      DB14      --->          D14                  PD9          |
//      DB15      --->          D15                  PD10        -|
//=======================================液晶屏控制线接线==========================================//
//     LCD模块 				     TFTLCD插槽引脚        STM32连接引脚 
//       WR       --->          WR                   PD5             //液晶屏写数据控制信号
//       RD       --->          RD                   PD4             //液晶屏读数据控制信号
//       RS       --->          RS                   PD13            //液晶屏数据/命令控制信号
//       RST      --->          RST                复位引脚          //液晶屏复位控制信号
//       CS       --->          CS                   PD7             //液晶屏片选控制信号
//       BL       --->          BL                   PB5             //液晶屏背光控制信号
//=========================================触摸屏触接线=========================================//
//如果模块不带触摸功能或者带有触摸功能，但是不需要触摸功能，则不需要进行触摸屏接线
//	   LCD模块             TFTLCD插槽引脚        STM32连接引脚 
//      PEN       --->          PEN                  PH7             //触摸屏触摸中断信号
//      MISO      --->          MISO                 PG3             //触摸屏SPI总线读信号
//      MOSI      --->          MOSI                 PI3             //触摸屏SPI总线写信号
//      T_CS      --->          TCS                  PI8             //触摸屏片选控制信号
//      CLK       --->          CLK                  PH6             //触摸屏SPI总线时钟信号
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

SRAM_HandleTypeDef SRAM_Handler;    //SRAM句柄(用于控制LCD)
	   
//管理LCD重要参数
//默认为竖屏
_lcd_dev lcddev;

//画笔颜色,背景颜色
u16 POINT_COLOR = 0x0000,BACK_COLOR = 0xFFFF;  
u16 DeviceCode;	 

u16 LCD_read(void)
{
	vu16 data;			//防止被优化
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
	data=data;		//使用-O2优化的时候,必须插入的延时
#if LCD_USE8BIT_MODEL
	LCD->LCD_REG=(data<<8); //写入要写的寄存器序号
#else
	LCD->LCD_REG=data; //写入要写的寄存器序号
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
	data=data;			//使用-O2优化的时候,必须插入的延时
#if LCD_USE8BIT_MODEL
	LCD->LCD_RAM=(data<<8);//写入要写的数据
#else
	LCD->LCD_RAM=data; //写入要写的数据
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
	LCD->LCD_REG = (LCD_Reg<<8);		//写入要写的寄存器序号	 
	LCD->LCD_RAM = (LCD_RegValue<<8);//写入数据
#else
	LCD->LCD_REG = LCD_Reg;		//写入要写的寄存器序号	 
	LCD->LCD_RAM = LCD_RegValue;//写入数据	    		 
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
	 LCD->LCD_RAM = Data<<8;  //写十六位颜色值
#else
	 LCD->LCD_RAM = Data;  //写十六位颜色值
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
	delay_us(1);//延时1us
	//8bit:red data	
	//16bit:red and green data
	r = LCD_RD_DATA();
	delay_us(1);//延时1us	
	//8bit:green data
	//16bit:blue data
	g = LCD_RD_DATA();
#if LCD_USE8BIT_MODEL
	delay_us(1);//延时1us	
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
	LCD_SetCursor(x,y);//设置光标位置 
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
		return 0;	//超过了范围,直接返回	
	}
	LCD_SetCursor(x,y);//设置光标位置 
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
		LCD->LCD_RAM = Color<<8;//写十六位颜色值
#else
		LCD->LCD_RAM = Color; //写十六位颜色值
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

	HAL_MPU_Disable();							//配置MPU之前先关闭MPU,配置完成以后在使能MPU	
	//外部SRAM为region0，大小为2MB，此区域可读写
	MPU_Initure.Enable=MPU_REGION_ENABLE;	    //使能region
	MPU_Initure.Number=LCD_REGION_NUMBER;		//设置region，外部SRAM使用的region0
	MPU_Initure.BaseAddress=LCD_ADDRESS_START;	//region基地址
	MPU_Initure.Size=LCD_REGION_SIZE;			//region大小
	MPU_Initure.SubRegionDisable=0X00;
	MPU_Initure.TypeExtField=MPU_TEX_LEVEL0;
	MPU_Initure.AccessPermission=MPU_REGION_FULL_ACCESS;	//此region可读写
	MPU_Initure.DisableExec=MPU_INSTRUCTION_ACCESS_ENABLE;	//允许读取此区域中的指令
	MPU_Initure.IsShareable=MPU_ACCESS_NOT_SHAREABLE;
	MPU_Initure.IsCacheable=MPU_ACCESS_NOT_CACHEABLE;
	MPU_Initure.IsBufferable=MPU_ACCESS_BUFFERABLE;
	HAL_MPU_ConfigRegion(&MPU_Initure);
	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);     //开启MPU
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
	
	__HAL_RCC_FMC_CLK_ENABLE();				//使能FMC时钟
	__HAL_RCC_GPIOD_CLK_ENABLE();			//使能GPIOD时钟
	__HAL_RCC_GPIOE_CLK_ENABLE();			//使能GPIOE时钟
	
	//初始化PD0,1,4,5,7,8,9,10,13,14,15
	GPIO_Initure.Pin=GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_8|\
                     GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
	GPIO_Initure.Mode=GPIO_MODE_AF_PP; 		//复用
	GPIO_Initure.Pull=GPIO_PULLUP;			//上拉
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;	//高速
	GPIO_Initure.Alternate=GPIO_AF12_FMC;	//复用为FMC
	HAL_GPIO_Init(GPIOD,&GPIO_Initure);
	
	//初始化PE7,8,9,10,11,12,13,14,15
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
		__HAL_RCC_GPIOB_CLK_ENABLE();			//开启GPIOB时钟
    GPIO_Initure.Pin=GPIO_PIN_5;           //PB5,背光控制
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;    //高速
    HAL_GPIO_Init(GPIOB,&GPIO_Initure); 
    LCD_MPU_Config();                       //使能MPU保护LCD区域
    SRAM_Handler.Instance=FMC_NORSRAM_DEVICE; //BANK1    
    SRAM_Handler.Extended=FMC_NORSRAM_EXTENDED_DEVICE;         
              
        
    SRAM_Handler.Init.NSBank=FMC_NORSRAM_BANK1;     //使用NE1
    SRAM_Handler.Init.DataAddressMux=FMC_DATA_ADDRESS_MUX_DISABLE;  //不复     用数据线
    SRAM_Handler.Init.MemoryType=FMC_MEMORY_TYPE_SRAM;              //SRAM
		SRAM_Handler.Init.MemoryDataWidth=FMC_NORSRAM_MEM_BUS_WIDTH_16; //16位数据宽度
    SRAM_Handler.Init.BurstAccessMode=FMC_BURST_ACCESS_MODE_DISABLE; //是否使能突发访问,仅对同步突发存储器有效,此处未用到
    SRAM_Handler.Init.WaitSignalPolarity=FMC_WAIT_SIGNAL_POLARITY_LOW;//等待信号的极性,仅在突发模式访问下有用
    SRAM_Handler.Init.WaitSignalActive=FMC_WAIT_TIMING_BEFORE_WS;   //存储器是在等待周期之前的一个时钟周期还是等待周期期间使能NWAIT
    SRAM_Handler.Init.WriteOperation=FMC_WRITE_OPERATION_ENABLE;    //存储器写使能
		SRAM_Handler.Init.WaitSignal=FMC_WAIT_SIGNAL_DISABLE;           //等待使能位,此处未用到
    SRAM_Handler.Init.ExtendedMode=FMC_EXTENDED_MODE_ENABLE;        //读写使用不同的时序
    SRAM_Handler.Init.AsynchronousWait=FMC_ASYNCHRONOUS_WAIT_DISABLE;//是否使能同步传输模式下的等待信号,此处未用到
    SRAM_Handler.Init.WriteBurst=FMC_WRITE_BURST_DISABLE;           //禁止突发写
    SRAM_Handler.Init.ContinuousClock=FMC_CONTINUOUS_CLOCK_SYNC_ASYNC;
        
    //FSMC读时序控制寄存器
    FSMC_ReadWriteTim.AddressSetupTime=0x11;    //地址建立时间(ADDSET)为17个HCLK 1/216M=4.6ns*17=78ns
    FSMC_ReadWriteTim.AddressHoldTime=0x00;
    FSMC_ReadWriteTim.DataSetupTime=0x55;       //数据保存时间(DATAST)为85个HCLK	=4.6*85=391ns
    FSMC_ReadWriteTim.AccessMode=FMC_ACCESS_MODE_A; //模式A
    //FSMC写时序控制寄存器
    FSMC_WriteTim.AddressSetupTime=0x15;        //地址建立时间(ADDSET)为21个HCLK=96ns
    FSMC_WriteTim.AddressHoldTime=0x00;
    FSMC_WriteTim.DataSetupTime=0x15;           //数据保存时间(DATAST)为4.6ns*21个HCLK=96ns
    FSMC_WriteTim.AccessMode=FMC_ACCESS_MODE_A;     //模式A
		
    HAL_SRAM_Init(&SRAM_Handler,&FSMC_ReadWriteTim,&FSMC_WriteTim);	
		//重新配置写时序控制寄存器的时序   	 							    
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
	LCD_GPIOInit();//LCD GPIO初始化	
//************* ILI9341初始化**********//	
//*************3.2inch ILI9341初始化**********//	
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

  LCD_direction(USE_HORIZONTAL);//设置LCD显示方向
	LCD_LED_SET();//点亮背光	 
	LCD_Clear(WHITE);//清全屏白色
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

	LCD_WriteRAM_Prepare();	//开始写入GRAM			
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
