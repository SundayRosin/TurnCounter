/* MAIN.C file
 * 
 * Copyright (c) 2002-2005 STMicroelectronics
 */
#include "stm8l15x.h"
#include "stm8l15x_clk.h"
#include "stm8l15x_tim2.h"
//#include "stm8l15x_tim4.h"
#include "binary.h"


__near __no_init const unsigned char Factory_VREFINT @ 0x4910;

uint8_t power=1;
uint8_t tftData[10]={0,0,0,11,10,0,0,0,0,0};//массив для дисплея
uint8_t dec[5]={0,0,0,0,0}; //массив для преобразования 
uint16_t counter=0;
uint8_t buffer[2];
uint16_t recv_array[2];

uint32_t Usumma,U=0;
uint16_t viborka;

uint8_t znakogen[12]=
{
0xEE,//0b11101110, //0
0x60, //1
0x2F, //2
0x6D, //3
0xE1, //4
0xCD, //5
0xCF, //6
0x68, //7
0xEF, //8
0xED, //9
0, //blank
B01000110//0b00000001, //-
//0b11101011, //A
//0b11000111, //b
//0b10001110, //c
//0b01100111, //d
//0b10001111, //e
//0b10001011, //f
//0b10000110, //L
//0b10001010, //A
//0b11100011, //I

};

void delay(void)
{
uint32_t delay_count;
for ( delay_count=0; delay_count<10000; delay_count++){};


}

void delayka(uint8_t i)
{  
	   for (i=0;i<255;i++)
		  {;
			};
	
}


//Установка данных на шине
void setBus(uint8_t data)
{
	uint8_t a;
	//Установка данных на шине
			GPIOB->ODR&=B00011111; //сбрасываю в нуль db0  db1 db2 
			GPIOC->ODR&=B10111111; //db3=0
			a=data<<5;
			//a&=0b11100000;
			GPIOB->ODR|=a;
			a=data<<3;
			a&=B01000000;
			GPIOC->ODR|=a;
	
}

void writeTft(void)
{
                        GPIOC->ODR&=B11011111; //PC5=WR=0
                        GPIOC->ODR|=B00100010; //WR=1
			GPIOC->ODR&=B11011111; //PC1=WR=0	
			
}

//разблокировка дисплея после включения
void unlock(void)
{
	setBus(0x0F);
	GPIOC->ODR&=B11101111; //A0=PC4=0
      	writeTft();
       	setBus(0x01);	
         GPIOC->ODR|=B00010000; //A0=PC4=1
	 writeTft();
	
}


void tft(uint8_t segs[10])
{
       uint8_t i,a;
	
	//устанавливаем адресс знакоместа
	setBus(0);
	GPIOC->ODR&=B11101111; //A0=PC4=0
	writeTft();
	GPIOC->ODR|=B00010000; //A0=PC4=1
			
	  for (i=0;i<10;i++)
		{
                  a=znakogen[segs[i]];
                  if (i==0){a+=B00010000;};
                  //точка в 0м разр.  
               
		setBus(a&B00001111); //SG(L)	
		writeTft();
		setBus(a>>4);
		writeTft();	
                
		}
	
	
	
	
}
//основная тактовая 1мгц
void mainCLK(void)
{
  //Переключаем тактовую частоту в автоматическом режиме
	 CLK->SWCR |= CLK_SWCR_SWEN;
	 CLK->SWR = CLK_SYSCLKSource_HSI;//выбираем hsi
	 CLK->CKDIVR=CLK_SYSCLKDiv_1;//CLK_SYSCLKDiv_1; // /1
   while ((CLK->SCSR)!=CLK_SYSCLKSource_HSI) {}; 
	 
  
}

//переключение на LSI генератор
void lsiCLK(void)
{
    CLK->CKDIVR=CLK_SYSCLKDiv_1;
    CLK->SWR = CLK_SYSCLKSource_LSI;//выбираем lsi
    //Переключаем тактовую частоту в автоматическом режиме
    CLK->SWCR |= CLK_SWCR_SWEN;
    while ((CLK->SCSR)!=CLK_SYSCLKSource_LSI) {}; 
    CLK->ICKCR&=~CLK_ICKCR_HSION;
}

//расчитывает напрежение питания
uint16_t getV(uint16_t U)
{
  uint16_t V;
  uint32_t a,Ref;
   
     Ref=0x600+Factory_VREFINT;//значение опорного напряжения (1667 при 3vref)
     a=Ref*300;       
      if (U>0)
      { V=a/U;} else {V=0;};
      return (V);
      
}

//преобразовует двочное число в десятичное
void hexdec(uint16_t x)
{
        dec[0]=0;
        dec[1]=0;
        dec[2]=0;
        dec[3]=0;
        dec[4]=0;
        while(x>=10000)
        {dec[4]++; x-=10000;
        }
        while(x>=1000)
        {dec[3]++; x-=1000;
        }
        while(x>=100)
        {dec[2]++; x-=100;
        }
         while(x>=10)
        {dec[1]++; x-=10;
        }
        while(x>=1)
        {dec[0]++; x-=1;
        }
        
}

//инициализация выходов
void initOut(void)
{
   //Инициализируем порты 
  
   GPIOA->DDR=0;
   GPIOB->DDR=B11100010; 
   GPIOC->DDR=B01110000; //назначаем выходы
   GPIOD->DDR=0;
   
   GPIOA->CR1=0xFF;
   GPIOC->CR1=0xFF;// выходы в режиме push pull
   GPIOB->CR1=0xff;//B11111010;//входы подятнуты к резисторам 
   GPIOD->CR1=0xFF;//
   GPIOE->CR1=0xFF;
}

void deinitOut(void)
{
  GPIOA->ODR=0;  //выключаю дисплей и датчики и ставлю на всех выходах 0
         GPIOB->ODR=0; //что бы не было кз
         GPIOC->ODR=0;
         GPIOD->ODR=0;
         
         GPIOA->DDR=0xFF; //делаю выходами все не задействованные выводы
         //иначе ловит помехи от рук,и ток поребления плавает от 5 до 15мкА
         GPIOB->DDR=B11101111;//тут кнопка PB4
         GPIOC->DDR=0xFF;
         GPIOD->DDR=0xFF;
}

//Настраиваем TIM2
void initTIM2(void)
{
    CLK->PCKENR1|=B00000001; //включаем тактирование TIM2
    TIM2->CCER1|=B00100010; //CC2P,CC1P: Capture/compare 2 output polarity
    TIM2->CCMR1|=B00000001; //CC1 channel is configured as input, IC1 is mapped on TI1FP1
    TIM2->CCMR2|=B00000001;//CC2 channel is configured as input, IC2 is mapped on TI2FP2
    TIM2->SMCR|=B00000001; //Set encoder mode
    TIM2->CR1|=TIM_CR1_CEN; //включаем таймер
  
  
}


int main(void)
{
   mainCLK();//включаем HSI,тактовая частота 16мгц
   initOut();//назначаем какие ножки выходы 
       
   //Включаю ЖКИ и датчики
   GPIOB->ODR|=B00000010;
   delay();
   initTIM2();
   
   
   //Настраиваем TIM4
   CLK->PCKENR1|=B00000100;
   TIM4->PSCR|=0x09;//512
   TIM4->IER|=B00000001; //разрешаем прерывания по переполнению
   TIM4->CR1|=B00000001; //включаем таймер
   
   //Настраиваю внешние прерывания для кнопки On/Off
   EXTI->CONF1|=B00000010;//PB[7:4] are used for EXTIB interrupt generation
   GPIOB->CR2|=B00010000;//активируем прерывания PB4
   
   //Настраиваю АЦП
   CLK->PCKENR2|=B00000001;//включаю тактирование АЦП PCKEN20 ADC1  
   ADC1->CR1 |= ADC_CR1_ADON;
   ADC1->SQR[0]=B10010000;//отключаю дма,включаю  канал Uref
   ADC1->TRIGR[0]|=ADC_TRIGR1_VREFINTON;//включаю опорное
   ADC1->CR2|=B00000111;//sample time fo 0-24 channals 111: 384 ADC clock cycles
   ADC1->CR1 |= ADC_CR1_START; //запускаем преобразование

   asm("RIM"); //разрешаем прерывания //  enableInterrupts();
   unlock(); //разблокирую дисплей,см. даташит МЭЛТ МТ-10t7-8-9   

	
loop:	
			
       //преход в спящий режим, ток портебления при 3v ~ 1.7мКа.      
        if (power==0)
        {
         buffer[0]=TIM2->CNTRL; //cохраняем значения 
         buffer[1]=TIM2->CNTRH;
         
         deinitOut();//порты подтягиваем к земле
                  
         ADC1->TRIGR[0]&=~ADC_TRIGR1_VREFINTON;//выключаю опорное
         ADC1->CR1 &= ~ADC_CR1_ADON;//выключаю АЦП
         halt(); //засыпаю
         EXTI->SR2|=B00000001;//очистка флага прерывания
         initOut(); //делаю нужные выводы вых,остальное входы
      
         ADC1->CR1 |= ADC_CR1_ADON;//включаю ацр
         ADC1->TRIGR[0]|=ADC_TRIGR1_VREFINTON;//включаю опорное
         GPIOB->ODR|=B00000010;///Включаю ЖКИ и датчики
         delay();
         ADC1->CR1 |= ADC_CR1_START; //запускаю преобразование      
         TIM2->CNTRL=buffer[0];
         TIM2->CNTRH=buffer[1];
                 
         power=1; //взвожу флаг
        }
        
        //Работа ацп
        
	 //Если ацп выполнил преобразование
         if ((ADC1->SR && ADC_SR_EOC) == 1)
         {
             
             if (viborka==16)
             {
               viborka=ADC1->DRL; viborka=0;
               U=Usumma/16;
               Usumma=0;
               ADC1->CR1 |= ADC_CR1_START; //сново запускаем преобразование
             }
             else
             {
               uint16_t temp=0;
               temp=(ADC1->DRH)<<8;
               temp=temp+ADC1->DRL;
               Usumma=Usumma+temp;
               viborka++;
               ADC1->CR1 |= ADC_CR1_START; //сново запускаем преобразование
               
             }
           
           
         }
            
	goto loop;	
	
	
}


void test(uint8_t y)
{
  tftData[0]=y;
    
        
	tft(tftData);
  
}


INTERRUPT_HANDLER(TIM4_IRQHandler, 25)

{
    TIM4->SR1=0;//флаг сбрасываем
 if ( ((GPIOD->IDR)&B00000001)==0 ) 
 {TIM2->CNTRL=0;TIM2->CNTRH=0;} //кнопка сброс
 
  counter=TIM2->CNTRH<<8; 
  counter=counter+TIM2->CNTRL;
        hexdec(counter/2);        
        
        tftData[9]=dec[0];
        tftData[8]=dec[1];
        tftData[7]=dec[2];
        tftData[6]=dec[3];
        tftData[5]=dec[4];
        
        hexdec(getV(U));
        tftData[0]=dec[2];
        tftData[1]=dec[1];
        tftData[2]=dec[0];
             
	tft(tftData);
  
}





INTERRUPT_HANDLER(TLI_IRQHandler,0 )
{

}

INTERRUPT_HANDLER(FLASH_IRQHandler,1 )
{

}

INTERRUPT_HANDLER(DMA1_IRQHandler,2 )
{
 
 DMA1_Channel0->CSPR &= ~DMA_CSPR_TCIF;
}

INTERRUPT_HANDLER(DMA12_IRQHandler,3 )
{

   DMA1_Channel0->CSPR &= ~DMA_CSPR_TCIF;
}
INTERRUPT_HANDLER(RTC_IRQHandler,4 )
{
test(1);
}

INTERRUPT_HANDLER(PVD_IRQHandler,5 )
{
}
//прерывание от кнопки ON/OFF
INTERRUPT_HANDLER(EXTIB_IRQHandler,6 ) //External interrupt port B/G
{

   EXTI->SR2|=B00000001;//jочищаем флаг
   while( ((GPIOB->IDR)&B00010000)==0 ){}; //пока кнопка нажата сидим и ждем пока отпустят
  power=0;
   
}

INTERRUPT_HANDLER(EXTID_IRQHandler,7 )
{
  test(3);
}

INTERRUPT_HANDLER(EXTI0_IRQHandler,8 )
{
   test(4); //!!!
   EXTI->SR1=0;  EXTI->SR2=0; 
}

INTERRUPT_HANDLER(EXTI1_IRQHandler,9 )
{
   test(5);
}

INTERRUPT_HANDLER(EXTI2_IRQHandler,10 )
{
   test(6);  //!!!
   EXTI->SR1=0;  EXTI->SR2=0; 
}

INTERRUPT_HANDLER(EXTI3_IRQHandler,11 )
{
   test(7);
}

INTERRUPT_HANDLER(EXTI4_IRQHandler,12 )
{
   test(8);
}

INTERRUPT_HANDLER(EXTI5_IRQHandler,13 )
{
   test(9);
}

INTERRUPT_HANDLER(EXTI6_IRQHandler,14 )
{
   test(1);
}

INTERRUPT_HANDLER(EXTI7_IRQHandler,15 )
{
   test(2);
}
//16 резервное
INTERRUPT_HANDLER(CLK_IRQHandler,17 )
{
   test(3);
}

INTERRUPT_HANDLER(ADC1_IRQHandler,18 )
{
   test(7);   //!!
}

INTERRUPT_HANDLER(TIM2_IRQHandler,19 )
{
   test(5);
}

INTERRUPT_HANDLER(TIM2_U_IRQHandler,20 )
{
   test(8); ///!
}

INTERRUPT_HANDLER(TIM3_IRQHandler,21 )
{
   test(7);
}

INTERRUPT_HANDLER(TIM3_C_IRQHandler,22 )
{
   test(8);
}

INTERRUPT_HANDLER(RI_IRQHandler,23 )
{
   test(9);
}

INTERRUPT_HANDLER(SPI1_IRQHandler,26 )
{
   test(1);
}

INTERRUPT_HANDLER(UASRT_IRQHandler,27 )
{
   test(2);
}

INTERRUPT_HANDLER(USART_R_IRQHandler,28 )
{
   test(3);
}


INTERRUPT_HANDLER(I2C_IRQHandler,29 )
{
   test(5);
}



