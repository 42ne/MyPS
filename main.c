//новый коментарий

#include <io.h>
#include <interrupt.h>
#include "avrlibdefs.h"
#include "avrlibtypes.h"
#include <math.h>


#define btn1	5
#define btn2	6
#define btn3	7
#define BTNPIN PIND
	

#define GREEN    0	// LED1
#define YELLOW 1	// LED2
#define RED  2	// LED3
#define LEDPORT PORTD
#define redon      cbi(LEDPORT,RED)
#define yellowon    cbi(LEDPORT,YELLOW)
#define greenon    cbi(LEDPORT,GREEN)

#define redoff      sbi(LEDPORT,RED)
#define yellowoff    sbi(LEDPORT,YELLOW)
#define greenoff   sbi(LEDPORT,GREEN)


#define dig1on cbi (PORTG,0)
#define dig2on cbi (PORTG,1)
#define dig3on cbi (PORTG,2)
#define dig1off sbi (PORTG,0)
#define dig2off sbi (PORTG,1)
#define dig3off sbi (PORTG,2)

#define ddoton cbi (PORTA,7)
#define udoton cbi (PORTC,7) 
#define ddotoff sbi (PORTA,7)
#define udotoff sbi (PORTC,7)


const float Vref=4.99; // опорное напряжение АЦП
//коэффициенты деления для соотв. каналов АЦП

const float kVSet=0.4; //ADC6
const float kVSense=0.167; //ADC0
 
u08 digtable[10]=
{
0b11000000,0b11111001,0b10100100,0b10110000,0b10011001,
0b10010010,0b10000010,0b11111000,0b10000000,0b10010000
};

u08 upnum[6],botnum[6]; //разряды и точки (можно оптимизировать точки, чтобы занимали один байт)
u08 key,countled;   //биты состояния клавиш (4 младших бита значащие)
// бит 0 = 1 - не используется
// бит 1 =1 - нажата кл.1 (необработанное событие)
// бит 2 =1 - нажата кл.2 (необработанное событие)
// бит 3 =1 - нажата кл.3 (необработанное событие)

float MCSet,MVSet,M18V,M12VRef,MTherm1,MTherm2,MCSense,MVSense;
u08 FanSpeed=0;
u08 PWMEnable=0; //включенный ШИМ
u08 DispMode=0;       //режим отображения дисплея
u08 CurLimMode=1; //режим ограничения тока если true, иначе - режим отключения при перегрузке по току

void init(void);
void setdig (u08 number, u08 pos);
void DispFloat (float fnum,u08 *s);
float MeasureVoltageSet (void);
float MeasureVoltageSense (void);
float MeasureTemp2(void);
void KeyHandling(void);
inline float ADCRead(u08 channel);
//==============================================================================================================================
//
// main function
//
//==============================================================================================================================

int main (void)
 {
	init();

while (1) 
	{
		
		
	float voltage=MeasureVoltageSet(); 
	DispFloat (voltage,upnum);
	voltage=MeasureTemp2(); 
	DispFloat (voltage,botnum);
	KeyHandling();
	
	
	}

}
//==============================================================================================================================
//
// Initialization
//
//==============================================================================================================================
void init(void)
{
DDRD=0b00000111;
DDRG=0b00000111;
DDRA=DDRC=0xff;
PORTA=PORTC=0xff;
DDRF=0x00;
PORTF=0x00;

             TCCR0 = 0x8e;               // Разрешение прерываний и инициализация таймера
             SREG = 0x80;               
             TIMSK = 2;                  
             OCR0 = 30;   

key=countled=0;
}
//==============================================================================================================================
inline float ADCRead(u08 channel)
{
                 ADMUX = channel&0x07;                   // 10-bit right adjust ADC3
                 ADCSR = 0xc7;                   // start ADC
			while(ADCSR & 0x40)
			;   
float result=((float)(ADCH*256)+ADCL)*Vref/1024;
return result;
}
//==============================================================================================================================
void KeyHandling(void)
{
	//обработка клавиш
	  if (key&(1<<1)) //нажата клавиша 1
	  {
			countled++;
			if (countled>2)
			countled=0;
		cbi(key,1); // событие уже обработано, установить соотв. флаг
	  }
	  if (key&(1<<2)) //нажата клавиша 2
	  {
			if (countled>0)
			countled--;
			else
			countled=2;
	  cbi(key,2); // событие уже обработано, установить соотв. флаг
	  }
	  if (key&(1<<3)) //нажата клавиша 3
	  {
		switch(countled)
		{
		case 0:
		countled=2;
		break;
		
		case 1:
		countled=0;
		break;
		
		case 2:
		countled=1;
		break;
		
		default:
		break;
		}
	  cbi(key,3); // событие уже обработано, установить соотв. флаг
	  }
	  	
		switch (countled)
		{
		case 0:
		redon;
		yellowoff;
		greenoff;
		break;
		
		case 1:
		redoff;
		yellowon;
		greenoff;
		break;
		
		case 2:
		redoff;
		yellowoff;
		greenon;
		break;
		
		default:
			break;
		}	  
	  } 

//==============================================================================================================================
//измерение температуры с датчика 2 с усреднением,количество выборок определяется константой vsamples,
// для того чтобы процедура вернула измененное значение, ее необходимо вызвать не менее
// vsamples раз.
float MeasureTemp2(void)
{
const u08 vsamples=100;

const float A1=3.354E-03;
const float B1=2.744E-04;
const float C1=3.667E-06;
float tmp;
static float sum=0,lastresult=0,result=0,U;
static u08 vsamplecount=0;
                ADMUX = 0x03;                   // 10-bit right adjust ADC3
                 ADCSR = 0xc7;                   // start ADC
			while(ADCSR & 0x40)
			;   
	u08 lbyte=ADCL;
	u08 hbyte =ADCH; 
		U=((float)(hbyte*256)+lbyte)*Vref/1024;
		
//		U=ADCRead(3);
		tmp=log(U/(Vref-U));
		result=1/(A1+B1*tmp+C1*tmp*tmp)-273.15;
		
		sum=sum+result;
if (vsamplecount>=vsamples)
	{
	result=sum/(vsamplecount+1);
	
		lastresult=result;
		vsamplecount=0;
		sum=0;
		return result;
	}
	else
	{
	vsamplecount++;
	return lastresult;
	}
return 0;
	
}
//==============================================================================================================================
//измерение напряжения с усреднением,количество выборок определяется константой vsamples,
// для того чтобы процедура вернула измененное значение, ее необходимо вызвать не менее
// vsamples раз.
float MeasureVoltageSet (void)
{
const u08 vsamples=100;
static float sum=0,lastresult=0,result=0;
static u08 vsamplecount=0;
                ADMUX = 0x06;                   // 10-bit right adjust ADC6
                 ADCSR = 0xc7;                   // start ADC
			while(ADCSR & 0x40)
			;   
	u08 lbyte=ADCL;
	u08 hbyte =ADCH; 
		result=((float)(hbyte*256)+lbyte)*Vref*(1/kVSet)/1024;
//	result=ADCRead(6)/kVSet;
		sum=sum+result;

if (vsamplecount>=vsamples)
	{
	result=sum/(vsamplecount+1);
	
		lastresult=result;
		vsamplecount=0;
		sum=0;
		return result;
	}
	else
	{
	vsamplecount++;
	return lastresult;
	}
return 0;
	
}

//==============================================================================================================================
//измерение напряжения с усреднением,количество выборок определяется константой vsamples,
// для того чтобы процедура вернула измененное значение, ее необходимо вызвать не менее
// vsamples раз.
float MeasureVoltageSense(void)
{
const u08 vsamples=100;
static float sum=0,lastresult=0,result=0;
static u08 vsamplecount=0;
              ADMUX = 0x00;                   // 10-bit right adjust ADC0
                ADCSR = 0xc7;                   // start ADC
			while(ADCSR & 0x40)
			;   
	u08 lbyte=ADCL;
	u08 hbyte =ADCH; 
		result=((float)(hbyte*256)+lbyte)*Vref*(1/kVSense)/1024;
//		result=ADCRead(0);
		sum=sum+result;

if (vsamplecount>=vsamples)
	{
	result=sum/(vsamplecount+1);
	
		lastresult=result;
		vsamplecount=0;
		sum=0;
		return result;
	}
	else
	{
	vsamplecount++;
	return lastresult;
	}
return 0;
	
}
//==============================================================================================================================
// вывод цифры в позицию
void setdig (u08 number, u08 pos)
{
if (pos==1)
	{
	PORTC=PORTC&0x80|digtable[number];
	}
if (pos==2)
	{
	PORTA=PORTA&0x80|digtable[number];
	}
}


//==============================================================================================================================
// отображение на индикаторе числа fnum с точкой в соотв.позицию *s (upnum/botnum)
void DispFloat (float fnum,u08 *s)
{
u08 count=0;
u16 inum;
u08 power;
float tempnum;
tempnum=fnum;
power=100;

// Проверка границ
if (fnum>999)
	fnum=999;

// Определение степени	
while (tempnum>=10)
	{
	tempnum=tempnum/10;
	count++;
	power=power/10;
	}

//очистка всех точек
s[3]=s[4]=s[5]=0;
// постановка точки
s[3+count]=1;


// Вывод числа без учета точки
inum=fnum*power;	

count=0;
s16 num=0;
power=1;
s16 digit;

num=inum;
	/* Определение степени числа*/
while (num>=10)
	{
	num=num/10;
	count++;
	power=power*10;
	}
num=inum;
	/* Составление строки */
u08 pos=2-count;


for (digit=0; digit<=2;digit++)
	{
	if (digit>=pos)
		{
		s[digit]=(u08)(num/power);
		num=num%power;
		power=power/10;
		}
		else
		{
		s[digit]=0;
		}
	}

}

//==============================================================================================================================
ISR(TIMER0_COMP_vect)
{


static u08 countdig=0;

// опрос клавиатуры
static u16 pushcount[3]={0,0,0};
static u16 releasecount[3]={0,0,0};
const u16 delaykey=50;


if (BTNPIN&(1<<btn1)) //отпущена кн.1
		//redoff;
		{
		if ((releasecount[0]<65535)&&(!(key&(1<<1))))
		releasecount[0]++;
		
		if ((pushcount[0]>delaykey)&&(!(key&(1<<1))))
			{
			cbi (key,1); //произошла смена состояния клавиши 
			releasecount[0]=pushcount[0]=0;
			}
		}
	else			   //нажата кн.1
		//redon;
		{
		if ((pushcount[0]<65535)&&(!(key&(1<<1))))
		pushcount[0]++;
		
		if ((releasecount[0]>delaykey)&&(!(key&(1<<1))))
			{
			sbi (key,1); //произошла смена состояния клавиши 
			releasecount[0]=pushcount[0]=0;
			}
		}
if (BTNPIN&(1<<btn2)) //отпущена кн.2
		//yellowoff;
		{
		if ((releasecount[1]<65535)&&(!(key&(1<<2))))
		releasecount[1]++;

		if ((pushcount[1]>delaykey)&&(!(key&(1<<2))))
			{
			cbi (key,2); //произошла смена состояния клавиши 
			releasecount[1]=pushcount[1]=0;
			}		
		
		}
	else			   //нажата кн.2
		//yellowon;
		{
		if ((pushcount[1]<65535)&&(!(key&(1<<2))))
		pushcount[1]++;
		
		if ((releasecount[1]>delaykey)&&(!(key&(1<<2))))
			{
			sbi (key,2); //произошла смена состояния клавиши 
			releasecount[1]=pushcount[1]=0;
			}
	
		}
if (BTNPIN&(1<<btn3)) //отпущена кн.3
		//greenoff;
		{
		if ((releasecount[2]<65535)&&(!(key&(1<<3))))
		releasecount[2]++;
		
		if ((pushcount[2]>delaykey)&&(!(key&(1<<3))))
			{
			cbi (key,3); //произошла смена состояния клавиши 
			releasecount[2]=pushcount[2]=0;
			}				
		}
	else			   //нажата кн.3
		//greenon;
		{
		if ((pushcount[2]<65535)&&(!(key&(1<<3))))
		pushcount[2]++;

		if ((releasecount[2]>delaykey)&&(!(key&(1<<3))))
			{
			sbi (key,3); //произошла смена состояния клавиши 
			releasecount[2]=pushcount[2]=0;
			}		
		}
	
	
switch (countdig)
{
	case 0:
		dig1on;
		dig2off;
		dig3off;
		setdig(upnum[0],1);
		setdig(botnum[0],2);
		// размещение точки
		if (!upnum[3])
			udotoff;
			else
			udoton;		
		if (!botnum[3])
			ddotoff;
			else
			ddoton;		
	
	break;
	case 1:
		dig1off;
		dig2on;
		dig3off;
		setdig(upnum[1],1);
		setdig(botnum[1],2);		
		// размещение точки
		if (!upnum[4])
			udotoff;
			else
			udoton;		
		if (!botnum[4])
			ddotoff;
			else
			ddoton;				
	break;
	case 2:
		dig1off;
		dig2off;
		dig3on;
		setdig(upnum[2],1);	
		setdig(botnum[2],2);	
		// размещение точки
		if (!upnum[5])
			udotoff;
			else
			udoton;		
		if (!botnum[5])
			ddotoff;
			else
			ddoton;			
	break;
	
	default:
	break;
}

 countdig++;
 if (countdig>2)
	countdig=0;
}

