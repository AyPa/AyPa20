//#include <avr/pgmspace.h>

//#define Pin2Input(port,pin){port&=~(1<<pin);}
//#define Pin2Output(port,pin){port|=(1<<pin);}
//#define Pin2HIGH(port,pin){port|=(1<<pin);}
//#define Pin2LOW(port,pin){port&=~(1<<pin);}
//#define NOP __asm__ __volatile__ ("nop\n\t")


extern uint32_t long timer0_millis;
uint32_t milli;
uint8_t HOUR;
uint16_t MINU;
uint32_t SECU;


uint16_t vcc;

void (* reboot) (void) = 0; //declare reset function @ address 0
byte Fan;

//#define FanON { __asm__ __volatile__("sbi 11,0\n\t"); FanState=1; }
//#define FanOFF { __asm__ __volatile__("cbi 11,0\n\t"); FanState=0; }

#define ADCon{ PRR&=~(1<<PRADC); ADCSRA|=(1<<ADEN); }
#define ADCoff{ ADCSRA&=~(1<<ADEN);   PRR|=(1<<PRADC);}
#define SerialON{ PRR&=~(1<<PRUSART0); Serial.begin(9600); }
#define SerialOFF{ Serial.end(); PRR|=(1<<PRUSART0);}

#define SetADC(bandgap,input,us){ ADCSRA|=(1<<ADEN);delayMicroseconds(2);ADMUX=(bandgap<<REFS1)|(1<<REFS0)|(0<<ADLAR)|input;delayMicroseconds(us);} // input (0..7,8,14) (bg/vcc analogReference )
#define mRawADC(v,p) { ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|p;do{}while(bit_is_set(ADCSRA,ADSC));v=(ADCH<<8)|ADCL; }
//#define mRawADC2(h,l,p) { ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|p;do{}while(bit_is_set(ADCSRA,ADSC));l=ADCL;h=ADCH;}
//#define mRawADC(v,p) ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|p;do{}while(bit_is_set(ADCSRA,ADSC));v=ADCW; 

byte volatile WDsleep;
byte volatile WDflag;

// Watchdog timeout values : //#define T16MS 0//#define T32MS 1//#define T64MS 2//#define T128MS 3//#define T250MS 4//#define T500MS 5//#define T1S 6
#define T2S 7//#define T4S 8//#define T8S 9
#define setup_watchdog(timeout){cli(); __asm__ __volatile__("wdr\n\t"); MCUSR&=~(1<<WDRF);WDTCSR|=(1<<WDCE)|(1<<WDE);WDTCSR=((1<<WDIE)|timeout);sei();}
 
ISR(WDT_vect) { 
if(WDsleep){WDflag=1; WDsleep=0; } else{ __asm__ __volatile__("call 0\n\t");}//reboot();}
//__asm__ __volatile__("call 0\n\t");
} // Watchdog timer interrupt

/*
void CheckPowerSupply(void)
{
        
      ADCon;
      ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);

      cli();milli=timer0_millis;sei();  
      HOUR = milli/3600000L;
       if (HOUR>=24){
     cli();timer0_millis-=86400000L;sei();  
     HOUR=0;
//         reboot();-не работает
     } 

//      SerialON;

  delayMicroseconds(900);  // Wait for Vref to settle
  ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIE)|2;  while (bit_is_set(ADCSRA,ADSC)); vcc = 1120300L / ADCW;  ADCoff;
  
  if (vcc<4600){  __asm__ __volatile__("sbi 5,5\n\t"); } else { __asm__ __volatile__("cbi 5,5\n\t"); }


//  Serial.println(milli);
  //Serial.println(HOUR);
//   Serial.println(tz);
   //Serial.println(t);
   }
*/

long nextm;



void  delay500ns(void) __attribute__((noinline)); 
void  delay500ns(void) { __asm__ __volatile__( "delay500:\n\t"    // "ret\n\t"
);} 
void  delay750ns(void) __attribute__((noinline)); 
void  delay750ns(void) {  __asm__ __volatile__( "delay750:\n\t"   "nop\n\t""nop\n\t"  "nop\n\t""nop\n\t"  //"ret\n\t"
);} 
void  delay1000ns(void) __attribute__((noinline)); 
void  delay1000ns(void) { __asm__ __volatile__( "delay1000:\n\t" "call delay500\n\t"   // "ret\n\t"
);} 
void  delay2500ns(void) __attribute__((noinline)); 
void  delay2500ns(void) {  __asm__ __volatile__( "delay2500:\n\t" 
"call delay1000\n\t"//"nop\n\t""nop\n\t"  "nop\n\t""nop\n\t"  //750"nop\n\t""nop\n\t"  "nop\n\t""nop\n\t" //1000
"call delay1000\n\t"//"nop\n\t""nop\n\t"  "nop\n\t""nop\n\t""nop\n\t""nop\n\t"  "nop\n\t""nop\n\t" //1500
//"ret\n\t"
);} 



void delay10500ns(void){__asm__ __volatile__( "delay10500:\n\t" 
"call delay2500\n\t"  "call delay2500\n\t"  "call delay2500\n\t"  "call delay2500\n\t"   
//"ret\n\t" 
); } // 500+10000=10500ns total delay

void delay21500ns(void){__asm__ __volatile__( "delay21500:\n\t" 
"call delay10500\n\t"  "call delay10500\n\t" 
//"ret\n\t" 
); } // 500+21000=21500ns total delay

void delay24000ns(void){__asm__ __volatile__( "delay24000:\n\t" 
"call delay10500\n\t"  "call delay10500\n\t"
"call delay2500\n\t"
//"ret\n\t" 
); } // 500+21000+2500=24000ns total delay

void delay26500ns(void){__asm__ __volatile__( "delay26500:\n\t" 
"call delay10500\n\t"  "call delay10500\n\t"
"call delay2500\n\t""call delay2500\n\t"
//"ret\n\t" 
); } // 500+21000+5000=26500ns total delay

void delay32000ns(void){__asm__ __volatile__( "delay32000:\n\t" 
"call delay10500\n\t"  "call delay10500\n\t" "call delay10500\n\t" 
//"ret\n\t" 
); } // 500+315000=32000ns total delay

byte Longer;

//6500 9000 if Longer flag is set

void Wait(void){__asm__ __volatile__( "wait:\n\t" 
//"call delay32000\n\t"//вялый фотосинтез //2050
//"call delay26500\n\t"// 2180 (2990)
//"call delay21500\n\t" // 2300
//"call delay10500\n\t""call delay2500\n\t""call delay2500\n\t""call delay2500\n\t"// 18500 всего // 2390
//"call delay10500\n\t"// 2620
//"call delay2500\n\t""call delay2500\n\t""call delay2500\n\t"// 2710
//"call delay2500\n\t""call delay2500\n\t""call delay1000\n\t"
"call delay2500\n\t"//"call delay2500\n\t"//"call delay1000\n\t"
"lds r20,Longer\n\t"
"cpi r20,0\n\t"
"breq 2f\n\t"
"call delay2500\n\t"
"2:\n\t"
//"call delay2500\n\t""call delay2500\n\t"// 2780
//"call delay2500\n\t"// 2860

); }



void delay86500ns(void){__asm__ __volatile__( "delay86500:\n\t" 
"call delay21500\n\t"  "call delay21500\n\t" "call delay21500\n\t"  "call delay21500\n\t" 
//"ret\n\t" 
); } // 500+86000=86500ns total delay

void delay43500ns(void){__asm__ __volatile__( "delay43500:\n\t" 
"call delay21500\n\t"  "call delay21500\n\t" 
//"ret\n\t" 
); } // 500+43000=43500ns total delay

void delay173500ns(void){__asm__ __volatile__( "delay173500:\n\t" 
"call delay86500\n\t"  "call delay86500\n\t" 
//"ret\n\t" 
); } // 500+86500+86500=173500ns total delay


void delay5500ns(void){__asm__ __volatile__( "delay5500:\n\t" 
"call delay2500\n\t"  "call delay2500\n\t"  
"ret\n\t" ); } // 500+5000=5500ns total delay

void delay3000ns(void){__asm__ __volatile__( "delay3000:\n\t" 
"call delay2500\n\t" 
"ret\n\t" ); } // 500+2500=3000ns total delay

byte counter;
void DayLight(void) __attribute__((noinline)); 
void DayLight(void)
{
  
//  Call-used registers (r18-r27, r30-r31):
//May be allocated by gcc for local data. You may use them freely in assembler subroutines.
//Call-saved registers (r2-r17, r28-r29):  
//    for (word e=1;e<65535;e++) {
  //  for (byte e=0;e<255;e++) {

    __asm__ __volatile__(
    
    "push r28\n\t"
    "push r29\n\t"
    "push r16\n\t"
    "push r17\n\t"
 
    "ldi r18,0b00000001\n\t"  // маски для портов B и C 012345
    "ldi r19,0b00000010\n\t"
    "ldi r20,0b00000100\n\t"
    "ldi r21,0b00001000\n\t"
    "ldi r22,0b00010000\n\t"
    "ldi r23,0b00100000\n\t"
 
//"cli\n\t" // с включенными прерываниями мерцает почем зря. 

//    "lds r31,counter\n\t"
  //  "andi r31,1\n\t"
    "lds r31,Fan\n\t" // лапка 0 управляет вентилятором.


    "ldi r24,0b00000100\n\t"  // маски для порта D 234567 (чтобы вентилятор пробрасывать) 
    "ldi r25,0b00001000\n\t"
    "ldi r26,0b00010000\n\t"
    "ldi r27,0b00100000\n\t"
    "ldi r28,0b01000000\n\t"
    "ldi r29,0b10000000\n\t"

    "or r24,r31\n\t"
    "or r25,r31\n\t"
    "or r26,r31\n\t"
    "or r27,r31\n\t"
    "or r28,r31\n\t"
    "or r29,r31\n\t"
  
  "ldi r17,11\n\t" // 60 sec
     
"888:\n\t"
    
    "ldi r30,0\n\t" 
    "mov r1,r30\n\t" // r1=0
    "ldi r16,253\n\t" 
    "ldi r30,248\n\t"
    //60000
    //60160 60161
//60378
"555:\n\t"// 
// (прерывание может выпасть на вспышку и она будет доооолго светить пока обрабатывается прерывание - посему запрещаем их на время пыхов)

// 123456 порядок активации выходов. По 6 выходов в каждом из 3х портов.
"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 456123 слегка меняем порядок активации выходов для избежания ненужных резонансов
"out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 234561
"out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r18\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r18\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r24\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 612345
"out 5,r23\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r23\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r29\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 456123
"out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 123456
"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 651234
"out 5,r22\n\t""out 5,r23\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r22\n\t""out 8,r23\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r28\n\t""out 11,r29\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 346512
"out 5,r20\n\t""out 5,r21\n\t""out 5,r23\n\t""out 5,r22\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r20\n\t""out 8,r21\n\t""out 8,r23\n\t""out 8,r22\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r26\n\t""out 11,r27\n\t""out 11,r29\n\t""out 11,r28\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D

// 123456 порядок активации выходов. По 6 выходов в каждом из 3х портов.
"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 456123 слегка меняем порядок активации выходов для избежания ненужных резонансов
"out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 234561
"out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r18\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r18\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r24\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 612345
"out 5,r23\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r23\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r29\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 456123
"out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 123456
"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 651234
"out 5,r22\n\t""out 5,r23\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r22\n\t""out 8,r23\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r28\n\t""out 11,r29\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 346512
"out 5,r20\n\t""out 5,r21\n\t""out 5,r23\n\t""out 5,r22\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r20\n\t""out 8,r21\n\t""out 8,r23\n\t""out 8,r22\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r26\n\t""out 11,r27\n\t""out 11,r29\n\t""out 11,r28\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D

// 16x3x7=336ops

// 123456 порядок активации выходов. По 6 выходов в каждом из 3х портов.
"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 456123 слегка меняем порядок активации выходов для избежания ненужных резонансов
"out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 234561
"out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r18\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r18\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r24\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 612345
"out 5,r23\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r23\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r29\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 456123
"out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 123456
"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 651234
"out 5,r22\n\t""out 5,r23\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r22\n\t""out 8,r23\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r28\n\t""out 11,r29\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 346512
"out 5,r20\n\t""out 5,r21\n\t""out 5,r23\n\t""out 5,r22\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r20\n\t""out 8,r21\n\t""out 8,r23\n\t""out 8,r22\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r26\n\t""out 11,r27\n\t""out 11,r29\n\t""out 11,r28\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D

// 123456 порядок активации выходов. По 6 выходов в каждом из 3х портов.
"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 456123 слегка меняем порядок активации выходов для избежания ненужных резонансов
"out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 234561
"out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r18\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r18\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r24\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 612345
"out 5,r23\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r23\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r29\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 456123
"out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 123456
"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 651234
"out 5,r22\n\t""out 5,r23\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r22\n\t""out 8,r23\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r28\n\t""out 11,r29\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 346512
"out 5,r20\n\t""out 5,r21\n\t""out 5,r23\n\t""out 5,r22\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r20\n\t""out 8,r21\n\t""out 8,r23\n\t""out 8,r22\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r26\n\t""out 11,r27\n\t""out 11,r29\n\t""out 11,r28\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D

// 32

// 123456 порядок активации выходов. По 6 выходов в каждом из 3х портов.
"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 456123 слегка меняем порядок активации выходов для избежания ненужных резонансов
"out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 234561
"out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r18\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r18\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r24\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 612345
"out 5,r23\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r23\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r29\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 456123
"out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 123456
"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 651234
"out 5,r22\n\t""out 5,r23\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r22\n\t""out 8,r23\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r28\n\t""out 11,r29\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 346512
"out 5,r20\n\t""out 5,r21\n\t""out 5,r23\n\t""out 5,r22\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r20\n\t""out 8,r21\n\t""out 8,r23\n\t""out 8,r22\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r26\n\t""out 11,r27\n\t""out 11,r29\n\t""out 11,r28\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D

// 123456 порядок активации выходов. По 6 выходов в каждом из 3х портов.
"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 456123 слегка меняем порядок активации выходов для избежания ненужных резонансов
"out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 234561
"out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r18\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r18\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r24\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 612345
"out 5,r23\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r23\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r29\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 456123
"out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 123456
"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 651234
"out 5,r22\n\t""out 5,r23\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r22\n\t""out 8,r23\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r28\n\t""out 11,r29\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 346512
"out 5,r20\n\t""out 5,r21\n\t""out 5,r23\n\t""out 5,r22\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r20\n\t""out 8,r21\n\t""out 8,r23\n\t""out 8,r22\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r26\n\t""out 11,r27\n\t""out 11,r29\n\t""out 11,r28\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
//48

// 123456 порядок активации выходов. По 6 выходов в каждом из 3х портов.
"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 456123 слегка меняем порядок активации выходов для избежания ненужных резонансов
"out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 234561
"out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r18\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r18\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r24\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 612345
"out 5,r23\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r23\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r29\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 456123
"out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 123456
"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 651234
"out 5,r22\n\t""out 5,r23\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r22\n\t""out 8,r23\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r28\n\t""out 11,r29\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 346512
"out 5,r20\n\t""out 5,r21\n\t""out 5,r23\n\t""out 5,r22\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r20\n\t""out 8,r21\n\t""out 8,r23\n\t""out 8,r22\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r26\n\t""out 11,r27\n\t""out 11,r29\n\t""out 11,r28\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D

// 123456 порядок активации выходов. По 6 выходов в каждом из 3х портов.
"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 456123 слегка меняем порядок активации выходов для избежания ненужных резонансов
"out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 234561
"out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r18\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r18\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r24\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 612345
"out 5,r23\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r23\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r29\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 456123
"out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 123456
"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 651234
"out 5,r22\n\t""out 5,r23\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r22\n\t""out 8,r23\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r28\n\t""out 11,r29\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 346512
"out 5,r20\n\t""out 5,r21\n\t""out 5,r23\n\t""out 5,r22\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r20\n\t""out 8,r21\n\t""out 8,r23\n\t""out 8,r22\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r26\n\t""out 11,r27\n\t""out 11,r29\n\t""out 11,r28\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
//64

// 16x3x7=336ops
// 1344ops

"dec r30\n\t" //+1
"breq 111f\n\t"//1 (not taken)
"rjmp 555b\n\t"//2
"111:\n\t" // 340ops per inner round 1348(64)

"dec r16\n\t"
"breq 222f\n\t"
"rjmp 555b\n\t"
"222:\n\t"

"dec r17\n\t"
"breq 333f\n\t"
"rjmp 888b\n\t"
"333:\n\t"

//"sei\n\t"

"pop r17\n\t"
"pop r16\n\t"
"pop r29\n\t"
"pop r28\n\t"

      ); 
}

void TwiLight(void) __attribute__((noinline)); 
void TwiLight(void)
{
  
//  Call-used registers (r18-r27, r30-r31):
//May be allocated by gcc for local data. You may use them freely in assembler subroutines.
//Call-saved registers (r2-r17, r28-r29):  
//    for (word e=1;e<65535;e++) {
  //  for (byte e=0;e<255;e++) {

    __asm__ __volatile__(
    
    "push r28\n\t"
    "push r29\n\t"
    "push r16\n\t"
    "push r17\n\t"
 
    "ldi r18,0b00000001\n\t"  // маски для портов B и C 012345
    "ldi r19,0b00000010\n\t"
    "ldi r20,0b00000100\n\t"
    "ldi r21,0b00001000\n\t"
    "ldi r22,0b00010000\n\t"
    "ldi r23,0b00100000\n\t"
 
//"cli\n\t" // с включенными прерываниями мерцает почем зря. 

//    "lds r31,counter\n\t"
  //  "andi r31,1\n\t"
    "lds r31,Fan\n\t" // лапка 0 управляет вентилятором.


    "ldi r24,0b00000100\n\t"  // маски для порта D 234567 (чтобы вентилятор пробрасывать) 
    "ldi r25,0b00001000\n\t"
    "ldi r26,0b00010000\n\t"
    "ldi r27,0b00100000\n\t"
    "ldi r28,0b01000000\n\t"
    "ldi r29,0b10000000\n\t"

    "or r24,r31\n\t"
    "or r25,r31\n\t"
    "or r26,r31\n\t"
    "or r27,r31\n\t"
    "or r28,r31\n\t"
    "or r29,r31\n\t"
  
  "ldi r17,11\n\t" // 60 sec
     
"888:\n\t"
    
    "ldi r30,0\n\t" 
    "mov r1,r30\n\t" // r1=0
    "ldi r16,253\n\t" 
//    "ldi r30,15\n\t"
    "ldi r30,248\n\t"
    //60000
    //60160 60161
//60378
  
  
  /*
// 123456 порядок активации выходов. По 6 выходов в каждом из 3х портов.
"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 456123 слегка меняем порядок активации выходов для избежания ненужных резонансов
"out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 234561
"out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r18\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r18\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r24\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 612345
"out 5,r23\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r23\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r29\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 456123
"out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 123456
"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 651234
"out 5,r22\n\t""out 5,r23\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r22\n\t""out 8,r23\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r28\n\t""out 11,r29\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
// 346512
"out 5,r20\n\t""out 5,r21\n\t""out 5,r23\n\t""out 5,r22\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r20\n\t""out 8,r21\n\t""out 8,r23\n\t""out 8,r22\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r26\n\t""out 11,r27\n\t""out 11,r29\n\t""out 11,r28\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D

////////////////////////////
"out 5,r1\n\t""out 5,r1\n\t""out 5,r1\n\t""out 5,r1\n\t""out 5,r1\n\t""out 5,r1\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r1\n\t""out 8,r1\n\t""out 8,r1\n\t""out 8,r1\n\t""out 8,r1\n\t""out 8,r1\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r31\n\t""out 11,r31\n\t""out 11,r31\n\t""out 11,r31\n\t""out 11,r31\n\t""out 11,r31\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D
  
  */
  
"555:\n\t"// 
// (прерывание может выпасть на вспышку и она будет доооолго светить пока обрабатывается прерывание - посему запрещаем их на время пыхов)

// 123456 порядок активации выходов. По 6 выходов в каждом из 3х портов.
"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"//в сумерках светим в 4 раза реже  
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

// 456123 слегка меняем порядок активации выходов для избежания ненужных резонансов
"out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"//в сумерках светим в 4 раза реже  
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

// 234561
"out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r18\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r18\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r24\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"//в сумерках светим в 4 раза реже  
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

// 612345
"out 5,r23\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r23\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r29\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"//в сумерках светим в 4 раза реже  
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

// 16x3x7=336ops

// 456123
"out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"//в сумерках светим в 4 раза реже  
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

// 123456
"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"//в сумерках светим в 4 раза реже  
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

// 651234
"out 5,r22\n\t""out 5,r23\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r22\n\t""out 8,r23\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r28\n\t""out 11,r29\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"//в сумерках светим в 4 раза реже  
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

// 346512
"out 5,r20\n\t""out 5,r21\n\t""out 5,r23\n\t""out 5,r22\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r20\n\t""out 8,r21\n\t""out 8,r23\n\t""out 8,r22\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r26\n\t""out 11,r27\n\t""out 11,r29\n\t""out 11,r28\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"//в сумерках светим в 4 раза реже  
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

// 32
// 123456 порядок активации выходов. По 6 выходов в каждом из 3х портов.
"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"//в сумерках светим в 4 раза реже  
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

// 456123 слегка меняем порядок активации выходов для избежания ненужных резонансов
"out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"//в сумерках светим в 4 раза реже  
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

// 234561
"out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r18\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r18\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r24\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"//в сумерках светим в 4 раза реже  
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

// 612345
"out 5,r23\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r23\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r29\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"//в сумерках светим в 4 раза реже  
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

// 48

// 456123
"out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"//в сумерках светим в 4 раза реже  
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

// 123456
"out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r22\n\t""out 5,r23\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r22\n\t""out 8,r23\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r28\n\t""out 11,r29\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"//в сумерках светим в 4 раза реже  
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

// 651234
"out 5,r22\n\t""out 5,r23\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r20\n\t""out 5,r21\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r22\n\t""out 8,r23\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r20\n\t""out 8,r21\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r28\n\t""out 11,r29\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r26\n\t""out 11,r27\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"//в сумерках светим в 4 раза реже  
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

// 346512
"out 5,r20\n\t""out 5,r21\n\t""out 5,r23\n\t""out 5,r22\n\t""out 5,r18\n\t""out 5,r19\n\t""out 5,r1\n\t" // B0-B5 и выключаем порт B
"out 8,r20\n\t""out 8,r21\n\t""out 8,r23\n\t""out 8,r22\n\t""out 8,r18\n\t""out 8,r19\n\t""out 8,r1\n\t" // С0-С5 и выключаем порт C
"out 11,r26\n\t""out 11,r27\n\t""out 11,r29\n\t""out 11,r28\n\t""out 11,r24\n\t""out 11,r25\n\t""out 11,r31\n\t" // D2-D7 и выключаем порт D

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"//в сумерках светим в 4 раза реже  
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"
"nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"

//64

// 16x3x7=336ops
// 1344ops

"dec r30\n\t" //+1
"breq 111f\n\t"//1 (not taken)
"rjmp 555b\n\t"//2
"111:\n\t" // 340ops per inner round 1348(64)

"dec r16\n\t"
"breq 222f\n\t"
"rjmp 555b\n\t"
"222:\n\t"

"dec r17\n\t"
"breq 333f\n\t"
"rjmp 888b\n\t"
"333:\n\t"

//"sei\n\t"

"pop r17\n\t"
"pop r16\n\t"
"pop r29\n\t"
"pop r28\n\t"

      ); 
}

uint8_t count2s=0;
uint16_t tz,tw;
long LastDark;
byte mmi;
/*
void LL(void){for(word r=0;r<10000;r++){LightAA2();}} // рассвет/закат
//void LH(void){for(counter=0;counter<2;counter++){Serial.begin(9600);Serial.print("+");delay(100); LightAA(); Serial.begin(9600);Serial.println("e");delay(100);}} // полная яркость
void LH(void){//Serial.begin(9600);Serial.print("+");delay(100);
for(counter=0;counter<60;counter++){ LightAA(); }

} // полная яркость
*/

void WaitM20(void) __attribute__((noinline)); 
void WaitM20(void)
{
    __asm__ __volatile__(    
    "ldi r21,38\n\t" //75000ms
"11111:\n\t"
    "ldi r20,245\n\t"     
"22222:\n\t"    
    "ldi r19,125\n\t" 
    "ldi r18,192\n\t"     
"33333:\n\t"

"dec r18\n\t"   // 1
"breq 111f\n\t" // 1
"rjmp 33333b\n\t" // 2  255x4+4=1024 clock
"111:\n\t"

"dec r19\n\t"
"breq 222f\n\t"
"rjmp 33333b\n\t"
"222:\n\t"
   
"dec r20\n\t"
"breq 333f\n\t"
"rjmp 22222b\n\t"
"333:\n\t"

"dec r21\n\t"
"breq 444f\n\t"
"rjmp 11111b\n\t"
"444:\n\t"
);
}
void Wait2S20(void) __attribute__((noinline)); 
void Wait2S20(void)
{
    __asm__ __volatile__(    
    "ldi r21,39\n\t" //2500ms
"11111:\n\t"
    "ldi r20,8\n\t"     
"22222:\n\t"    
    "ldi r19,125\n\t" 
    "ldi r18,29\n\t"     
"33333:\n\t"

"dec r18\n\t"   // 1
"breq 111f\n\t" // 1
"rjmp 33333b\n\t" // 2  255x4+4=1024 clock
"111:\n\t"

"dec r19\n\t"
"breq 222f\n\t"
"rjmp 33333b\n\t"
"222:\n\t"
   
"dec r20\n\t"
"breq 333f\n\t"
"rjmp 22222b\n\t"
"333:\n\t"

"dec r21\n\t"
"breq 444f\n\t"
"rjmp 11111b\n\t"
"444:\n\t"
);
}

void WaitM16(void) __attribute__((noinline)); 
void WaitM16(void)
{
    __asm__ __volatile__(    
//    "ldi r21,150\n\t" //300039ms
//60376 без прерываний 1.00627
//1.003611111 60594
    "ldi r21,30\n\t" //60000ms-9:56
"11111:\n\t"
    "ldi r20,242\n\t"     
"22222:\n\t"    
    "ldi r19,130\n\t" 
//    "ldi r18,251\n\t"     //60000 9:56
//    "ldi r18,252\n\t"     
//    "ldi r18,254\n\t"     //19:53
//    "ldi r18,200\n\t"     
    "ldi r18,63\n\t"     
"33333:\n\t"

"dec r18\n\t"   // 1
"breq 111f\n\t" // 1
"rjmp 33333b\n\t" // 2  255x4+4=1024 clock
"111:\n\t"

"dec r19\n\t"
"breq 222f\n\t"
"rjmp 33333b\n\t"
"222:\n\t"
   
"dec r20\n\t"
"breq 333f\n\t"
"rjmp 22222b\n\t"
"333:\n\t"

"dec r21\n\t"
"breq 444f\n\t"
"rjmp 11111b\n\t"
"444:\n\t"
);
}

void WaitM24(void) __attribute__((noinline)); 
void WaitM24(void)
{
    __asm__ __volatile__(    
    "ldi r21,45\n\t" //90000ms
"11111:\n\t"
    "ldi r20,242\n\t"     
"22222:\n\t"    
    "ldi r19,128\n\t" 
    "ldi r18,251\n\t"     
"33333:\n\t"

"dec r18\n\t"   // 1
"breq 111f\n\t" // 1
"rjmp 33333b\n\t" // 2  255x4+4=1024 clock
"111:\n\t"

"dec r19\n\t"
"breq 222f\n\t"
"rjmp 33333b\n\t"
"222:\n\t"
   
"dec r20\n\t"
"breq 333f\n\t"
"rjmp 22222b\n\t"
"333:\n\t"

"dec r21\n\t"
"breq 444f\n\t"
"rjmp 11111b\n\t"
"444:\n\t"
);
}

void Wait2S24(void) __attribute__((noinline)); 
void Wait2S24(void)
{
    __asm__ __volatile__(   //3000 
"11111:\n\t"
    "ldi r20,242\n\t"     
"22222:\n\t"    
    "ldi r19,192\n\t" 
    "ldi r18,255\n\t"     
"33333:\n\t"

"dec r18\n\t"   // 1
"breq 111f\n\t" // 1
"rjmp 33333b\n\t" // 2  255x4+4=1024 clock
"111:\n\t"

"dec r19\n\t"
"breq 222f\n\t"
"rjmp 33333b\n\t"
"222:\n\t"
   
"dec r20\n\t"
"breq 333f\n\t"
"rjmp 22222b\n\t"
"333:\n\t"
);
}

void Wait2S16(void) __attribute__((noinline)); 
void Wait2S16(void)
{
    __asm__ __volatile__(    //1999ms need 2011.5 2018.7
"11111:\n\t"
    "ldi r20,242\n\t"     
"22222:\n\t"    
    "ldi r19,130\n\t"      
    "ldi r18,50\n\t"     //2011: [177..193]
"33333:\n\t"

"dec r18\n\t"   // 1
"breq 111f\n\t" // 1
"rjmp 33333b\n\t" // 2  255x4+4=1024 clock
"111:\n\t"

"dec r19\n\t"
"breq 222f\n\t"
"rjmp 33333b\n\t"
"222:\n\t"
   
"dec r20\n\t"
"breq 333f\n\t"
"rjmp 22222b\n\t"
"333:\n\t"

);
}


void loop() {

    __asm__ __volatile__("Start:\n\t");
//    __asm__ __volatile__("wdr\n\t");//  wdt_reset();
   

// start @ 18:00
/*
long t1=timer0_millis;
//Wait2S16();
//Wait2S20();
//Wait2S24();
//WaitM16();
//WaitM20();
//WaitM24();
//DayLight();
TwiLight(); // 60473?
t1=timer0_millis-t1;

Serial.begin(9600);
Serial.println(">>>");
//Serial.println(timer0_millis,DEC);
Serial.println(t1,DEC);
delay(1000);*/

    __asm__ __volatile__("cli\n\t");

/*
PORTD=0x0;
Wait2S16();Wait2S16();
PORTD=0xff;
for(counter=0;counter<20;counter++){ WaitM16(); } 
PORTD=0x0;
Wait2S16();Wait2S16();
PORTD=0xff;
for(counter=0;counter<10;counter++){ WaitM16(); } 
PORTD=0x0;
Wait2S16();Wait2S16();
PORTD=0xff;
for(counter=0;counter<10;counter++){ WaitM16(); } 
PORTD=0x0;
Wait2S16();Wait2S16();
*/


// 16mhz
// вентиляторы включаются каждую нечетную минуту днем.
    for(byte r=0;r<4;r++){for(counter=0;counter<60;counter++){ Fan=(counter&1); DayLight(); Fan=0;} Wait2S16();} // 4h  1800-2200
    for(counter=0;counter<60;counter++){ TwiLight();} // evening 1h                2200-2300
    
    for(byte r=0;r<5;r++){for(counter=0;counter<60;counter++){ WaitM16(); } } // 5h  2300-0400
    for(counter=0;counter<59;counter++){ WaitM16(); } // 59m 0400-0459

    for(byte r=0;r<14;r++){ Wait2S16();  } // выравнивающий довесок
    
    for(counter=0;counter<60;counter++){ TwiLight();} // morning 1h                0500-0600
    for(byte r=0;r<12;r++){for(counter=0;counter<60;counter++){ Fan=(counter&1); DayLight(); Fan=0; } Wait2S16();} //12h 0600-1800

// 20mhz
  /*  for(byte r=0;r<4;r++){for(counter=0;counter<75;counter++){ LightAA(); }  delay(5000);} // 4h  1800-2200
    for(counter=0;counter<75;counter++){ TwiLight();} delay(5000); // evening 1h                2200-2300
    for(byte r=0;r<6;r++){for(counter=0;counter<75;counter++){ WaitM20(); }  delay(5000);} // 6h  2300-0500
    for(counter=0;counter<75;counter++){ TwiLight();} delay(5000); // morning 1h                0500-0600
    for(byte r=0;r<12;r++){for(counter=0;counter<75;counter++){ LightAA(); } delay(5000);} //12h 0600-1800
*/
//      cli();milli=timer0_millis;sei();  
//if(milli>=nextm)
//{
//  HOUR=milli/(3600000L>>2);
//  HOUR=milli/3600000L;
//  long tail=milli-HOUR*3600000L;
//  MINU=tail/60000L;
//  tail=tail-MINU*60000L;
//  SECU=tail/1000;

//   if (HOUR>=24){ cli();timer0_millis-=(86400000L>>2);sei(); HOUR=0;MINU=0;SECU=0;LastDark=0;}
//   if (HOUR>=24){ cli();timer0_millis-=21600000L;sei(); HOUR=0;MINU=0;SECU=0;LastDark=0;}
//  cli();timer0_millis=0L;sei();
 //   if ((milli-LastDark)>(900000L>>2)) {delay(250);LastDark=milli;} // 1 sec (x4)
//   if ((milli-LastDark)>900000L) {delay(1000);LastDark=milli;} // 1 sec
// if ((milli-LastDark)>1200000L) {delay(1000);LastDark=milli;} // 1 sec

//if ((!SECU)&&(!(MINU&0x1F))){delay(1000);} // примерно раз в полчаса (1я и 33я минуты) пауза в 1 секунду для сброса/отдыха/перезарядки энзимов и/или вообще для кругозора как оно без света.
//if ((HOUR>=7)&&(HOUR<=21)) {for(word j=0;j<1500;j++){LightMix85();}delay(1000);} // период между пыхами 8.5 мкс
//if ((HOUR>=7)&&(HOUR<=21)) {for(word j=0;j<1500;j++){LightMix100();}delay(1000);} // период между пыхами 10 мкс
//if ((HOUR>=5)&&(HOUR<=22)) {

  //if ((HOUR==5)||(HOUR==22)) {FanOFF;LL();}//<38w <480lux
//  if ((HOUR==6)||(HOUR==20)) {FanOFF;for(word r=0;r<10000;r++){LightAA1();delayMicroseconds(7);}LightAA();}//38w 480lux
//  if ((HOUR==6)||(HOUR==20)) {FanOFF;for(word r=0;r<10000;r++){LightAA6250();}} // 44.7w 530lux
//  if ((HOUR==6)||(HOUR==20)) {FanOFF;for(word r=0;r<100;r++){LightAA6250();}LightAA1();} // 44.6w 535lux
//  if ((HOUR==6)||(HOUR==20)) {FanOFF;for(word r=0;r<10;r++){LightAA6250();}} // 44.6w 523lux
//  if ((HOUR==6)||(HOUR==20)) {FanOFF;LightAA6250();} // 44.2w 508lux
//  else
  //{

//  if ((milli>>16)&0x3){FanON;}else{FanOFF;} // каждую четвертую минуту (65.5c) тушим вентиляторы для облегчения доступа CO2 в листья.
//  if ((milli>>16)&0x3){FanON;for(word r=0;r<100;r++){LightAA1();delayMicroseconds(3);LightAA1();}}else{FanOFF;LightAA();} // каждую четвертую минуту (65.5c) тушим вентиляторы для облегчения доступа CO2 в листья.
// 1110 77.5w
// 906 50.2w

//  if ((milli>>16)&0x3){FanON;for(word r=0;r<10000;r++){LightAA1();delayMicroseconds(2);LightAA1();}}else{FanOFF;for(byte r=0;r<25;r++){LightAA();}} // каждую четвертую минуту (65.5c) тушим вентиляторы для облегчения доступа CO2 в листья.
//  78.2w
// 914 50.9w
//  if ((milli>>16)&0x3){FanON;for(word r=0;r<10000;r++){LightAA1();delayMicroseconds(1);LightAA1();}}else{FanOFF;for(byte r=0;r<25;r++){LightAA();}} // каждую четвертую минуту (65.5c) тушим вентиляторы для облегчения доступа CO2 в листья.
//  78.2w
// 927 52.2w
  //if ((milli>>16)&0x3){FanON;for(word r=0;r<10000;r++){LightAAA();}}else{FanOFF;for(byte r=0;r<100;r++){LightAA();}} // каждую четвертую минуту (65.5c) тушим вентиляторы для облегчения доступа CO2 в листья.
// 1107 78.0w
// 1049 66.1w
//byte k=((milli>>16)&3);
//byte k=((milli>>14)&3);

//if (k){FanON;}else{FanOFF;}  

//LH();

//}}


 
    __asm__ __volatile__("rjmp Start\n\t");
}

void setup() 
{  
    PORTD=0;
    PORTB=0;
    PORTC=0;
  DDRD=0b11111111; // set D pins to output
  DDRB=0b11111111; // set B pins to output
  DDRC=0b11111111; // set C pins to output

  //delay(1500); 
  //SerialON;  Serial.println("Start"); delay(500);  SerialOFF;

  ACSR|=(1<<ACD);// analog comparator off
  PRR|=(1<<PRTWI)|(1<<PRTIM2)|(1<<PRTIM1)|(1<<PRSPI)|(1<<PRADC);

//  TCCR0B=4; // 256 часы в 4 раза медленнее
//  TCCR0B=5; // 1024 в 16 раз медленнее


//  setup_watchdog(T2S); // если в течении 2s не сбросить сторожевого пса то перезагрузка. (защита от зависаний)
    
  
//    Pin2Output(DDRD,0);
//    Pin2Output(DDRD,1);
//    Pin2Output(DDRD,2);
//    Pin2Output(DDRD,3);
//    Pin2Output(DDRD,4);
//    Pin2Output(DDRD,5);
    
//    Pin2LOW(PORTD,0);
//    Pin2LOW(PORTD,1);
//    Pin2LOW(PORTD,2);
//    Pin2LOW(PORTD,3);
//    Pin2LOW(PORTD,4);
//    Pin2LOW(PORTD,5);

  //  SetADC(1,8,500);  //  select temperature sensor 352
    
    // initial  hour  settings
    
  //  cli();timer0_millis=0L;sei();    // 12 ночи
 //   cli();timer0_millis=(1800000L>>2);sei();    // 0:30 ночи
   // cli();timer0_millis=3600000L;sei();    // 1 ночи
  //  cli();timer0_millis=7200000L;sei();    // 2 ночи

 //   cli();timer0_millis=21600000L;sei();    // 6 утра
//    cli();timer0_millis=25200000L;sei();    // 7 утра
//  cli();timer0_millis=26100000L;sei();    // 7:15 утра

//    cli();timer0_millis=28795000L;sei();    // почти 8 утра

  //  cli();timer0_millis=36000000L;sei();    // 10 утра
 //  cli();timer0_millis=39600000L;sei();    // 11 утра
 //   cli();timer0_millis=43200000L;sei();    // полдень
 //cli();timer0_millis=46800000L;sei();    // час дня


//    cli();timer0_millis=50400000L;sei();    // 2 часа дня
//    cli();timer0_millis=54000000L;sei();    // 3 часа дня
 //  cli();timer0_millis=(55800000L>>2);sei();    // 3:30 часа дня
//    cli();timer0_millis=57600000L;sei();    // 4 часа дня
//    cli();timer0_millis=61200000L;sei();    // 5 вечера
//    cli();timer0_millis=64080000L;sei();    // почти 6 вечера
   //cli();timer0_millis=64800000L;sei();    // 6 вечера

//    cli();timer0_millis=68400000L;sei();    // 7 вечера
//    cli();timer0_millis=71000000L;sei();    // почти 8 вечера

    //cli();timer0_millis=72000000L;sei();    // 8 вечера
//    cli();timer0_millis=73800000L;sei();    // 8:30 вечера
  //cli();timer0_millis=(73800000L>>2);sei();    // 8:30 вечера
//    cli();timer0_millis=75000000L;sei();    // почти 9 вечера
 //   cli();timer0_millis=75600000L;sei();    // 9 вечера
//    cli();timer0_millis=77400000L;sei();    // 9:30 вечера
//    cli();timer0_millis=78500000L;sei();    // почти 10 вечера
 
    // cli();timer0_millis=79200000L;sei();    // 10 вечера
//    cli();timer0_millis=82700000L;sei();    // почти 11 вечера
 //   cli();timer0_millis=20675000L;sei();    // почти 11 вечера
  //  cli();timer0_millis=(82800000L>>2)-10;sei();    // 11 вечера
  // cli();timer0_millis=(86395000L>>2);sei();    // почти полночь
 //cli();timer0_millis=86000000L;sei();    // почти полночь
// cli();timer0_millis=(85900000L>>2);sei();    // почти полночь

//milli=timer0_millis;
//  HOUR=milli/3600000L;
//  long tail=milli-HOUR*3600000L;
//  MINU=tail/60000L;
//  tail=tail-MINU*60000L;
//  SECU=tail/1000;

//SerialON;  Serial.println(SECU);Serial.println(MINU);Serial.println(HOUR); delay(500);  SerialOFF;

 //CheckPowerSupply();CheckPowerSupply();
// nextm=timer0_millis+3600000L;
}

