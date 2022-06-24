/*
 * LCD_PortC.c - ATmega2560
 *
 * Created: 2020-06-22 3:29:33 PM
 * Modified: By Pat
 */ 

#include <avr/io.h>
//#include <util/delay.h>

#include "lcd.h"

void mTimer (int count)
{
   /***
      Setup Timer1 as a ms timer
	  Using polling method not Interrupt Driven
   ***/
	  
   int i;

   i = 0;

   TCCR1B |= _BV (CS11);  // Set prescaler (/8) clock 16MHz/8 -> 2MHz
   /* Set the Waveform gen. mode bit description to clear
     on compare mode only */
   TCCR1B |= _BV(WGM12);

   /* Set output compare register for 2000 cycles, 1ms */
   OCR1A = 0x07D0;
 
   /* Initialize Timer1 to zero */
   TCNT1 = 0x0000;

   /* Enable the output compare interrupt */
   //TIMSK1 |= _BV(OCIE1A);  //remove if global interrups is set (sei())

   /* Clear the Timer1 interrupt flag and begin timing */
   TIFR1 |= _BV(OCF1A);

   /* Poll the timer to determine when the timer has reached 1ms */
   while (i < count)
   {
      while ((TIFR1 & 0x02) != 0x02);
	
	   /* Clear the interrupt flag by WRITING a ONE to the bit */
	   TIFR1 |= _BV(OCF1A);
	   i++;
   } /* while */
   TCCR1B &= ~_BV (CS11);  //  disable prescalar, no Clock
   return;
}  /* mTimer */



int main(int argc,char*argv[])
{
	unsigned char i;

	//Initialize LCD module
	InitLCD(LS_BLINK|LS_ULINE);

	//Clear the screen
	LCDClear();

	//Simple string printing
	LCDWriteString("Congrats ");
	
	//A string on line 2
	LCDWriteStringXY(0,1,"Loading ");

	//Print some numbers
	for (i=0;i<99;i+=1)
	{
		LCDWriteIntXY(9,1,i,3);
		LCDWriteStringXY(12,1,"%");
		//_delay_loop_2(0);	
		//_delay_loop_2(0);	
		//_delay_loop_2(0);	
		//_delay_loop_2(0);	
		mTimer(1000);
	}

	//Clear the screen
	LCDClear();

	//Some more text

	LCDWriteString("Hello world");
	LCDWriteStringXY(0,1,"By Pat");		// <--- Write ur NAME HERE !!!!!!!!!!!

	//Wait ~ 25 secs
	//for(i=0;i<100;i++) _delay_loop_2(0);
	mTimer (25000);

	//Some More ......
	LCDClear();
	//mTimer(100);
	LCDWriteString("    eXtreme");
	LCDWriteStringXY(0,1,"  Electronics");

   return(0);

}
