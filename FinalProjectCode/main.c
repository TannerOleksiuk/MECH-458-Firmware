/*#########################################################################
# MILESTONE: 5
# PROGRAM: 5
# PROJECT: Final Project
# GROUP: 5
# NAME 1: Tanner, Oleksiuk, V00867082
# NAME 2: Paulo, Dait, V00787012
# DESC: This program sorts materials based on different properties measured by a sensor-conveyor belt network.
Includes self-homing of sort bin, item classification, stepper driven sorting bin, pause function and ramp down.
# DATA
# REVISED
########################################################################*/
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "lcd.h"
#include "LinkedQueue.h"

#define ALUMINUM 0
#define STEEL 1
#define WHITE 2
#define BLACK 3

#define INIT_SYS 0
#define SYSTEM_ON 1
#define PAUSED 2
#define RAMPDN 3


void mTimer(int count);
int step(int dir, int steps, int speed, int tabIndex); //steps motor by specified number of steps
int trapezoidalStep(int dir, int steps, int speedInit, int speedTarget, int tabIndex); //Steps motor with acceleration and deceleration
void setPos(int dir, int speedInit, int speedTarget, int degrees, int tabIndex); //Sets exact position of motor in degrees
void continuousRotation(int dir, int speed, int timeToRotate, int tabIndex); //Sets motor to continuously rotate for set amount of time, time = 0 is infinite rotation
void homeStepper();
void pause(int queueSize);
int rampDown();
void getADCVal();
int getMaterial(int adcVal);
int setBucketPos(int MATERIAL, int currPos);

//Global Variables
volatile unsigned int ADC_result; //store ADC result
volatile unsigned int lowestADCVal = 1024; //Store lowest value from ADC
volatile unsigned int ADC_result_flag; //Flag for ADC conversion completion
volatile unsigned int home_flag = 0; //Flag upon homing of stepper
volatile unsigned int tableIndex = 0; //Keeps track of table index to avoid hissy fits
volatile unsigned int measureADC = 0; //Flag for when to store values from ADC
volatile unsigned int popItem = 0; //Flag to pop item from the linked list
volatile unsigned int currPosition = 3; //Current position of the stepper motor
volatile unsigned int sortedMats[4] = {0,0,0,0}; //AL, STL, WHT, BLK
volatile unsigned int isRampDown = 0; //Has a ramped down sequence been initiated
volatile uint32_t millis = 0; //Global time in milliseconds that the program has been running
volatile uint32_t debounce = 0; //Time passed for debouncing purpose
volatile uint32_t popTime = 0; //Time passed since last item was popped from queue
	
volatile unsigned int state = INIT_SYS; //initialize system state

//const int stepTable[4] = {0x08, 0x04, 0x02, 0x01}; //Sets the step instructions for the 28BYJ-48 Motor
//const int stepTable[4] = {0x30, 0x06, 0x28, 0x05 }; //Step table for project apparatus unpiolar
const int stepTable[4] = {0x36, 0x2E, 0x2D, 0x35}; //Bipolar lab apparatus
	
//Sets the amount of steps to do a full rotation
//const float stepsToFullRotation = 2048.0; //2BYJ-48 Motor
const float stepsToFullRotation = 200.0; //Lab apparatus

//constants for controlling DC motor direction
const int DC_BACK = 0x0B;
const int DC_FORWARD = 0x07;
const int DC_BRAKE = 0x0F;

//global speed settings for stepper and DC motor
const int DC_SPEED = 0x60; //DC motor PWM setting
const int STEP_SPEED_INIT = 20;
const int STEP_SPEED_STEADY = 10;

//Constants for classfication
const int ALUMINUM_UPPER = 255;
const int STEEL_LOWER = 400;
const int STEEL_UPPER = 700;
const int WHITE_LOWER = 900;
const int WHITE_UPPER = 955;
const int BLACK_LOWER = 956;

//Positions in degrees of materials: AL STL WHT BLK
const int materialPos[4] = {90,270,180,0}; //Positions in degrees relative to black, of each material


int main(void)
{
	//Setup Linked List
	link *head;
	link *tail;
	link *newLink;
	link *rtnLink;
	
	rtnLink = NULL;
	newLink = NULL;
	
	setup(&head, &tail);
	
	
	/***** Begin Initializations ******/
	
	CLKPR = 0x80; //Allow prescaler to be changed
	CLKPR = 0x01; //Adjust prescaler to set clk to 8MHz
	
	TCCR1B |= _BV(CS11); //configure timer control register

	
	//Interrupt initialization
	cli();
	//Setup data direction for interrupt ports
	DDRD = 0b11110000;
	DDRE = 0b11001111;
	
	//Setup interrupts to trigger on rising edge
	EIMSK |= (_BV(INT1))|(_BV(INT2))|(_BV(INT3))|(_BV(INT4))|(_BV(INT5)); //enable interrupts 1-5
	EICRA |= 0xBA; //Setup edge triggers for int 0-3, all on falling edge except int 2 triggers rising edge
	EICRB |= 0x0A; //Setup int 4,5, to trigger on falling edge
	
	//Setup interrupt for timer3
	TCCR3A = 0;
	TCCR3A = 0;
	TCNT3 = 57537; //Set counter so interrupt triggers 1 per millisecond 
	TIMSK3 |= _BV(TOIE3);
	TCCR3B = _BV(CS10); //Start timer with no prescaler
	
	//ADC configuration
	ADCSRA |= _BV(ADEN); //enable ADC
	ADCSRA |= _BV(ADIE); //enable ADC interrupt
	ADMUX |= _BV(REFS0)|_BV(MUX0); //Configure multiplexer to use voltage reference 0 and to left adjust result
	
	sei();
	
	/*PWM init*/
	//Sets OCRA to be updated at TOP, TOP to be max value (0xFF), and TOV set at MAX
	TCCR0A = 0x03;
	TCCR0B = 0x00;
	
	//Set compare register to clear on compare and set at TOP
	TCCR0A |= 0x80;
	
	//Set prescaler/clock frequency
	TCCR0B |= 0x03;
	
	//Set duty cycle through OCRA
	OCR0A = DC_SPEED;
	
	//Set DDRB to appropriate pin (PB7)
	DDRB = 0x8F;
	DDRF = 0x00;
	DDRA = 0x0F;
	DDRC = 0xFF;
	DDRL = 0xF0;
	
	
	//begin conversion of ADC as an inital test
	ADCSRA |= _BV(ADSC);
	while(!ADC_result_flag);
	ADC_result_flag = 0;
	
	lowestADCVal = 1024;
	
	//init LCD
	InitLCD(LS_ULINE);
	LCDClear();
	
	//Home Stepper Motor
	homeStepper();
	LCDClear();
	/******** END INIT *******/
	
	state = SYSTEM_ON;
	
	//Start DC motor
	PORTB |= DC_FORWARD;
	
	/*****************************Main Loop*************************************/
	/**************************************************************************/
	
	volatile unsigned int readyToStore = 0;
	volatile uint32_t rampTime = 0;
	
	while(1){
		switch(state){
			case SYSTEM_ON:
				LCDWriteIntXY(7,1,size(&head,&tail),1)
				if(measureADC){
					while(PIND & 0x04){
						getADCVal();
						LCDWriteIntXY(0,1,lowestADCVal,4)
					}
					measureADC = 0;
					readyToStore = 1;
				}
				if(readyToStore){
					if(size(&head,&tail) == 0){
						setup(&head,&tail);
					}
					initLink(&newLink);
					newLink->e.itemCode = getMaterial(lowestADCVal);;
					enqueue(&head, &tail, &newLink);
					lowestADCVal = 1024;
					readyToStore = 0;
				}
				//Check whether to pop item from conveyor belt and linked list
				if(popItem && size(&head, &tail) > 0){
					//Dequeue item
					dequeue(&head, &rtnLink);
					//Check class
					sortedMats[(int)rtnLink->e.itemCode]++;
					PORTL = rtnLink->e.itemCode << 4;
					//Stop DC Motor
					PORTB = DC_BRAKE;
					mTimer(5); //Wait 5 milliseconds for complete stop
					//Set stepper to correct position
					currPosition = setBucketPos(rtnLink->e.itemCode, currPosition);
					//Start DC Motor
					PORTB = DC_FORWARD;
					free(rtnLink);
					popItem = 0;
					popTime = millis;
				}
				else if((popItem && size(&head, &tail)==0)){
					popItem = 0;
				}
				
				if(isRampDown){
					//Check if it has been 8 seconds
					if((millis-rampTime) > 8000){
						//Check if there are still items on conveyor
						if(size(&head,&tail)==0){
							//delete linked list and free memory
							clearQueue(&head,&tail);
							//Call ramp down subroutine
							return rampDown();
						}
					}
				}
				
				break;
				
				
			case PAUSED:
				pause(size(&head,&tail));
				break;
			case RAMPDN:
			//initiate ramp down sequence
				rampTime = millis;
				isRampDown = 1;
				state = SYSTEM_ON;
				break;
			default:
				LCDWriteStringXY(0,0,"Invalid State");
				break;
		}
	}
	
	
	return 0;
}


/*********************************Functions:*************************************/
/********************************************************************************/

//Timer function for miliseconds
void mTimer(int count){
	int i;  // keeps track of loop number
	i=0;  // initializes loop counter2333

	//set the waveform generator mode bit description to clear timer on compare math mode (ctc) only
	TCCR1B |= _BV(WGM12);
	
	OCR1A = 0x03E8;
	
	TCNT1 = 0x0000;
	
	TIFR1 |= _BV(OCF1A);
	
	while (i<count){
		if((TIFR1 & 0x02) == 0x02){
			
			TIFR1 |= _BV(OCF1A);
			i++; //increment loop number
			
		} //end if
		
	} // end while
	return;
}

//Step tells motor to step a specific number of times at a set speed and direction
int step(int dir, int steps, int speed, int tabIndex){
	
	//Clockwise dir
	if(dir){
		for(int i = 0; i<steps; i++){
			PORTA = stepTable[tabIndex];
			mTimer(speed);
			if(tabIndex < 3){
				tabIndex++;
			}
			else{
				tabIndex = 0;
			}
		}
	}
	//CCW
	else{
		for(int i = 0; i<steps; i++){
			PORTA = stepTable[(3-tabIndex)];
			mTimer(speed);
			if(tabIndex < 3){
				tabIndex++;
			}
			else{
				tabIndex = 0;
			}
		}
	}
	return tabIndex;
}

//Trapezoidal acceleration profile for stepper motor
int trapezoidalStep(int dir, int steps, int speedInit, int speedTarget, int tabIndex){
	volatile int stepsToAcel = 0;
	volatile int speed = speedInit;
	volatile int isAccel = 1;
	for(int i = 0; i < steps; i++){
		//Accelerate
		if(speed > speedTarget && isAccel){
			tabIndex = step(dir,1,speed,tabIndex);
			speed--;
			stepsToAcel++;
		}
		//Constant Speed
		else if((steps - i) > stepsToAcel){
			tabIndex = step(dir,1,speed,tabIndex);
		}
		//Decelerate
		else{
			isAccel = 0;
			tabIndex = step(dir,1,speed,tabIndex);
			speed++;
		}
	}
	return tabIndex;
}

//Set stepper position in degrees
void setPos(int dir, int speedInit, int speedTarget, int degrees, int tabIndex){
	int steps = ((stepsToFullRotation/360.0)*degrees);
	//PORTC = steps; //Used for debugging
	tableIndex = trapezoidalStep(dir, steps, speedInit, speedTarget, tabIndex);
}

//Set the bucket position to a material in optimal direction
int setBucketPos(int MATERIAL, int currPos){
	//Get position in degrees
	int deg = materialPos[MATERIAL];
	int currPosDeg = materialPos[currPos];
	//Determine optimal direction and set stepper to that position
	if(deg == currPosDeg){
		return currPos;
	}
	if((currPosDeg+90) == deg || ((currPosDeg+90) == 360 && MATERIAL==BLACK)){
		setPos(1,17,5,90,tableIndex);
	}
	else if((currPosDeg-90) == deg || ((currPosDeg+270) == deg && MATERIAL == STEEL)){
		setPos(0,17,5,90,tableIndex);
	}
	else{
		setPos(1,17,5,180,tableIndex);
	}
	return MATERIAL;
}

//Retrieve material type based on ADC value
int getMaterial(int adcVal){
	if(adcVal < ALUMINUM_UPPER){ 
		return ALUMINUM;
	}
	if(adcVal >= STEEL_LOWER && adcVal <= STEEL_UPPER){
		return STEEL;
	}
	if(adcVal >= WHITE_LOWER && adcVal <= WHITE_UPPER){
		return WHITE;
	}
	if(adcVal >= BLACK_LOWER){
		return BLACK;
	}
	
	//If classified as none of the items
	return 5;
}

//Home stepper by spinning until interrupt is triggered
void homeStepper(){
	LCDClear();
	LCDWriteStringXY(0,0,"Homing Stepper..")
	LCDWriteStringXY(7,1, "3");
	mTimer(1000);
	LCDWriteStringXY(7,1, "2");
	mTimer(1000);
	LCDWriteStringXY(7,1, "1");
	mTimer(1000);
	LCDWriteStringXY(7,1, " ");
	while(!home_flag){
		tableIndex = step(1,1,15,tableIndex);
	}
	currPosition = 3;
}

//Begin pause subroutine
void pause(int queueSize){
	PORTB = DC_BRAKE;
	mTimer(5);
	LCDClear();
	LCDWriteStringXY(0,0,"Belt:")
	LCDWriteIntXY(5,0,queueSize,2);
	LCDWriteStringXY(0,1,"A:");
	LCDWriteIntXY(2,1,sortedMats[ALUMINUM],2);
	LCDWriteStringXY(4,1,"S:");
	LCDWriteIntXY(6,1,sortedMats[STEEL],2);
	LCDWriteStringXY(8,1,"W:");
	LCDWriteIntXY(10,1,sortedMats[WHITE],2);
	LCDWriteStringXY(12,1,"B:");
	LCDWriteIntXY(14,1,sortedMats[BLACK],2);
	while(1){
		if(state != PAUSED){
			LCDClear();
			PORTB = DC_FORWARD;
			break;
		}
	}
}

//Begin ramp down sequence
int rampDown(){
	//Brake DC Motor
	PORTB = DC_BRAKE;
	mTimer(20);
	//Disable all ports (except LCD)
	PORTB = 0x00;
	PORTL = 0x00;
	PORTA = 0x00;
	PORTF = 0x00;
	PORTE = 0x00;
	PORTD = 0x00;
	//Dsiplay Items
	LCDClear();
	LCDWriteStringXY(0,0, "Sys. Disabled");
	LCDWriteStringXY(0,1,"A:");
	LCDWriteIntXY(2,1,sortedMats[ALUMINUM],2);
	LCDWriteStringXY(4,1,"S:");
	LCDWriteIntXY(6,1,sortedMats[STEEL],2);
	LCDWriteStringXY(8,1,"W:");
	LCDWriteIntXY(10,1,sortedMats[WHITE],2);
	LCDWriteStringXY(12,1,"B:");
	LCDWriteIntXY(14,1,sortedMats[BLACK],2);
	//Return 0 to main
	return 0;
}

//Retrieve ADC value
void getADCVal(){
	//begin conversion
	ADCSRA |= _BV(ADSC);
	//wait for conversion to complete
	while(!ADC_result_flag);
	//clear flag
	ADC_result_flag = 0;
}

/*****************Interrupt service routines:*************************************/
/********************************************************************************/

//Hall Effect
ISR(INT1_vect){
	if(state == INIT_SYS){
		home_flag = 1;
	}
}

//Optical Sensor Refl
ISR(INT2_vect){
	mTimer(1);
	if(PIND & 0x04){
		if(state == SYSTEM_ON){
			measureADC = 1;
		}
	}
}

//Exit Sensor
ISR(INT3_vect){
	//mTimer(15);
	if((millis - popTime)>100){
		if(state == SYSTEM_ON){
			popItem = 1;
		}
	}
}

//Pause
ISR(INT4_vect){
	if((millis - debounce) > 100){
		mTimer(15);
		if(!(PINE & _BV(PINE4))){
			if(state == PAUSED){
				state = SYSTEM_ON;
			}
			else{
				state = PAUSED;
			}
			debounce = millis;
		}
	}
}

//Ramp Down
ISR(INT5_vect){
	if(state != INIT_SYS){
		state = RAMPDN;
	}
}

//Timer 3 overflow interrupt
ISR(TIMER3_OVF_vect){
	TCNT3 = 57537;
	millis++;
	
}

//Faulty interrupt detection
ISR(BADISR_vect){
	
}

//interrupt will be trigged when ADC is finished conversion
ISR(ADC_vect){
	ADC_result = ADCL + (ADCH<<8);
	if(ADC_result < lowestADCVal){
		lowestADCVal = ADC_result;
	}
	ADC_result_flag = 1;
}
