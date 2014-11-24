//-----------------------------------------------------------------
// Name:	Coulston
// File:	lab5.c
// Date:	Fall 2014
// Purp:	Demo the decoding of an IR packet
//-----------------------------------------------------------------
#include <msp430g2553.h>
#include "start5.h"

int8	newIrPacket = FALSE;
int16	packetData[40];
int16	bitString;
int8	packetIndex = 0;
int8	packetIndex2 = 0;


#define		TRUE			1
#define		FALSE			0
#define		UP_BUTTON		(P2IN & BIT5)
#define		DOWN_BUTTON		(P2IN & BIT4)
#define		AUX_BUTTON		(P2IN & BIT3)
#define		LEFT_BUTTON		(P2IN & BIT2)
#define		RIGHT_BUTTON	(P2IN & BIT1)

// -----------------------------------------------------------------------
// -----------------------------------------------------------------------
void main(void) {

	unsigned char x, y, button_press, color;

	// === Initialize system ================================================
	IFG1 = 0; /* clear interrupt flag1 */
	WDTCTL = WDTPW + WDTHOLD; /* stop WD */
	WDTCTL = WDTPW | WDTHOLD;                 // stop the watchdog timer



//	initMSP430();				// Setup MSP to process IR and buttons

	//copy over from lab 4 Required Functionality
//	init();
//	initNokia();
//	clearDisplay();
//	x = 4;
//	y = 4;
//	drawBlock( y, x, color);

//	_enable_interrupt();

	while(1)  {


		bothForward();
		stopRobot();
		bothBackward();
		stopRobot();
		tankRightForty();
		stopRobot();
		bothForward();
		stopRobot();
		tankLeftForty();
		stopRobot();
		tankRightNinety();
		stopRobot();
		tankLeftNinety();
		stopRobot();

//		moveLeftBackward(dutyCycle);
//
//		_delay_cycles(1000000);



//		packetIndex2 = 0;
//
//		if (newIrPacket) {
//			_disable_interrupt();
//
//			while ((packetData[packetIndex2] != 2) && (packetIndex2 < 40)) {
//				packetIndex2++;
//			}
//
//			int length = 0;
//
//			while (length < 16) {
//				bitString+=packetData[packetIndex2++];
//				bitString<<=1;
//				length++;
//			}
//
//			init();
//			initNokia();
//
//
//
//
//				if (bitString == CH_UP) {
//
//					if (y >= 1)
//						y = y - 1;
//					button_press = TRUE;
//				}
//				if (bitString == CH_DW) {
//
//					if (y <= 6)
//						y = y + 1;
//					button_press = TRUE;
//						}
//				if (bitString == ONE) {
//					if (color == TRUE) {
//						color = FALSE;
//					} else {
//						color = TRUE;
//					}
//						}
//				if (bitString == VOL_UP) {
//					if (x <= 10)
//						x = x + 1;
//					button_press = TRUE;
//						}
//				if (bitString == VOL_DW) {
//					if (x >= 1)
//						x = x - 1;
//					button_press = TRUE;
//						}
//
//				if (button_press) {
//					button_press = FALSE;
//					drawBlock(y, x, color);
//				}
//
//				bitString = 0;
//				packetIndex = 0;
//				newIrPacket = FALSE;
//
//				int i;
//				for (i = 0; i<0xFFFF; i++);
//
//				drawBlock( y, x, color);
//				initMSP430();


	} // end infinite loop
} // end main

// -----------------------------------------------------------------------
// In order to decode IR packets, the MSP430 needs to be configured to
// tell time and generate interrupts on positive going edges.  The
// edge sensitivity is used to detect the first incoming IR packet.
// The P2.6 pin change ISR will then toggle the edge sensitivity of
// the interrupt in order to measure the times of the high and low
// pulses arriving from the IR decoder.
//
// The timer must be enabled so that we can tell how long the pulses
// last.  In some degenerate cases, we will need to generate a interrupt
// when the timer rolls over.  This will indicate the end of a packet
// and will be used to alert main that we have a new packet.
// -----------------------------------------------------------------------
void initMSP430() {

	IFG1=0; 					// clear interrupt flag1
	WDTCTL=WDTPW+WDTHOLD; 		// stop WD

	BCSCTL1 = CALBC1_8MHZ;
	DCOCTL = CALDCO_8MHZ;

	P2SEL  &= ~BIT6;						// Setup P2.6 as GPIO not XIN
	P2SEL2 &= ~BIT6;
	P2DIR &= ~BIT6;
	P2IFG &= ~BIT6;						// Clear any interrupt flag
	P2IE  |= BIT6;						// Enable PORT 2 interrupt on pin change

	HIGH_2_LOW;
	P1DIR |= BIT0 | BIT6;				// Enable updates to the LED
	P1OUT &= ~(BIT0 | BIT6);			// An turn the LED off

	TA0CCR0 = 0x8000;					// create a 16mS roll-over period
	TACTL &= ~TAIFG;					// clear flag before enabling interrupts = good practice
	TACTL = ID_3 | TASSEL_2 | MC_1;		// Use 1:1 presclar off MCLK and enable interrupts

	_enable_interrupt();
}


// -----------------------------------------------------------------------
// Since the IR decoder is connected to P2.6, we want an interrupt
// to occur every time that the pin changes - this will occur on
// a positive edge and a negative edge.
//
// Negative Edge:
// The negative edge is associated with end of the logic 1 half-bit and
// the start of the logic 0 half of the bit.  The timer contains the
// duration of the logic 1 pulse, so we'll pull that out, process it
// and store the bit in the global irPacket variable. Going forward there
// is really nothing interesting that happens in this period, because all
// the logic 0 half-bits have the same period.  So we will turn off
// the timer interrupts and wait for the next (positive) edge on P2.6
//
// Positive Edge:
// The positive edge is associated with the end of the logic 0 half-bit
// and the start of the logic 1 half-bit.  There is nothing to do in
// terms of the logic 0 half bit because it does not encode any useful
// information.  On the other hand, we going into the logic 1 half of the bit
// and the portion which determines the bit value, the start of the
// packet, or if the timer rolls over, the end of the ir packet.
// Since the duration of this half-bit determines the outcome
// we will turn on the timer and its associated interrupt.
// -----------------------------------------------------------------------
#pragma vector = PORT2_VECTOR			// This is from the MSP430G2553.h file

__interrupt void pinChange (void) {

	int8	pin;
	int16	pulseDuration;			// The timer is 16-bits

	if (IR_PIN)		pin=1;	else pin=0;

	switch (pin) {					// read the current pin level
		case 0:						// !!!!!!!!!NEGATIVE EDGE!!!!!!!!!!
			pulseDuration = TAR;

			if ((pulseDuration < maxStartPulse) && (pulseDuration > minStartPulse)) {
				pulseDuration = 2;
			} else if ((pulseDuration < maxLogic1Pulse) && (pulseDuration > minLogic1Pulse)) {
				pulseDuration = 1;
			} else if ((pulseDuration < maxLogic0Pulse) && (pulseDuration > minLogic0Pulse)) {
				pulseDuration = 0;
			}

			packetData[packetIndex++] = pulseDuration;

			LOW_2_HIGH; 				// Setup pin interrupr on positive edge
			break;

		case 1:							// !!!!!!!!POSITIVE EDGE!!!!!!!!!!!
			TAR = 0x0000;						// time measurements are based at time 0
			HIGH_2_LOW; 						// Setup pin interrupr on positive edge
			break;
	} // end switch

	if (packetIndex > 40) {
		newIrPacket = TRUE;
	}

	P2IFG &= ~BIT6;			// Clear the interrupt flag to prevent immediate ISR re-entry

} // end pinChange ISR



// -----------------------------------------------------------------------
//			0 half-bit	1 half-bit		TIMER A COUNTS		TIMER A COUNTS
//	Logic 0	xxx
//	Logic 1
//	Start
//	End
//
// -----------------------------------------------------------------------
#pragma vector = TIMER0_A1_VECTOR			// This is from the MSP430G2553.h file
__interrupt void timerOverflow (void) {

	TACTL &= ~TAIFG;
}

// ------------------------------------------------------------------------
//	Function for moveRightWheelForward();
//	moves the right wheel forward with a specific duty cycle (aka. power)
//
//
//
//--------------------------------------------------------------------------

void bothForward() {

	P2DIR |= BIT2 | BIT5;
	P2SEL |= BIT2 | BIT5;

	P2DIR |= BIT3 | BIT4;
	P2OUT &= ~(BIT3 | BIT4);

	TA1CTL &= ~MC1 | MC0;       // stop timer A0

	TA1CTL |= TACLR;


	TA1CTL |= TASSEL1;			//


	TA1CCR0 = 100;
	TA1CCR1 = 50;
	TA1CCR2 = 50;

	TA1CCTL1 |= OUTMOD_3;
	TA1CCTL2 |= OUTMOD_3;


	TA1CTL |= MC_1;

	_delay_cycles(1000000);


}

void bothBackward() {

	P2DIR |= BIT2 | BIT5;
	P2SEL |= BIT2 | BIT5;

	P2DIR |= BIT3 | BIT4;
	P2OUT |= BIT3 | BIT4;

	TA1CTL &= ~MC1 | MC0;       // stop timer A0

	TA1CTL |= TACLR;


	TA1CTL |= TASSEL1;			//


	TA1CCR0 = 100;
	TA1CCR1 = 50;
	TA1CCR2 = 50;

	TA1CCTL1 |= OUTMOD_7;
	TA1CCTL2 |= OUTMOD_7;


	TA1CTL |= MC_1;

	_delay_cycles(1000000);


}

void tankRightForty() {

		P2DIR |= BIT2 | BIT5;
		P2SEL |= BIT2 | BIT5;

		P2DIR |= BIT3 | BIT4;
		P2OUT &= ~(BIT4);
		P2OUT |= BIT3;

		TA1CTL &= ~MC1 | MC0;       // stop timer A0

		TA1CTL |= TACLR;


		TA1CTL |= TASSEL1;			//


		TA1CCR0 = 100;
		TA1CCR1 = 50;
		TA1CCR2 = 50;

		TA1CCTL1 |= OUTMOD_3;
		TA1CCTL2 |= OUTMOD_7;


		TA1CTL |= MC_1;

		_delay_cycles(300000);


}
void tankLeftForty(){
	P2DIR |= BIT2 | BIT5;
	P2SEL |= BIT2 | BIT5;

	P2DIR |= BIT3 | BIT4;
	P2OUT |= BIT4;
	P2OUT &= ~BIT3;

	TA1CTL &= ~MC1 | MC0;       // stop timer A0

	TA1CTL |= TACLR;


	TA1CTL |= TASSEL1;			//


	TA1CCR0 = 100;
	TA1CCR1 = 50;
	TA1CCR2 = 50;

	TA1CCTL1 |= OUTMOD_7;
	TA1CCTL2 |= OUTMOD_3;


	TA1CTL |= MC_1;

	_delay_cycles(370000);

}

void tankRightNinety(){
	P2DIR |= BIT2 | BIT5;
			P2SEL |= BIT2 | BIT5;

			P2DIR |= BIT3 | BIT4;
			P2OUT &= ~(BIT4);
			P2OUT |= BIT3;

			TA1CTL &= ~MC1 | MC0;       // stop timer A0

			TA1CTL |= TACLR;


			TA1CTL |= TASSEL1;			//


			TA1CCR0 = 100;
			TA1CCR1 = 50;
			TA1CCR2 = 50;

			TA1CCTL1 |= OUTMOD_3;
			TA1CCTL2 |= OUTMOD_7;


			TA1CTL |= MC_1;

			_delay_cycles(580000);
}
void tankLeftNinety(){
	P2DIR |= BIT2 | BIT5;
	P2SEL |= BIT2 | BIT5;

	P2DIR |= BIT3 | BIT4;
	P2OUT |= BIT4;
	P2OUT &= ~BIT3;

	TA1CTL &= ~MC1 | MC0;       // stop timer A0

	TA1CTL |= TACLR;


	TA1CTL |= TASSEL1;			//


	TA1CCR0 = 100;
	TA1CCR1 = 50;
	TA1CCR2 = 50;

	TA1CCTL1 |= OUTMOD_7;
	TA1CCTL2 |= OUTMOD_3;


	TA1CTL |= MC_1;

	_delay_cycles(650000);
}
void stopRobot(){

	P2DIR &= ~(BIT2 | BIT5);

	P2DIR &= ~(BIT3 | BIT4);
	P2OUT &= ~(BIT3 | BIT4);

	TA1CTL &= ~MC1 | MC0;       // stop timer A0

	TA1CTL |= TACLR;

	_delay_cycles(1000000);
}
