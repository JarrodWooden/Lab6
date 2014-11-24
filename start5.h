//-----------------------------------------------------------------
// Name:	Coulston
// File:	lab5.h
// Date:	Fall 2014
// Purp:	Include file for the MSP430
//-----------------------------------------------------------------

//-----------------------------------------------------------------
// Page 76 : MSP430 Optimizing C/C++ Compiler v 4.3 User's Guide
//-----------------------------------------------------------------
typedef		unsigned char		int8;
typedef		unsigned short		int16;
typedef		unsigned long		int32;
typedef		unsigned long long	int64;

#define		TRUE				1
#define		FALSE				0

//-----------------------------------------------------------------
// Function prototypes found in lab5.c
//-----------------------------------------------------------------
void initMSP430();
__interrupt void pinChange (void);
__interrupt void timerOverflow (void);

void bothForward();
void bothBackward();
void tankRightForty();
void tankLeftForty();
void tankRightNinety();
void tankLeftNinety();
void stopRobot();


//-----------------------------------------------------------------
// Each PxIES bit selects the interrupt edge for the corresponding I/O pin.
//	Bit = 0: The PxIFGx flag is set with a low-to-high transition
//	Bit = 1: The PxIFGx flag is set with a high-to-low transition
//-----------------------------------------------------------------

#define		IR_PIN			(P2IN & BIT6)
#define		HIGH_2_LOW		P2IES |= BIT6
#define		LOW_2_HIGH		P2IES &= ~BIT6


#define		averageLogic0Pulse	450
#define		averageLogic1Pulse	0x05DF
#define		averageStartPulse	22500
#define		minLogic0Pulse		averageLogic0Pulse - 200
#define		maxLogic0Pulse		averageLogic0Pulse + 200
#define		minLogic1Pulse		averageLogic1Pulse - 200
#define		maxLogic1Pulse		averageLogic1Pulse + 200
#define		minStartPulse		averageStartPulse - 1000
#define		maxStartPulse		averageStartPulse + 1000

#define		PWR		0xC2D0
#define		ONE		0xC284
#define		TWO		0xC244
#define		THR		0xC2C4

#define		VOL_UP	0xC228 //right
#define		VOL_DW	0xC2A8 //left
#define		CH_UP	0xC298 //up
#define		CH_DW	0xC218 //down
