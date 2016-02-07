/*
 * RobotOne.c
 *
 * atmega2560
 * 16 MHz
 * Created: 2/6/2016 11:40:44 PM
 * Author : kidwidget
 */ 


// includes
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// defines
#define HEARTRATE 156	// about 20 ms or 50 Hz
#define THROTTLE 0
#define FRONTSERVO 1
#define REARSERVO 2
#define NEUTRAL 127
#define CENTER 127
#define YES 1
#define NO 0
#define TRUE 1
#define FALSE 0
#define ONTIME 300

// globals
volatile uint8_t heartBeat = 0;
uint8_t muscles[8];
uint8_t gameOver = 0;

// prototypes
void checkSensors(void);
void behaviorLogic(void);
void actions(void);
void endgame(void);
void putSPI(uint8_t);
void startSPI(void);
void endSPI(void);

int main(void){
	_delay_ms(500);
	uint8_t i;
	// set servos to neutral
	for(i = 0; i < 8; i++){
		muscles[i] = NEUTRAL;
	}
	// set up input and output ports
	// mosi as output, miso as input, sck as output, ss as output
	DDRB = (1<<PB2) | (1<<PB1) | (1<<PB0);
	PORTB |= (1<<4);
	
	// set up spi
	// spi enabled as master with fclk/16
	// MOSI	user defined
	// MISO	input
	// SCK	user defined
	// SS	user defined
	SPCR = (1<<SPE) | (1<<MSTR) | (1<<SPR0);
	
	// set up 8 bit timer
	// CLK/1024, ctc, ctc interrupt
	OCR0A = HEARTRATE;
	TCCR0A = (1<<WGM01);
	TCCR0B = (1<<CS02) | (1<<CS01);
	TIMSK0 = (1<<OCIE0A);
	sei();
	
	while(1){
		if(gameOver == TRUE){
			endgame();
		}
		while(heartBeat == TRUE){
			heartBeat = FALSE;
			checkSensors();
			behaviorLogic();
			actions();
		}
	}	
}

// functions
void checkSensors(void){
	return;
}

void behaviorLogic(void){
	static uint8_t timer = 0;
	// forward for 1 second then cut throttle
	if(timer < ONTIME){
		muscles[THROTTLE] = NEUTRAL + 127;
		timer++;
	}
	else{
		muscles[THROTTLE] = NEUTRAL;
		// timer = 0;
	}
	return;
}

void actions(void){
	uint8_t i;
	startSPI();
	for(i = 0; i < 8; i++){
		putSPI(i);
		putSPI(muscles[i]);
	}
	endSPI();
	return;
}

void endgame(void){
	while(1){
		TCCR0A = 0;
	}
}

void putSPI(uint8_t c){
	SPDR = c;
	while(!(SPSR & (1 <<SPIF))){
		;
	}
	return;
}
void startSPI(void){
	PORTB &= ~(1 << 4);
	return;
}

void endSPI(void){
	PORTB |= (1 << 4);
	return;
}

// interrupts
ISR(TIMER0_COMPA_vect){
	static uint8_t i = 0;
	// set heartbeat every other time
	if(i%2){
		heartBeat = TRUE;
	}
}
