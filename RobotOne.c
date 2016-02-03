/*
 * RobotOne.c
 *
 * Created: 1/21/2014 12:22:39 PM
 *  Author: Daniel Mack
 */ 


// includes
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// defines

#define HEARTRATE 156 // about 20 miliseconds or 50 Hertz
#define THROTTLE 0
#define FRONT 1
#define REAR 2
#define NEUTRAL 127
#define CENTER 127
#define YES 1
#define NO 0
#define TRUE 1
#define FALSE 0
#define ONTIME 150


// globals
volatile uint8_t heartBeat = 0;
uint8_t muscles[8];
uint8_t gameOver = 0;

// prototypes
void checkSensors(void);
void behaviorLogic(void);
void actions(void);
void endGame(void);
void putSPI(uint8_t);
void startSPI(void);
void endSPI(void);

int main(void){
	_delay_ms(500);
	uint8_t i;		
	// set muscles to neutral
	for(i = 0; i < 8; i++){
		muscles[i] = 127;
	}
	
	/* set up input and output ports
	MOSI as output, MISO as input, SCK as output, SS as output */
	DDRB =	0b10110000; 
	PORTB |= (1 << 4);
	/* set up spi
	spi enabled as master with fclk/16
	MOSI	user defined
	MISO	input
	SCK		user defined
	SS		user defined */
	SPCR = (1<<SPE) | (1<<MSTR) | (1<<SPR0);
	

	/* set up 8bit timer
	clk/1024, ctc, ctc interrupt */
	OCR0 = HEARTRATE;
	TCCR0 = (1<<WGM01) | (1<<CS02) | (1<<CS00);
	TIMSK = (1<<OCIE0);
	sei();
		
    while(1)
    {
		if(gameOver == TRUE){
			endGame();
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
	;
	return;
}

void behaviorLogic(void){
	static uint8_t timer = 0;
	// forward give throttle, wait 1 second and then cut throttle
	if(timer < ONTIME){
		muscles[THROTTLE] = NEUTRAL + 127;
		timer++;
		
	}
	else{
		muscles[THROTTLE] = NEUTRAL;
		timer = 0;
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
void endGame(void){
	while(1){
		TCCR0 = 0;
	}
}

void putSPI(uint8_t c){
	SPDR = c;
	while(!(SPSR & (1 << SPIF))){
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

void velocity(int8_t velocity uint8_t time){
	if(time > 0){
		muscle[THROTTLE] = NEUTRAL + velocity
		timer--;
	}
	else{
		return;
	}
}
void steering(int8_t frontServo)


// interrupts
ISR(TIMER0_COMP_vect){
	heartBeat = TRUE;
} 

