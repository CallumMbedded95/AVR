//********** USI_UART_Tx functions **********//
// Author: Callum McNeish

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "USI.h" 

#include <stdint-gcc.h>

// UART Parameters
#define USI_REG_CTR_SIZE		8
#define CYCLES_PER_BIT     	( F_CPU / BAUD ) // defined in makefile

volatile static unsigned char Tx_Static = 0b01101000; // Letter a char

//termporary section
enum Tx_State {First, Second};
volatile static enum Tx_State test = First;
//


// Reverses the order of bits in a byte.
// I.e. MSB is swapped with LSB, etc.
unsigned char Bit_Reverse( unsigned char x ) {
    x = ((x >> 1) & 0x55) | ((x << 1) & 0xaa);
    x = ((x >> 2) & 0x33) | ((x << 2) & 0xcc);
    x = ((x >> 4) & 0x0f) | ((x << 4) & 0xf0);
    return x;    
}

void initialiseTx() {
	sei(); // set global interrupts - not sure if required
	USICR = 0; //usi disabled - likely not necessary
	PORTB = (1<<PB1); // DO/Tx Pin
	DDRB |= (1<<PB1); // USI Output pin
}

void transmitBytes(unsigned char data) {
	Tx_Static = Bit_Reverse(data); //Tx data

	TCCR0A = (1<<WGM01)|(0<<WGM00); // Set CTC mode (Compare to OCRA and clear)
	GTCCR |= 1 << PSR0; 
	TCCR0B = (0<<WGM02)|(1<<CS00);	// Setting CTC mode part 2 AND no prescaling (for now)

	TCNT0 = 0;	// Timer counter set to 0

	OCR0A = CYCLES_PER_BIT; // Used as CTC compare against TCNT0 - triggers USI Overflow

	// Delete after *************
	USIDR = 0x00|(Tx_Static >> 1); // 0 start bit apparently
	// Delete after *************


	USICR = (1<<USIOIE) |								// Counter overflow interrupt flag
			(0<<USIWM1) | (1<<USIWM0) |					// Three wire mode set
			(0<<USICS1) | (1<<USICS0) | (0<<USICLK);	// Set to timer/counter0 compare match/Clock source

	DDRB  |= (1<<PB1);                      
    USISR = 1<<USIOIF | (16 - USI_REG_CTR_SIZE);     //Clear USI int flag from status reg AND set ctr to 8 - so we can count to 8
}

ISR(USI_OVF_vect) {
	if (test == First) {
		test = Second;
		USIDR = (Tx_Static<<7)|(0x7F); //High edge and start bit (start = low edge)

		USISR = (1<<USIOIF)| // Clear interrupt flag on usi status reg
				(16 - STOP_BITS - 1); // Set counter to 16 - stop bits and - last USIDR bit
	} else if (test == Second) {
		test = First;

		USICR = 0x00; //turn off
		PORTB = (1<<PB1); // DO/Tx Pin
		DDRB |= (1<<PB1); // USI Output pin
		USISR = 1<<USIOIF;
	}
}