//********** USI_UART functions **********//
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "USI.h" 

// UART Parameters
#define BAUD				19200
#define USI_REG_CTR_SIZE		8
#define CYCLES_PER_BIT     	( F_CPU / BAUD ) // defined in makefile

// Reverses the order of bits in a byte.
// I.e. MSB is swapped with LSB, etc.
unsigned char Bit_Reverse( unsigned char x )
{
    x = ((x >> 1) & 0x55) | ((x << 1) & 0xaa);
    x = ((x >> 2) & 0x33) | ((x << 2) & 0xcc);
    x = ((x >> 4) & 0x0f) | ((x << 4) & 0xf0);
    return x;    
}

void initialiseTx() {
	TCCR0A = (1<<WGM01)|(0<<WGM00); // Set CTC mode (Compare to OCRA and clear)
	TCCR0B = (0<<WGM02)|(1<<CS00);	// Setting CTC mode part 2 AND no prescaling (for now)

	TCNT0 = 0;	// Timer counter set to 0

	OCR0A = CYCLES_PER_BIT;

	sei(); // set global interrupts - not sure if required

	USICR = (1<<USIOIE) |								// Counter overflow interrupt flag
			(0<<USIWM1) | (1<<USIWM0) |					// Three wire mode set
			(0<<USICS1) | (1<<USICS0) | (0<<USICLK);	// Set to timer/counter0 compare match/Clock source

	DDRB |= (1<<PB1); // USI Output pin

	USIDR = 0x00; //Reset USI Data Reg

	USISR = (1<<USIOIF)| // Clear interrupt flag on usi status reg
				(USI_REG_CTR_SIZE-TX_REGISTER_SIZE); // Set counter to 8 (it has a 16 (4-bit ctr), our tx register is size 8, we want it to count to 8)
}

void transmitBytes(unsigned char buffer) {
	USISR = (1<<USIOIF)| // Clear interrupt flag on usi status reg
		(USI_REG_CTR_SIZE-TX_REGISTER_SIZE); // REPLAE THIS LATER
}

ISR(USI_OVF_vect) {
    PORTB ^= (1 << 3); // use 3 for test (PB2 I think?)
    _delay_ms(1000);

    // PORTB = (0 << 3); 
    // _delay_ms(1000);
}