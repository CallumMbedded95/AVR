//********** USI_UART_Tx functions **********//
// Author: Callum McNeish

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "USI_Tx.h" 

#include <stdint-gcc.h>

// UART Parameters
#define USI_USISR_CTR_LEN 		16
#define USI_FIRST_FRAME_LEN		7
#define USI_TOTAL_FRAME_LEN		10
#define CYCLES_PER_BIT     	( F_CPU / BAUD ) // defined in makefile
#define TRUE					1
#define FALSE 					0

volatile static unsigned char Tx_Buffer[TX_BUFFER_LEN]; // Buffer for Tx data
volatile static unsigned char Tx_Head;	// Circular buffer head
volatile static unsigned char Tx_Tail;	// Circular buffer tail

struct UART_Status_Struct {
	unsigned char Tx_Active:1;
	unsigned char Tx_Transferring:1;
	unsigned char Tx_Idle:1;
} volatile static UART_Status;

// Reverses the order of bits in a byte.
// I.e. MSB is swapped with LSB, etc.
unsigned char Bit_Reverse( unsigned char x ) {
    x = ((x >> 1) & 0x55) | ((x << 1) & 0xaa);
    x = ((x >> 2) & 0x33) | ((x << 2) & 0xcc);
    x = ((x >> 4) & 0x0f) | ((x << 4) & 0xf0);
    return x;    
}

void flushBuffers() {
	Tx_Head = 0x00;
	Tx_Tail = 0x00;
}

// Initialises Rx PB1 for receiving of data 
void initialiseRx() {
	cli();
	PORTB = (0<<PB0); // make sure PB1 is input
	DDRB |= (0<<PB0);

	TCNT0 = 0; //set timer cntr to 0
	TCCR0A = (1<<WGM01)|(0<<WGM00);
	TCCR0B = (0<<WGM02)|(1<<CS00);

	OCR0A = CYCLES_PER_BIT;

	USISR = (1<<USISIF)|0xFF; // clear interrupt flag
	USICR = (1<<USISIE)|(1<<USIOIE)| // start and counter cond interrupt enable
			(0<<USIWM1)|(1<<USIWM0)| // wire mode 3 wire
			(0<<USICS1)|(1<<USICS0)|(0<<USICLK); // counter/timer compare match CTC mode

	DDRB |= (1<<PB3); // test led to see if receive triggers

	sei();
}

// Interrupt vector for DI USI pin
ISR(USI_START_vect) {
	PORTB |= (1<<PB3);
}


// Initialises Tx PB1 pin for transfer of data
void initialiseTx() {
	//sei(); // set global interrupts - not sure if required
	USICR = 0; //usi disabled - likely not necessary
	PORTB = (1<<PB1); // DO/Tx Pin
	DDRB |= (0<<PB1); // USI Input pin

	UART_Status.Tx_Idle = TRUE;
}

// Sets up the buffer with data to transmit
void transmitBytes(unsigned char data) {
	unsigned char Tx_Buf_Ind = (Tx_Head+1) & TX_BUFFER_MASK; // Will account for overflow
	while(Tx_Buf_Ind == Tx_Tail); // We must wait for room in buffer

	Tx_Head = Tx_Buf_Ind; // Set new head
	Tx_Buffer[Tx_Head] = Bit_Reverse(data);

	setInternal_Tx();

	while (!UART_Status.Tx_Idle);
	UART_Status.Tx_Active = TRUE;
}

// Called by transmit bytes to set Timer/Count0 and USI status and data registers
void setInternal_Tx() {
	cli(); // cannot have an interrupt happening during this - for half duplex
	TCCR0A = (1<<WGM01)|(0<<WGM00); // Set CTC mode (Compare to OCRA and clear)
	TCCR0B = (0<<WGM02)|(1<<CS00);	// Setting CTC mode part 2 AND no prescaling (for now)

	TCNT0 = 0;	// Timer counter set to 0

	OCR0A = CYCLES_PER_BIT; // Used as CTC compare against TCNT0 - triggers USI Overflow

	USIDR = 0xFF; //start bit is low- keep high

	USICR = (1<<USIOIE) |								// Counter overflow interrupt flag
			(0<<USIWM1) | (1<<USIWM0) |					// Three wire mode set
			(0<<USICS1) | (1<<USICS0) | (0<<USICLK);	// Set to timer/counter0 compare match/Clock source

	DDRB  |= (1<<PB1);
	DDRB  |= (1<<3);

	USISR = 1<<USIOIF | 0xFF;     //Clear USI int flag from status reg AND set ctr to 8 - so we can count to 8

	sei(); // set global interrupts again
}

ISR(USI_OVF_vect) {
	if (UART_Status.Tx_Active ) {
		unsigned char tmptail = (Tx_Tail + 1) & TX_BUFFER_MASK;
		USIDR = 0x80|(Tx_Buffer[tmptail] >> 2);

		USISR = 1<<USIOIF | // Clear USI interrupt flag
				(USI_USISR_CTR_LEN - USI_FIRST_FRAME_LEN);     //Clear USI int flag from status reg AND set ctr to 8 - so we can count to 8

		Tx_Tail = tmptail;
		UART_Status.Tx_Active = FALSE;
		UART_Status.Tx_Transferring = TRUE;

	} else if (UART_Status.Tx_Transferring) {		
		USIDR = (Tx_Buffer[Tx_Tail] << 5)|(0x1F);

		USISR = (1<<USIOIF)| // Clear interrupt flag on usi status reg
			(USI_USISR_CTR_LEN - (USI_TOTAL_FRAME_LEN - USI_FIRST_FRAME_LEN)); // Set counter to 16 - stop bits and - last USIDR bit

		UART_Status.Tx_Transferring = FALSE;
		UART_Status.Tx_Idle = TRUE;
	} else if (UART_Status.Tx_Idle) {		
		USICR = 0x00; // Turn off USI
		USISR = 1<<USIOIF; // Clear interrupt flag			
	}
}