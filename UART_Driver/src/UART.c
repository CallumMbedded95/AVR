//********** USI_UART_Tx functions **********//
// Author: Callum McNeish

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "UART.h" 

#include <stdint-gcc.h>

#define CYCLES_PER_BIT     	( F_CPU / BAUD ) // defined in makefile
#define USI_USISR_CTR_LEN 		16

// UART Rx Parameters
#define HALF_CYCLES_PER_BIT	(CYCLES_PER_BIT * 1/2)
#define CLOCK_CYCLE_DELAY		21

// UART Tx Parameters
#define USI_FIRST_FRAME_LEN		7
#define USI_TOTAL_FRAME_LEN		10

#define TRUE					1
#define FALSE 					0

volatile static unsigned char Tx_Buffer[TX_BUFFER_LEN]; // Buffer for Tx data
volatile static unsigned char Tx_Head;	// Circular buffer head
volatile static unsigned char Tx_Tail;	// Circular buffer tail

volatile static unsigned char Rx_Buffer[RX_BUFFER_LEN];
volatile static unsigned char Rx_Head;	// Circular buffer head
volatile static unsigned char Rx_Tail;	// Circular buffer tail

unsigned char ReturnReceiveBufferTail() {
	unsigned char tmpTail = (Rx_Tail + 1) & RX_BUFFER_MASK;
	Rx_Tail = tmpTail;
	return Rx_Buffer[tmpTail];
}

unsigned char DataInReceiveBuffer() {
	if (Rx_Tail != Rx_Head)
		return 0x01; // true
	return 0x00;
}

struct UART_Status_Struct {
	unsigned char Tx_Active:1;
	unsigned char Tx_Transferring:1;
	unsigned char Tx_Idle:1;
	unsigned char Rx_Receive:1;
} volatile static UART_Status;

// Reverses the order of bits in a byte.
// I.e. MSB is swapped with LSB, etc.
unsigned char Bit_Reverse( unsigned char x ) {
    x = ((x >> 1) & 0x55) | ((x << 1) & 0xaa);
    x = ((x >> 2) & 0x33) | ((x << 2) & 0xcc);
    x = ((x >> 4) & 0x0f) | ((x << 4) & 0xf0);
    return x;    
}

unsigned char isIdle() {
	return UART_Status.Tx_Idle;
}

void flushBuffers() {
	Tx_Head = 0x00;
	Tx_Tail = 0x00;
	Rx_Head = 0x00;
	Rx_Tail = 0x00;
}

// Initialises Rx PB1 for receiving of data 
void initialiseRx() {	
	DDRB |= (0<<PB0)||(0<<PB1); // PB1 set to input and PB3 to make sure no data shifting occurs
	PORTB |= (1<<PB0); // INPUT SET HIGH 

	GIFR = 1<<PCIF; // pin change interrupt flag
	GIMSK = (1<<PCIE); //PIN CHANGE GLOBAL INTERRUPT ENABLE (PCINT0_vect)
	PCMSK = (1<<PCINT0); // Sets PinB0 to have interrupt vector enabled

	sei();
	PORTB &= ~(1<<PB3);
		
}

void receiveBytes() {
	cli();
	TCCR0A = (1<<WGM01)|(0<<WGM00);
	TCCR0B = (0<<WGM02)|(1<<CS00);
	
	TCNT0 = HALF_CYCLES_PER_BIT - CLOCK_CYCLE_DELAY; //set timer cntr to 0

	OCR0A = CYCLES_PER_BIT;

	USISR = (1<<USIOIF)|0x08; // start and clear interrupt flag AND counter to 16
	USICR = (1<<USIOIE)| // dont use this part? (1<<USISIE)|// start and counter cond interrupt enable
			(0<<USIWM1)|(1<<USIWM0)| // wire mode 3 wire
			(0<<USICS1)|(1<<USICS0)|(0<<USICLK); // counter/timer compare match CTC mode

	UART_Status.Rx_Receive = TRUE;

	sei();
}

// Interrupt vector for DI USI pin
ISR(PCINT0_vect) {
	unsigned char pinState = PINB;
	if (!(pinState & (0<<PB0))) { // And and if result is > 0 then true
		GIFR = 1<<PCIF; // pin change interrupt flag
		GIMSK &= ~(1<<PCIE); //PIN CHANGE GLOBAL INTERRUPT DISABLED (So PCINT0_vect doesnt continuously trigger)
		PCMSK &= ~(1<<PCINT0); // Sets PinB0 to have interrupt vector disabled like above ^
			
		receiveBytes(); // set USI for receive mode		
	}	
}


// Initialises Tx PB1 pin for transfer of data
void initialiseTx() {
	//sei(); // set global interrupts - not sure if required
	USICR = 0; //usi disabled - likely not necessary
	PORTB |= (1<<PB1); // DO/Tx Pin
	DDRB |= (1<<PB1); // USI Input pin

	UART_Status.Tx_Idle = TRUE;

	PORTB |= 1<<PB3;
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

	USICR = (1<<USIOIE) |								// Counter overflow interrupt flag
			(0<<USIWM1) | (1<<USIWM0) |					// Three wire mode set
			(0<<USICS1) | (1<<USICS0) | (0<<USICLK);	// Set to timer/counter0 compare match/Clock source
	USIDR = 0xFF; //start bit is low- keep high

	DDRB |= (1<<PB1);

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
	} else if (UART_Status.Rx_Receive) {
		unsigned char tmpHead = (Rx_Head + 1) & RX_BUFFER_MASK;
		Rx_Buffer[tmpHead] = USIDR;
		Rx_Head = tmpHead;
		USICR = 0x00;
		USISR = 1<<USIOIF|0xFF; // Clear interrupt flag
		UART_Status.Rx_Receive = FALSE;
		GIMSK = (1<<PCIE); //PIN CHANGE GLOBAL INTERRUPT ENABLE (PCINT0_vect)
		PCMSK = (1<<PCINT0); // Sets PinB0 to have interrupt vector enabled
	}
}