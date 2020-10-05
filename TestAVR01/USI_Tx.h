//********** USI_UART_Tx header **********//
// Author: Callum McNeish

#define TX_REGISTER_SIZE	4
#define STOP_BITS 			1
#define BAUD				19200

unsigned char Bit_Reverse( unsigned char x );
void initialiseTx();
void transmitBytes(unsigned char byte);