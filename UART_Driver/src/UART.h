//********** USI_UART_Tx header **********//
// Author: Callum McNeish

#define TX_BUFFER_LEN		4
#define TX_BUFFER_MASK 		(TX_BUFFER_LEN-1)
#define RX_BUFFER_LEN		4
#define RX_BUFFER_MASK 		(RX_BUFFER_LEN-1)
#define STOP_BITS 			1
//#define BAUD				19200
#define BAUD 9600

unsigned char Bit_Reverse(unsigned char x);
void initialiseTx();
void initialiseRx();
void transmitBytes(unsigned char byte);
void setInternal_Tx();
void flushBuffers();
unsigned char ReturnReceiveBufferTail();
unsigned char DataInReceiveBuffer();
unsigned char isIdle();