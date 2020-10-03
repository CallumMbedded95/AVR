#define TX_REGISTER_SIZE	4
#define STOP_BITS 			1

unsigned char Bit_Reverse( unsigned char x );
void initialiseTx();
void transmitBytes(unsigned char buffer);