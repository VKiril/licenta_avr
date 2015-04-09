#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#define F_CPU 8000000UL

#define STX 0x02 
#define ETX 0x03

#define IN_STX_POS          0 
#define IN_LENGTH_BIT_POS   1
#define IN_CMD_POS          2
#define IN_CRC_POS          3   
#define IN_ETX1_POS         4  
#define IN_ETX2_POS         5  

#define OUT_STX_POS          0
#define OUT_LENGTH_BIT_POS   1
#define OUT_CMD_POS          2
#define OUT_BIT0_POS         3
#define OUT_BIT1_POS         4
#define OUT_CRCL_POS         5
#define OUT_CRCH_POS         6
#define OUT_ETX1_POS         7
#define OUT_ETX2_POS         8


#define INCOMMING_ARRAY_LENGTH 6
#define OUT_COMMING_ARRAY_LENGTH 9

//cmd commands

#define START_CONVERSION_AND_TRANSMITING 0x06
#define STOP_CONVERSION_AND_TRANSMITING  0x07

//aliases
#define INCOMMING_ARRAY_ALIAS    0x04
#define OUT_COMMING_ARRAY_ALIAS  0x05

char counter = 0;
char startFlag = 0 ; 

char higgerBit = 0; 
char lowerBit = 0 ; 

//                                               |stx|length|cmd|crc|etx1|etx2|                 
char inCommingArray [INCOMMING_ARRAY_LENGTH]   = { 0,    0,   0,  0,  0,   0  } ;
//                                              |stx|length|cmd|bit0|bit1|crcL|crchH|etx1|etx2|
char outCommingArray[OUT_COMMING_ARRAY_LENGTH] = {0,    0,   0,   0,   0,   0,    0,  0,   0  }	;
//functions prototypes

void _portInit(void);
void _ADCinit(void);
void _timer1Init(void);
void _uartInit(void);
void _construct(void);
void _externInterruptInit(void);
void adcConversion(void);
void pingPulse(void);
void receiveArray(char);

void send(void);
void stopConversion(void);
void startConversion(char);

void initArray(char );

ISR(ADC_vect){
	adcConversion();
}

ISR(TIMER0_OVF_vect){
	//
}

ISR(INT0_vect){
	PORTB = 0xFF;
	send();
}

ISR(INT1_vect){
	PORTB = 0x00;
	counter = 0 ;
}

ISR(USART_UDRE_vect){
	send();
}

ISR(USART_RXC_vect){
	receiveArray(UDR);
}

void adcConversion(void){
	
	lowerBit  = ADCL ;
	higgerBit = ADCH;
	DDRC = 0XFF;
	if(startFlag == 1){
		PORTC=0b11111110;
		
		int crc = lowerBit + higgerBit + inCommingArray[IN_CMD_POS] + OUT_COMMING_ARRAY_LENGTH  ;
		outCommingArray[OUT_LENGTH_BIT_POS] = OUT_COMMING_ARRAY_LENGTH ;
		outCommingArray[OUT_CMD_POS] = inCommingArray[IN_CMD_POS] ;
		outCommingArray[OUT_BIT0_POS] = lowerBit ;
		outCommingArray[OUT_BIT1_POS] = higgerBit ;
		outCommingArray[OUT_CRCH_POS] = (char) crc << 8 ;
		outCommingArray[OUT_CRCL_POS] = (char) crc ;
		send();
	} else {
		PORTC=0b00000001;
	}
}


void initArray(char array){
	if(array == INCOMMING_ARRAY_ALIAS ){
		outCommingArray [OUT_STX_POS]  = STX ; 
		outCommingArray [OUT_ETX1_POS] = ETX ; 
		outCommingArray [OUT_ETX2_POS] = ETX ; 
	} 
}

void receiveArray(char bit){
	for (char i = 0 ; i < INCOMMING_ARRAY_LENGTH-1 ; i ++ ){
		inCommingArray[i] = inCommingArray[i+1] ;		
	}
	inCommingArray[INCOMMING_ARRAY_LENGTH-1] = bit ; 
	char crc  = 0 ;
	
	for(char i = 1 ; i < INCOMMING_ARRAY_LENGTH-2 ; i++){
		crc += inCommingArray[i] ; 
	}
	
	if(crc == inCommingArray[IN_CRC_POS] && inCommingArray[IN_STX_POS] == STX && inCommingArray[IN_ETX1_POS] == ETX && inCommingArray[IN_ETX2_POS] == ETX ){
		if( inCommingArray[IN_CMD_POS] == START_CONVERSION_AND_TRANSMITING ){
			startConversion(inCommingArray[IN_CMD_POS]);
		} 
		if( inCommingArray[IN_CMD_POS] == STOP_CONVERSION_AND_TRANSMITING  ){
			stopConversion();
		}
	}
}

void startConversion(char cmd){
	UCSRC |= (1<<URSEL);
	startFlag = 1 ; 
	send();
}

void stopConversion(void){
	UCSRB &=~(1<<UDRIE);
	startFlag = 0 ; 
}

void send(void){
	if(counter < OUT_COMMING_ARRAY_LENGTH){
		for(char i = 0 ; i < OUT_COMMING_ARRAY_LENGTH ; i++ ){
			UDR = outCommingArray[i] ;
		}
		
		UCSRB |=(1<<UDRIE);
		counter++;
	} else {
		UCSRB &=~(1<<UDRIE);
		counter = 0;
	}
}

void pingPulse(void){
	UCSRC |= (1<<URSEL);
	UDR = 0x0f;
	UCSRC &= ~(1<<URSEL);
}

int main(void){
	
	//_construct();
	_portInit   ();	
	_uartInit   ();
	_timer1Init ();
	_externInterruptInit();	
	_ADCinit    ();
	
	sei();
	pingPulse();
	
    while(1) 
	{ 		
    }
	return 1;
}



void _portInit(void){
	//adc port
	PORTA = 0X00;
	DDRA = 0XFF;
	
	//uart port
	PORTD = 0b00111100;
	DDRD  = 0b11111111;
	
	//led port
	PORTB = 0x00;
	DDRB = 0xFF;
}

void _ADCinit(void){
	
	
	ADMUX  |= ( 1<<REFS1 ) | ( 1<<REFS0 );//Internal 2.56V Voltage Reference with external capacitor at AREF pin
	ADCSRA |= ( 1<<ADEN  ) | ( 1<<ADSC  ) | ( 1<<ADIF ) | ( 1<<ADIE ) | ( 1<<ADPS2 ) | ( 1<<ADPS1 );//Division Factor 64 frequency 125000Hz
	
	SFIOR  |= ( 1<<ADTS2 );// trigger Timer/Counter0 Overflow
}

void _timer1Init(void){
	TCCR0 |= ( 1<<CS01)|(1<<CS00);//division factor 64
	TIMSK |= ( 1<<TOIE0) ;
}

void _uartInit(void){
	char baud = 12 ;
	UBRRH = (unsigned char)(baud>>8);
	UBRRL = (unsigned char) baud;
	//UBRRH = 0x00;
	//UBRRL = 0x05;//0b00000101

	//Receive Data Bit 8
	//TXB8: Transmit Data Bit 8
	UCSRB |=(1<<RXCIE)|(1<<TXCIE)|(1<<RXEN)|(1<<TXEN)|(1<<RXB8)|(1<<TXB8)|(1<<UDRIE);
	UCSRC &= ~(1<<URSEL);
}

void _externInterruptInit(void){
	//The falling edge of INT0 generates an interrupt request.
	MCUCR |= (1<<ISC01);
	GICR  |= (1<<INT0) ;
	GIFR  |= (1<<INTF0) ;

	MCUCR |= ( 1 << ISC11 ) ;//falling edge front
	GICR  |= ( 1 << INT1  ) ;//int1
	GIFR  |= ( 1 << INTF1 ) ;//int1_vect
}

void _construct(void){	
	_portInit   ();	
	_uartInit   ();
	_timer1Init ();
	_externInterruptInit();
	_ADCinit    ();	
}


