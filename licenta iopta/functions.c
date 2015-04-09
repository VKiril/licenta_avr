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