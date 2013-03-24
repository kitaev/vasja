/*
 * controller.c
 *
 */ 

#define F_CPU 11059000UL
#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

#define PE 2
#define CP 3
#define Q7 1

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <avr/sleep.h>

char message[] = {'h','e','l','l','o',',',' ','v','a','s','j','a','!','\r','\n','\0'};

unsigned char read_state_spi() {
	// latch off
	PORTD &= ~(1 << 2);
	// latch on
	PORTB |= (1 << 7);
	PORTD |= (1 << 2);
	USIDR = 0xAA;
	USISR = (1 << USIOIF);
	do {
		USICR = (1<<USIWM0)|(1<<USICS1)|(1<<USICS0)|(1<<USICLK)|(1<<USITC);
	} while ((USISR & (1<<USIOIF)) == 0);
	return ~USIDR;
}

void send_char(char ch) {
    while ((UCSRA & (1 << UDRE)) == 0) {};
    UDR = ch;
}

ISR(WDT_OVERFLOW_vect) {
    unsigned char state = read_state_spi();

    // enable rs485 driver
    PORTD |= 0b00110000;
    send_char(0x30 + ((state & 0x70) >> 4));
    send_char(0x20);
    char *ch = message;
    while(*ch) {
        send_char(*ch++);
    }
    // wait for transmission to complete
    while ((UCSRA & (1 << TXC)) == 0) {};
    UCSRA |= (1 << TXC);

    // disable rs485 driver
    PORTD &= ~0b00110000;
}

void configure_uart() {
    UBRRH = (BAUD_PRESCALE >> 8);
    UBRRL = BAUD_PRESCALE;  
    UCSRB |= (1 << UCSZ2)| (1 << TXEN);
    UCSRC |= (1 << USBS) | (1 << UCSZ0) | (1 << UCSZ1);
}

void configure_timer() {
    wdt_reset();
    WDTCSR = (1 << WDCE) | (1 << WDIE);
    WDTCSR = (1 << WDIE) | (1 << WDP2) | (1 << WDP1);
}

void init_ports() {
    // 4 : de
    // 5 : re
	// 2 : latch pin
    DDRD  |= 0b00110100;
	PORTD |= 0b00000100;

    // 7 : usck
    // 6 : do (out)
    DDRB  |= 0b11000000;
    // 5 : di (pull up?)
    PORTB |= 0b00100000;
}

int main(void) {
    MCUSR = 0;

    init_ports();
    configure_uart();
    configure_timer();
    
    sleep_enable();
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);

    sei();
    while(1) {
        sleep_mode();
    }
}
