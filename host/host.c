/*
 * host.c
 *
 * Created: 19-Mar-13 02:52:24
 * Author: kitae_000
 */ 

#define F_CPU 16000000UL
#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>

void init() {
    MCUSR = 0;
    // disable unused features.
    PRR0 = 0b11101101;
    PRR1 = 0b00111011;
    // led
    DDRB = 0b10000000;
}

void configure_uarts() {
    // same baud rate
    UBRR0H = (BAUD_PRESCALE >> 8);
    UBRR0L = BAUD_PRESCALE;
    UBRR3H = (BAUD_PRESCALE >> 8);
    UBRR3L = BAUD_PRESCALE;

    // uart0: 8 bits, 1 stop for uart0, transmit
    UCSR0B |= (1 << TXEN0);
    UCSR0C |= (1 << UCSZ00) | (1 << UCSZ01);
    
    // uart3: 9 bits, 2 stop for uart3, receive, receive interrupt
    UCSR3B |= (1 << UCSZ32) | (1 << RXEN3)| (1 << RXCIE3);
    UCSR3C |= (1 << USBS3) | (1 << UCSZ30) | (1 << UCSZ31);

    // enable receive for rs485
    DDRE |= (1 << PINE5); // re, pin3, inverted, enabled
    DDRG |= (1 << PING5); // de, pin4, disabled
}

// on char receipt from uart3
ISR(USART3_RX_vect) {
    PORTB |= 0b10000000;
    char recevied_byte;
    while(UCSR3A & (1 << RXC3)) {
        // receive from 3
        recevied_byte = UDR3;
        // send to 0
        while((UCSR0A & (1 << UDRE0)) == 0);
        UDR0 = recevied_byte;
    }
    // flush 0
    while ((UCSR0A & (1 << TXC0)) == 0) {};
    UCSR0A |= (1 << TXC0);
    PORTB &= ~0b10000000;
}

int main(void) {
    init();
    configure_uarts();
    
    sleep_enable();
    set_sleep_mode(SLEEP_MODE_IDLE);

    sei();
    while(1) {
        sleep_mode();
    }
}
