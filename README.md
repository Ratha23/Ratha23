#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>

#define TRIG_PIN PB0
#define ECHO_PIN PB1


void initUART() {
  UBRR0H = 0x00;
  UBRR0L = 8; 
  UCSR0B = (1<<TXEN0) | (1<<RXEN0); // enable transmitter and receiver
  UCSR0C = (1<<UCSZ01) | (1<<UCSZ00); // 8-bit data, no parity, 1 stop bit
} 

void initTimer() {
  TCCR1B |= (1<<CS10); // set prescaler to 1
}

void sendUART(uint8_t data) {
  while (!(UCSR0A & (1<<UDRE0))); // wait for empty transmit buffer
  UDR0 = data; // send data
}

uint16_t pulseIn() {
  uint16_t pulseWidth = 0;
  // send a 10us HIGH pulse to the trigger pin
  PORTB |= (1<<TRIG_PIN);
  _delay_us(10);
  PORTB &= ~(1<<TRIG_PIN);
  // measure the duration of the HIGH pulse on the echo pin
  while (!(PINB & (1<<ECHO_PIN))); // wait for HIGH state
  TCNT1 = 0; // reset counter
  while (PINB & (1<<ECHO_PIN)) { // measure pulse width
    if (TCNT1 > 50000) break; // timeout after 50000 cycles (about 60ms)
    pulseWidth = TCNT1;
  }
  return pulseWidth * 0.017; // convert to microseconds
}



int main() {
  initUART();
  initTimer();
  //timer2 fast pwm;
  DDRB = 1<<PB3;
  DDRD = 1<<PD3;
  TCCR2A = (1<<COM2A1)| (1<<COM2B1) | (1<<WGM21) | (1<<WGM20); // Non-inverting Mode and Fast PWM 8-BIT
  TCCR2B =  1<<CS21;

  
  DDRB |= (1<<TRIG_PIN); // set trigger pin as output
  PORTB &= ~(1<<TRIG_PIN); // set trigger pin LOW
  DDRB &= ~(1<<ECHO_PIN); // set echo pin as input
  
  DDRD = 0b11101000; // set LED pins as output
  uint16_t distance;
  
  while (1) {
    distance = (pulseIn()/ 58)*10; // convert pulse duration to distance in cm
    sendUART(distance); // send distance data via UART
        
    if (distance <= 50) {
      
      OCR2A = 1;
      OCR2B = 1;
      PORTD = 0b01000000;
      _delay_ms(2000);
      OCR2B = 1;
      OCR2A = 120;
      PORTD = 0b01000000;  
      _delay_ms(1500);
      
      } else if (distance >50 && distance <= 100) {
        OCR2A =120;
      OCR2B =120;
      PORTD = 0b01100000;
      
    }
    else if (distance > 100)
    {
      OCR2A =254;
      OCR2B =254;
      PORTD = 0b01110000;
      
    }
    _delay_ms(10); 
  }
  return 0;
}
