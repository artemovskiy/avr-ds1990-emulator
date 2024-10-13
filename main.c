#ifndef F_CPU
#define F_CPU 16000000
#endif

#define BAUD 9600L

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define MIN_RESET_LENGTH 300
#define MAX_WAITING_FOR_RISE 1000
#define RISE_STEP 5

#define S_IDLE 0
#define S_RSET 1
#define S_PRSN 2
#define S_RCMD 3
#define S_DIAL 4
#define S_RSND 5

volatile uint8_t s = S_IDLE;

volatile uint16_t rise_cnt;
volatile char tick_idx;
uint8_t cmd;
volatile uint8_t ri;
volatile uint8_t n;

char bytes_len = 8;
volatile uint8_t byte_idx;
const uint8_t serial[8] = {0x01, 0xF1, 0xCF, 0x7B, 0x14, 0, 0, 0xDD};

ISR(INT0_vect)
{
   if (s == S_DIAL)
   {
      PORTD |= (1 << PD4);
      s = S_RSND;
      return;
   }
   if (s == S_IDLE)
   {
      s = S_RSET;
   }
}

void main(void)
{
   DDRC |= (1 << PC0);
   DDRD = (1 << PD4);
   PORTD &= ~(1 << PD4);

   // int setup
   EICRA |= (1 << ISC01); // trigger on falling edge
   EIMSK |= (1 << INT0);  // enable int on INT0 pin

   // init completed signal
   PORTC |= (1 << PC0);
   _delay_ms(100);
   PORTC &= ~(1 << PC0);

   sei();
   while (1)
   {
      if (s == S_RSET)
      {
         if (rise_cnt > MAX_WAITING_FOR_RISE)
         {
            s = S_IDLE;
            rise_cnt = 0;
            continue;
         }
         if (PIND & (1 << PD2))
         {
            if (rise_cnt < MIN_RESET_LENGTH)
            {
               s = S_IDLE;
               rise_cnt = 0;
               continue;
            }
            _delay_us(15);

            PORTD |= (1 << PD4);
            _delay_us(60);
            PORTD &= ~(1 << PD4);
            _delay_us(10);
            s = S_RCMD;
            rise_cnt = 0;
            continue;
         }
         _delay_us(RISE_STEP);
         rise_cnt += RISE_STEP;
      }
      if (s == S_RCMD)
      {

         while (PIND & (1 << PD2))
            ;
         _delay_us(40);
         if (PIND & (1 << PD2))
         {
            cmd += 1 << tick_idx;
         }
         else
         {
            while (!(PIND & (1 << PD2)))
               ;
         }
         tick_idx++;

         if (tick_idx > 7)
         {
            tick_idx = 0;
            if (cmd == 0x33)
            {
               n = serial[byte_idx];
               s = S_DIAL;
            }
            else
            {
               s = S_IDLE;
            }
            cmd = 0;
         }
         continue;
      }
      if (s == S_RSND)
      {
         if (n % 2)
         {
            _delay_us(2);
            PORTD &= ~(1 << PD4);
         }
         else
         {
            _delay_us(40);
            PORTD &= ~(1 << PD4);
         }
         s = S_DIAL;
         n = n >> 1;
         ri++;
         if (ri > 7)
         {
            ri = 0;
            byte_idx++;

            if (byte_idx >= bytes_len)
            {
               s = S_IDLE;
               byte_idx = 0;
               PORTC |= (1 << PC0);
               _delay_ms(1000);
               PORTC &= ~(1 << PC0);
               continue;
            }
            else
            {
               n = serial[byte_idx];
               continue;
            }
         }
      }
   }
}