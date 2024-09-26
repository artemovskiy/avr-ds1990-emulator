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

volatile uint8_t wr = 0;
volatile char wait_reset = 0;
volatile uint16_t reset_counter = 0;
volatile uint16_t wait_rise = 0;
volatile uint16_t rise_cnt = 0;
volatile char wait_tick = 0;
volatile char tick_idx = 0;
uint8_t cmd = 0;
volatile uint8_t wait_r = 0;
volatile uint8_t ri = 0;
volatile uint8_t n = 0xC5;

char bytes_len = 8;
volatile uint8_t byte_idx = 0;
const uint8_t serial[8] = {0x01, 0xF1, 0xCF, 0x7B, 0x14, 0, 0, 0xDD};
volatile uint8_t start_rt = 0;

ISR(INT0_vect)
{
   if (wait_r)
   {
      PORTD |= (1 << PD4);
      start_rt = 1;
      return;
   }
   if (wr)
   {
      wait_rise = 1;
      wr = 0;
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
   wr = 1;
   while (1)
   {
      if (wait_rise)
      {
         if (rise_cnt > MAX_WAITING_FOR_RISE)
         {
            wait_rise = 0;
            rise_cnt = 0;
            wr = 1;
            continue;
         }
         if (PIND & (1 << PD2))
         {
            if (rise_cnt < MIN_RESET_LENGTH)
            {
               wait_rise = 0;
               rise_cnt = 0;
               wr = 1;
               continue;
            }
            _delay_us(15);

            PORTD |= (1 << PD4);
            _delay_us(60);
            PORTD &= ~(1 << PD4);
            _delay_us(10);
            wait_rise = 0;
            wait_tick = 1;
            rise_cnt = 0;
            continue;
         }
         _delay_us(RISE_STEP);
         rise_cnt += RISE_STEP;
      }
      if (wait_tick)
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
            wait_tick = 0;
            if (cmd == 0x33)
            {
               n = serial[byte_idx];
               wait_r = 1;
            }
            else
            {
               wr = 1;
            }
            cmd = 0;
         }
         continue;
      }
      if (start_rt)
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
         n = n >> 1;
         ri++;
         start_rt = 0;
         if (ri > 7)
         {
            ri = 0;
            byte_idx++;

            wr = 1;
            if (byte_idx >= bytes_len)
            {
               wait_r = 0;
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