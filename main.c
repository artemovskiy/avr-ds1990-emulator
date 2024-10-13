#ifndef F_CPU
#define F_CPU 16000000
#endif

#define BAUD 9600L

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "onewire.h"
#include "pindef.h"

#define RESET_LENGTH 350
#define MAX_WAITING_FOR_RISE 200

volatile uint8_t wr = 0;
volatile char wait_reset = 0;
volatile uint16_t reset_counter = 0;
volatile uint16_t wait_rise = 0;
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
      wait_reset = 1;
      wr = 0;
   }
}

gpin_t onewire_pin = {&PORTD, &PIND, &DDRD, PD2};
gpin_t D_pin = {&PORTC, &PINC, &DDRC, PC0};
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
      if (wait_reset)
      {
         if (reset_counter >= RESET_LENGTH)
         {
            wait_reset = 0;
            wait_rise = 1;
            reset_counter = 0;
         }
         else
         {
            _delay_us(10);
            reset_counter += 10;
         }
         continue;
      }

      if (wait_rise)
      {

         while (!(PIND & (1 << PD2)))
            ;
         wait_rise = 0;
         _delay_us(15);
         PORTD |= (1 << PD4);
         _delay_us(60);
         PORTD &= ~(1 << PD4);
         _delay_us(10);
         wait_tick = 1;

         continue;
      }
      if (wait_tick)
      {

         while (PIND & (1 << PD2))
            ;
         PORTC |= (1 << PC0);
         _delay_us(40);
         PORTC &= ~(1 << PC0);
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
            wait_tick = false;
            if (cmd == 0x33)
            {
               n = serial[byte_idx];
               wait_r = 1;
            }
         }
         continue;
      }
      if (start_rt)
      {
         // PORTC &= ~(1 << PC0);
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

            if (byte_idx >= bytes_len)
            {
               wait_r = 0;
               byte_idx = 0;
               PORTC |= (1 << PC0);
               _delay_ms(2000);
               PORTC &= ~(1 << PC0);
               wr = 1;
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