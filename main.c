#ifndef F_CPU
#define F_CPU 16000000
#endif

#define BAUD 9600L
#define UBRR_value F_CPU / (BAUD * 16) - 1

#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#define MIN_RESET_LENGTH 300
#define MAX_WAITING_FOR_RISE 1000
#define RISE_STEP 5

#define S_IDLE 0
#define S_RSET 1
#define S_PRSN 2
#define S_RCMD 3
#define S_DIAL 4
#define S_RSND 5

#define SERIAL_LENGTH 8

uint8_t s = S_IDLE;
uint16_t reset_time_cnt;
uint8_t tick_idx;
uint8_t recieved_bit_index;
uint8_t current_byte;
uint8_t response_bit_idx;
uint8_t response_byte_idx;
uint8_t serial[SERIAL_LENGTH];

void UART_SendByte(uint8_t u8Data)
{
   // Wait until last byte has been transmitted
   while ((UCSR0A & (1 << UDRE0)) == 0)
      ;

   // Transmit data
   UDR0 = u8Data;
}

char digitToASCII(char d)
{
   if (0 <= d && d <= 9)
   {
      return d + '0';
   }
   if (0xA <= d && d <= 0xF)
   {
      return d - 10 + 'A';
   }
   return 0;
}
void bytoToHexASCII(uint8_t byte, char *result)
{
   result[1] = digitToASCII(byte / 16);
   result[0] = digitToASCII(byte % 16);
}
void snedInt(uint8_t tmp)
{
   char out[2];
   bytoToHexASCII(tmp, out);

   UART_SendByte(out[1]);
   UART_SendByte(out[0]);
}

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
   // uart setup
   UCSR0B |= (1 << TXEN0);
   UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);
   UBRR0 = UBRR_value;
   // int setup
   EICRA |= (1 << ISC01); // trigger on falling edge
   EIMSK |= (1 << INT0);  // enable int on INT0 pin
   // read the serial and transmit to UART
   eeprom_read_block((void *)&serial, (const void *)0, sizeof(serial));
   for (uint8_t i = 0; i < SERIAL_LENGTH; i++)
   {
      snedInt(serial[i]);
   }
   UART_SendByte('\r');
   UART_SendByte('\n');

   // init completed signal
   PORTC |= (1 << PC0);
   _delay_ms(100);
   PORTC &= ~(1 << PC0);

   sei();
   while (1)
   {
      if (s == S_RSET)
      {
         if (reset_time_cnt > MAX_WAITING_FOR_RISE)
         {
            s = S_IDLE;
            reset_time_cnt = 0;
            continue;
         }
         if (PIND & (1 << PD2))
         {
            if (reset_time_cnt < MIN_RESET_LENGTH)
            {
               s = S_IDLE;
               reset_time_cnt = 0;
               continue;
            }
            _delay_us(15);

            PORTD |= (1 << PD4);
            _delay_us(60);
            PORTD &= ~(1 << PD4);
            _delay_us(10);
            s = S_RCMD;
            reset_time_cnt = 0;
            continue;
         }
         _delay_us(RISE_STEP);
         reset_time_cnt += RISE_STEP;
      }
      if (s == S_RCMD)
      {

         while (PIND & (1 << PD2))
            ;
         _delay_us(40);
         if (PIND & (1 << PD2))
         {
            recieved_bit_index += 1 << tick_idx;
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
            if (recieved_bit_index == 0x33)
            {
               current_byte = serial[response_byte_idx];
               s = S_DIAL;
            }
            else
            {
               s = S_IDLE;
            }
            recieved_bit_index = 0;
         }
         continue;
      }
      if (s == S_RSND)
      {
         if (current_byte % 2)
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
         current_byte = current_byte >> 1;
         response_bit_idx++;
         if (response_bit_idx > 7)
         {
            response_bit_idx = 0;
            response_byte_idx++;

            if (response_byte_idx >= SERIAL_LENGTH)
            {
               s = S_IDLE;
               response_byte_idx = 0;
               // respond completed signal
               PORTC |= (1 << PC0);
               _delay_ms(1000);
               PORTC &= ~(1 << PC0);
               continue;
            }
            else
            {
               current_byte = serial[response_byte_idx];
               continue;
            }
         }
      }
   }
}