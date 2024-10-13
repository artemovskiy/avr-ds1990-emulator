#ifndef F_CPU
#define F_CPU 16000000
#endif

#define BAUD 9600L

#define UBRR_value F_CPU / (BAUD * 16) - 1

#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>
#include "pindef.h"
#include "onewire.h"
#include "ds18b20.h"

void Transmit_UData(unsigned char data)
{
   while (!(UCSR0A & (1 << UDRE0)))
      ; /// wait till transmit buffer empty
   UDR0 = data;
}

volatile unsigned char t;
unsigned char k;

const gpin_t sensorPin = {&PORTC, &PINC, &DDRC, PC2};
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

   Transmit_UData(out[1]);
   Transmit_UData(out[0]);
}

void UartSendString(char *str, uint8_t len)
{
   for (uint8_t i = 0; i < len; i++)
   {
      Transmit_UData(str[i]);
   }
}
char USART_Receive(void)
{
   // venter på at data bliver modtaget
   while (!(UCSR0A & (1 << RXC0)))
      ;
   // hent og retuner modtaget data fra bufferen
   return UDR0;
}

void main(void)
{
   DDRC |= 0b00000011;
   // PORTC |= 0b00000010;
   UCSR0C = 0b00000110;
   UCSR0B |= (1 << TXEN0) | (1 << RXEN0);
   // speed
   UBRR0 = UBRR_value;

   char wordBuffer[32];
   unsigned char wordSize;

         Transmit_UData('>');
   while (1)
   {
      unsigned char r = USART_Receive();
      if (false)
      {
         char debug[16];
         int len = sprintf(debug, "Debug: %X\r\n", r);
         UartSendString(debug, len);
      }
      switch (r)
      {
      case 0xD: // ENTER
         Transmit_UData('\r');
         Transmit_UData('\n');
         UartSendString(wordBuffer, wordSize);
         Transmit_UData('\r');
         Transmit_UData('\n');
         Transmit_UData('>');
         wordSize = 0;
         break;
      case 0x7F:
         wordSize--;
         Transmit_UData(r);
         break;
      default:
         wordBuffer[wordSize] = r;
         Transmit_UData(r);
         wordSize++;
      }
      // PORTC &= ~((1 << PC0) | (1 << PC1));
      // onewire_search_state state;
      // if (onewire_reset(&sensorPin))
      // {
      //    onewire_search_init(&state);
      //    bool result = onewire_search(&sensorPin, &state);
      //    if (result)
      //    {
      //       PORTC |= (1 << PC0);
      //       UartSendString("Number: ", 8);
      //       for (uint8_t i = 0; i < 8; i++)
      //       {
      //          snedInt(state.address[i]);
      //       }
      //    }
      //    else
      //    {
      //       UartSendString("Read failed", 11);
      //       PORTC |= (1 << PC1);
      //    }
      //    Transmit_UData('\r');
      //    Transmit_UData('\n');
      //    // // Start a temperature reading (this includes skiprom)
      //    // ds18b20_convert(&sensorPin);

      //    // // Wait for measurement to finish (750ms for 12-bit value)
      //    // _delay_ms(750);

      //    // // Get the raw 2-byte temperature reading
      //    // int16_t reading = ds18b20_read_single(&sensorPin);

      //    // if (reading != kDS18B20_CrcCheckFailed)
      //    // {
      //    //    // Convert to floating point (or keep as a Q12.4 fixed point value)
      //    //    float temperature = ((float)reading) / 16;

      //    //    // Transmit_UData((int)temperature);
      //    //    //  Transmit_UData('\r');
      //    //    //  Transmit_UData('\n');

      //    //    uint16_t tmp = (int) temperature;

      //    //    PORTC |= (1 << PC0);
      //    // }
      //    // else
      //    // {
      //    //    PORTC |= (1 << PC1);
      //    // }

      //    _delay_ms(1000);
      // }
   }
}