
#define BAUD 9600L
#define UBRR_value F_CPU / (BAUD * 16) - 1

#include <avr/io.h>

void UART_setup()
{
    // uart setup
    UCSR0B |= (1 << TXEN0);
    UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);
    UBRR0 = UBRR_value;
}

void UART_send_byte(uint8_t u8Data)
{
    // Wait until last byte has been transmitted
    while ((UCSR0A & (1 << UDRE0)) == 0)
        ;

    // Transmit data
    UDR0 = u8Data;
}
char digit_to_ASCII(char d)
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
void byto_to_hex_ASCII(uint8_t byte, char *result)
{
    result[1] = digit_to_ASCII(byte / 16);
    result[0] = digit_to_ASCII(byte % 16);
}
void UART_send_int(uint8_t tmp)
{
    char out[2];
    byto_to_hex_ASCII(tmp, out);

    UART_send_byte(out[1]);
    UART_send_byte(out[0]);
}