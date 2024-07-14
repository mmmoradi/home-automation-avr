/*
mmahdim
Home Automation with AVR

chip     : ATmega32
frequency: 8.000000 MHz
*/

#include <avr/io.h>
#include <util/delay.h>
#include <lcd.h>
#include <stdio.h>

// Voltage Reference: AREF pin
#define ADC_VREF_TYPE ((0 << REFS1) | (0 << REFS0) | (0 << ADLAR))

// Read the AD conversion result
uint16_t read_adc(uint8_t adc_input)
{
    ADMUX = adc_input | ADC_VREF_TYPE;
    // Delay needed for the stabilization of the ADC input voltage
    _delay_us(10);
    // Start the AD conversion
    ADCSRA |= (1 << ADSC);
    // Wait for the AD conversion to complete
    while ((ADCSRA & (1 << ADIF)) == 0)
        ;

    ADCSRA |= (1 << ADIF);
    return ADCW;
}

int main(void)
{
    uint16_t pot_adc = 0;
    uint16_t pot_adc_update = 0;
    uint16_t pot_pin_update = 0;

    uint16_t temp_adc = 0;
    uint16_t temp_adc_update = 0;
    uint16_t temp = 0;

    uint8_t button_state = 0;
    uint8_t button_update = 0;

    uint8_t lcd_update = 1;
    uint8_t lcd_state = 1;

    uint8_t speed = 1;
    uint8_t threshold = 20;
    uint8_t enable = 0;

    // Input/Output Ports initialization
    // Port A
    // Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
    DDRA = (0 << DDA7) | (0 << DDA6) | (0 << DDA5) | (0 << DDA4) | (0 << DDA3) | (0 << DDA2) | (0 << DDA1) | (0 << DDA0);
    // State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
    PORTA = (0 << PORTA7) | (0 << PORTA6) | (0 << PORTA5) | (0 << PORTA4) | (0 << PORTA3) | (0 << PORTA2) | (0 << PORTA1) | (0 << PORTA0);

    // Port B
    // Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=Out Bit2=In Bit1=In Bit0=In
    DDRB = (0 << DDB7) | (0 << DDB6) | (0 << DDB5) | (0 << DDB4) | (1 << DDB3) | (0 << DDB2) | (0 << DDB1) | (0 << DDB0);
    // State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=0 Bit2=T Bit1=T Bit0=T
    PORTB = (1 << PORTB7) | (1 << PORTB6) | (1 << PORTB5) | (1 << PORTB4) | (0 << PORTB3) | (0 << PORTB2) | (0 << PORTB1) | (1 << PORTB0);

    // Port C
    // Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
    DDRC = (0 << DDC7) | (0 << DDC6) | (0 << DDC5) | (0 << DDC4) | (0 << DDC3) | (0 << DDC2) | (0 << DDC1) | (0 << DDC0);
    // State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
    PORTC = (0 << PORTC7) | (0 << PORTC6) | (0 << PORTC5) | (0 << PORTC4) | (0 << PORTC3) | (0 << PORTC2) | (0 << PORTC1) | (0 << PORTC0);

    // Port D
    // Function: Bit7=Out Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
    DDRD = (1 << DDD7) | (0 << DDD6) | (0 << DDD5) | (0 << DDD4) | (0 << DDD3) | (0 << DDD2) | (0 << DDD1) | (0 << DDD0);
    // State: Bit7=0 Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
    PORTD = (0 << PORTD7) | (0 << PORTD6) | (0 << PORTD5) | (0 << PORTD4) | (0 << PORTD3) | (0 << PORTD2) | (0 << PORTD1) | (0 << PORTD0);

    // Timer/Counter 0 initialization
    // Clock source: System Clock
    // Clock value: 1000.000 kHz
    // Mode: Fast PWM top=0xFF
    // OC0 output: Non-Inverted PWM
    // Timer Period: 0.256 ms
    // Output Pulse(s):
    // OC0 Period: 0.256 ms Width: 0.026102 ms
    TCCR0 = (1 << WGM00) | (1 << COM01) | (0 << COM00) | (1 << WGM01) | (0 << CS02) | (1 << CS01) | (0 << CS00);
    TCNT0 = 0x00;
    OCR0 = 0x1A;

    // Timer/Counter 1 initialization
    // Clock source: System Clock
    // Clock value: Timer1 Stopped
    // Mode: Normal top=0xFFFF
    // OC1A output: Disconnected
    // OC1B output: Disconnected
    // Noise Canceler: Off
    // Input Capture on Falling Edge
    // Timer1 Overflow Interrupt: Off
    // Input Capture Interrupt: Off
    // Compare A Match Interrupt: Off
    // Compare B Match Interrupt: Off
    TCCR1A = (0 << COM1A1) | (0 << COM1A0) | (0 << COM1B1) | (0 << COM1B0) | (0 << WGM11) | (0 << WGM10);
    TCCR1B = (0 << ICNC1) | (0 << ICES1) | (0 << WGM13) | (0 << WGM12) | (0 << CS12) | (0 << CS11) | (0 << CS10);
    TCNT1H = 0x00;
    TCNT1L = 0x00;
    ICR1H = 0x00;
    ICR1L = 0x00;
    OCR1AH = 0x00;
    OCR1AL = 0x00;
    OCR1BH = 0x00;
    OCR1BL = 0x00;

    // Timer/Counter 2 initialization
    // Clock source: System Clock
    // Clock value: 1000.000 kHz
    // Mode: Fast PWM top=0xFF
    // OC2 output: Non-Inverted PWM
    // Timer Period: 0.256 ms
    // Output Pulse(s):
    // OC2 Period: 0.256 ms Width: 0.0512 ms
    ASSR = 0 << AS2;
    TCCR2 = (1 << WGM20) | (1 << COM21) | (0 << COM20) | (1 << WGM21) | (0 << CS22) | (1 << CS21) | (0 << CS20);
    TCNT2 = 0x00;
    OCR2 = 0x33;

    // Timer(s)/Counter(s) Interrupt(s) initialization
    TIMSK = (0 << OCIE2) | (0 << TOIE2) | (0 << TICIE1) | (0 << OCIE1A) | (0 << OCIE1B) | (0 << TOIE1) | (0 << OCIE0) | (0 << TOIE0);

    // External Interrupt(s) initialization
    // INT0: Off
    // INT1: Off
    // INT2: Off
    MCUCR = (0 << ISC11) | (0 << ISC10) | (0 << ISC01) | (0 << ISC00);
    MCUCSR = (0 << ISC2);

    // USART initialization
    // USART disabled
    UCSRB = (0 << RXCIE) | (0 << TXCIE) | (0 << UDRIE) | (0 << RXEN) | (0 << TXEN) | (0 << UCSZ2) | (0 << RXB8) | (0 << TXB8);

    // Analog Comparator initialization
    // Analog Comparator: Off
    // The Analog Comparator's positive input is
    // connected to the AIN0 pin
    // The Analog Comparator's negative input is
    // connected to the AIN1 pin
    ACSR = (1 << ACD) | (0 << ACBG) | (0 << ACO) | (0 << ACI) | (0 << ACIE) | (0 << ACIC) | (0 << ACIS1) | (0 << ACIS0);

    // ADC initialization
    // ADC Clock frequency: 500.000 kHz
    // ADC Voltage Reference: AREF pin
    // ADC Auto Trigger Source: ADC Stopped
    ADMUX = ADC_VREF_TYPE;
    ADCSRA = (1 << ADEN) | (0 << ADSC) | (0 << ADATE) | (0 << ADIF) | (0 << ADIE) | (1 << ADPS2) | (0 << ADPS1) | (0 << ADPS0);
    SFIOR = (0 << ADTS2) | (0 << ADTS1) | (0 << ADTS0);

    // SPI initialization
    // SPI disabled
    SPCR = (0 << SPIE) | (0 << SPE) | (0 << DORD) | (0 << MSTR) | (0 << CPOL) | (0 << CPHA) | (0 << SPR1) | (0 << SPR0);

    // TWI initialization
    // TWI disabled
    TWCR = (0 << TWEA) | (0 << TWSTA) | (0 << TWSTO) | (0 << TWEN) | (0 << TWIE);

    // LCD initialization
    // RS - PORTC Bit 0
    // RD - PORTC Bit 1
    // EN - PORTC Bit 2
    // D4 - PORTC Bit 4
    // D5 - PORTC Bit 5
    // D6 - PORTC Bit 6
    // D7 - PORTC Bit 7
    lcd_init();
    lcd_on();
    // lcd_enable_cursor();
    // lcd_enable_blinking();
    // lcd_enable_autoscroll();
    lcd_set_left_to_right();

    while (1)
    {
        // push buttons
        if (!(PINB & 1 << 4))
        {
            if (!button_state)
            {
                button_update = 1;
                button_state = 1;
            }
        }
        else if (!(PINB & 1 << 5))
        {
            if (!button_state)
            {
                button_update = 2;
                button_state = 1;
            }
        }
        else if (!(PINB & 1 << 6))
        {
            if (!button_state)
            {
                button_update = 3;
                button_state = 1;
            }
        }
        else if (!(PINB & 1 << 7))
        {
            if (!button_state)
            {
                button_update = 4;
                button_state = 1;
            }
        }
        else
        {
            button_state = 0;
        }

        // Control Flow
        if (button_update)
        {

            switch (button_update)
            {
            case 1: // UP
                if ((lcd_state & 0b00000011) && lcd_state != 0b00000001)
                    lcd_state--;
                if ((lcd_state & 0b00110000) && lcd_state != 0b00010000)
                    lcd_state -= 0b00010000;
                else if (lcd_state == 0b01000000)
                    enable = 1;
                else if (lcd_state == 0b10000000)
                    threshold++;
                else if (lcd_state == 0b11000000 && speed != 3)
                    speed++;

                lcd_update = 1;
                break;

            case 2: // DOWN
                if ((lcd_state & 0b00000011) && lcd_state != 0b00000011)
                    lcd_state++;
                if ((lcd_state & 0b00110000) && lcd_state != 0b00110000)
                    lcd_state += 0b00010000;
                else if (lcd_state == 0b01000000)
                    enable = 0;
                else if (lcd_state == 0b10000000)
                    threshold--;
                else if (lcd_state == 0b11000000 && speed != 0)
                    speed--;

                lcd_update = 1;
                break;

            case 3: // SELECT/ENTER
                if (lcd_state == 0b00000001)
                    lcd_state = 0b00000100;
                else if (lcd_state == 0b00000010)
                    lcd_state = 0b00001000;
                else if (lcd_state == 0b00000011)
                    lcd_state = 0b00010000;
                else if (lcd_state & 0b00110000)
                    lcd_state <<= 2;

                lcd_update = 1;
                break;

            case 4: // BACK
                if (lcd_state & 0b11000000)
                    lcd_state >>= 2;
                else
                    lcd_state = 0b00000001;

                lcd_update = 1;
                break;

            default:
                lcd_state = 100;
                lcd_update = 1;
                break;
            }

            button_update = 0;
        }

        // LED
        if ((PINB & 1 << 0))
        {
            pot_adc = read_adc(0);
            OCR0 = pot_adc >> 2;
        }
        else
        {
            OCR0 = 0x00;
        }

        if ((pot_adc ^ pot_adc_update || pot_pin_update ^ (PINB & 1 << 0)) && lcd_state == 0b00000100)
        // send lcd update signal when needed
        {
            lcd_update = 1;
            pot_adc_update = pot_adc;
            pot_pin_update = (PINB & 1 << 0);
        }

        // Motor
        temp_adc = read_adc(1);
        if (temp_adc ^ temp_adc_update && lcd_state == 0b00001000) // send lcd update signal when needed
        {
            lcd_update = 1;
            temp_adc_update = temp_adc;
        }
        temp = temp_adc * ((float)500 / 1024);

        if (enable)
        {
            if (temp > threshold)
            {
                if (speed == 0)
                    OCR2 = 0;
                else if (speed == 1)
                    OCR2 = 50;
                else if (speed == 2)
                    OCR2 = 150;
                else if (speed == 3)
                    OCR2 = 255;
            }
            else
                OCR2 = 0;
        }
        else
        {
            if (speed == 0)
                OCR2 = 0;
            else if (speed == 1)
                OCR2 = 50;
            else if (speed == 2)
                OCR2 = 150;
            else if (speed == 3)
                OCR2 = 255;
        }

        // LCD
        if (lcd_update)
        {
            lcd_clear();
            lcd_set_cursor(0, 0);

            if (lcd_state & 0b00000011) // lcd_state = 1 or 2 or 3
            {
                lcd_puts("     Smart Home     ");
                lcd_set_cursor(0, 1);
                lcd_puts("       LED          ");
                lcd_set_cursor(0, 2);
                lcd_puts("       TEMP         ");
                lcd_set_cursor(0, 3);
                lcd_puts("       FAN          ");

                lcd_set_cursor(0, (lcd_state & 0b00000011));
                lcd_write('>');
            }
            else if (lcd_state == 0b00000100) // lcd_state = 4
            {
                lcd_puts("     Smart Home     ");
                lcd_set_cursor(0, 1);
                lcd_puts("       LED          ");
                lcd_set_cursor(0, 2);
                lcd_printf(" state: %3d          value: %3d", (PINB & 1 << 0), pot_adc * (PINB & 1 << 0));
                lcd_set_cursor(0, 3);
                lcd_printf(" value: %3d", pot_adc * (PINB & 1 << 0));
            }
            else if (lcd_state == 0b00001000) // lcd_state = 8
            {
                lcd_puts("     Smart Home     ");
                lcd_set_cursor(0, 1);
                lcd_puts("      TEMP          ");
                lcd_set_cursor(0, 2);
                lcd_printf("temp: %3d   thr: %3d", temp, threshold);
                lcd_set_cursor(0, 3);
                lcd_printf("speed: %3d auto: %3d", OCR2, enable);
            }
            else if (lcd_state & 0b11110000) // lcd_state = 32 or
            {
                lcd_puts("     Smart Home FAN ");
                lcd_set_cursor(0, 1);
                lcd_printf("      auto: %3d     ", enable);

                lcd_set_cursor(0, 2);
                lcd_printf("      threshold: %3d", threshold);

                lcd_set_cursor(0, 3);
                lcd_printf("      speed: %3d    ", speed);

                if (lcd_state & 0b00110000)
                {
                    lcd_set_cursor(0, (lcd_state & 0b00110000) >> 4);
                    lcd_puts(">");
                }
                else
                {
                    lcd_set_cursor(0, (lcd_state & 0b11000000) >> 6);
                    lcd_puts("edit>");
                }
            }
            else
            {
                lcd_printf("DEBUG: %d", lcd_state);
            }

            lcd_update = 0;
        }
    }
    return 0;
}