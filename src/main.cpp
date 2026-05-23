/* ═══════════════════════════════════════════════════════════════════════════
 * Smart Trash Can — ATmega328P
 * Direct register manipulation, no Arduino libraries.
 *
 * Pin map
 * ───────────────────────────────────────────────────────────────────────────
 *  PD0  │ USART0 RX           │ (mEDBG CDC bridge — not used in code)
 *  PD1  │ USART0 TX           │ Serial debug output -> COM3 serial monitor
 *  PD2  │ Active buzzer       │ Output
 *  PD3  │ Red LEDs (×2)       │ Output
 *  PD4  │ Green LEDs (×2)     │ Output
 *  PD5  │ HC-SR04 ECHO        │ Input  (PCINT21 -> ISR PCINT2_vect)
 *  PD6  │ HC-SR04 TRIG        │ Output
 *  PB1  │ SG90 servo signal   │ Output (Timer1 OC1A Fast PWM)
 *  PC4  │ I2C SDA             │ TWI
 *  PC5  │ I2C SCL             │ TWI
 * ═══════════════════════════════════════════════════════════════════════════ */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>

/* ─── Servo pulse widths (Timer1, prescaler=8, 16 MHz -> 0.5 µs/tick) ───────
 *   ICR1  = 39999  -> period = 20 ms (50 Hz)
 *   OCR1A = 3000   -> 1.5 ms -> arm at closed position
 *   OCR1A = 1000   -> 0.5 ms -> arm rotates CCW to open position
 *
 *   Range = 2000 ticks = 1.0 ms -> full 180° travel on SG90.
 *   Direction: increasing OCR1A = CW (arm DOWN); decreasing = CCW (arm UP).
 *   After uploading, reposition the servo horn so that OCR1A=3000 is the
 *   correct "lid closed" angle, then OCR1A=1000 gives maximum opening.
 *   If the servo buzzes at 1000, raise SERVO_OPEN in steps of 100.
 * ─────────────────────────────────────────────────────────────────────────── */
#define SERVO_CLOSED  3000u   /* 1.5 ms -> arm at closed position             */
#define SERVO_OPEN    1000u   /* 0.5 ms -> arm at open position (CCW, max)    */

/* ─── PCF8574 LCD backpack bit masks ────────────────────────────────────────
 *   P7-P4 -> D7-D4,  P3 -> BL,  P2 -> E,  P1 -> RW (0=write),  P0 -> RS
 * ─────────────────────────────────────────────────────────────────────────── */
#define LCD_BL  0x08
#define LCD_EN  0x04
#define LCD_RS  0x01

/* ─── USART baud rate ────────────────────────────────────────────────────── */
#define BAUD      9600UL
#define UBRR_VAL  ((F_CPU / 16UL / BAUD) - 1UL)   /* = 103 at 16 MHz */

/* ─── Globals ────────────────────────────────────────────────────────────── */
static volatile uint16_t echo_start    = 0;
static volatile uint16_t echo_duration = 0;
static volatile uint8_t  echo_ready    = 0;

static uint16_t usage_count = 0;

#define LCD_ADDR  0x27u   /* PCF8574 backpack, A2=A1=A0=1 */


/* ═══════════════════════════════════════════════════════════════════════════
 * LAB 2 — ISR: PCINT2_vect  (PD5 / PCINT21)
 *
 * Rising edge  -> capture Timer1 counter as start of echo pulse
 * Falling edge -> compute duration; handle single Timer1 wrap at ICR1=39999
 * ═══════════════════════════════════════════════════════════════════════════ */
ISR(PCINT2_vect)
{
    if (PIND & (1 << PD5)) {
        echo_start = TCNT1;
    } else {
        uint16_t now = TCNT1;
        echo_duration = (now >= echo_start)
                        ? (now - echo_start)
                        : ((uint16_t)(40000u - echo_start) + now);
        echo_ready = 1;
    }
}


/* ═══════════════════════════════════════════════════════════════════════════
 * USART0 — 9600 8N1, TX only (debug output through mEDBG CDC -> COM3)
 *
 *   UBRR0 = F_CPU / (16 × BAUD) − 1 = 103 -> actual baud = 9615 (0.16% err)
 *   UCSR0B: TXEN0=1    enable transmitter
 *   UCSR0C: UCSZ01:00=11  8-bit frame, 1 stop bit, no parity
 * ═══════════════════════════════════════════════════════════════════════════ */
static void usart_init(void)
{
    UBRR0H = (uint8_t)(UBRR_VAL >> 8);
    UBRR0L = (uint8_t)(UBRR_VAL);
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

static void usart_putchar(char c)
{
    while (!(UCSR0A & (1 << UDRE0)));   /* wait for TX buffer empty */
    UDR0 = c;
}

static void usart_print(const char *s)
{
    while (*s) usart_putchar(*s++);
}

static void usart_println(const char *s)
{
    usart_print(s);
    usart_putchar('\r');
    usart_putchar('\n');
}

/* Print a uint16_t without needing printf */
static void usart_print_u16(uint16_t n)
{
    char buf[6];
    uint8_t i = 5;
    buf[i] = '\0';
    if (n == 0) {
        buf[--i] = '0';
    } else {
        while (n > 0) { buf[--i] = (char)('0' + n % 10); n /= 10; }
    }
    usart_print(&buf[i]);
}


/* ═══════════════════════════════════════════════════════════════════════════
 * init_gpio
 * ═══════════════════════════════════════════════════════════════════════════ */
void init_gpio(void)
{
    DDRD |=  (1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD6);
    DDRD &= ~(1 << PD5);
    PORTD &= ~(1 << PD5);   /* no pull-up on echo */

    PORTD &= ~(1 << PD2);   /* buzzer OFF */
    PORTD &= ~(1 << PD3);   /* red   OFF  */
    PORTD |=  (1 << PD4);   /* green ON   */
    PORTD &= ~(1 << PD6);   /* trig  LOW  */
}


/* ═══════════════════════════════════════════════════════════════════════════
 * LAB 3 — init_pwm: Timer1 Fast PWM Mode 14 (TOP = ICR1)
 *
 *   TCCR1A: COM1A1=1 (non-inverting OC1A), WGM11=1
 *   TCCR1B: WGM13=1, WGM12=1 (Fast PWM, TOP=ICR1), CS11=1 (prescaler 8)
 * ═══════════════════════════════════════════════════════════════════════════ */
void init_pwm(void)
{
    DDRB  |= (1 << PB1);
    ICR1   = 39999;
    OCR1A  = SERVO_CLOSED;
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13)  | (1 << WGM12) | (1 << CS11);
}


/* ═══════════════════════════════════════════════════════════════════════════
 * LAB 2 — init_interrupts: PCINT2 on PD5 (PCINT21)
 *
 *   PCICR  PCIE2=1   enable Port D pin-change group
 *   PCMSK2 PCINT21=1 unmask PD5 only
 * ═══════════════════════════════════════════════════════════════════════════ */
void init_interrupts(void)
{
    PCICR  |= (1 << PCIE2);
    PCMSK2 |= (1 << PCINT21);
}


/* ═══════════════════════════════════════════════════════════════════════════
 * LAB 6 — TWI / I2C
 *
 *   TWSR = 0x00   prescaler = 1
 *   TWBR = 72     SCL = 16 000 000 / (16 + 2×72×1) = 100 000 Hz
 *   TWCR TWEN=1   enable TWI block
 * ═══════════════════════════════════════════════════════════════════════════ */

/*
 * i2c_bus_recovery — clock SCL 9 times to release a slave that is holding
 * SDA low from a previously aborted transaction (e.g. from a prior firmware).
 * Must be called BEFORE TWEN is set so we can drive the pins as GPIO.
 */
static void i2c_bus_recovery(void)
{
    TWCR = 0;   /* disable TWI — return PC4/PC5 to GPIO control */

    DDRC  |=  (1 << PC5) | (1 << PC4);
    PORTC |=  (1 << PC5) | (1 << PC4);   /* SCL=HIGH, SDA=HIGH */

    for (uint8_t i = 0; i < 9; i++) {
        PORTC &= ~(1 << PC5);   /* SCL LOW  */
        _delay_us(5);
        PORTC |=  (1 << PC5);   /* SCL HIGH */
        _delay_us(5);
    }

    /* Manual STOP condition: SDA LOW while SCL HIGH -> SDA HIGH */
    PORTC &= ~(1 << PC4);   _delay_us(5);   /* SDA LOW  */
    PORTC |=  (1 << PC4);   _delay_us(5);   /* SDA HIGH */

    /* Release pins back to TWI */
    DDRC  &= ~((1 << PC5) | (1 << PC4));
    PORTC &= ~((1 << PC5) | (1 << PC4));
}

/* Returns 1 on timeout, 0 on success */
static uint8_t i2c_start(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    uint16_t t = 10000;
    while (!(TWCR & (1 << TWINT)) && --t);
    return (t == 0);
}

static void i2c_stop(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

/* Returns 1 on timeout, 0 on success (note: NACK ≠ timeout — TWINT still fires) */
static uint8_t i2c_write_byte(uint8_t data)
{
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    uint16_t t = 10000;
    while (!(TWCR & (1 << TWINT)) && --t);
    return (t == 0);
}


void init_i2c(void)
{
    i2c_bus_recovery();
    TWSR = 0x00;
    TWBR = 72;
    TWCR = (1 << TWEN);
    PORTC |= (1 << PC4) | (1 << PC5);   /* internal pull-ups on SDA/SCL (~100 kΩ) */
}

/* Send one byte to PCF8574; silently aborts on any bus error */
static void pcf8574_write(uint8_t val)
{
    if (i2c_start())                                { i2c_stop(); return; }
    if (i2c_write_byte((LCD_ADDR << 1) | 0x00))  { i2c_stop(); return; }
    i2c_write_byte(val);
    i2c_stop();
}


/* ═══════════════════════════════════════════════════════════════════════════
 * LCD driver — HD44780 via PCF8574 4-bit interface
 * ═══════════════════════════════════════════════════════════════════════════ */

static void lcd_pulse_nibble(uint8_t nibble, uint8_t rs)
{
    uint8_t base = (uint8_t)((nibble << 4) | LCD_BL | (rs ? LCD_RS : 0));
    pcf8574_write(base); 
    _delay_us(1);
    pcf8574_write(base | LCD_EN);   
    _delay_us(2);
    pcf8574_write(base);            
    _delay_us(50);
}

static void lcd_send_byte(uint8_t byte, uint8_t rs)
{
    lcd_pulse_nibble(byte >> 4,   rs);
    lcd_pulse_nibble(byte & 0x0F, rs);
}

void lcd_command(uint8_t cmd) { lcd_send_byte(cmd, 0); }
void lcd_data_char(uint8_t c) { lcd_send_byte(c,   1); }

void lcd_set_cursor(uint8_t row, uint8_t col)
{
    lcd_command(0x80 | ((row ? 0x40u : 0x00u) + col));
}

void lcd_print(const char *s)
{
    while (*s) lcd_data_char((uint8_t)*s++);
}

static void lcd_clear(void)
{
    lcd_command(0x01);
    _delay_ms(2);
}

void lcd_init(void)
{
    _delay_ms(50);
    lcd_pulse_nibble(0x03, 0); _delay_ms(5);
    lcd_pulse_nibble(0x03, 0); _delay_ms(1);
    lcd_pulse_nibble(0x03, 0); _delay_us(150);
    lcd_pulse_nibble(0x02, 0); _delay_us(150);   /* switch to 4-bit */
    lcd_command(0x28);   /* 4-bit, 2 lines, 5×8 font */
    lcd_command(0x08);   /* display OFF */
    lcd_command(0x01);   /* clear */
    _delay_ms(2);
    lcd_command(0x06);   /* entry mode: cursor right, no shift */
    lcd_command(0x0C);   /* display ON, cursor OFF, blink OFF */
}


/* ═══════════════════════════════════════════════════════════════════════════
 * Servo
 * ═══════════════════════════════════════════════════════════════════════════ */

static void servo_set(uint16_t ocr)
{
    OCR1A = ocr;
}

/*
 * Smooth close: SERVO_OPEN=1000 -> SERVO_CLOSED=3000 (increasing OCR1A = CW).
 * 50 steps × 20 ms = 1 s total.
 */
static void servo_close_smooth(void)
{
    uint16_t pos = SERVO_OPEN;           /* start at 1000 (arm open) */
    while (pos < SERVO_CLOSED) {         /* increase toward 3000     */
        pos = (pos + 40 < SERVO_CLOSED) ? (pos + 40) : SERVO_CLOSED;
        OCR1A = pos;
        _delay_ms(20);
    }
}


/* ═══════════════════════════════════════════════════════════════════════════
 * HC-SR04
 * ═══════════════════════════════════════════════════════════════════════════ */

static void hcsr04_trigger(void)
{
    echo_ready = 0;
    PORTD |=  (1 << PD6);
    _delay_us(10);
    PORTD &= ~(1 << PD6);
}

/*
 * Returns distance in cm, 0 if no echo within ~30 ms.
 * Each Timer1 tick = 0.5 µs -> distance = echo_duration / 116
 */
static uint16_t measure_distance_cm(void)
{
    hcsr04_trigger();
    uint16_t timeout = 30000;
    while (!echo_ready && --timeout) _delay_us(1);
    if (!echo_ready) return 0;
    return (uint16_t)(echo_duration / 116u);
}


/* ═══════════════════════════════════════════════════════════════════════════
 * LCD display helpers
 * ═══════════════════════════════════════════════════════════════════════════ */

static void display_idle(void)
{
    char buf[17];
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Apropie mana");
    lcd_set_cursor(1, 0);
    sprintf(buf, "Utilizari: %u", usage_count);
    lcd_print(buf);
}

static void display_active(void)
{
    char buf[17];
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Capac Deschis!");
    lcd_set_cursor(1, 0);
    sprintf(buf, "Utilizari: %u", usage_count);
    lcd_print(buf);
}


/* ═══════════════════════════════════════════════════════════════════════════
 * main
 * ═══════════════════════════════════════════════════════════════════════════ */
int main(void)
{
    usart_init();          /* first — enables debug output on COM3 */
    init_gpio();
    init_pwm();
    init_i2c();            /* includes bus recovery + LCD address scan */
    init_interrupts();
    lcd_init();

    sei();

    usart_println("[SYS] Smart Trash Can started");
    usart_println("[SYS] Open serial monitor at 9600 baud on COM3");
    display_idle();

    while (1) {
        uint16_t dist = measure_distance_cm();

        /* Print every distance reading so you can see what the sensor measures */
        usart_print("[DIST] ");
        usart_print_u16(dist);
        usart_println(" cm");

        if (dist > 0 && dist <= 8) {
            usart_println("[STATE] Object detected -> opening lid");

            PORTD &= ~(1 << PD4);   /* green OFF */
            PORTD |=  (1 << PD3);   /* red   ON  */

            servo_set(SERVO_OPEN);
            usage_count++;
            display_active();

            PORTD |= (1 << PD2);

            usart_println("[STATE] Holding open for 4 s");
            _delay_ms(4000);

            PORTD &= (1 << PD2);

            usart_println("[STATE] Closing lid");
            servo_close_smooth();

            PORTD &= ~(1 << PD3);   /* red   OFF */
            PORTD |=  (1 << PD4);   /* green ON  */

            display_idle();
            _delay_ms(500);         /* dead-band before next trigger */

            usart_println("[STATE] Back to idle");

        } else {
            PORTD |=  (1 << PD4);
            PORTD &= ~(1 << PD3);
            PORTD &= ~(1 << PD2);
        }

        _delay_ms(60);
    }

    return 0;
}
