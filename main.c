#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

/* =========================================================================
   MEGA 2560 SYSTEM CONSTANTS & PIN DEFINITIONS
   ========================================================================= */
#define SPINDLE_COUNT 5
#define HISTORY_SIZE 12
#define EVENT_HISTORY_SIZE 16
#define EVENT_QUEUE_SIZE 10

#define SAMPLE_MS 250
#define DISPLAY_MS 500
#define REPORT_MS 2500
#define DEBOUNCE_MS 180

/* --- ENUMS --- */
typedef enum
{
    STATE_RUNNING,
    STATE_PACKAGE_LOW,
    STATE_YARN_BREAK,
    STATE_VIBRATION,
    STATE_OVERHEAT,
    STATE_ELECTRICAL,
    STATE_STOPPED
} State;
typedef enum
{
    SEV_NORMAL,
    SEV_WARNING,
    SEV_MAJOR,
    SEV_CRITICAL
} Severity;

/* --- STRUCTS --- */
typedef struct
{
    unsigned long timestamp;
    float rpm, vibration, temperatureC, currentA, packagePct;
} Sample;

typedef struct
{
    char id[4];
    State state;
    Severity severity;
    float rpm, targetRpm, vibrationRms, temperatureC, currentA, packagePct, health, outputMeters;
    unsigned long downtimeMs, breakCount;
    bool yarnPresent, packageLow, vibrationInjected;
    Sample history[HISTORY_SIZE];
    uint8_t historyHead, historyCount;
} Spindle;

typedef struct
{
    char spindleId[4];
    char message[28];
    uint8_t severity;
    unsigned long timestamp;
} FaultEvent;

typedef struct
{
    char text[36];
    unsigned long timestamp;
} EventRecord;

/* --- GLOBALS --- */
Spindle spindles[SPINDLE_COUNT];
FaultEvent eventQueue[EVENT_QUEUE_SIZE];
EventRecord eventHistory[EVENT_HISTORY_SIZE];

uint8_t eventHead = 0, eventTail = 0, eventCount = 0;
uint8_t historyHead = 0, selectedSpindle = 0;
unsigned long lastSampleMs = 0, lastDisplayMs = 0, lastReportMs = 0, lastButtonMs = 0;

volatile int speedSetpoint = 900;
volatile uint8_t lastEncoderClk = 0;

uint32_t totalFaults = 0;
float busCurrent = 0.0f; // Measured via Analog Slider
float mpuVibrationRaw = 0.0f;
bool alarmActive = false, buzzerState = false;

/* =========================================================================
   HARDWARE INTERRUPTS
   ========================================================================= */

// --- ENCODER ROTATION INTERRUPT (INT4 on PE4) ---
ISR(INT4_vect)
{
    uint8_t clk = (PINE & (1 << PE4)) ? 1 : 0;
    uint8_t dt = (PINE & (1 << PE5)) ? 1 : 0;

    if (clk != lastEncoderClk)
    {
        if (dt != clk)
            speedSetpoint += 25;
        else
            speedSetpoint -= 25;

        if (speedSetpoint < 300)
            speedSetpoint = 300;
        if (speedSetpoint > 1800)
            speedSetpoint = 1800;
        lastEncoderClk = clk;
    }
}

// --- TIMER0 (millis) ---
volatile unsigned long timer0_millis = 0;
ISR(TIMER0_COMPA_vect) { timer0_millis++; }
unsigned long millis(void)
{
    unsigned long m;
    cli();
    m = timer0_millis;
    sei();
    return m;
}

void timer_init(void)
{
    TCCR0A = (1 << WGM01);
    TCCR0B = (1 << CS01) | (1 << CS00);
    OCR0A = 249;
    TIMSK0 |= (1 << OCIE0A);
    sei();
}

/* =========================================================================
   BARE-METAL C DRIVERS FOR MEGA 2560
   ========================================================================= */

// --- ADC (Analog Current Sensor / Slide Potentiometer) ---
void adc_init(void)
{
    ADMUX = (1 << REFS0);                                              // AVCC Reference (5V)
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Enable ADC, Prescaler 128
}

uint16_t adc_read(uint8_t ch)
{
    ADMUX = (ADMUX & 0xF8) | (ch & 0x07); // Select ADC Channel (0-7)
    ADCSRA |= (1 << ADSC);                // Start Conversion
    while (ADCSRA & (1 << ADSC))
        ; // Wait for completion
    return ADC;
}

// --- UART0 (Serial Monitor 115200 Baud) ---
int uart_putchar(char c, FILE *stream)
{
    if (c == '\n')
        uart_putchar('\r', stream);
    while (!(UCSR0A & (1 << UDRE0)))
        ;
    UDR0 = c;
    return 0;
}
static FILE uart_output = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
void uart_init(void)
{
    UBRR0 = 16;
    UCSR0A = (1 << U2X0);
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
    stdout = &uart_output;
}

// --- I2C/TWI (Pins 20 & 21 on Mega) ---
void twi_init(void)
{
    PORTD |= (1 << PD0) | (1 << PD1); // Mega Pull-ups
    TWSR = 0x00;
    TWBR = 72;
    TWCR = (1 << TWEN);
}
void twi_start(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    uint16_t t = 10000;
    while (!(TWCR & (1 << TWINT)) && t--)
        _delay_us(1);
}
void twi_stop(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    uint16_t t = 10000;
    while ((TWCR & (1 << TWSTO)) && t--)
        _delay_us(1);
}
void twi_write(uint8_t data)
{
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    uint16_t t = 10000;
    while (!(TWCR & (1 << TWINT)) && t--)
        _delay_us(1);
}
uint8_t twi_read_ack(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    uint16_t t = 10000;
    while (!(TWCR & (1 << TWINT)) && t--)
        _delay_us(1);
    return TWDR;
}
uint8_t twi_read_nack(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN);
    uint16_t t = 10000;
    while (!(TWCR & (1 << TWINT)) && t--)
        _delay_us(1);
    return TWDR;
}

// --- HARDWARE SPI & MOCK SD CARD WRITER ---
void spi_init(void)
{
    DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB2); // CS=53, SCK=52, MOSI=51
    PORTB |= (1 << PB0);
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
}
void sd_log(const char *id, const char *msg, const char *sev)
{
    PORTB &= ~(1 << PB0); // Pull CS Low
    SPDR = 0xFF;
    while (!(SPSR & (1 << SPIF)))
        ;                // Send dummy byte
    PORTB |= (1 << PB0); // Pull CS High
    printf("[SD LOG] %lu,%s,%s,%s\n", millis(), id, msg, sev);
}

// --- 1-WIRE DS18B20 DRIVER ---
bool ds_reset(void)
{
    bool presence;
    DDRG |= (1 << PG5);
    PORTG &= ~(1 << PG5);
    _delay_us(480);
    DDRG &= ~(1 << PG5);
    PORTG |= (1 << PG5);
    _delay_us(70);
    presence = !(PING & (1 << PG5));
    _delay_us(410);
    return presence;
}
void ds_write_bit(bool bit)
{
    DDRG |= (1 << PG5);
    PORTG &= ~(1 << PG5);
    _delay_us(2);
    if (bit)
    {
        DDRG &= ~(1 << PG5);
        PORTG |= (1 << PG5);
    }
    _delay_us(60);
    DDRG &= ~(1 << PG5);
    PORTG |= (1 << PG5);
    _delay_us(2);
}
void ds_write_byte(uint8_t byte)
{
    for (int i = 0; i < 8; i++)
    {
        ds_write_bit(byte & 0x01);
        byte >>= 1;
    }
}
bool ds_read_bit(void)
{
    bool bit;
    DDRG |= (1 << PG5);
    PORTG &= ~(1 << PG5);
    _delay_us(2);
    DDRG &= ~(1 << PG5);
    PORTG |= (1 << PG5);
    _delay_us(10);
    bit = (PING & (1 << PG5));
    _delay_us(50);
    return bit;
}
uint8_t ds_read_byte(void)
{
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++)
    {
        if (ds_read_bit())
            byte |= (1 << i);
    }
    return byte;
}
float read_ds18b20(void)
{
    if (!ds_reset())
        return -127.0f;
    ds_write_byte(0xCC);
    ds_write_byte(0x44);
    uint16_t t = 1000;
    while (!ds_read_bit() && t--)
        _delay_us(100);
    if (!ds_reset())
        return -127.0f;
    ds_write_byte(0xCC);
    ds_write_byte(0xBE);
    uint8_t l = ds_read_byte();
    uint8_t h = ds_read_byte();
    int16_t raw = (h << 8) | l;
    return (float)raw / 16.0f;
}

// --- FULL MPU6050 (Acceleration + Gyroscope) ---
void mpu_init(void)
{
    twi_start();
    twi_write(0xD0);
    twi_write(0x6B);
    twi_write(0x00);
    twi_stop();
}
bool readMpu6050(void)
{
    twi_start();
    twi_write(0xD0);
    twi_write(0x3B);
    twi_start();
    twi_write(0xD1);
    uint8_t d[14];

    // Read all 14 bytes to capture Wokwi rotation sliders
    for (int i = 0; i < 13; i++)
        d[i] = twi_read_ack();
    d[13] = twi_read_nack();
    twi_stop();

    float ax = (int16_t)((d[0] << 8) | d[1]) / 16384.0f;
    float ay = (int16_t)((d[2] << 8) | d[3]) / 16384.0f;
    float az = (int16_t)((d[4] << 8) | d[5]) / 16384.0f;

    float gx = (int16_t)((d[8] << 8) | d[9]) / 131.0f;
    float gy = (int16_t)((d[10] << 8) | d[11]) / 131.0f;
    float gz = (int16_t)((d[12] << 8) | d[13]) / 131.0f;

    float aMag = sqrt(ax * ax + ay * ay + az * az);
    float gMag = sqrt(gx * gx + gy * gy + gz * gz);

    // Vibration reacts to Acceleration Deviation AND Rotation Speed
    mpuVibrationRaw = fabs(aMag - 1.0f) + (gMag / 200.0f);
    return true;
}

// --- OLED DRIVER (SSD1306) ---
void oled_cmd(uint8_t cmd)
{
    twi_start();
    twi_write(0x78);
    twi_write(0x00);
    twi_write(cmd);
    twi_stop();
}
void oled_data(uint8_t data)
{
    twi_start();
    twi_write(0x78);
    twi_write(0x40);
    twi_write(data);
    twi_stop();
}
void oled_init(void)
{
    uint8_t cmds[] = {0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40, 0x8D, 0x14, 0x20, 0x02, 0xA1, 0xC8, 0xDA, 0x12, 0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0xAF};
    for (int i = 0; i < sizeof(cmds); i++)
        oled_cmd(cmds[i]);
}
void oled_set_cursor(uint8_t col, uint8_t page)
{
    oled_cmd(0xB0 | page);
    oled_cmd(0x00 | (col & 0x0F));
    oled_cmd(0x10 | (col >> 4));
}
void oled_clear(void)
{
    for (uint8_t p = 0; p < 8; p++)
    {
        oled_set_cursor(0, p);
        for (uint8_t c = 0; c < 128; c++)
            oled_data(0x00);
    }
}

const uint8_t font5x7[59][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, {0x00, 0x00, 0x5F, 0x00, 0x00}, {0x00, 0x07, 0x00, 0x07, 0x00}, {0x14, 0x7F, 0x14, 0x7F, 0x14}, {0x24, 0x2A, 0x7F, 0x2A, 0x12}, {0x23, 0x13, 0x08, 0x64, 0x62}, {0x36, 0x49, 0x55, 0x22, 0x50}, {0x00, 0x05, 0x03, 0x00, 0x00}, {0x00, 0x1C, 0x22, 0x41, 0x00}, {0x00, 0x41, 0x22, 0x1C, 0x00}, {0x08, 0x2A, 0x1C, 0x2A, 0x08}, {0x08, 0x08, 0x3E, 0x08, 0x08}, {0x00, 0x50, 0x30, 0x00, 0x00}, {0x08, 0x08, 0x08, 0x08, 0x08}, {0x00, 0x60, 0x60, 0x00, 0x00}, {0x20, 0x10, 0x08, 0x04, 0x02}, {0x3E, 0x51, 0x49, 0x45, 0x3E}, {0x00, 0x42, 0x7F, 0x40, 0x00}, {0x42, 0x61, 0x51, 0x49, 0x46}, {0x21, 0x41, 0x45, 0x4B, 0x31}, {0x18, 0x14, 0x12, 0x7F, 0x10}, {0x27, 0x45, 0x45, 0x45, 0x39}, {0x3C, 0x4A, 0x49, 0x49, 0x30}, {0x01, 0x71, 0x09, 0x05, 0x03}, {0x36, 0x49, 0x49, 0x49, 0x36}, {0x06, 0x49, 0x49, 0x29, 0x1E}, {0x00, 0x36, 0x36, 0x00, 0x00}, {0x00, 0x56, 0x36, 0x00, 0x00}, {0x00, 0x08, 0x14, 0x22, 0x41}, {0x14, 0x14, 0x14, 0x14, 0x14}, {0x41, 0x22, 0x14, 0x08, 0x00}, {0x02, 0x01, 0x51, 0x09, 0x06}, {0x32, 0x49, 0x79, 0x41, 0x3E}, {0x7E, 0x11, 0x11, 0x11, 0x7E}, {0x7F, 0x49, 0x49, 0x49, 0x36}, {0x3E, 0x41, 0x41, 0x41, 0x22}, {0x7F, 0x41, 0x41, 0x22, 0x1C}, {0x7F, 0x49, 0x49, 0x49, 0x41}, {0x7F, 0x09, 0x09, 0x09, 0x01}, {0x3E, 0x41, 0x41, 0x51, 0x32}, {0x7F, 0x08, 0x08, 0x08, 0x7F}, {0x00, 0x41, 0x7F, 0x41, 0x00}, {0x20, 0x40, 0x41, 0x3F, 0x01}, {0x7F, 0x08, 0x14, 0x22, 0x41}, {0x7F, 0x40, 0x40, 0x40, 0x40}, {0x7F, 0x02, 0x04, 0x02, 0x7F}, {0x7F, 0x04, 0x08, 0x10, 0x7F}, {0x3E, 0x41, 0x41, 0x41, 0x3E}, {0x7F, 0x09, 0x09, 0x09, 0x06}, {0x3E, 0x41, 0x51, 0x21, 0x5E}, {0x7F, 0x09, 0x19, 0x29, 0x46}, {0x46, 0x49, 0x49, 0x49, 0x31}, {0x01, 0x01, 0x7F, 0x01, 0x01}, {0x3F, 0x40, 0x40, 0x40, 0x3F}, {0x1F, 0x20, 0x40, 0x20, 0x1F}, {0x7F, 0x20, 0x18, 0x20, 0x7F}, {0x63, 0x14, 0x08, 0x14, 0x63}, {0x03, 0x04, 0x78, 0x04, 0x03}, {0x61, 0x51, 0x49, 0x45, 0x43}};

void oled_print(uint8_t col, uint8_t page, const char *str)
{
    oled_set_cursor(col, page);
    while (*str)
    {
        if (*str >= 32 && *str <= 90)
        {
            for (int i = 0; i < 5; i++)
                oled_data(font5x7[*str - 32][i]);
        }
        else
        {
            for (int i = 0; i < 5; i++)
                oled_data(0);
        }
        oled_data(0x00);
        str++;
    }
}

/* =========================================================================
   TEXTILE LOGIC
   ========================================================================= */

const char *stateName(State s)
{
    switch (s)
    {
    case STATE_RUNNING:
        return "RUN";
    case STATE_PACKAGE_LOW:
        return "PKG LOW";
    case STATE_YARN_BREAK:
        return "YARN BREAK";
    case STATE_VIBRATION:
        return "VIB FAULT";
    case STATE_OVERHEAT:
        return "OVERHEAT";
    case STATE_ELECTRICAL:
        return "OVERLOAD";
    default:
        return "STOPPED";
    }
}
const char *severityName(Severity s)
{
    switch (s)
    {
    case SEV_WARNING:
        return "WARNING";
    case SEV_MAJOR:
        return "MAJOR";
    case SEV_CRITICAL:
        return "CRITICAL";
    default:
        return "NORMAL";
    }
}

void logEvent(const char *id, const char *message, Severity severity)
{
    char combined[36];
    snprintf(combined, sizeof(combined), "%s: %s", id, message);
    strncpy(eventHistory[historyHead].text, combined, 35);
    eventHistory[historyHead].timestamp = millis();
    historyHead = (historyHead + 1) % EVENT_HISTORY_SIZE;
    totalFaults++;

    if (eventCount < EVENT_QUEUE_SIZE)
    {
        strncpy(eventQueue[eventTail].spindleId, id, 3);
        strncpy(eventQueue[eventTail].message, message, 27);
        eventQueue[eventTail].severity = severity;
        eventQueue[eventTail].timestamp = millis();
        eventTail = (eventTail + 1) % EVENT_QUEUE_SIZE;
        eventCount++;
    }

    printf("[EVENT] %lu ms | %s | %s | %s\n", millis(), id, message, severityName(severity));
    sd_log(id, message, severityName(severity));
}

void initializeSpindles()
{
    for (uint8_t i = 0; i < SPINDLE_COUNT; i++)
    {
        sprintf(spindles[i].id, "S%02u", i + 1);
        spindles[i].state = STATE_RUNNING;
        spindles[i].severity = SEV_NORMAL;
        spindles[i].rpm = 850.0f + i * 20.0f;
        spindles[i].targetRpm = spindles[i].rpm;
        spindles[i].vibrationRms = 0.12f;
        spindles[i].temperatureC = 32.0f;
        spindles[i].currentA = 0.35f;
        spindles[i].packagePct = 100.0f - i * 4.0f;
        spindles[i].health = 98.0f;
        spindles[i].yarnPresent = true;
    }
}

void updateSpindle(Spindle *s, uint8_t index)
{
    s->targetRpm = (float)speedSetpoint;
    float desired = s->yarnPresent ? s->targetRpm : 0.0f;
    s->rpm += (desired - s->rpm) * 0.25f;

    s->vibrationRms = 0.10f + 0.015f * index + fabs(s->rpm) / 9000.0f;

    if (index == selectedSpindle)
    {
        float physicalTemp = read_ds18b20();
        if (physicalTemp > -50.0f)
            s->temperatureC = physicalTemp;

        // Update physical Current from Slide Potentiometer
        s->currentA = busCurrent;

        // Update physical Vibration
        if (readMpu6050())
            s->vibrationRms += mpuVibrationRaw;
    }
    else
    {
        s->temperatureC += (32.0f + index * 0.6f - s->temperatureC) * 0.04f;
        s->currentA = 0.25f + s->rpm / 4000.0f;
    }

    if (s->vibrationInjected)
        s->vibrationRms = 0.65f;
    if (s->yarnPresent && !s->packageLow)
    {
        s->packagePct -= 0.0035f * (s->rpm / 900.0f);
        if (s->packagePct < 0.0f)
            s->packagePct = 0.0f;
        s->outputMeters += (s->rpm / 900.0f) * 0.035f;
    }

    if (!s->yarnPresent)
    {
        s->state = STATE_YARN_BREAK;
        s->severity = SEV_CRITICAL;
    }
    else if (s->vibrationInjected || s->vibrationRms > 0.42f)
    {
        s->state = STATE_VIBRATION;
        s->severity = SEV_MAJOR;
    }
    else if (s->temperatureC > 65.0f)
    {
        s->state = STATE_OVERHEAT;
        s->severity = SEV_CRITICAL;
    }
    else if (s->currentA > 1.8f)
    {
        s->state = STATE_ELECTRICAL;
        s->severity = SEV_CRITICAL;
    }
    else if (s->packageLow || s->packagePct <= 10.0f)
    {
        s->state = STATE_PACKAGE_LOW;
        s->severity = SEV_WARNING;
    }
    else
    {
        s->state = STATE_RUNNING;
        s->severity = SEV_NORMAL;
    }

    float penalty = 0.0f;
    if (s->vibrationRms > 0.42f)
        penalty += 35.0f;
    if (s->temperatureC > 55.0f)
        penalty += (s->temperatureC - 55.0f) * 0.8f;
    if (s->currentA > 1.2f)
        penalty += (s->currentA - 1.2f) * 15.0f;
    if (!s->yarnPresent)
        penalty += 45.0f;
    if (s->packagePct <= 10.0f || s->packageLow)
        penalty += 18.0f;

    s->health = 100.0f - penalty;
    if (s->health < 0.0f)
        s->health = 0.0f;
}

// --- Mega 2560 Button Control ---
bool buttonPressed(volatile uint8_t *port, uint8_t pin)
{
    if (!(*port & (1 << pin)) && (millis() - lastButtonMs > DEBOUNCE_MS))
    {
        lastButtonMs = millis();
        return true;
    }
    return false;
}

void processControls()
{
    // Encoder Press (PE3=SW)
    if (buttonPressed(&PINE, PE3))
    {
        selectedSpindle = (selectedSpindle + 1) % SPINDLE_COUNT;
        spindles[selectedSpindle].targetRpm = (float)speedSetpoint;
        printf("Selected spindle: %s\n", spindles[selectedSpindle].id);
    }

    if (buttonPressed(&PINH, PH3))
    { // Yarn Break (Pin 6)
        Spindle *s = &spindles[selectedSpindle];
        s->yarnPresent = !s->yarnPresent;
        if (!s->yarnPresent)
            logEvent(s->id, "YARN BREAK", SEV_CRITICAL);
        else
            logEvent(s->id, "YARN RESTORED", SEV_WARNING);
    }

    if (buttonPressed(&PINH, PH4))
    { // Package (Pin 7)
        Spindle *s = &spindles[selectedSpindle];
        if (!s->packageLow)
        {
            s->packageLow = true;
            s->packagePct = 8.0f;
            logEvent(s->id, "PACKAGE LOW", SEV_WARNING);
        }
        else
        {
            s->packageLow = false;
            s->packagePct = 100.0f;
            logEvent(s->id, "PACKAGE REPLACED", SEV_NORMAL);
        }
    }

    if (buttonPressed(&PINH, PH5))
    { // Ack (Pin 8)
        alarmActive = false;
        PORTC &= ~(1 << PC4); // Buzzer off
        printf("Operator acknowledged alarm.\n");
    }
}

void updateIndicators()
{
    bool warning = false, critical = false;
    for (uint8_t i = 0; i < SPINDLE_COUNT; i++)
    {
        if (spindles[i].severity == SEV_WARNING)
            warning = true;
        if (spindles[i].severity == SEV_MAJOR || spindles[i].severity == SEV_CRITICAL)
            critical = true;
    }

    // LEDs: PC7(G), PC6(Y), PC5(R)
    PORTC &= ~((1 << PC7) | (1 << PC6) | (1 << PC5));
    if (!warning && !critical)
        PORTC |= (1 << PC7);
    if (warning && !critical)
        PORTC |= (1 << PC6);
    if (critical)
        PORTC |= (1 << PC5);

    if (critical && alarmActive)
    {
        if ((millis() / 500) % 2 == 0)
            PORTC |= (1 << PC4);
        else
            PORTC &= ~(1 << PC4);
    }
    else if (warning && alarmActive)
    {
        PORTC |= (1 << PC4);
    }
    else
    {
        PORTC &= ~(1 << PC4);
    }
}

void drawDisplay()
{
    Spindle *s = &spindles[selectedSpindle];
    char buf[32];
    oled_clear();
    oled_print(0, 0, "TEXTILE MONITOR");
    sprintf(buf, "%s %s H:%d", s->id, stateName(s->state), (int)s->health);
    oled_print(0, 2, buf);
    sprintf(buf, "RPM:%d Set:%d", (int)s->rpm, (int)s->targetRpm);
    oled_print(0, 3, buf);

    char vT[10], vI[10], vV[10];
    dtostrf(s->temperatureC, 3, 1, vT);
    dtostrf(s->currentA, 3, 2, vI);
    dtostrf(s->vibrationRms, 3, 2, vV);
    sprintf(buf, "Temp:%sC I:%sA", vT, vI);
    oled_print(0, 4, buf);
    sprintf(buf, "Vib:%s Pkg:%d%%", vV, (int)s->packagePct);
    oled_print(0, 5, buf);
    sprintf(buf, "Out:%dm Q:%d", (int)s->outputMeters, eventCount);
    oled_print(0, 6, buf);
}

void printReport()
{
    printf("\n========== TEXTILE SPINNING MONITOR ==========\n");
    printf("Uptime: %lu s | Selected: %s | Setpoint: %d RPM\n", millis() / 1000, spindles[selectedSpindle].id, speedSetpoint);
    for (uint8_t i = 0; i < SPINDLE_COUNT; i++)
    {
        Spindle *s = &spindles[i];
        printf("%s | %s | RPM %d | Vib %.2f g | Temp %.1f C | I %.2f A | Pkg %.1f%% | Health %d\n",
               s->id, stateName(s->state), (int)s->rpm, s->vibrationRms, s->temperatureC, s->currentA, s->packagePct, (int)s->health);
    }
    printf("Fault events: %lu | Queue: %d\n", totalFaults, eventCount);
    printf("==============================================\n");
}

/* =========================================================================
   MAIN CONTINUOUS LOOP
   ========================================================================= */
int main(void)
{
    timer_init();
    uart_init();
    adc_init();
    twi_init();
    spi_init();

    // --- Configure Interrupt for Encoder ---
    EICRB |= (1 << ISC40);
    EICRB &= ~(1 << ISC41);
    EIMSK |= (1 << INT4);
    lastEncoderClk = (PINE & (1 << PE4)) ? 1 : 0;

    // Inputs (Encoder SW on PE3, Buttons on PH3-5)
    DDRE &= ~(1 << PE3);
    PORTE |= (1 << PE3);
    DDRH &= ~((1 << PH3) | (1 << PH4) | (1 << PH5));
    PORTH |= (1 << PH3) | (1 << PH4) | (1 << PH5);

    // Outputs (LEDs & Buzzer on PC4-7)
    DDRC |= (1 << PC4) | (1 << PC5) | (1 << PC6) | (1 << PC7);
    PORTC &= ~((1 << PC4) | (1 << PC5) | (1 << PC6) | (1 << PC7));

    initializeSpindles();
    mpu_init();
    oled_init();

    printf("\nAdvanced Textile Monitor started (PURE BARE-METAL C)\n");

    while (1)
    {
        unsigned long now = millis();
        processControls();

        if (eventCount > 0)
        {
            FaultEvent ev = eventQueue[eventHead];
            eventHead = (eventHead + 1) % EVENT_QUEUE_SIZE;
            eventCount--;
            printf("[QUEUE] Processed %s / %s\n", ev.spindleId, ev.message);
        }

        if (now - lastSampleMs >= SAMPLE_MS)
        {
            lastSampleMs = now;

            // Read the ADC (Channel 1 for Pin A1) and scale 0-1023 to 0.0 - 5.0 Amps
            uint16_t adcVal = adc_read(1);
            busCurrent = (adcVal / 1023.0f) * 5.0f;

            bool fault = false;
            for (uint8_t i = 0; i < SPINDLE_COUNT; i++)
            {
                updateSpindle(&spindles[i], i);
                if (spindles[i].severity >= SEV_WARNING)
                    fault = true;
            }
            if (fault)
                alarmActive = true;
        }

        updateIndicators();

        if (now - lastDisplayMs >= DISPLAY_MS)
        {
            lastDisplayMs = now;
            drawDisplay();
        }

        if (now - lastReportMs >= REPORT_MS)
        {
            lastReportMs = now;
            printReport();
        }
    }
    return 0;
}