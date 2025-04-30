#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "ff.h"     // FatFs library
#include "diskio.h" // FatFs low level disk I/O

// Define F_CPU if not already defined
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

// UART settings
#define BAUD 9600
#define BAUD_PRESCALE ((F_CPU / 16 / BAUD) - 1)

// Pin definitions using direct port manipulation
#define BUTTON_PIN      PIND2   // INT0
#define LIMIT_SW_PIN    PINB2   // PB2
#define DIR_R_PIN       PIND7   // PD7
#define STEP_R_PIN      PINB0   // PB0
#define ENABLE_R_PIN    PINB1   // PB1
#define DIR_Z_PIN       PIND3   // PD3
#define STEP_Z_PIN      PIND5   // PD5
#define ENABLE_Z_PIN    PIND6   // PD6
#define SD_CS_PIN       PINB4   // PB4 (Arduino pin 4)

// Port manipulation macros
#define SET_PIN(port, pin) ((port) |= (1 << (pin)))
#define CLEAR_PIN(port, pin) ((port) &= ~(1 << (pin)))
#define READ_PIN(port, pin) ((port) & (1 << (pin)))
#define TOGGLE_PIN(port, pin) ((port) ^= (1 << (pin)))

// Scanner parameters
#define SCAN_AMOUNT 5
#define Z_AXIS_HEIGHT 20.0f
#define STEP_DELAY_US 50
#define Z_LAYER_HEIGHT 0.5f
#define LEAD_SCREW_ROTATIONS_PER_CM 8
#define STEPS_PER_ROTATION 1600
#define DISTANCE_TO_CENTER 15.2f
#define EMPTY_SCAN_THRESHOLD 3
#define MAX_OBJECT_DISTANCE 25.0f
#define DEBOUNCE_DELAY_MS 50
#define STATUS_LED_PIN    PINB5  // Built-in LED on Arduino

// Global variables
FATFS fs;
FIL file;
volatile uint8_t scan = 0;
float distance = 0.0f;
float angle = 0.0f;
float x = 0.0f;
float y = 0.0f;
float z = 0.0f;
uint16_t z_loop = 0;
uint16_t steps_z_height = 0;
uint8_t empty_scan_count = 0;
uint8_t object_found = 0;
uint16_t file_number = 1;
char file_name[13];
volatile uint32_t milliseconds = 0;
volatile uint32_t last_debounce_time = 0;
volatile uint8_t last_button_state = 1;
volatile uint8_t button_state = 1;
uint8_t sensor_error = 0;
const float RADIANS = (3.141592f / 180.0f) * (360.0f / STEPS_PER_ROTATION);

// Function prototypes
void init_system(void);
void init_timer(void);
void init_uart(void);
void init_adc(void);
void init_spi(void);
void init_gpio(void);
uint32_t timer_get_ms(void);
void uart_send_char(char c);
void uart_send_string(const char* str);
uint16_t read_adc(uint8_t channel);
float get_distance(void);
void home_z_axis(void);
uint8_t check_for_object(void);
void get_next_filename(void);
void write_file_header(void);
void write_to_sd(float x, float y, float z);
void blink_status_led(uint8_t times);

// System initialization
void init_system(void) {
    // Disable watchdog if enabled by bootloader
    MCUSR &= ~(1 << WDRF);
    wdt_disable();
    
    // Initialize all subsystems
    init_gpio();
    init_timer();
    init_uart();
    init_adc();
    init_spi();
    
    // Enable global interrupts
    sei();
}

// Timer1 initialization for millisecond counting
void init_timer(void) {
    TCCR1B |= (1 << WGM12) | (1 << CS11) | (1 << CS10);  // CTC mode, prescaler 64
    OCR1A = 249;  // For 1ms at 16MHz
    TIMSK1 |= (1 << OCIE1A);  // Enable compare match interrupt
}

// Timer1 Compare Match A interrupt
ISR(TIMER1_COMPA_vect) {
    milliseconds++;
}

// Get current time in milliseconds
uint32_t timer_get_ms(void) {
    uint32_t ms;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        ms = milliseconds;
    }
    return ms;
}

// UART initialization
void init_uart(void) {
    UBRR0H = (uint8_t)(BAUD_PRESCALE >> 8);
    UBRR0L = (uint8_t)BAUD_PRESCALE;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);  // 8-bit data
}

// Send single character via UART
void uart_send_char(char c) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
}

// Send string via UART
void uart_send_string(const char* str) {
    while (*str) {
        uart_send_char(*str++);
    }
}

// ADC initialization
void init_adc(void) {
    // Set ADC reference to internal 1.1V
    ADMUX |= (1 << REFS1) | (1 << REFS0);
    
    // Enable ADC, set prescaler to 128
    ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

// SPI initialization for SD card
void init_spi(void) {
    // Set MOSI, SCK, and SS as outputs
    DDRB |= (1 << DDB3) | (1 << DDB5) | (1 << DDB2);
    
    // Enable SPI, set as master, set clock rate
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
}

// GPIO initialization
void init_gpio(void) {
    // Set motor control pins as outputs
    DDRD |= (1 << DIR_R_PIN) | (1 << DIR_Z_PIN) | 
            (1 << STEP_Z_PIN) | (1 << ENABLE_Z_PIN);
    DDRB |= (1 << STEP_R_PIN) | (1 << ENABLE_R_PIN);
    
    // Set button and limit switch as inputs with pull-up
    DDRD &= ~(1 << BUTTON_PIN);
    DDRB &= ~(1 << LIMIT_SW_PIN);
    PORTD |= (1 << BUTTON_PIN);
    PORTB |= (1 << LIMIT_SW_PIN);
    
    // Set status LED as output
    DDRB |= (1 << STATUS_LED_PIN);
    
    // Enable pin change interrupt for button
    EICRA |= (1 << ISC01);    // Falling edge
    EIMSK |= (1 << INT0);     // Enable INT0
}

// Status LED control
void blink_status_led(uint8_t times) {
    for (uint8_t i = 0; i < times; i++) {
        SET_PIN(PORTB, STATUS_LED_PIN);
        _delay_ms(100);
        CLEAR_PIN(PORTB, STATUS_LED_PIN);
        _delay_ms(100);
    }
}
// ADC read function
uint16_t read_adc(uint8_t channel) {
    // Select ADC channel
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
    
    // Start conversion
    ADCSRA |= (1 << ADSC);
    
    // Wait for conversion to complete
    while (ADCSRA & (1 << ADSC));
    
    return ADC;
}

// Get distance from sensor
float get_distance(void) {
    float analog = 0.0f;
    float measured_analog = 0.0f;
    
    for (uint8_t i = 0; i < SCAN_AMOUNT; i++) {
        measured_analog = (float)read_adc(0);
        analog += measured_analog;
        _delay_ms(2);
    }
    
    float raw_distance = analog / SCAN_AMOUNT;
    float voltage = raw_distance * 3.3f / 1023.0f;
    float sensor_distance = 13.0f * powf(voltage, -1);
    
    if (sensor_distance < 4.0f || sensor_distance > 30.0f) {
        sensor_error = 1;
        return -1.0f;
    }
    
    distance = fabsf(DISTANCE_TO_CENTER - sensor_distance);
    x = distance * cosf(angle);
    y = distance * sinf(angle);
    
    return distance;
}

// Home Z-axis function
void home_z_axis(void) {
    char status_buf[32];
    uart_send_string("Homing Z-axis...\r\n");
    
    CLEAR_PIN(PORTD, ENABLE_Z_PIN);
    _delay_ms(5);  // Wait for driver to enable
    
    // Move up slightly first
    uart_send_string("Moving up slightly...\r\n");
    CLEAR_PIN(PORTD, DIR_Z_PIN);
    for (uint8_t i = 0; i < 50; i++) {
        SET_PIN(PORTD, STEP_Z_PIN);
        _delay_us(STEP_DELAY_US);
        CLEAR_PIN(PORTD, STEP_Z_PIN);
        _delay_us(STEP_DELAY_US);
    }
    
    // Move down until limit switch
    uart_send_string("Moving down until limit switch...\r\n");
    SET_PIN(PORTD, DIR_Z_PIN);
    
    while(READ_PIN(PINB, LIMIT_SW_PIN)) {
        SET_PIN(PORTD, STEP_Z_PIN);
        _delay_us(STEP_DELAY_US);
        CLEAR_PIN(PORTD, STEP_Z_PIN);
        _delay_us(STEP_DELAY_US);
    }
    
    uart_send_string("Limit switch triggered!\r\n");
    
    // Move up to clear switch
    uart_send_string("Moving up to clear switch...\r\n");
    CLEAR_PIN(PORTD, DIR_Z_PIN);
    for(uint8_t i = 0; i < 50; i++) {
        SET_PIN(PORTD, STEP_Z_PIN);
        _delay_us(STEP_DELAY_US);
        CLEAR_PIN(PORTD, STEP_Z_PIN);
        _delay_us(STEP_DELAY_US);
    }
    
    SET_PIN(PORTD, ENABLE_Z_PIN);
    z = 0.0f;
    uart_send_string("Z-axis homed successfully!\r\n");
}

// Check for object presence
uint8_t check_for_object(void) {
    uint16_t valid_points = 0;
    uint16_t total_points = STEPS_PER_ROTATION / 16;
    
    for (uint16_t i = 0; i < total_points; i++) {
        sensor_error = 0;
        get_distance();
        
        if (!sensor_error && distance < MAX_OBJECT_DISTANCE) {
            valid_points++;
        }
        
        for (uint8_t j = 0; j < 16; j++) {
            CLEAR_PIN(PORTB, ENABLE_R_PIN);
            CLEAR_PIN(PORTD, DIR_R_PIN);
            SET_PIN(PORTB, STEP_R_PIN);
            _delay_us(STEP_DELAY_US);
            CLEAR_PIN(PORTB, STEP_R_PIN);
            _delay_us(STEP_DELAY_US);
        }
    }
    
    SET_PIN(PORTB, ENABLE_R_PIN);
    return (valid_points > (total_points / 4));
}

// SD card operations
void get_next_filename(void) {
    FRESULT fr;
    
    while (1) {
        sprintf(file_name, "SCAN%03d.TXT", file_number);
        fr = f_open(&file, file_name, FA_READ);
        if (fr == FR_NO_FILE) {
            uart_send_string("Creating new file: ");
            uart_send_string(file_name);
            uart_send_string("\r\n");
            break;
        }
        f_close(&file);
        file_number++;
        if (file_number > 999) file_number = 1;
    }
}

void write_file_header(void) {
    FRESULT fr;
    
    fr = f_open(&file, file_name, FA_WRITE | FA_CREATE_ALWAYS);
    if (fr == FR_OK) {
        f_printf(&file, "X,Y,Z\n");
        f_printf(&file, "# Scan parameters: Height: %d cm, Layer height: %.1f mm\n",
                (int)Z_AXIS_HEIGHT, Z_LAYER_HEIGHT);
        f_printf(&file, "# Data points below:\n");
        f_close(&file);
    } else {
        uart_send_string("Error opening file for header!\r\n");
    }
}

void write_to_sd(float x, float y, float z) {
    FRESULT fr;
    
    fr = f_open(&file, file_name, FA_WRITE | FA_OPEN_APPEND);
    if (fr == FR_OK) {
        f_printf(&file, "%.3f,%.3f,%.3f\n", x, y, z);
        f_close(&file);
    } else {
        uart_send_string("Error writing to file!\r\n");
    }
}

// Button interrupt handler
ISR(INT0_vect) {
    uint32_t current_time = timer_get_ms();
    
    if ((current_time - last_debounce_time) > DEBOUNCE_DELAY_MS) {
        if (!READ_PIN(PIND, BUTTON_PIN)) {  // Button pressed (active low)
            if (scan == 1) {
                scan = 0;
                SET_PIN(PORTD, ENABLE_Z_PIN);
                SET_PIN(PORTB, ENABLE_R_PIN);
                uart_send_string("Scan stopped by user.\r\n");
                home_z_axis();
            } else {
                uart_send_string("Starting new scan...\r\n");
                scan = 1;
                z = 0.0f;
                angle = 0.0f;
                z_loop = 0;
                empty_scan_count = 0;
                object_found = 0;
                get_next_filename();
                write_file_header();
                home_z_axis();
            }
        }
        last_debounce_time = current_time;
    }
}

// Main function
int main(void) {
    // System initialization
    init_system();
    
    // Initialize SD card
    if (f_mount(&fs, "", 0) != FR_OK) {
        uart_send_string("SD card initialization failed!\r\n");
        blink_status_led(5);  // Error indication
        while (1);
    }
    
    // Calculate steps for Z-axis movement
    steps_z_height = (uint16_t)((Z_LAYER_HEIGHT * STEPS_PER_ROTATION * 
                                LEAD_SCREW_ROTATIONS_PER_CM) / 10.0f);
    
    uart_send_string("3D Scanner initialized!\r\n");
    uart_send_string("Press button to start scanning.\r\n");
    blink_status_led(2);  // Ready indication
    
    // Main loop
    while (1) {
        if (scan == 1) {
            if (z < Z_AXIS_HEIGHT) {
                // Check for object presence at start of each layer
                if (!object_found) {
                    object_found = check_for_object();
                    if (!object_found) {
                        empty_scan_count++;
                        char count_buf[32];
                        sprintf(count_buf, "Empty scan count: %d\r\n", empty_scan_count);
                        uart_send_string(count_buf);
                        
                        if (empty_scan_count >= EMPTY_SCAN_THRESHOLD) {
                            uart_send_string("Object no longer detected. Stopping scan.\r\n");
                            scan = 0;
                            SET_PIN(PORTD, ENABLE_Z_PIN);
                            SET_PIN(PORTB, ENABLE_R_PIN);
                            home_z_axis();
                            continue;
                        }
                    } else {
                        empty_scan_count = 0;
                    }
                }

                // Perform full rotation scan for current layer
                for (uint16_t loop_cont = 0; loop_cont < STEPS_PER_ROTATION; loop_cont++) {
                    // Check if scan was stopped
                    if (scan == 0) {
                        SET_PIN(PORTD, ENABLE_Z_PIN);
                        SET_PIN(PORTB, ENABLE_R_PIN);
                        home_z_axis();
                        break;
                    }

                    sensor_error = 0;
                    get_distance();

                    if (!sensor_error) {
                        // Rotate turntable one step
                        CLEAR_PIN(PORTB, ENABLE_R_PIN);
                        CLEAR_PIN(PORTD, DIR_R_PIN);
                        SET_PIN(PORTB, STEP_R_PIN);
                        _delay_us(STEP_DELAY_US);
                        CLEAR_PIN(PORTB, STEP_R_PIN);
                        _delay_us(STEP_DELAY_US);

                        angle += RADIANS;
                        write_to_sd(x, y, z);
                    }
                }

                // Check if scan was stopped during rotation
                if (scan == 0) continue;

                // Move Z-axis up by one layer
                while (z_loop < steps_z_height) {
                    // Check if scan was stopped
                    if (scan == 0) {
                        SET_PIN(PORTD, ENABLE_Z_PIN);
                        SET_PIN(PORTB, ENABLE_R_PIN);
                        home_z_axis();
                        break;
                    }

                    // Move Z-axis one step
                    CLEAR_PIN(PORTD, ENABLE_Z_PIN);
                    CLEAR_PIN(PORTD, DIR_Z_PIN);
                    SET_PIN(PORTD, STEP_Z_PIN);
                    _delay_us(STEP_DELAY_US);
                    CLEAR_PIN(PORTD, STEP_Z_PIN);
                    _delay_us(STEP_DELAY_US);
                    z_loop++;
                }

                // Check if scan was stopped during Z movement
                if (scan == 0) continue;

                // Update Z position and reset loop counter
                z += Z_LAYER_HEIGHT;
                z_loop = 0;

                // Status update
                char status_buf[32];
                sprintf(status_buf, "Layer complete. Z = %.2f\r\n", z);
                uart_send_string(status_buf);
            } else {
                // Scan complete - reached maximum height
                SET_PIN(PORTD, ENABLE_Z_PIN);
                SET_PIN(PORTB, ENABLE_R_PIN);
                scan = 0;
                
                uart_send_string("Scan complete!\r\n");
                blink_status_led(3);  // Completion indication
                
                home_z_axis();
            }
        } else {
            // Not scanning - idle mode
            TOGGLE_PIN(PORTB, STATUS_LED_PIN);  // Blink status LED
            _delay_ms(500);
        }
    }
    
    return 0;  // Never reached in embedded system
}