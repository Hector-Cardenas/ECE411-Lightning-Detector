#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/interp.h"
#include "hardware/timer.h"
#include "hardware/watchdog.h"
#include "hardware/clocks.h"
#include "pico/cyw43_arch.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

#define LED_DELAY_MS 250

//Debug macro expansions
#ifdef FLASHPRINT
#define FLASH
#define PRINT
#endif

#ifdef PRINTFORCE
#define PRINT
#define FORCEINT
#endif

#ifdef DEBUG
#define FLASH
#define PRINT
#define FORCEINT
#endif

// SPI Defines
// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS 17
#define PIN_SCK 18
#define PIN_MOSI 19

// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9

// ADC defines
#define THRESHOLD 1
#define ADC_PIN 26
#define ADC_IRQ 22
#define REFERENCE_VOLTAGE 3.3

// UART defines
// By default the stdout UART is `uart0`, so we will use the second one
#define UART_ID uart1
#define BAUD_RATE 115200

// Use pins 4 and 5 for UART1
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define UART_TX_PIN 4
#define UART_RX_PIN 5

//ADF7020 defines (Values to be determined)

#define R0 UINT32_C(0x1138DCA0)
#define R1 UINT32_C(0x00021011)
#define R2 UINT32_C(0x00003ED2)

//7020 transmission defines
#define TXRXDATA_PIN 12
#define TX_T         200
#define PREAMBLE     0xAA
#define DEVICE_ID    0x96
#define CALLSIGN     "KK7ETD"
#define CALL_WIDTH   6


// Function declarations
void overThreshold();
void writeRegister(uint32_t data); //Write to device over spi  
void transceiverInit();
void transmit_id();
void txPacket(char packet);

int main()
{
    // IO initialization
    #ifdef PRINT
    printf("Performing initialization");
    #endif
    stdio_init_all();
    gpio_set_dir(TXRXDATA_PIN, true);
    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(0);

    // variables for ADC conversion
    uint16_t rawInput;
    float conversion = (REFERENCE_VOLTAGE) / (1 << 12);
    uint16_t rfInput;

    // Initialise the Wi-Fi chip
    if (cyw43_arch_init())
    {
        printf("Wi-Fi init failed\n");
        return -1;
    }

    // SPI initialisation. This example will use SPI at 1MHz.
    spi_init(SPI_PORT, 1000 * 1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_init(PIN_CS);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);

    //Initializing transceiver
    #ifdef PRINT
    printf("Initializing transceiver");
    #endif
    transceiverInit();

    while (true)
    {
	#ifndef FORCEINT
        rfInput = adc_read() * conversion;
	#else
	rfInput = 10;
	#endif
        if (rfInput > THRESHOLD){
	#ifdef PRINT
	printf("We are checking if the input exceeds the threshold");
	#endif
            overThreshold();
	}
    }
}

//function definitions
void overThreshold(){ // we have gone over our threshold this is where our transceiver/wifi output goes
    #ifdef PRINT
    printf("We are over the threshold, doing something");
    #endif
    #ifdef FLASH
    while (1){ // this will just flash the onboard LED when we interrupt
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN,true);
        sleep_ms(LED_DELAY_MS);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN,false);
        sleep_ms(LED_DELAY_MS);
    }
    #endif
    transmit_id();
}

void writeRegister(uint32_t data){ // Sending data over SPI
    #ifdef PRINT
    printf("We are sending %x over SPI", data);
    #endif
    uint16_t buffer[2]; //When we send the data over SPI its in one word so we jsut need to split the word into two halfwords
    buffer[0] = data & 0xFFFF; // Lower bits 
    buffer[1] = (data >> 16) & 0xFFFF; // Upper bits
    spi_write16_blocking(spi0,buffer,2);
}

void transceiverInit(){ // Initializing the AD7020 over SPI
    #ifdef PRINT
    printf("We are initializing the 7020 transceiver\n");
    #endif
    writeRegister(R0);
    writeRegister(R1);
    writeRegister(R2);
}

void transmit_id(){    
    #ifdef PRINT
    printf("We are transmitting using GPIO from Pico W to 7020\n");
    #endif
    // Transmit  preamble, callsign, and device ID
    char id = DEVICE_ID;
    char callsign[CALL_WIDTH] = CALLSIGN;
    char preamble = PREAMBLE;
    
    // Send message components
    txPacket(preamble);
    for (int i = 0; i < CALL_WIDTH; i++) txPacket(callsign[i]);
    txPacket(id); 

    gpio_put(TX_T, 0); // Set pin low to mute PA
}
    

void txPacket(char packet){
    #ifdef PRINT
    printf("We are in the packet transmission function, part of transmit_id\n");
    #endif
    for (int i = sizeof(char)-1; i >= 0; i--) {     
    // Transmit starting from MSB, using IEEE Manchester Code
        gpio_put(TXRXDATA_PIN, (~(packet >> i) & 1)); 
        sleep_ms(TX_T/2);
        gpio_put(TXRXDATA_PIN, ( (packet >> i) & 1));
        sleep_ms(TX_T/2);
	}
}
