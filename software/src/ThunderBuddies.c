#include "hardware/irq.h"
#include "pico/stdlib.h"

#ifdef CYW43_WL_GPIO_LED_PIN
#include "pico/cyw43_arch.h"
#endif

#ifdef LED_DELAY_MS
#define LED_DELAY_MS 250
#endif



void ADCIRQHandler(void);
void overThreshold(void);

/*TO BE PUT IN MAIN */
irq_set_exclusive_handler(ADC_IRQ_FIFO,&ADCIRQHandler);


// if the interrupt bit is set, we have gone over our threshold and need to respond accordingly
irq_handler_t ADCIRQHandler(void){
   overThreshold();
   irq_clear(ADC_IRQ_FIFO); //Clear the IRQ bit so we can respond to another one in teh future
}

void overThreshold(void){ // we have gone over our threshold this is where our transceiver/wifi output goes
   //TODO: do what we need to do ie wifi/transceiver output
   //For now (11-7-24), just going to add the on board LED blink to verify the interrupt is working as expected 
    int rc =  cyw43_arch_init();
    hard_assert(rc == PICO_OK);
    while (1){
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN,true);
        sleep_ms(LED_DELAY_MS);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN,false);
        sleep_ms(LED_DELAY_MS);
    }
}