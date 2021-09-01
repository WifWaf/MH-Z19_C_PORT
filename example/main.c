
/*** !!! this is not a functional demonstration, purely an example of what is required ***/

#include <string.h>
#include <stdio.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "MH-Z19.h"

MHZ19_mem_t mhz19_mem;

int MHZ19_link_available();
uint8_t MHZ19_link_read(uint8_t *buff, uint8_t len);
void MHZ19_link_write(uint8_t *buff, uint8_t len);
void MHZ19_link_print(char *text);
void MHZ19_link_delay_ms(int duration);
int MHZ19_link_elapse_ms();

void app_main(void)
{
   // Provided pointer adresses to the typedef structure MHZ19_mem_t //
    mhz19_mem.available = MHZ19_link_available;
    mhz19_mem.read_ = MHZ19_link_read;
    mhz19_mem.write = MHZ19_link_write;
    mhz19_mem.print = MHZ19_link_print;
    mhz19_mem.delay_ms = MHZ19_link_delay_ms;
    mhz19_mem.elapse_ms = MHZ19_link_elapse_ms;

    // Set configuration options for the config (this is not essential for most applications) //
    mhz19_mem.cfg |= MHZ19_FILTER_CLR_EN;

   // Pass struct to the library for initialisation
    MHZ19_begin(&mhz19_mem);

   for(;;)
   {
      int co2_val = MHZ19_getCO2(true);
      printf("CO2: %d ppm", co2_val);
      vTaskDelay(pdMS_TO_TICKS(1000));
   }
}


int MHZ19_link_available()
{
    return uart_available();
}

void MHZ19_link_read(uint8_t *buff, uint8_t len)
{
    uart_read(buff, len);
}  

void MHZ19_link_write(uint8_t *data, uint8_t len)
{
    uart_write(data, len);
}

void MHZ19_link_print(char *text)
{
    printf("%s", text);
}

void MHZ19_link_delay_ms(int duration)
{
    vTaskDelay(pdMS_TO_TICKS(duration));
}

int MHZ19_link_elapse_ms()
{
    int64_t stamp = get_time(); 
    stamp /= 1e3;  // us to ms as an example if it is needed.

    return (int)stamp;
}
