#include <stdio.h>
#include "pico/stdlib.h"


int main()
{
    stdio_init_all();
    gpio_init(25);
    gpio_set_dir(25,1);
    gpio_put(25,1);
}
