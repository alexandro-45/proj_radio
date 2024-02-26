#include "delay.h"
#include <stdint.h>

static volatile uint32_t _millis = 0L;  // millis since start of the program

void vApplicationTickHook(void) {
    ++_millis;
}

uint32_t millis(void) {
    return _millis;
}

void delay(uint32_t time) {
    uint32_t old_millis = _millis;
    while (_millis - old_millis < time);
}