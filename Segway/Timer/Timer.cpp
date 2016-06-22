
/*
	Written by Niklas Fauth at 4am. No warranty...
*/

#include "Timer.h"

Timer::Timer() {}

Timer::~Timer() {}

bool Timer::prepareTimer(unsigned long frequency) {
    volatile int* CMR0 = (volatile int*)(TIMER_MODULE + TIMER_CMR0_OFFSET);
    volatile int* RC0 = (volatile int*)(TIMER_MODULE + TIMER_RC0_OFFSET);
    setIsTimerInterruptEnabled(true);
    *CMR0 |= 1 << 15; // waveform mode enabled
    *CMR0 |= 1 << 14; // RC is chosen
    *CMR0 |= 1 << 1; // clock defined in configur
    if (2000000 / frequency <= 0xFFFF) {
        *RC0 = 2000000 / frequency;
        return true;
    }
    return false;
}

bool Timer::initTimer(unsigned long frequency) {
    if (prepareTimer(frequency) == true) {
        setIsTimerInterruptEnabled(true);
        return true;
    }
    return false;
}

void Timer::setIsTimerEnabled(bool enabled) {
    volatile int* CCR0 = (volatile int*)(TIMER_MODULE + TIMER_CCR0_OFFSET);
    if (enabled == true) {
        *CCR0 |= 1; //enable the clock
        *CCR0 |= 1 << 2; //clock is started
    } else *CCR0 |= 1 << 1; //disable the clock
}


void Timer::setIsTimerInterruptEnabled(bool enabled) {
    volatile int* IER0 = (volatile int*)(TIMER_MODULE + TIMER_IER0_OFFSET);
    volatile int* IDR0 = (volatile int*)(TIMER_MODULE + TIMER_IDR0_OFFSET);
    if (enabled == true)
        *IER0 |= 1 << 4; //a RC compare has occured
    else *IDR0 |= 1 << 4; //Interrupt Mask Register, read-only
}

void Timer::resetInterruptFlag(void) {
    BIT_IS_SET(*(volatile int*)(TIMER_MODULE + TIMER_SR0_OFFSET), 4);
}

bool Timer::getIsTimerEnabled(void) {
    return (bool)(*(volatile int*)(TIMER_MODULE + TIMER_SR0_OFFSET) & 0x10000);
}

bool Timer::getIsInterruptEnabled(void) {
    return (bool)(*(volatile int*)(TIMER_MODULE + TIMER_IMR0_OFFSET) & 0x10);      // RC Compare
}
