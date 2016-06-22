// Taken from interrupt_avr32.h (Copyright (c) 2010-2012 Atmel Corporation. All rights reserved.)

#ifndef INTERRUPTTOOLS_H_
#define INTERRUPTTOOLS_H_

// barrier() prevents compiler from relocating code across this barrier
#define barrier()                              \
    asm volatile( "" ::: "memory" )

// cpu_irq_enable() enables interrupts globally
#define cpu_irq_enable()                       \
    do {                                       \
        barrier();                             \
        __builtin_csrf( 16 );                  \
    } while( 0 )
// cpu_irq_enable() disables interrupts globally
#define cpu_irq_disable()                      \
    do {                                       \
        __builtin_ssrf( 16 );                  \
        barrier();                             \
    } while( 0 )



#endif /* INTERRUPTTOOLS_H_ */