#include "interruptTools.h"
#include "Configuration/Configuration.h"
#include "Segway/Segway.h"
#include "Timer/Timer.h"

#include "bitmacros.h"

const char* MYAVR32_PM_ADDRESS = (char*) 0xFFFF0C00;
const unsigned int MYAVR32_PM_OFFSET_OSCCTL0 = 0x0028;
const unsigned int MYAVR32_PM_OFFSET_MCCTRL = 0x0000;
const unsigned int MYAVR32_PM_OFFSET_POSCSR = 0x0054;

const char* MYAVR32_INTC_ADDRESS = (char*) 0xFFFF0800;
const unsigned int MYAVR32_INTC_OFFSET_IPR0 = 0x0000;
const unsigned int MYAVR32_INTC_SIZE_IPR = 0x0004;

/*! \def SIMULATION_TEST
\brief Don't run the segway controlling code, but the code for checking the helper classes.
*/
//#define SIMULATION_TEST

Segway* mySegway = 0;

// Import the symbols _evba, _int0, _int1, _int2, _int3 from exception.s
extern "C" void _evba(), _int0(), _int1(), _int2(), _int3();

/*! \brief  Interrupt handler, called when an interrupt occurs. Selects timer interrupts and calls timer function.
    \return zero, so no further action is taken.
*/
extern "C" void* _get_interrupt_handler(unsigned long int_level) {
    if (int_level == 0) {
        // If mySegway is present, call it's timer function.
        if (mySegway != 0)
            mySegway->timerFunction();
        // Reset interrupt flag
        Timer::resetInterruptFlag();
    }
    return 0;
}

/*! \brief  Switches system clock to external OSC0
*/
void init_system_clock() {
    // Set oscillator gain class "G3" (as f_OSC >= 8 MHz)
    //AVR32_PM.OSCCTRL0.mode = AVR32_PM_OSCCTRL0_MODE_CRYSTAL_G3;
    SET_BITS(* (VINTP)(MYAVR32_PM_ADDRESS + MYAVR32_PM_OFFSET_OSCCTL0), (0x007 << 0));
    // Startup time: 4096 cycles
    //AVR32_PM.OSCCTRL0.startup = AVR32_PM_OSCCTRL0_STARTUP_0_RCOSC;
    CLEAR_BITS(* (VINTP)(MYAVR32_PM_ADDRESS + MYAVR32_PM_OFFSET_OSCCTL0), (0x007 << 8));
    // Enable Oscillator 0
    //AVR32_PM.MCCTRL.osc0en = 1;
    SET_BIT(* (VINTP)(MYAVR32_PM_ADDRESS + MYAVR32_PM_OFFSET_MCCTRL), 2);
    // Wait for oscillator 0 to be ready
    //while(!AVR32_PM.POSCSR.osc0rdy);
    while (!BIT_IS_SET(* (VINTP)(MYAVR32_PM_ADDRESS + MYAVR32_PM_OFFSET_POSCSR), 7));
    // Select oscillator 0 as source for the main clock
    //AVR32_PM.MCCTRL.mcsel = 1;
    CLEAR_BITS(* (VINTP)(MYAVR32_PM_ADDRESS + MYAVR32_PM_OFFSET_MCCTRL), (0x003 << 0));
    SET_BIT(* (VINTP)(MYAVR32_PM_ADDRESS + MYAVR32_PM_OFFSET_MCCTRL), 0);
}

/*! \brief  Initializes the interrupt system to redirect timer interrupts to _int0(), which is implemented in assembler.
*/
void init_interrupt_system() {
    // Set exception handler (defined in exception.s)
    __builtin_mtsr(0x0004, (unsigned long) &_evba);
    // The timer's interrupt group is 14 for all timers.
    // Set the interrupt level of the timer's interrupt group to 0.
    //AVR32_INTC.IPR[14].intlevel = AVR32_INTC_INT0;
    CLEAR_BITS(* (VINTP)(MYAVR32_INTC_ADDRESS + MYAVR32_INTC_OFFSET_IPR0 + 14 * MYAVR32_INTC_SIZE_IPR), (0x003 << 30));
    // Set the interrupt handler offset of timer's interrupt group. => Timer interrupts will be called having int_level = 0.
    //AVR32_INTC.IPR[14].autovector = ( int )&_int0 - ( int )&_evba;
    CLEAR_BITS(* (VINTP)(MYAVR32_INTC_ADDRESS + MYAVR32_INTC_OFFSET_IPR0 + 14 * MYAVR32_INTC_SIZE_IPR), (0x3FFF << 0));
    SET_BITS(* (VINTP)(MYAVR32_INTC_ADDRESS + MYAVR32_INTC_OFFSET_IPR0 + 14 * MYAVR32_INTC_SIZE_IPR), (((int) &_int0 - (int) &_evba) << 0));
    // Low Priority for interrupts caused by the PWM controller
    //AVR32_INTC.IPR[12].intlevel = AVR32_INTC_INT3;
    SET_BITS(* (VINTP)(MYAVR32_INTC_ADDRESS + MYAVR32_INTC_OFFSET_IPR0 + 12 * MYAVR32_INTC_SIZE_IPR), (0x003 << 30));
    // Globally enable interrupts
    cpu_irq_enable();
}

/*! \brief  Interrupt handler, called when an interrupt occurs. Selects timer interrupts and calls timer function.
    \return zero
*/
int main(void) {
    // Use external OSC0
    init_system_clock();
    // Prepare interrupt handler (needed for the timer)
    init_interrupt_system();
    // Initialize configuration
    Configuration::init();
    while (true) {
#ifdef SIMULATION_TEST
        // Enter Simulation mode
        Simulation mySimulation;
        mySimulation.main();
#else
        // Enter Segway mode
        mySegway = new Segway;
        mySegway->main();
        delete mySegway;
        mySegway = 0;
#endif
    }
    return 0;
}