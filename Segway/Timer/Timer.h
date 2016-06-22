/*
    Timer class for AVR32UC3B

    DEPENDENCIES:
    An interrupt handling system has to be set up. This class only manages the timer registers.

    EXAMPLE INTERRUPT HANDLING SYSTEM:
    See main.cpp.example, exception.s.example


    Written by Niklas Fauth at 4am. No warranty...
    

 */


// adresses
#define GPIO_MODULE 0xFFFF1000
#define GPIO_GPER_OFFSET 0x00 // Enable
#define GPIO_PMR0_OFFSET 0x10 // Peripheral mux 0
#define GPIO_PMR1_OFFSET 0x20 // Peripheral mux 1
#define GPIO_ODER_OFFSET 0x40 // Output driver
#define GPIO_OVR_OFFSET 0x50 // Output value
#define GPIO_PVR_OFFSET 0x60 // Value
#define GPIO_PUER_OFFSET 0x70 // Pull-up
#define GPIO_GFER_OFFSET 0xC0 // Glitch filter
#define GPIO_PORT_OFFSET 0x0100 // Port size

#define PWM_MODULE 0xFFFF3000
#define PWM_ENA_OFFSET 0x004 // Enable
#define PWM_DIS_OFFSET 0x008 // Disable
#define PWM_SR_OFFSET 0x00C // Status
#define PWM_CMR0_OFFSET 0x200 // Channel 0
#define PWM_CHANNEL_OFFSET 0x020 // Channel size
#define PWM_CDTY0_OFFSET 0x004 // Duty cycle
#define PWM_CPRD0_OFFSET 0x008 // Period
#define PWM_CCNT0_OFFSET 0x00C // Counter
#define PWM_CUPD0_OFFSET 0x010 // Update

#define TIMER_MODULE 0xFFFF3800
#define TIMER_CCR0_OFFSET 0x00 // Control
#define TIMER_CMR0_OFFSET 0x04 // Mode
#define TIMER_CV0_OFFSET 0x10 // Counter Value
#define TIMER_RA0_OFFSET 0x14 // Register A
#define TIMER_RB0_OFFSET 0x18 // Register B
#define TIMER_RC0_OFFSET 0x1C // Register C
#define TIMER_SR0_OFFSET 0x20 // Status
#define TIMER_IER0_OFFSET 0x24 // Interrupt Enable
#define TIMER_IDR0_OFFSET 0x28 // Interrupt Disable
#define TIMER_IMR0_OFFSET 0x2C // Interrupt Mask

#define ADC_MODULE 0xFFFF3C00
#define ADC_CR_OFFSET 0x00 // Control
#define ADC_MR_OFFSET 0x04 // Mode
#define ADC_CHER_OFFSET 0x10 // Channel enable
#define ADC_CHDR_OFFSET 0x14 // Channel disable
#define ADC_CHSR_OFFSET 0x18 // Channel Status
#define ADC_LCDR_OFFSET 0x20 // Last converted data
#define ADC_SR_OFFSET 0x1C // Status

#include "../Configuration/Configuration.h"
#include "../bitmacros.h"

#ifndef TIMER_H_
#define TIMER_H_

class Timer {
  private:

  public:
    Timer();
    ~Timer();

	/** In dieser Methode wird die Kurve 2 (WAVSEL 2) gewaehlt, um das Hochzaehlen des Timers zu realisieren. Diese Kurve ist im Daten Blatt auf Seite 484 dargestellt.
	Die Kurve ist eine Saegezahnkurve mit einem variablen RC-Wert als Maximalwert. Ausserdem wird die Frequenz uebergeben, mit welcher dann der Maximale RC-Wert berechnet wird und somit auch die Haeufigkeit der Interrupts pro Sekunde.
	*/
  bool prepareTimer(unsigned long frequency);
	bool initTimer(unsigned long frequency);
	void setIsTimerEnabled(bool enabled);
	void setIsTimerInterruptEnabled(bool enabled);
  static void resetInterruptFlag(void);
	bool getIsTimerEnabled(void);
  bool getIsInterruptEnabled(void);
};

#endif /* TIMER_H_ */
