#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "stdlib.h"


#define MICROSTEPS_PER_REV 3200.0f
#define DIAMETRO_POLEA_MM 10.0f
#define M_PI 3.14159265358979323846f

// Motor A (M1) - PC6, PC5, PC4
#define M1_PUL_PORT GPIO_PORTC_BASE
#define M1_PUL_PIN  GPIO_PIN_6
#define M1_DIR_PORT GPIO_PORTC_BASE
#define M1_DIR_PIN  GPIO_PIN_5
#define M1_ENA_PORT GPIO_PORTC_BASE
#define M1_ENA_PIN  GPIO_PIN_4

// Motor B (M2) - PL3, PL2, PL1
#define M2_PUL_PORT GPIO_PORTL_BASE
#define M2_PUL_PIN  GPIO_PIN_3
#define M2_DIR_PORT GPIO_PORTL_BASE
#define M2_DIR_PIN  GPIO_PIN_2
#define M2_ENA_PORT GPIO_PORTL_BASE
#define M2_ENA_PIN  GPIO_PIN_1

void delay_us(uint32_t us) {
    SysCtlDelay((120000000 / 3 / 1000000) * us);
}

void delay_ms(uint32_t ms) {
    SysCtlDelay((120000000 / 3 / 1000) * ms);
}

void setup_motor_pins(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC)) {}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOL)) {}

    // Motor A
    GPIOPinTypeGPIOOutput(M1_PUL_PORT, M1_PUL_PIN);
    GPIOPinTypeGPIOOutput(M1_DIR_PORT, M1_DIR_PIN);
    GPIOPinTypeGPIOOutput(M1_ENA_PORT, M1_ENA_PIN);
    GPIOPinWrite(M1_ENA_PORT, M1_ENA_PIN, 0);  // Activar driver

    // Motor B
    GPIOPinTypeGPIOOutput(M2_PUL_PORT, M2_PUL_PIN);
    GPIOPinTypeGPIOOutput(M2_DIR_PORT, M2_DIR_PIN);
    GPIOPinTypeGPIOOutput(M2_ENA_PORT, M2_ENA_PIN);
    GPIOPinWrite(M2_ENA_PORT, M2_ENA_PIN, 0);  // Activar driver
}

void step_motor_direct(uint8_t motor_id, bool direction) {
    uint32_t pul_port, pul_pin;
    uint32_t dir_port, dir_pin;

    if (motor_id == 1) {
        pul_port = M1_PUL_PORT; pul_pin = M1_PUL_PIN;
        dir_port = M1_DIR_PORT; dir_pin = M1_DIR_PIN;
    } else {
        pul_port = M2_PUL_PORT; pul_pin = M2_PUL_PIN;
        dir_port = M2_DIR_PORT; dir_pin = M2_DIR_PIN;
    }

    GPIOPinWrite(dir_port, dir_pin, direction ? dir_pin : 0);
    GPIOPinWrite(pul_port, pul_pin, pul_pin);
    delay_us(300);
    GPIOPinWrite(pul_port, pul_pin, 0);
    delay_us(300);
}

void movement_xy(float x_mm, float y_mm) {
    float circ_mm = M_PI * DIAMETRO_POLEA_MM;

    float revA = (x_mm + y_mm) / circ_mm;
    float revB = (-x_mm + y_mm) / circ_mm;

    int32_t stepsA = (int32_t)(revA * MICROSTEPS_PER_REV);
    int32_t stepsB = (int32_t)(revB * MICROSTEPS_PER_REV);

    bool dirA = stepsA >= 0;
    bool dirB = stepsB >= 0;

    stepsA = abs(stepsA);
    stepsB = abs(stepsB);

    int32_t max_steps = (stepsA > stepsB) ? stepsA : stepsB;

    for (int32_t i = 0; i < max_steps; i++) {
        if (i < stepsA) step_motor_direct(1, dirA);
        if (i < stepsB) step_motor_direct(2, dirB);
    }
}

int main(void) {
    SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                        SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);

    setup_motor_pins();

    while (1) {
        movement_xy(0.0f, 20.0f);   // 20 mm a la derecha
        delay_ms(1000);
        movement_xy(0.0f, -20.0f);   // 20 mm hacia adelante
        delay_ms(1000);
    }
}

// Evita error del linker si startup_gcc.o declara esta ISR
void Timer0IntHandler(void) {}
