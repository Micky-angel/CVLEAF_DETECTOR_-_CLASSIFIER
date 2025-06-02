#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/adc.h"



// === LED N0 definido para parpadeo con Timer ===
#define LED0_TIMER_PORT GPIO_PORTN_BASE
#define LED0_TIMER_PIN  GPIO_PIN_0

// === LED N1 definido para parpadeo con Timer ===
#define LED_TIMER_PORT GPIO_PORTN_BASE
#define LED_TIMER_PIN  GPIO_PIN_1

// === Definiciones de botones ===
#define BOTON1_PORT    GPIO_PORTC_BASE
#define BOTON1_PIN     GPIO_PIN_5
#define BOTON1_PULL    PULL_UP

#define BOTON2_PORT    GPIO_PORTJ_BASE
#define BOTON2_PIN     GPIO_PIN_1
#define BOTON2_PULL    PULL_DOWN


// === PWM 1: PK4 - M0PWM6
#define PWM1_PORT GPIO_PORTK_BASE
#define PWM1_PIN  GPIO_PIN_4
#define PWM1_BASE PWM0_BASE
#define PWM1_GEN  PWM_GEN_3
#define PWM1_OUT  PWM_OUT_6
#define PWM1_BIT  PWM_OUT_6_BIT

// === PWM 2: PF2 - M0PWM2
#define PWM2_PORT GPIO_PORTF_BASE
#define PWM2_PIN  GPIO_PIN_2
#define PWM2_BASE PWM0_BASE
#define PWM2_GEN  PWM_GEN_1
#define PWM2_OUT  PWM_OUT_2
#define PWM2_BIT  PWM_OUT_2_BIT

// === CONFIGURACIÓN ADC PARA POTENCIÓMETRO ===
#define ADC_PERIPHERAL     SYSCTL_PERIPH_ADC0
#define ADC_BASE           ADC0_BASE
#define ADC_SEQUENCER      3
#define ADC_CHANNEL        ADC_CTL_CH1
#define ADC_PIN            GPIO_PIN_2
#define ADC_GPIO_PERIPH    SYSCTL_PERIPH_GPIOE
#define ADC_GPIO_BASE      GPIO_PORTE_BASE


#define PWM_FREQUENCY 1000
uint32_t pwmLoad;

typedef enum {
    NO_PULL,
    PULL_UP,
    PULL_DOWN
} gpio_resistencia_t;


// === Retardo en milisegundos basado en SysCtlDelay()
void delay_ms(uint32_t ms) {
    SysCtlDelay((120000000 / 3000) * ms);
}


// === Función para configurar LEDs como salida digital ===
void configurar_led(uint32_t puerto, uint8_t pin) {
    GPIOPinTypeGPIOOutput(puerto, pin);
}

// === Prender un LED (coloca el pin en nivel alto) ===
void prender_led_gpio(uint32_t puerto, uint8_t pin) {
    GPIOPinWrite(puerto, pin, pin);
}

// === Apagar un LED (coloca el pin en nivel bajo) ===
void apagar_led_gpio(uint32_t puerto, uint8_t pin) {
    GPIOPinWrite(puerto, pin, 0);
}

void toggle_led(uint32_t puerto, uint8_t pin) {
    uint8_t estado = GPIOPinRead(puerto, pin);
    GPIOPinWrite(puerto, pin, estado ^ pin);
}

// === Función para configurar BOTONES como entrada digital ===
void configurar_boton(uint32_t puerto, uint8_t pin, gpio_resistencia_t tipo_resistencia) {
    // Habilita el periférico del GPIO correspondiente
    SysCtlPeripheralEnable(
        (puerto == GPIO_PORTA_BASE) ? SYSCTL_PERIPH_GPIOA :
        (puerto == GPIO_PORTB_BASE) ? SYSCTL_PERIPH_GPIOB :
        (puerto == GPIO_PORTC_BASE) ? SYSCTL_PERIPH_GPIOC :
        (puerto == GPIO_PORTD_BASE) ? SYSCTL_PERIPH_GPIOD :
        (puerto == GPIO_PORTE_BASE) ? SYSCTL_PERIPH_GPIOE :
        (puerto == GPIO_PORTF_BASE) ? SYSCTL_PERIPH_GPIOF :
        (puerto == GPIO_PORTG_BASE) ? SYSCTL_PERIPH_GPIOG :
        (puerto == GPIO_PORTH_BASE) ? SYSCTL_PERIPH_GPIOH :
        (puerto == GPIO_PORTJ_BASE) ? SYSCTL_PERIPH_GPIOJ :
        (puerto == GPIO_PORTK_BASE) ? SYSCTL_PERIPH_GPIOK :
        (puerto == GPIO_PORTL_BASE) ? SYSCTL_PERIPH_GPIOL :
        (puerto == GPIO_PORTM_BASE) ? SYSCTL_PERIPH_GPIOM :
        (puerto == GPIO_PORTN_BASE) ? SYSCTL_PERIPH_GPION :
        (puerto == GPIO_PORTQ_BASE) ? SYSCTL_PERIPH_GPIOQ : 0);

    // Configurar como entrada
    GPIOPinTypeGPIOInput(puerto, pin);

    // Configurar tipo de resistencia
    switch (tipo_resistencia) {
        case PULL_UP:
            GPIOPadConfigSet(puerto, pin, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
            break;
        case PULL_DOWN:
            GPIOPadConfigSet(puerto, pin, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);
            break;
        case NO_PULL:
        default:
            GPIOPadConfigSet(puerto, pin, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
            break;
    }
}

// === Función para checkear BOTONES como entrada digital ===
bool boton_presionado(uint32_t puerto, uint8_t pin, gpio_resistencia_t tipo_resistencia) {
    uint8_t estado = GPIOPinRead(puerto, pin) & pin;

    if (tipo_resistencia == PULL_UP) {
        return (estado == 0);  // Activo bajo
    } else {
        return (estado != 0);  // Activo alto
    }
}


// === Configura PWM en un pin específico
void configurar_pwm_led(uint32_t pwm_base, uint32_t gen, uint32_t out, uint32_t bit,
                        uint32_t port, uint8_t pin, uint32_t pin_config_macro)
{
    SysCtlPeripheralEnable(pwm_base == PWM0_BASE ? SYSCTL_PERIPH_PWM0 : SYSCTL_PERIPH_PWM1);
    SysCtlPeripheralEnable(port == GPIO_PORTK_BASE ? SYSCTL_PERIPH_GPIOK : SYSCTL_PERIPH_GPIOF);



    GPIOPinConfigure(pin_config_macro);
    GPIOPinTypePWM(port, pin);

    PWMClockSet(pwm_base, PWM_SYSCLK_DIV_64);
    uint32_t pwmClock = SysCtlClockGet() / 64;
    pwmLoad = (pwmClock / PWM_FREQUENCY) - 1;

    PWMGenConfigure(pwm_base, gen, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(pwm_base, gen, pwmLoad);
    PWMPulseWidthSet(pwm_base, out, (uint32_t)(pwmLoad * 0.2f));

    PWMOutputState(pwm_base, bit, true);
    PWMGenEnable(pwm_base, gen);
}

// === Ajusta el duty cycle de un pin PWM
void ajustar_pwm_duty(uint32_t pwm_base, uint32_t out, float duty_percent)
{
    if (duty_percent > 1.0f) duty_percent = 1.0f;
    if (duty_percent < 0.0f) duty_percent = 0.0f;
    PWMPulseWidthSet(pwm_base, out, (uint32_t)(pwmLoad * duty_percent));
}

// === Configuracion del ADC
void configurar_adc(void) {
    SysCtlPeripheralEnable(ADC_PERIPHERAL);
    SysCtlPeripheralEnable(ADC_GPIO_PERIPH);
    GPIOPinTypeADC(ADC_GPIO_BASE, ADC_PIN);

    ADCSequenceConfigure(ADC_BASE, ADC_SEQUENCER, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC_BASE, ADC_SEQUENCER, 0, ADC_CHANNEL | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC_BASE, ADC_SEQUENCER);
    ADCIntClear(ADC_BASE, ADC_SEQUENCER);
}
// === Lectura del ADC
uint32_t leer_adc_valor(void) {
    uint32_t resultado;

    ADCProcessorTrigger(ADC_BASE, ADC_SEQUENCER);
    while (!ADCIntStatus(ADC_BASE, ADC_SEQUENCER, false)) {}
    ADCIntClear(ADC_BASE, ADC_SEQUENCER);
    ADCSequenceDataGet(ADC_BASE, ADC_SEQUENCER, &resultado);

    return resultado;  // Valor entre 0 y 4095
}


//--------------------------------------------------------------------------------TIMER 100ms
// === Interrupción del Timer0A cada 100 ms para parpadear PN1 ===
void Timer0IntHandler(void) {
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    toggle_led(LED_TIMER_PORT, LED_TIMER_PIN);
}


// --------------------------------------------------------------------------------------





int main(void)
{
    // === Configurar sistema a 120 MHz ===
    SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                        SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);

    // === LED PN1 (N1) salida digital ===
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    //--add the others if needed ------------------ - - - - - 
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION)) {}
    configurar_led(LED_TIMER_PORT, LED_TIMER_PIN);
    
    // === LED PN0 (N0) salida digital ===
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION)) {}
    configurar_led(LED0_TIMER_PORT, LED0_TIMER_PIN);
    
    // === BOTON1 entrada digital ===
    configurar_boton(BOTON1_PORT, BOTON1_PIN, NO_PULL);


    // === Configurar PWM en PK4 (M0PWM6) ===
    configurar_pwm_led(PWM1_BASE, PWM1_GEN, PWM1_OUT, PWM1_BIT,
                       PWM1_PORT, PWM1_PIN, GPIO_PK4_M0PWM6);

    // === Configurar PWM en PF2 (M0PWM2) ===
    configurar_pwm_led(PWM2_BASE, PWM2_GEN, PWM2_OUT, PWM2_BIT,
                       PWM2_PORT, PWM2_PIN, GPIO_PF2_M0PWM2);

    // === Configurar Timer0A para parpadeo LED cada 100 ms ===
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, (120000000 / 10) - 1);  // 100 ms
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    IntEnable(INT_TIMER0A);
    IntMasterEnable();
    TimerEnable(TIMER0_BASE, TIMER_A);
    
    // === Configurar el ADC
    configurar_adc();
    
    // === Alternancia de duty para los 2 PWM cada 1s ===
    bool high = true;
    uint32_t valor_pot;
    float duty;
    

//------------------------------------------------------------------------------MAIN WHILE
    while (1)
    {
        valor_pot = leer_adc_valor();
        duty = 0.05+valor_pot /5100.0f;
        ajustar_pwm_duty(PWM1_BASE, PWM1_OUT, duty);
        
        if (boton_presionado(BOTON1_PORT, BOTON1_PIN, NO_PULL )) {
            
            ajustar_pwm_duty(PWM2_BASE, PWM2_OUT, 1.0f);
            prender_led_gpio(GPIO_PORTN_BASE, LED0_TIMER_PIN);
            
        } else {
            
            ajustar_pwm_duty(PWM2_BASE, PWM2_OUT, 0.05f);
            apagar_led_gpio(GPIO_PORTN_BASE, LED0_TIMER_PIN);
            
        }

        high = !high;

        delay_ms(10); 
    }

// --------------------------------------------------------------------------------------

}
