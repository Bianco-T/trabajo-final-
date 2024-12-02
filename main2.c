#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"

#define ADC_PIN 26
#define PWM_PIN 15

float Kp = 1.0, Ki = 0.1, Kd = 0.01;
float referencia = 2048.0;

float error = 0, integral = 0, derivative = 0, prev_error = 0;
float control = 0;

// Límites para el término integral y el control
float integral_max = 1000.0; // Límite máximo del término integral
float integral_min = -1000.0; // Límite mínimo del término integral
float control_max = 4095.0;  // Límite máximo del control (duty cycle)
float control_min = 0.0;     // Límite mínimo del control (duty cycle)

void setup() {
    stdio_init_all();

    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(0);

    // Configuración del PWM
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(PWM_PIN);
    pwm_set_wrap(slice_num, 4095);
    pwm_set_gpio_level(PWM_PIN, 0);
    pwm_set_enabled(slice_num, true);
}

int main() {
    setup();

    while (1) {
        // Leer el valor del ADC
        uint16_t adc_raw = adc_read();
        float feedback = (float)adc_raw;

        // Calcular el error
        error = referencia - feedback;

        // Calcular el término integral con límite
        integral += error;
        if (integral > integral_max) integral = integral_max;
        if (integral < integral_min) integral = integral_min;

        // Calcular el término derivativo
        derivative = error - prev_error;

        // Calcular el control
        control = Kp * error + Ki * integral + Kd * derivative;

        // Limitar el valor del control
        if (control > control_max) control = control_max;
        if (control < control_min) control = control_min;

        // Actualizar el error previo
        prev_error = error;

        // Actualizar el PWM
        pwm_set_gpio_level(PWM_PIN, (uint16_t)control);

        
        printf("Feedback: %.2f, Error: %.2f, Control: %.2f, PWM Output: %.2f%%\n",
               feedback, error, control, (control / 4095.0) * 100.0);
        printf("Kp: %.2f, Ki: %.2f, Kd: %.2f\n", Kp, Ki, Kd);

        sleep_ms(500);
    }

    return 0;
}
