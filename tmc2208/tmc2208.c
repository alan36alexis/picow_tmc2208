#include "tmc2208.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "pico/stdlib.h"

#include <stdio.h>

// Puntero global para ser usado por el callback del timer.
// Limitación: Este diseño simple solo permite controlar un motor.
// Para múltiples motores, se requeriría un diseño más avanzado (ej. un array de punteros).
static TMC2208_t *_motor_ptr_for_callback;

/**
 * @brief Callback del temporizador. Se ejecuta a intervalos regulares para generar los pulsos STEP.
 * Simplemente invierte el estado del pin STEP.
 */
bool repeating_timer_callback(struct repeating_timer *t) {
    if (_motor_ptr_for_callback) {
        if(_motor_ptr_for_callback->mode == TMC2208_MODE_NSTEPS) {
            _motor_ptr_for_callback->nsteps--;
            if (_motor_ptr_for_callback->nsteps == 0) {
                // Si se han completado los pasos, detener el motor
                tmc2208_stop(_motor_ptr_for_callback);
                return false; // Detener el temporizador
            }
            else {
                // Invierte el pin STEP
                gpio_put(_motor_ptr_for_callback->step_pin, !gpio_get(_motor_ptr_for_callback->step_pin));
                return true; // Mantener el temporizador en ejecución
            }
        }
        else if (_motor_ptr_for_callback->mode == TMC2208_MODE_RUN_CW || _motor_ptr_for_callback->mode == TMC2208_MODE_RUN_CCW) {
            // Invierte el pin STEP
            gpio_put(_motor_ptr_for_callback->step_pin, !gpio_get(_motor_ptr_for_callback->step_pin));
            return true; // Mantener el temporizador en ejecución
        }
    }
    return false; // Detener si el puntero no es válido
}

void tmc2208_init(TMC2208_t *motor, uint8_t step_pin, uint8_t dir_pin, uint8_t enable_pin, uint16_t steps_per_rev, uint8_t microsteps, uint8_t ms1_pin, uint8_t ms2_pin) {
    // Guardar la configuración en la estructura
    motor->step_pin = step_pin;
    motor->dir_pin = dir_pin;
    motor->enable_pin = enable_pin;
    motor->ms1_pin = ms1_pin;
    motor->ms2_pin = ms2_pin;
    motor->steps_per_rev = steps_per_rev;
    motor->microsteps = microsteps;
    motor->alarm_id = -1; // Inicialmente no hay temporizador activo

    // Asignar el puntero para el callback
    _motor_ptr_for_callback = motor;

    // Inicializar pines GPIO
    gpio_init(motor->step_pin);
    gpio_set_dir(motor->step_pin, GPIO_OUT);
    gpio_init(motor->dir_pin);
    gpio_set_dir(motor->dir_pin, GPIO_OUT);
    gpio_init(motor->enable_pin);
    gpio_set_dir(motor->enable_pin, GPIO_OUT);

    // Por defecto, el driver está deshabilitado (ENA a nivel alto)
    tmc2208_enable(motor, false);
    motor->mode = TMC2208_MODE_STANDBY_HOLD; // Modo inicial

    add_repeating_timer_us(-10000, repeating_timer_callback, NULL, &motor->timer_to_steps);
}

void tmc2208_enable(TMC2208_t *motor, bool enable) {
    // El pin ENA es activo a nivel bajo
    gpio_put(motor->enable_pin, !enable);
    sleep_ms(2);
}

void tmc2208_set_microstepping_by_pins(TMC2208_t *motor, TMC2208_Microsteps_t microsteps) {
    if (!motor) return;

    // Inicializar pines como salida
    gpio_init(motor->ms1_pin);
    gpio_set_dir(motor->ms1_pin, GPIO_OUT);
    gpio_init(motor->ms2_pin);
    gpio_set_dir(motor->ms2_pin, GPIO_OUT);

    bool ms1 = 0, ms2 = 0;

    switch (microsteps) {
        case TMC2208_MICROSTEPS_1:
            ms1 = 0; ms2 = 0;
            motor->microsteps = 1;
            break;
        case TMC2208_MICROSTEPS_2:
            ms1 = 1; ms2 = 0;
            motor->microsteps = 2;
            break;
        case TMC2208_MICROSTEPS_4:
            ms1 = 0; ms2 = 1;
            motor->microsteps = 4;
            break;
        case TMC2208_MICROSTEPS_16:
            ms1 = 1; ms2 = 1;
            motor->microsteps = 16;
            break;
        default:
            printf("⚠️ Microstepping %d no soportado por pines. Usar UART.\n", microsteps);
            return;
    }

    gpio_put(motor->ms1_pin, ms1);
    gpio_put(motor->ms2_pin, ms2);

    printf("Microstepping configurado por pines: %d (MS1=%d, MS2=%d)\n",
           motor->microsteps, ms1, ms2);
}

void tmc2208_set_direction(TMC2208_t *motor, bool direction) {
    motor->direction = direction;
    gpio_put(motor->dir_pin, motor->direction);
}

void tmc2208_set_rpm(TMC2208_t *motor, float rpm) {
    // Cancelar cualquier temporizador anterior
    cancel_repeating_timer(&motor->timer_to_steps);

    // Si RPM es 0 o negativo, el motor se detiene
    if (rpm <= 0) {
        gpio_put(motor->step_pin, 0); // Asegurar que el pin STEP quede en bajo
        return;
    }

    // Calcular la frecuencia de pulso en Hz
    float pulse_freq = (rpm / 60.0f) * motor->steps_per_rev * motor->microsteps;

    // El temporizador debe alternar el pin, por lo que su frecuencia es el doble
    // El periodo en microsegundos es 1,000,000 / (2 * frecuencia)
    long delay_us = (long)(1000000.0f / (2.0f * pulse_freq));
    
    // Validar que el delay no sea demasiado corto para el sistema
    if (delay_us < 2) {
        // La frecuencia es demasiado alta, no se puede lograr con fiabilidad
        printf("Error: RPM muy altas, el delay del timer es demasiado corto.\n");
        return;
    }

    // Crear un nuevo temporizador repetitivo
    // Usamos un valor negativo para el delay para que llame al callback inmediatamente
    add_repeating_timer_us(-delay_us, repeating_timer_callback, NULL, &motor->timer_to_steps);
    if(motor->direction == true) {
        motor->mode = TMC2208_MODE_RUN_CW; // Modo avance
    } else {
        motor->mode = TMC2208_MODE_RUN_CCW; // Modo retroceso
    }

}

void tmc2208_stop(TMC2208_t *motor) {
    // Cancelar el temporizador repetitivo
    cancel_repeating_timer(&motor->timer_to_steps);
    // Asegurar que el pin STEP quede en bajo
    gpio_put(motor->step_pin, 0);
    // Cambiar el modo a STANDBY_HOLD
    motor->mode = TMC2208_MODE_STANDBY_HOLD;
}

void tmc2208_stop_and_disable(TMC2208_t *motor) {
    // Detener el motor
    tmc2208_stop(motor);
    // Deshabilitar el driver
    tmc2208_enable(motor, false);
    // Reiniciar el modo a STANDBY_HOLD
    motor->mode = TMC2208_MODE_STANDBY_HOLD;
}

void tmc2208_set_turns_at_rpm(TMC2208_t *motor, float rpm, float turns) {
    // Calcular el número total de micropasos para las vueltas deseadas
    uint64_t total_microsteps = (uint64_t)(turns * motor->steps_per_rev * motor->microsteps);

    // Llamar a la función para enviar un número específico de pasos a una frecuencia calculada
    // La frecuencia se calcula a partir de las RPM deseadas
    float freq = (rpm / 60.0f) * motor->steps_per_rev * motor->microsteps;
    tmc2208_send_nsteps_at_freq(motor, total_microsteps, freq);
}

void tmc2208_send_nsteps_at_freq(TMC2208_t *motor, int nsteps, float freq) {
    // Cancelar cualquier temporizador anterior
    cancel_repeating_timer(&motor->timer_to_steps);

    // Si nsteps es 0 o negativo, el motor se detiene
    if (nsteps <= 0 || freq <= 0) {
        gpio_put(motor->step_pin, 0); // Asegurar que el pin STEP quede en bajo
        return;
    }

    // Calcular la frecuencia de pulso en Hz
    float pulse_freq = freq;
    motor->nsteps = nsteps * 2; // Duplicar el número de pasos porque hay doble entrada al callback de timer por ciclo de step
    motor->mode = TMC2208_MODE_NSTEPS;

    // El temporizador debe alternar el pin, por lo que su frecuencia es el doble
    long delay_us = (long)(1000000.0f / (2.0f * pulse_freq));

    // Validar que el delay no sea demasiado corto para el sistema
    if (delay_us < 2) {
        printf("Error: Frecuencia muy alta, el delay del timer es demasiado corto.\n");
        return;
    }

    // Crear un nuevo temporizador repetitivo
    add_repeating_timer_us(-delay_us, repeating_timer_callback, NULL, &motor->timer_to_steps);
}