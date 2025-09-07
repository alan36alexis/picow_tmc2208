#include "pico/stdlib.h"
#include "tmc2208/tmc2208.h"

// --- Configuracion ---
// Deficino de pines
#define MOTOR_STEP_PIN 3
#define MOTOR_DIR_PIN  2
#define MOTOR_ENA_PIN  8
#define MOTOR_MS1_PIN  7
#define MOTOR_MS2_PIN  6

// Parámetros de resolucion
#define MOTOR_STEPS_PER_REV 200
#define MOTOR_MICROSTEPS    TMC2208_MICROSTEPS_16

int main() {
    stdio_init_all();

    TMC2208_t motor1;

    tmc2208_init(&motor1, MOTOR_STEP_PIN, MOTOR_DIR_PIN, MOTOR_ENA_PIN, MOTOR_STEPS_PER_REV, MOTOR_MICROSTEPS, MOTOR_MS1_PIN, MOTOR_MS2_PIN);
    sleep_ms(200);

    tmc2208_enable(&motor1, true);
    tmc2208_set_microstepping_by_pins(&motor1, MOTOR_MICROSTEPS);
    // tmc2208_set_direction(&motor1, true);
    // tmc2208_set_rpm(&motor1, 60);

    //tmc2208_send_nsteps_at_freq(&motor1, 3200, 1600.0f);
    while (1) {

        //enviar 500 pasos a 1000 Hz y esperar 1 segundo
        //reaccion: durante 500 ms hay pulsos en step_pin y durante 500 ms no hay pulsos

        tmc2208_enable(&motor1, true);
        tmc2208_set_direction(&motor1, true);
        tmc2208_send_nsteps_at_freq(&motor1, 3200, 1600.0f);
        sleep_ms(2000);
        tmc2208_enable(&motor1, false);
        sleep_ms(1000);

        tmc2208_enable(&motor1, true);
        tmc2208_set_direction(&motor1, false);
        tmc2208_send_nsteps_at_freq(&motor1, 3200, 1600.0f);
        sleep_ms(2000);
        tmc2208_enable(&motor1, false); 
        sleep_ms(1000);
    }

    return 0;
}
