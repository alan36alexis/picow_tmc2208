#include "pico/stdlib.h"
#include "tmc2208/tmc2208.h"

// --- Configuracion ---
// Deficino de pines
#define MOTOR_STEP_PIN 14
#define MOTOR_DIR_PIN  15
#define MOTOR_ENA_PIN  16

// Parámetros de resolucion
#define MOTOR_STEPS_PER_REV 200
#define MOTOR_MICROSTEPS    32

int main() {
    stdio_init_all();

    TMC2208_t motor1;

    // Inicialización básica sin UART (para compatibilidad)
    tmc2208_init(&motor1, MOTOR_STEP_PIN, MOTOR_DIR_PIN, MOTOR_ENA_PIN, MOTOR_STEPS_PER_REV, MOTOR_MICROSTEPS, NULL, 0, 0, 0);
    sleep_ms(200);

    tmc2208_enable(&motor1, true);
    // tmc2208_set_direction(&motor1, true);
    // tmc2208_set_rpm(&motor1, 60);

    while (1) {

        //enviar 500 pasos a 1000 Hz y esperar 1 segundo
        //reaccion: durante 500 ms hay pulsos en step_pin y durante 500 ms no hay pulsos

        tmc2208_set_direction(&motor1, true);
        tmc2208_send_nsteps_at_freq(&motor1, 10, 20.0f); // Enviar 500 pasos a 1000 Hz

        sleep_ms(1000);
    }

    return 0;
}
