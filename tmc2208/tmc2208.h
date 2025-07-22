#ifndef TMC2208_H
#define TMC2208_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/timer.h"
#include "pico/time.h"

typedef enum {
    TMC2208_MODE_STANDBY_FREE = 0,  // Motor en reposo, ENA deshabilitado
    TMC2208_MODE_STANDBY_HOLD = 1,  // Motor en reposo, ENA habilitado
    // TODO: Cambiar RUN_CW y RUN_CCW por nombres más descriptivos, como AVANCE y RETROCESO
    TMC2208_MODE_RUN_CW = 2, // Motor en movimiento sentido horario
    TMC2208_MODE_RUN_CCW = 3, // Motor en movimiento sentido antihorario
    TMC2208_MODE_NSTEPS = 4,    // Motor en movimiento por un número específico de pasos
    TMC2208_MODE_ALARM = 5  // Modo de alarma, motor detenido
} TMC2208_Mode_t;

// Estructura para contener la configuración del motor y sus pines
typedef struct {
    // Pines GPIO
    uint8_t step_pin;
    uint8_t dir_pin;
    uint8_t enable_pin;

    // Parámetros del motor
    uint16_t steps_per_rev;
    uint8_t microsteps;

    // Estado interno
    TMC2208_Mode_t mode; // Modo actual del driver
    int64_t alarm_id; // ID del temporizador repetitivo
    bool direction;   // true: adelante, false: atrás
    struct repeating_timer timer_to_steps; // Temporizador para pasos
    uint64_t nsteps; // Número de pasos restantes para el callback 
} TMC2208_t;

/**
 * @brief Inicializa el driver TMC2208 y los pines GPIO correspondientes.
 *
 * @param motor Puntero a la estructura de configuración del motor.
 * @param step_pin Pin GPIO para STEP.
 * @param dir_pin Pin GPIO para DIR.
 * @param enable_pin Pin GPIO para ENA.
 * @param steps_per_rev Pasos completos por revolución del motor (ej: 200).
 * @param microsteps Micropasos configurados en el driver (ej: 16).
 */
void tmc2208_init(TMC2208_t *motor, uint8_t step_pin, uint8_t dir_pin, uint8_t enable_pin, uint16_t steps_per_rev, uint8_t microsteps);

/**
 * @brief Habilita o deshabilita el driver del motor.
 *
 * @param motor Puntero a la estructura del motor.
 * @param enable true para habilitar (energizar), false para deshabilitar.
 */
void tmc2208_enable(TMC2208_t *motor, bool enable);

/**
 * @brief Establece la dirección de rotación.
 *
 * @param motor Puntero a la estructura del motor.
 * @param direction true para una dirección, false para la opuesta.
 */
void tmc2208_set_direction(TMC2208_t *motor, bool direction);

/**
 * @brief Establece la velocidad de rotación del motor en RPM.
 * Si RPM es 0, el motor se detiene.
 *
 * @param motor Puntero a la estructura del motor.
 * @param rpm Velocidad deseada en Revoluciones Por Minuto.
 */
void tmc2208_set_rpm(TMC2208_t *motor, float rpm);

/**
 * @brief Detiene el motor, cancela el timer y pone en LOW el pin de step.
 *
 * @param motor Puntero a la estructura del motor.
 */
void tmc2208_stop(TMC2208_t *motor);

/**
 * @brief Detiene el motor, cancela el timer, pone en LOW el pin de step y deshabilita el driver.
 *
 * @param motor Puntero a la estructura del motor.
 */
void tmc2208_stop_and_disable(TMC2208_t *motor);

/**
 * @brief Establece el número de pasos a enviar al motor a una frecuencia dada.
 * Esta función es útil para pruebas o movimientos específicos.
 *
 * @param motor Puntero a la estructura del motor.
 * @param nsteps Número de pasos a enviar.
 * @param freq Frecuencia en Hz a la que se enviarán los pasos.
 */
void tmc2208_send_nsteps_at_freq(TMC2208_t *motor, int nsteps, float freq);

#endif // TMC2208_H