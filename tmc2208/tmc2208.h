#ifndef TMC2208_H
#define TMC2208_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/timer.h"
#include "hardware/uart.h"
#include "pico/time.h"

// Constantes para comunicación UART TMC2208
#define TMC2208_UART_BAUD_RATE 115200
#define TMC2208_DEFAULT_ADDRESS 0b00

// Registros TMC2208 para configuración UART
#define TMC2208_REG_GCONF        0x00
#define TMC2208_REG_GSTAT        0x01
#define TMC2208_REG_IFCNT        0x02
#define TMC2208_REG_SLAVECONF    0x03
#define TMC2208_REG_OTP_PROG     0x04
#define TMC2208_REG_OTP_READ     0x05
#define TMC2208_REG_IOIN         0x06
#define TMC2208_REG_FACTORY_CONF 0x07
#define TMC2208_REG_IHOLD_IRUN   0x10
#define TMC2208_REG_TPOWERDOWN   0x11
#define TMC2208_REG_TSTEP        0x12
#define TMC2208_REG_TPWMTHRS     0x13
#define TMC2208_REG_TCOOLTHRS    0x14
#define TMC2208_REG_THIGH        0x15
#define TMC2208_REG_XDIRECT      0x2D
#define TMC2208_REG_VDCMIN       0x33
#define TMC2208_REG_MSLUT0       0x60
#define TMC2208_REG_MSLUT1       0x61
#define TMC2208_REG_MSLUT2       0x62
#define TMC2208_REG_MSLUT3       0x63
#define TMC2208_REG_MSLUT4       0x64
#define TMC2208_REG_MSLUT5       0x65
#define TMC2208_REG_MSLUT6       0x66
#define TMC2208_REG_MSLUT7       0x67
#define TMC2208_REG_MSLUTSEL     0x68
#define TMC2208_REG_MSLUTSTART   0x69
#define TMC2208_REG_MSCNT        0x6A
#define TMC2208_REG_MSCURACT     0x6B
#define TMC2208_REG_CHOPCONF     0x6C
#define TMC2208_REG_COOLCONF     0x6D
#define TMC2208_REG_DCCTRL       0x6E
#define TMC2208_REG_DRV_STATUS   0x6F
#define TMC2208_REG_PWMCONF      0x70
#define TMC2208_REG_PWMSCALE     0x71
#define TMC2208_REG_PWM_AUTO     0x72

// Máscaras para registros específicos
#define TMC2208_CHOPCONF_MRES_MASK    0x0F
#define TMC2208_CHOPCONF_INTPOL_MASK  0x10
#define TMC2208_CHOPCONF_DEDGE_MASK   0x20
#define TMC2208_CHOPCONF_MRES_SHIFT   0
#define TMC2208_CHOPCONF_INTPOL_SHIFT 4
#define TMC2208_CHOPCONF_DEDGE_SHIFT  5

#define TMC2208_IHOLD_IRUN_IHOLD_MASK 0x1F
#define TMC2208_IHOLD_IRUN_IRUN_MASK  0x1F00
#define TMC2208_IHOLD_IRUN_IHOLDDELAY_MASK 0x0F0000
#define TMC2208_IHOLD_IRUN_IHOLD_SHIFT 0
#define TMC2208_IHOLD_IRUN_IRUN_SHIFT 8
#define TMC2208_IHOLD_IRUN_IHOLDDELAY_SHIFT 16

// Valores de microstepping
typedef enum {
    TMC2208_MICROSTEPS_1 = 0,
    TMC2208_MICROSTEPS_2 = 1,
    TMC2208_MICROSTEPS_4 = 2,
    TMC2208_MICROSTEPS_8 = 3,
    TMC2208_MICROSTEPS_16 = 4,
    TMC2208_MICROSTEPS_32 = 5,
    TMC2208_MICROSTEPS_64 = 6,
    TMC2208_MICROSTEPS_128 = 7,
    TMC2208_MICROSTEPS_256 = 8
} TMC2208_Microsteps_t;

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

    // Configuración UART
    uart_inst_t *uart_instance;
    uint8_t uart_tx_pin;
    uint8_t uart_rx_pin;
    uint8_t uart_address;

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
 * @param uart_instance Instancia UART a utilizar (uart0 o uart1).
 * @param uart_tx_pin Pin GPIO para TX UART.
 * @param uart_rx_pin Pin GPIO para RX UART.
 * @param uart_address Dirección UART del driver (0-3).
 */
void tmc2208_init(TMC2208_t *motor, uint8_t step_pin, uint8_t dir_pin, uint8_t enable_pin, uint16_t steps_per_rev, uint8_t microsteps, uart_inst_t *uart_instance, uint8_t uart_tx_pin, uint8_t uart_rx_pin, uint8_t uart_address);

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
 * @brief Gira el motor un número específico de vueltas a una velocidad dada en RPM.
 *
 * @param motor Puntero a la estructura del motor.
 * @param rpm Velocidad deseada en Revoluciones Por Minuto.
 * @param turns Número de vueltas a girar.
 */
void tmc2208_set_turns_at_rpm(TMC2208_t *motor, float rpm, float turns);

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

// Funciones UART para configuración avanzada

/**
 * @brief Inicializa la comunicación UART con el driver TMC2208.
 *
 * @param motor Puntero a la estructura del motor.
 * @return true si la inicialización fue exitosa, false en caso contrario.
 */
bool tmc2208_uart_init(TMC2208_t *motor);

/**
 * @brief Lee un registro del driver TMC2208 a través de UART.
 *
 * @param motor Puntero a la estructura del motor.
 * @param register_address Dirección del registro a leer.
 * @param data Puntero donde se almacenará el valor leído.
 * @return true si la lectura fue exitosa, false en caso contrario.
 */
bool tmc2208_uart_read_register(TMC2208_t *motor, uint8_t register_address, uint32_t *data);

/**
 * @brief Escribe un valor en un registro del driver TMC2208 a través de UART.
 *
 * @param motor Puntero a la estructura del motor.
 * @param register_address Dirección del registro a escribir.
 * @param data Valor a escribir en el registro.
 * @return true si la escritura fue exitosa, false en caso contrario.
 */
bool tmc2208_uart_write_register(TMC2208_t *motor, uint8_t register_address, uint32_t data);

/**
 * @brief Configura el microstepping del driver TMC2208.
 *
 * @param motor Puntero a la estructura del motor.
 * @param microsteps Valor de microstepping deseado.
 * @return true si la configuración fue exitosa, false en caso contrario.
 */
bool tmc2208_set_microstepping(TMC2208_t *motor, TMC2208_Microsteps_t microsteps);

/**
 * @brief Configura las corrientes de operación (Irun) y reposo (Ihold) del driver.
 *
 * @param motor Puntero a la estructura del motor.
 * @param irun Corriente de operación (0-31, escala interna del driver).
 * @param ihold Corriente de reposo (0-31, escala interna del driver).
 * @param ihold_delay Delay antes de reducir a corriente de reposo (0-15).
 * @return true si la configuración fue exitosa, false en caso contrario.
 */
bool tmc2208_set_currents(TMC2208_t *motor, uint8_t irun, uint8_t ihold, uint8_t ihold_delay);

/**
 * @brief Configura el tiempo de transición antes de reducir la corriente en reposo.
 *
 * @param motor Puntero a la estructura del motor.
 * @param tpowerdown Tiempo en unidades de 2^18 clk cycles (aproximadamente 0-512ms).
 * @return true si la configuración fue exitosa, false en caso contrario.
 */
bool tmc2208_set_tpowerdown(TMC2208_t *motor, uint8_t tpowerdown);

/**
 * @brief Habilita o deshabilita el driver a través del registro GCONF.
 *
 * @param motor Puntero a la estructura del motor.
 * @param enable true para habilitar, false para deshabilitar.
 * @return true si la configuración fue exitosa, false en caso contrario.
 */
bool tmc2208_uart_enable(TMC2208_t *motor, bool enable);

/**
 * @brief Lee el estado del driver desde el registro GSTAT.
 *
 * @param motor Puntero a la estructura del motor.
 * @param gstat Puntero donde se almacenará el estado.
 * @return true si la lectura fue exitosa, false en caso contrario.
 */
bool tmc2208_get_status(TMC2208_t *motor, uint32_t *gstat);

/**
 * @brief Configura múltiples parámetros del driver en una sola operación.
 *
 * @param motor Puntero a la estructura del motor.
 * @param microsteps Valor de microstepping.
 * @param irun Corriente de operación.
 * @param ihold Corriente de reposo.
 * @param ihold_delay Delay para corriente de reposo.
 * @param tpowerdown Tiempo de transición.
 * @return true si la configuración fue exitosa, false en caso contrario.
 */
bool tmc2208_configure_all(TMC2208_t *motor, TMC2208_Microsteps_t microsteps, uint8_t irun, uint8_t ihold, uint8_t ihold_delay, uint8_t tpowerdown);

#endif // TMC2208_H