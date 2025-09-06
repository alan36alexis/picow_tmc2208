#include "tmc2208.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include <stdio.h>
#include <string.h>

// Puntero global para ser usado por el callback del timer.
// Limitación: Este diseño simple solo permite controlar un motor.
// Para múltiples motores, se requeriría un diseño más avanzado (ej. un array de punteros).
static TMC2208_t *_motor_ptr_for_callback;

// Tabla de conversión: índice = valor MRES, contenido = microsteps reales
static const uint16_t microstep_table[9] = {
    256, 128, 64, 32, 16, 8, 4, 2, 1
};

/**
 * @brief Callback del temporizador. Se ejecuta a intervalos regulares para generar los pulsos STEP.
 * Simplemente invierte el estado del pin STEP.
 */
bool repeating_timer_callback(struct repeating_timer *t) {
    if (_motor_ptr_for_callback) {
        if(_motor_ptr_for_callback->mode == TMC2208_MODE_NSTEPS) {
            _motor_ptr_for_callback->nsteps--;
            if (_motor_ptr_for_callback->nsteps == 0) {
                printf("Mode: %d\n", _motor_ptr_for_callback->mode);
                // Si se han completado los pasos, detener el motor
                tmc2208_stop(_motor_ptr_for_callback);
                return false; // Detener el temporizador
            }
            else {
                // Invierte el pin STEP
                gpio_put(_motor_ptr_for_callback->step_pin, !gpio_get(_motor_ptr_for_callback->step_pin));
                //printf("Mode: %d\n", _motor_ptr_for_callback->mode);
                return true; // Mantener el temporizador en ejecución
            }
        }
        else if (_motor_ptr_for_callback->mode == TMC2208_MODE_RUN_CW || _motor_ptr_for_callback->mode == TMC2208_MODE_RUN_CCW) {
            // Invierte el pin STEP
            //printf("Mode: %d\n", _motor_ptr_for_callback->mode);
            gpio_put(_motor_ptr_for_callback->step_pin, !gpio_get(_motor_ptr_for_callback->step_pin));
            
            return true; // Mantener el temporizador en ejecución
        }
    }
    return false; // Detener si el puntero no es válido
}

void tmc2208_init(TMC2208_t *motor, uint8_t step_pin, uint8_t dir_pin, uint8_t enable_pin, uint16_t steps_per_rev, uint16_t microsteps, uart_inst_t *uart_instance, uint8_t uart_tx_pin, uint8_t uart_rx_pin, uint8_t uart_address) {
    // Guardar la configuración en la estructura
    motor->step_pin = step_pin;
    motor->dir_pin = dir_pin;
    motor->enable_pin = enable_pin;
    motor->steps_per_rev = steps_per_rev;
    motor->microsteps = microsteps;
    motor->alarm_id = -1; // Inicialmente no hay temporizador activo

    // Configuración UART
    motor->uart_instance = uart_instance;
    motor->uart_tx_pin = uart_tx_pin;
    motor->uart_rx_pin = uart_rx_pin;
    motor->uart_address = uart_address;

    // Asignar el puntero para el callback
    _motor_ptr_for_callback = motor;

    // Inicializar pines GPIO
    gpio_init(motor->step_pin);
    gpio_set_dir(motor->step_pin, GPIO_OUT);
    gpio_init(motor->dir_pin);
    gpio_set_dir(motor->dir_pin, GPIO_OUT);
    gpio_init(motor->enable_pin);
    gpio_set_dir(motor->enable_pin, GPIO_OUT);

    // Inicializar UART si se proporcionó una instancia válida
    if (motor->uart_instance != NULL) {
        tmc2208_uart_init(motor);
    }

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

// Funciones UART para configuración avanzada

/**
 * @brief Calcula el CRC8 para comunicación UART con TMC2208
 */
static uint8_t tmc2208_calculate_crc8(uint8_t *data, uint8_t length) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < length; i++) {
        uint8_t byte = data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if ((crc ^ byte) & 0x01) {
                crc = (crc >> 1) ^ 0x8C;
            } else {
                crc = crc >> 1;
            }
            byte = byte >> 1;
        }
    }
    return crc;
}

/**
 * @brief Construye un paquete UART para comunicación con TMC2208
 */
static void tmc2208_build_uart_packet(uint8_t address, uint8_t register_addr, uint32_t data, bool read, uint8_t *packet) {
    packet[0] = 0x05; // SYNC byte
    packet[1] = (address & 0x03) | (read ? 0x80 : 0x00); // Address + R/W bit
    packet[2] = register_addr; // Register address
    packet[3] = (data >> 24) & 0xFF; // Data MSB
    packet[4] = (data >> 16) & 0xFF;
    packet[5] = (data >> 8) & 0xFF;
    packet[6] = data & 0xFF; // Data LSB
    packet[7] = tmc2208_calculate_crc8(packet, 7); // CRC
}

bool tmc2208_uart_init(TMC2208_t *motor) {
    if (motor->uart_instance == NULL) {
        return false;
    }

    // Configurar pines UART
    gpio_set_function(motor->uart_tx_pin, GPIO_FUNC_UART);
    gpio_set_function(motor->uart_rx_pin, GPIO_FUNC_UART);

    // Inicializar UART
    uart_init(motor->uart_instance, TMC2208_UART_BAUD_RATE);
    uart_set_hw_flow(motor->uart_instance, false, false);
    uart_set_format(motor->uart_instance, 8, 1, UART_PARITY_NONE);

    // Pequeña pausa para estabilización
    sleep_ms(10);

    return true;
}

bool tmc2208_uart_read_register(TMC2208_t *motor, uint8_t register_address, uint32_t *data) {
    if (motor->uart_instance == NULL || data == NULL) {
        return false;
    }

    uint8_t packet[8];
    uint8_t response[8];
    
    // Construir paquete de lectura
    tmc2208_build_uart_packet(motor->uart_address, register_address, 0, true, packet);
    
    // Enviar paquete
    for (int i = 0; i < 8; i++) {
        uart_putc_raw(motor->uart_instance, packet[i]);
    }
    
    // Esperar respuesta (timeout de 100ms)
    uint32_t timeout = 100000; // 100ms en microsegundos
    uint32_t start_time = time_us_32();
    uint8_t bytes_received = 0;
    
    while (bytes_received < 8 && (time_us_32() - start_time) < timeout) {
        if (uart_is_readable(motor->uart_instance)) {
            response[bytes_received] = uart_getc(motor->uart_instance);
            bytes_received++;
        }
    }
    
    if (bytes_received != 8) {
        return false; // Timeout o datos incompletos
    }
    
    // Verificar CRC
    uint8_t calculated_crc = tmc2208_calculate_crc8(response, 7);
    if (calculated_crc != response[7]) {
        return false; // CRC incorrecto
    }
    
    // Extraer datos
    *data = ((uint32_t)response[3] << 24) | 
            ((uint32_t)response[4] << 16) | 
            ((uint32_t)response[5] << 8) | 
            response[6];
    
    return true;
}

bool tmc2208_uart_write_register(TMC2208_t *motor, uint8_t register_address, uint32_t data) {
    if (motor->uart_instance == NULL) {
        return false;
    }

    uint8_t packet[8];
    
    // Construir paquete de escritura
    tmc2208_build_uart_packet(motor->uart_address, register_address, data, false, packet);
    
    // Enviar paquete completo
    for (int i = 0; i < 8; i++) {
        uart_putc_raw(motor->uart_instance, packet[i]);
    }

    // Asegurar vaciado del buffer TX
    uart_tx_wait_blocking(motor->uart_instance);

    return true;
}

bool tmc2208_set_microstepping(TMC2208_t *motor, TMC2208_Microsteps_t microsteps) {
    if (motor->uart_instance == NULL) {
        return false;
    }

    uint32_t chopconf;
    
    // Leer registro CHOPCONF actual
    if (!tmc2208_uart_read_register(motor, TMC2208_REG_CHOPCONF, &chopconf)) {
        return false;
    }
    
    // Limpiar bits de microstepping y establecer nuevo valor
    chopconf &= ~TMC2208_CHOPCONF_MRES_MASK;
    chopconf |= (microsteps & TMC2208_CHOPCONF_MRES_MASK);
    // chopconf &= ~TMC2208_CHOPCONF_MRES_MASK;
    // chopconf |= (microsteps << 24);

    printf("CHOPCONF modificado: 0x%08X\n", chopconf);
    
    
    // Escribir registro modificado
    if (!tmc2208_uart_write_register(motor, TMC2208_REG_CHOPCONF, chopconf)) {
        return false;
    }
    
    // Guardar valor real en la estructura
    if (microsteps <= 8) {
        motor->microsteps = microstep_table[microsteps];
    } else {
        motor->microsteps = 256; // fallback
    }
    
    return true;
}

uint16_t tmc2208_get_microstepping(TMC2208_t *motor) {
    if (motor->uart_instance == NULL) {
        return 0; // error
    }

    uint32_t chopconf;

    // Leer el registro CHOPCONF
    if (!tmc2208_uart_read_register(motor, TMC2208_REG_CHOPCONF, &chopconf)) {
        return 0; // error
    }

    // Extraer el campo MRES (bits 24..27)
    uint8_t mres = (chopconf >> 24) & 0x0F;

    // Validar rango y devolver microsteps reales
    if (mres <= 8) {
        return microstep_table[mres];
    }

    return 0; // valor inválido
}

bool tmc2208_set_currents(TMC2208_t *motor, uint8_t irun, uint8_t ihold, uint8_t ihold_delay) {
    if (motor->uart_instance == NULL) {
        return false;
    }

    // Validar parámetros
    if (irun > 31 || ihold > 31 || ihold_delay > 15) {
        return false;
    }
    
    // Construir valor del registro IHOLD_IRUN
    uint32_t ihold_irun = (ihold_delay << TMC2208_IHOLD_IRUN_IHOLDDELAY_SHIFT) |
                          (irun << TMC2208_IHOLD_IRUN_IRUN_SHIFT) |
                          (ihold << TMC2208_IHOLD_IRUN_IHOLD_SHIFT);
    
    // Escribir registro
    return tmc2208_uart_write_register(motor, TMC2208_REG_IHOLD_IRUN, ihold_irun);
}

bool tmc2208_set_tpowerdown(TMC2208_t *motor, uint8_t tpowerdown) {
    if (motor->uart_instance == NULL) {
        return false;
    }

    // Escribir registro TPOWERDOWN
    return tmc2208_uart_write_register(motor, TMC2208_REG_TPOWERDOWN, tpowerdown);
}

bool tmc2208_uart_enable(TMC2208_t *motor, bool enable) {
    if (motor->uart_instance == NULL) {
        return false;
    }

    uint32_t gconf;
    
    // Leer registro GCONF actual
    if (!tmc2208_uart_read_register(motor, TMC2208_REG_GCONF, &gconf)) {
        return false;
    }
    
    // Modificar bit de habilitación (bit 0)
    if (enable) {
        gconf |= 0x01; // Habilitar driver
    } else {
        gconf &= ~0x01; // Deshabilitar driver
    }
    
    // Escribir registro modificado
    return tmc2208_uart_write_register(motor, TMC2208_REG_GCONF, gconf);
}

bool tmc2208_get_status(TMC2208_t *motor, uint32_t *gstat) {
    if (motor->uart_instance == NULL || gstat == NULL) {
        return false;
    }

    return tmc2208_uart_read_register(motor, TMC2208_REG_GSTAT, gstat);
}

bool tmc2208_configure_all(TMC2208_t *motor, TMC2208_Microsteps_t microsteps, uint8_t irun, uint8_t ihold, uint8_t ihold_delay, uint8_t tpowerdown) {
    if (motor->uart_instance == NULL) {
        return false;
    }

    bool success = true;
    
    // Configurar microstepping
    success &= tmc2208_set_microstepping(motor, microsteps);
    
    // Configurar corrientes
    success &= tmc2208_set_currents(motor, irun, ihold, ihold_delay);
    
    // Configurar tiempo de transición
    success &= tmc2208_set_tpowerdown(motor, tpowerdown);
    
    return success;
}

int tmc2208_read_chopconf(TMC2208_t *motor, uint32_t *chopconf)
{
    if (!motor || !chopconf) {
        return -1; // error de punteros
    }

    uint32_t val = 0;
    int status = tmc2208_uart_read_register(motor, TMC2208_REG_CHOPCONF, &val);
    if (status < 0) {
        return status; // error de lectura
    }

    *chopconf = val;
    printf("CHOPCONF leido: 0x%08X\n", val);

    return 0;
}

void tmc2208_debug_chopconf(TMC2208_t *motor)
{
    uint32_t chopconf;
    if (!tmc2208_uart_read_register(motor, TMC2208_REG_CHOPCONF, &chopconf)) {
        printf("Error leyendo CHOPCONF\n");
        return;
    }

    printf("CHOPCONF = 0x%08X\n", chopconf);

    uint8_t mres   = (chopconf >> 24) & 0x0F;
    uint8_t toff   = (chopconf >> 0) & 0x0F;
    uint8_t hstrt  = (chopconf >> 4) & 0x07;
    uint8_t hend   = (chopconf >> 7) & 0x0F;
    uint8_t intpol = (chopconf >> 28) & 0x01;
    uint8_t dedge  = (chopconf >> 29) & 0x01;

    int microsteps = 256 >> mres;

    printf("  Microstepping: %d (MRES=%d)\n", microsteps, mres);
    printf("  TOFF        : %d\n", toff);
    printf("  HSTRT       : %d\n", hstrt);
    printf("  HEND        : %d\n", hend);
    printf("  INTPOL      : %s\n", intpol ? "ON" : "OFF");
    printf("  DEDGE       : %s\n", dedge ? "ON" : "OFF");
}

bool tmc2208_set_chopconf(TMC2208_t *motor,
                          TMC2208_Microsteps_t microsteps,
                          uint8_t toff,
                          uint8_t hstrt,
                          uint8_t hend,
                          bool intpol,
                          bool dedge)
{
    if (motor->uart_instance == NULL) {
        return false;
    }

    // Codificar MRES (bits 27..24), se guarda como log2(256/microsteps)
    uint8_t mres = 0;
    switch (microsteps) {
        case TMC2208_MICROSTEPS_1:   
            mres = 8; 
            break;
        case TMC2208_MICROSTEPS_2:   
            mres = 7; 
            break;
        case TMC2208_MICROSTEPS_4:   
            mres = 6; 
            break;
        case TMC2208_MICROSTEPS_8:   
            mres = 5; 
            break;
        case TMC2208_MICROSTEPS_16:  
            mres = 4; 
            break;
        case TMC2208_MICROSTEPS_32:  
            mres = 3; 
            break;
        case TMC2208_MICROSTEPS_64:  
            mres = 2; 
            break;
        case TMC2208_MICROSTEPS_128: 
            mres = 1; 
            break;
        case TMC2208_MICROSTEPS_256: 
            mres = 0; 
            break;
        default: 
            mres = 0; 
            break;
    }

    uint32_t chopconf = 0;
    chopconf |= (toff   & 0x0F);          // TOFF[3:0]
    chopconf |= (hstrt  & 0x07) << 4;     // HSTRT[2:0]
    chopconf |= (hend   & 0x0F) << 7;     // HEND[3:0]
    chopconf |= (mres   & 0x0F) << 24;    // MRES[3:0]
    chopconf |= (intpol ? 1U : 0U) << 28; // INTPOL
    chopconf |= (dedge  ? 1U : 0U) << 29; // DEDGE

    // Escribir al TMC2209
    if (!tmc2208_uart_write_register(motor, TMC2208_REG_CHOPCONF, chopconf)) {
        return false;
    }

    // Guardar en la estructura interna
    motor->microsteps = (1 << microsteps);

    printf("CHOPCONF seteado: 0x%08X\n", chopconf);

    return true;
}

bool tmc2208_disable_pdn_uart(TMC2208_t *motor) {
    if (motor->uart_instance == NULL) {
        return false;
    }

    uint32_t gconf = 0;

    // Leer el GCONF actual
    if (!tmc2208_uart_read_register(motor, TMC2208_REG_GCONF, &gconf)) {
        return false;
    }

    // Limpiar el bit pdn_disable (bit 6)
    gconf &= ~(1 << 6);

    // Escribir el nuevo valor
    if (!tmc2208_uart_write_register(motor, TMC2208_REG_GCONF, gconf)) {
        return false;
    }

    printf("PDN_UART deshabilitado. GCONF=0x%08X\n", gconf);
    return true;
}
