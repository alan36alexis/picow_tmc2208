#include "pico/stdlib.h"
#include "tmc2208/tmc2208.h"
#include <stdio.h>

// Ejemplo de configuración avanzada del TMC2208 via UART
// Este archivo demuestra cómo usar las funciones UART para configurar
// múltiples parámetros del driver TMC2208

#define MOTOR_STEP_PIN 3
#define MOTOR_DIR_PIN  2
#define MOTOR_ENA_PIN  8
#define UART_TX_PIN 4
#define UART_RX_PIN 5
#define TMC2208_UART_ADDRESS 0
#define UART_PORT uart1

#define microsteps 256

int main() {
    stdio_init_all();
    
    TMC2208_t motor;
    
    // Inicializar motor con UART
    tmc2208_init(&motor, MOTOR_STEP_PIN, MOTOR_DIR_PIN, MOTOR_ENA_PIN, 
                 200, microsteps, UART_PORT, UART_TX_PIN, UART_RX_PIN, TMC2208_UART_ADDRESS);
    
    sleep_ms(200);
    
    printf("=== Configuración Avanzada TMC2208 ===\n");

    tmc2208_uart_enable(&motor, false);
    tmc2208_set_microstepping_by_pins(&motor, TMC2208_MICROSTEPS_32);
    
    // // Método 1: Configurar parámetros individualmente
    // printf("\n1. Configuración individual:\n");
    
    // if (tmc2208_set_microstepping(&motor, TMC2208_MICROSTEPS_256)) {
    //     printf("✓ Microstepping: 256 micropasos\n");
    // } else {
    //     printf("✗ Error configurando microstepping\n");
    // }
    
    // if (tmc2208_set_currents(&motor, 25, 15, 8)) {
    //     printf("✓ Corrientes: Irun=25, Ihold=15, delay=8\n");
    // } else {
    //     printf("✗ Error configurando corrientes\n");
    // }
    
    // if (tmc2208_set_tpowerdown(&motor, 30)) {
    //     printf("✓ Tiempo de transición: 30\n");
    // } else {
    //     printf("✗ Error configurando tiempo de transición\n");
    // }

    // // Configuración típica: 16 microsteps, interpolación activada, toff=3, hstrt=4, hend=2
    // tmc2208_set_chopconf(&motor,
    //                     TMC2208_MICROSTEPS_16,
    //                     3,  // TOFF
    //                     4,  // HSTRT
    //                     2,  // HEND
    //                     true,  // INTPOL
    //                     false  // DEDGE
    // );

    
    // // Método 2: Configurar múltiples parámetros de una vez
    // // printf("\n2. Configuración múltiple:\n");
    
    // // if (tmc2208_configure_all(&motor, TMC2208_MICROSTEPS_256, 30, 20, 10, 40)) {
    // //     printf("✓ Configuración múltiple exitosa:\n");
    // //     printf("  - Microstepping: 64 micropasos\n");
    // //     printf("  - Corrientes: Irun=30, Ihold=20, delay=10\n");
    // //     printf("  - Tiempo de transición: 40\n");
    // // } else {
    // //     printf("✗ Error en configuración múltiple\n");
    // // }
    
    // // Leer y mostrar estado del driver
    // printf("\n3. Estado del driver:\n");
    // uint32_t status;
    // if (tmc2208_get_status(&motor, &status)) {
    //     printf("Estado GSTAT: 0x%08X\n", status);
        
    //     // Interpretar bits de estado
    //     if (status & 0x01) printf("  - Reset detectado\n");
    //     if (status & 0x02) printf("  - Error de comunicación\n");
    //     if (status & 0x04) printf("  - Error de voltaje\n");
    //     if (status & 0x08) printf("  - Error de temperatura\n");
    //     if (status == 0) printf("  - Sin errores detectados\n");
    // } else {
    //     printf("✗ Error leyendo estado del driver\n");
    // }

    // uint32_t chopconf_val;
    // if (tmc2208_read_chopconf(&motor, &chopconf_val) == 0) {
    //     printf("Microstepping actual = %d\n", tmc2208_get_microstepping(&motor));
    // }
    // tmc2208_debug_chopconf(&motor);



    // Habilitar motor y realizar movimiento de prueba
    printf("\n4. Prueba de movimiento:\n");
    tmc2208_enable(&motor, true);
    printf("Motor habilitado\n");
    
    for (int i = 0; i < 3; i++) {
        printf("Movimiento %d/3\n", i + 1);
        
        // Movimiento hacia adelante
        tmc2208_set_direction(&motor, true);
        //tmc2208_send_nsteps_at_freq(&motor, steps_per_rev, 500.0f);
        //tmc2208_send_nsteps_at_freq(&motor, 1000, 500.0f);
        tmc2208_set_turns_at_rpm(&motor, 60.0f, 2.0f); // 2 vueltas a 60 RPM
        sleep_ms(10000);
        
        // Movimiento hacia atrás
        tmc2208_set_direction(&motor, false);
        tmc2208_send_nsteps_at_freq(&motor, 1000, 500.0f);
        sleep_ms(10000);
    }
    
    printf("\n=== Configuración completada ===\n");
    
    return 0;
}



// #include "pico/stdlib.h"
// #include "tmc2208/tmc2208.h"

// // --- Configuracion ---
// // Deficino de pines
// #define MOTOR_STEP_PIN 14
// #define MOTOR_DIR_PIN  15
// #define MOTOR_ENA_PIN  16

// // Parámetros de resolucion
// #define MOTOR_STEPS_PER_REV 200
// #define MOTOR_MICROSTEPS    32

// int main() {
//     stdio_init_all();

//     TMC2208_t motor1;

//     // Inicialización básica sin UART (para compatibilidad)
//     tmc2208_init(&motor1, MOTOR_STEP_PIN, MOTOR_DIR_PIN, MOTOR_ENA_PIN, MOTOR_STEPS_PER_REV, MOTOR_MICROSTEPS, NULL, 0, 0, 0);
//     sleep_ms(200);

//     tmc2208_enable(&motor1, true);
//     // tmc2208_set_direction(&motor1, true);
//     // tmc2208_set_rpm(&motor1, 60);

//     while (1) {

//         //enviar 500 pasos a 1000 Hz y esperar 1 segundo
//         //reaccion: durante 500 ms hay pulsos en step_pin y durante 500 ms no hay pulsos

//         tmc2208_set_direction(&motor1, true);
//         tmc2208_send_nsteps_at_freq(&motor1, 10, 20.0f); // Enviar 500 pasos a 1000 Hz

//         sleep_ms(1000);
//     }

//     return 0;
// }