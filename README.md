# picow_tmc2208

Librería para controlar drivers de motor paso a paso TMC2208 con Raspberry Pi Pico W, incluyendo comunicación UART PDN para configuración avanzada de parámetros.

## Características

- Control básico de motores paso a paso (STEP, DIR, ENA)
- Configuración avanzada via comunicación UART PDN
- Configuración de microstepping (1, 2, 4, 8, 16, 32, 64, 128, 256)
- Control de corrientes (Irun, Ihold)
- Configuración de tiempos de transición
- Lectura de estado del driver
- Soporte para múltiples instancias UART

## Conexiones Hardware

### Pines básicos
- **STEP**: Pin GPIO para pulsos de paso
- **DIR**: Pin GPIO para dirección
- **ENA**: Pin GPIO para habilitar/deshabilitar el driver

### Pines UART PDN
- **TX**: Pin GPIO conectado al pin PDN_UART del TMC2208 (con resistencia de 1kΩ)
- **RX**: Pin GPIO conectado al pin PDN_UART del TMC2208 (sin resistencia)

## Uso Básico

```c
#include "pico/stdlib.h"
#include "tmc2208/tmc2208.h"

int main() {
    stdio_init_all();
    
    TMC2208_t motor;
    
    // Inicialización básica (sin UART)
    tmc2208_init(&motor, STEP_PIN, DIR_PIN, ENA_PIN, 200, 16, NULL, 0, 0, 0);
    
    tmc2208_enable(&motor, true);
    tmc2208_set_direction(&motor, true);
    tmc2208_set_rpm(&motor, 60.0f);
    
    return 0;
}
```

## Uso Avanzado con UART

```c
#include "pico/stdlib.h"
#include "tmc2208/tmc2208.h"

int main() {
    stdio_init_all();
    
    TMC2208_t motor;
    
    // Inicialización con UART
    tmc2208_init(&motor, STEP_PIN, DIR_PIN, ENA_PIN, 200, 16, 
                 uart0, UART_TX_PIN, UART_RX_PIN, 0);
    
    // Configurar parámetros avanzados
    tmc2208_set_microstepping(&motor, TMC2208_MICROSTEPS_16);
    tmc2208_set_currents(&motor, 20, 10, 5); // Irun=20, Ihold=10, delay=5
    tmc2208_set_tpowerdown(&motor, 20);
    
    // Leer estado del driver
    uint32_t status;
    tmc2208_get_status(&motor, &status);
    
    tmc2208_enable(&motor, true);
    
    return 0;
}
```

## Funciones Principales

### Inicialización
- `tmc2208_init()` - Inicializa el driver con configuración básica y UART opcional

### Control Básico
- `tmc2208_enable()` - Habilita/deshabilita el driver
- `tmc2208_set_direction()` - Establece dirección de rotación
- `tmc2208_set_rpm()` - Establece velocidad en RPM
- `tmc2208_stop()` - Detiene el motor
- `tmc2208_send_nsteps_at_freq()` - Envía número específico de pasos

### Configuración UART Avanzada
- `tmc2208_set_microstepping()` - Configura microstepping
- `tmc2208_set_currents()` - Configura corrientes Irun e Ihold
- `tmc2208_set_tpowerdown()` - Configura tiempo de transición
- `tmc2208_get_status()` - Lee estado del driver
- `tmc2208_configure_all()` - Configura múltiples parámetros

## Parámetros de Configuración

### Microstepping
- `TMC2208_MICROSTEPS_1` a `TMC2208_MICROSTEPS_256`
- Valores: 1, 2, 4, 8, 16, 32, 64, 128, 256 micropasos por paso completo

### Corrientes
- **Irun**: Corriente de operación (0-31, escala interna del driver)
- **Ihold**: Corriente de reposo (0-31, escala interna del driver)
- **Ihold_delay**: Delay antes de reducir corriente (0-15)

### Tiempo de Transición
- **TPOWERDOWN**: Tiempo en unidades de 2^18 clk cycles (aproximadamente 0-512ms)

## Compilación

```bash
mkdir build
cd build
cmake ..
make
```

## Notas Importantes

1. **Conexión UART**: El pin PDN_UART del TMC2208 debe conectarse correctamente para habilitar la comunicación UART.

2. **Resistencias**: Se requiere una resistencia de 1kΩ entre TX del Pico y PDN_UART del TMC2208.

3. **Direcciones**: Cada driver TMC2208 debe tener una dirección UART única (0-3).

4. **Compatibilidad**: La librería es compatible con Raspberry Pi Pico W y otros microcontroladores con SDK de Pico.

## Licencia

Este proyecto está bajo licencia MIT. Ver archivo LICENSE para más detalles.