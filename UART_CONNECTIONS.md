# Conexiones UART para TMC2208

## Diagrama de Conexiones

```
Raspberry Pi Pico W          TMC2208 Driver
┌─────────────────┐          ┌─────────────────┐
│                 │          │                 │
│ GPIO 0 (UART0_TX)├─────────┤ PDN_UART        │
│                 │   1kΩ   │                 │
│                 │          │                 │
│ GPIO 1 (UART0_RX)├────────┤ PDN_UART        │
│                 │          │                 │
│                 │          │                 │
│ GPIO 2 (DIR)    ├─────────┤ DIR             │
│                 │          │                 │
│ GPIO 3 (STEP)   ├─────────┤ STEP            │
│                 │          │                 │
│ GPIO 8 (ENA)    ├─────────┤ ENA             │
│                 │          │                 │
│ GND             ├─────────┤ GND             │
│                 │          │                 │
│ 3.3V            ├─────────┤ VCC             │
│                 │          │                 │
└─────────────────┘          └─────────────────┘
```

## Notas Importantes

### 1. Resistencia de 1kΩ
- **OBLIGATORIA**: Se debe colocar una resistencia de 1kΩ entre el pin TX del Pico (GPIO 0) y el pin PDN_UART del TMC2208
- **Propósito**: Proteger el pin PDN_UART del TMC2208 de posibles sobrevoltajes

### 2. Conexión RX
- El pin RX del Pico (GPIO 1) se conecta **directamente** al pin PDN_UART del TMC2208
- **Sin resistencia**: Esta conexión no requiere resistencia adicional

### 3. Configuración de Dirección UART
- Cada TMC2208 debe tener una dirección UART única (0-3)
- La dirección se configura mediante jumpers o pines en la placa del driver
- En el código, se especifica la dirección en la función `tmc2208_init()`

### 4. Pines UART Alternativos
- Se pueden usar otros pines GPIO para UART
- Para UART0: Cualquier pin compatible con UART0_TX/UART0_RX
- Para UART1: Cualquier pin compatible con UART1_TX/UART1_RX

## Ejemplo de Configuración de Pines

```c
// Configuración básica
#define UART_TX_PIN 0    // GPIO 0 (UART0_TX)
#define UART_RX_PIN 1    // GPIO 1 (UART0_RX)

// Configuración alternativa
#define UART_TX_PIN 4    // GPIO 4 (UART1_TX)
#define UART_RX_PIN 5    // GPIO 5 (UART1_RX)

// En la inicialización
tmc2208_init(&motor, STEP_PIN, DIR_PIN, ENA_PIN, 
             200, 16, 
             uart0, UART_TX_PIN, UART_RX_PIN, 0);  // Dirección 0
```

## Solución de Problemas

### Error de Comunicación UART
1. **Verificar conexiones**: Asegurar que TX y RX estén conectados correctamente
2. **Verificar resistencia**: Confirmar que la resistencia de 1kΩ esté presente
3. **Verificar dirección**: Confirmar que la dirección UART sea correcta
4. **Verificar alimentación**: Asegurar que el TMC2208 esté alimentado correctamente

### Driver No Responde
1. **Verificar configuración de pines**: Confirmar que los pines UART estén configurados correctamente
2. **Verificar baud rate**: El TMC2208 usa 115200 baud por defecto
3. **Verificar inicialización**: Asegurar que `tmc2208_uart_init()` se ejecute correctamente

### Lectura de Registros Fallida
1. **Verificar CRC**: El TMC2208 usa CRC8 para verificar integridad de datos
2. **Verificar timeout**: Aumentar el tiempo de espera si es necesario
3. **Verificar conexión RX**: Asegurar que la conexión RX funcione correctamente

## Múltiples Drivers TMC2208

Para controlar múltiples drivers TMC2208:

```c
TMC2208_t motor1, motor2;

// Driver 1 - Dirección 0
tmc2208_init(&motor1, STEP_PIN1, DIR_PIN1, ENA_PIN1, 
             200, 16, uart0, UART_TX_PIN, UART_RX_PIN, 0);

// Driver 2 - Dirección 1  
tmc2208_init(&motor2, STEP_PIN2, DIR_PIN2, ENA_PIN2, 
             200, 16, uart0, UART_TX_PIN, UART_RX_PIN, 1);
```

**Nota**: Todos los drivers pueden compartir la misma conexión UART, pero cada uno debe tener una dirección única.
