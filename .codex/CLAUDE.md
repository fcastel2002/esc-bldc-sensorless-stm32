# ESC BLDC Sensorless STM32 — Contexto para Claude

> [!IMPORTANT]
> Si el usuario pide actualizar este archivo, actualizar también `AGENTS.md` en el mismo directorio (`.agents/`) con los mismos cambios de contenido. Ambos archivos deben mantenerse sincronizados.

## 1. Qué es este proyecto

Controlador electrónico de velocidad (ESC) open-source y open-hardware para motores BLDC/PMSM sin sensores de posición. Usa un STM32F103C8T6 (Blue Pill), conmutación trapezoidal de seis pasos con detección sensorless de cruces por cero de la BEMF, arranque sinusoidal open-loop y control de velocidad mediante PI (con término KD opcional sobre la velocidad medida). Fue validado con motores BLDC de discos rígidos y etapa de potencia con módulos L298N.

Licencia: MIT. Autor: Francisco Castel.

## 2. Estructura general del repositorio

```text
.
├── firmware/               Proyecto embebido STM32F103C8T6
│   ├── Core/
│   │   ├── Inc/            Headers del firmware
│   │   └── Src/            Fuentes del firmware
│   ├── Drivers/            HAL y CMSIS de ST
│   ├── Middlewares/        USB Device middleware
│   ├── COMM_PROTOCOL.md    Referencia del protocolo binario de 64 bytes
│   └── CMakeLists.txt      Build con CMake + Ninja (arm-none-eabi-gcc)
│
├── gui/                    GUI de escritorio
│   └── EscGui/
│       ├── src/
│       │   ├── Esc.Protocol/    Frames, CRC16, opcodes, payloads
│       │   ├── Esc.Transport/   Enumeración y transporte USB HID (HidSharp)
│       │   ├── Esc.Bridge/      Servicio central, estado, arbitraje, telemetría, UDP
│       │   └── Esc.Web/         Dashboard Blazor Server (C#/.NET)
│       ├── tests/Esc.Tests/     Tests de protocolo y bridge
│       ├── GUI_AGENT_CONTEXT.md Contexto compacto para la GUI
│       ├── GUI_BLAZOR_WALKTHROUGH.md  Walkthrough completo de la GUI
│       └── BRIDGE_WALKTHROUGH.md      Walkthrough del Bridge
│
├── docs/
│   ├── PROJECT_FLOW.md     Flujo detallado del firmware
│   ├── PIL_SIMULINK.md     Guía de integración PIL con Simulink
│   └── BRIDGE_WALKTHROUGH.md  (copia del walkthrough del Bridge)
│
├── INFORME/                Paper/informe LaTeX del proyecto
└── README.md               Resumen visual y técnico
```

## 3. Firmware — Arquitectura y flujo

### 3.1. Microcontrolador y periféricos

- **MCU**: STM32F103C8T6, clock a 72 MHz (HSE + PLL).
- **TIM1**: PWM complementario para las 3 fases del inversor (6 canales).
- **TIM2**: Captura de cruces por cero de la BEMF (3 canales, uno por fase).
- **TIM3**: Base temporal auxiliar.
- **TIM4**: Base temporal para arranque SPWM y output compare para el tick de control PI.
- **USART1**: Comunicación UART (PB6/PB7, 115200 8N1).
- **USB FS**: Custom HID en PA11/PA12 (reports de 64 bytes).
- **CRC**: Periférico hardware para CRC32 de configuración Flash.
- **Flash**: Dos páginas rotativas al final de Flash para persistencia de parámetros.
- **GPIO PB8**: Selección de transporte al boot (HIGH=UART, LOW=USB).

### 3.2. Máquina de estados (`state_machine.c`)

```text
IDLE → FOC_STARTUP → RUNNING → READY → CLOSEDLOOP
  ↑         ↓            ↓         ↓
  ←── STOPPED ←── stall/timeout ──←
  ↓
CONFIG (aplica parámetros diferidos)
  ↓
HARD_ERROR → Error_Handler()
```

| Estado        | Descripción |
|---------------|-------------|
| `IDLE`        | Motor detenido, recibe comandos cada 50 ms. |
| `FOC_STARTUP` | Arranque sinusoidal open-loop (no es FOC completo). |
| `RUNNING`     | Six-step sensorless, esperando cruces por cero consistentes. |
| `READY`       | Configura TIM4 para el tick del PI. |
| `CLOSEDLOOP`  | Control de velocidad en lazo cerrado. |
| `CONFIG`      | Aplica parámetros que no se pueden cambiar en caliente. |
| `STOPPED`     | Detiene interrupciones de control. |
| `HARD_ERROR`  | Entra a `Error_Handler()`. |

### 3.3. Archivos clave del firmware

| Archivo | Responsabilidad |
|---------|----------------|
| `main.c` | Entrada, clocks, periféricos, loop principal. |
| `state_machine.c` | Estados y coordinación del firmware. |
| `comm.c` | Fachada de comunicación y telemetría periódica. |
| `comm_transport.c` | Selección UART/USB y transporte de frames. |
| `comm_protocol.c` | Protocolo binario, CRC, opcodes y efectos de comandos. |
| `hard_config.c` | Parámetros ESC, límites, getters/setters, actualización TIM1. |
| `flash_config.c` | Persistencia en Flash con páginas rotativas y CRC. |
| `startup.c` | Arranque sinusoidal open-loop y transición a six-step. |
| `bldc_driver.c` | Conmutación de seis pasos y duty PWM. |
| `motor_control.c` | Control PI, stall, zero-crossing handler, conversiones. |
| `speed_sensor.c` | Medición de velocidad por cruces por cero y consenso trifásico. |
| `isr_callbacks.c` | Callbacks HAL para captura, output compare y update de timers. |

### 3.4. Control de velocidad (PI + KD)

- El PI actúa ajustando duty PWM, no la secuencia de conmutación.
- `KD` se aplica sobre la derivada de la velocidad medida (no sobre el error) para evitar derivative kick ante cambios de setpoint.
- Anti-windup por saturación entre `min_limit_pwm` y `max_limit_pwm`.
- En `CLOSEDLOOP`, los cambios a `PWM_FREQ`, `KP`, `KI`, `KD` se aplican en caliente sin salir del lazo cerrado.
- Parámetros por defecto: `KP=0.75`, `KI=1.35`, `KD=0.0`, PWM 18 kHz, 2 pares de polos.

### 3.5. Protocolo binario

- Frames fijos de 64 bytes, little-endian.
- Magic `0xEC 0xB1`, version `0x01`, CRC16-CCITT-FALSE.
- Referencia completa en `firmware/COMM_PROTOCOL.md`.
- Opcodes principales: PING, GET_STATUS, RUN, STOP, ESTOP, SET_SPEED_RPM, GET/SET/RESET/SAVE_CONFIG, LOG_START/STOP/RATE, HIL_*.

### 3.6. Build del firmware

```powershell
# Desde firmware/
cmake --preset default
cmake --build build
```

Toolchain: `arm-none-eabi-gcc`. Genera `.elf` para flashear con ST-Link o similar.

## 4. GUI — Arquitectura y flujo

### 4.1. Stack tecnológico

- **Blazor Server** (C# / .NET), corre localmente en `http://localhost:5187`.
- **HidSharp** para USB HID.
- No es una web pública: es una aplicación local que controla el ESC por USB.

### 4.2. Capas de la GUI

```text
Browser → Blazor Server → EscBridgeService → EscProtocol → HidSharpTransport → USB HID → MCU
Simulink → UDP 127.0.0.1:5055 → HilUdpBridgeService → EscBridgeService → mismo camino HID
```

| Proyecto | Responsabilidad |
|----------|----------------|
| `Esc.Protocol` | Frames, CRC16, opcodes, payloads, modelos de dominio. |
| `Esc.Transport` | Enumeración HID y transporte USB con HidSharp. |
| `Esc.Bridge` | Fuente de verdad de estado, arbitraje GUI/Simulink/HTTP, telemetría. |
| `Esc.Web` | Dashboard Blazor, controles modulares, gráfico SVG. |
| `Esc.Tests` | Tests de protocolo y bridge con transporte falso. |

### 4.3. Regla central del Bridge

**Solo `EscBridgeService` decide cuándo y cómo se usa el HID.** La UI no abre HID, y Simulink tampoco. Todo pasa por el Bridge.

### 4.4. Modos de control

| Modo | GUI puede | Simulink puede | ESTOP |
|------|-----------|----------------|-------|
| `GuiControl` | Run, Stop, SetSpeed, Config | No | Siempre |
| `SimulinkControl` | Solo monitoreo | Run, Stop, SetSpeed | Siempre |
| `MonitorOnly` | Solo monitoreo | No | Siempre |

### 4.5. Controles modulares

- Definidos en `wwwroot/controls.json`.
- Cargados una sola vez por `ModularControlProvider` (no hay recarga en caliente).
- Soportan: `set_speed_rpm`, `set_config`, `reset_config`, `log_speed_start`, `log_speed_stop`.

### 4.6. Telemetría

- El firmware emite `TelemetryEvent` (type `0x82`, opcode `0x33`).
- `TelemetryStore` conserva hasta 2000 muestras por variable.
- El gráfico es SVG manual (no ScottPlot, aunque el paquete está referenciado).
- La lectura de telemetría comparte el mismo lock que los requests HID.

### 4.7. Ejecución de la GUI

```powershell
dotnet run --project gui\EscGui\src\Esc.Web\Esc.Web.csproj
```

Para publicar `.exe`:

```powershell
powershell -ExecutionPolicy Bypass -File .\gui\EscGui\publish-gui-win-x64.ps1
```

### 4.8. HIL / Simulink

- `HilUdpBridgeService` escucha UDP en `127.0.0.1:5055`.
- Acepta CSV: `seq,speed_rpm,enable` o `PIL,speed_rpm,enable`.
- Coalescencia: si llegan paquetes mientras el HID está ocupado, se aplica el más reciente (no es FIFO).
- ESTOP tiene prioridad sobre paquetes coalescidos.
- Timeout de 50 ms en firmware: si no llegan inputs HIL, vuelve a IDLE.

## 5. Documentación de referencia

| Documento | Ubicación | Cuándo leer |
|-----------|-----------|-------------|
| Protocolo binario | `firmware/COMM_PROTOCOL.md` | Cambios de opcodes, payloads o frames. |
| Flujo del firmware | `docs/PROJECT_FLOW.md` | Entender flujo completo del firmware. |
| Contexto GUI compacto | `gui/EscGui/GUI_AGENT_CONTEXT.md` | Cambios en la GUI (lectura mínima). |
| Walkthrough Blazor | `gui/EscGui/GUI_BLAZOR_WALKTHROUGH.md` | Entender la GUI en profundidad. |
| Walkthrough Bridge | `gui/EscGui/BRIDGE_WALKTHROUGH.md` | Entender el Bridge en profundidad. |
| PIL Simulink | `docs/PIL_SIMULINK.md` | Integración con Simulink. |

## 6. Reglas y convenciones

### Código firmware (C)

- Formateo con `.clang-format` (está en la raíz del repositorio).
- Linting con `.clang-tidy`.
- Los archivos generados por CubeMX tienen secciones `USER CODE BEGIN/END`; no editar fuera de esas secciones en archivos generados.
- Usar `extern` en headers, definir variables globales en `.c`.
- Las ganancias del PI se almacenan como `float` internamente pero se transmiten como `int16` en centésimas (ej: `135` = `1.35`).

### Código GUI (C#)

- Solución con 4 proyectos en `gui/EscGui/src/` y tests en `gui/EscGui/tests/`.
- No abrir nunca el HID desde la UI o desde código externo; siempre pasar por `EscBridgeService`.
- Si un cambio afecta estado observable, debe terminar actualizando `Snapshot` y llamando `Notify()`.
- `SaveConfig` es separado de `SetConfig`: modificar un parámetro no persiste a flash automáticamente.

### General

- El proyecto está en español (comentarios, documentación, nombres de parámetros en la GUI).
- Ser conservador con cambios en el protocolo binario: cualquier cambio debe reflejarse en firmware Y en GUI.
- No asumir que funcionalidades marcadas como `NOT_IMPLEMENTED` en el protocolo ya funcionan (ej: `CURRENT_LIMIT`, `TEMP_LIMIT`).

## 7. Errores comunes a evitar

> [!CAUTION]
> Estas son lecciones aprendidas de iteraciones anteriores. Revisarlas antes de hacer cambios.

1. **No saltear `Notify()` en el Bridge**: Si se actualiza estado en `EscBridgeService` sin llamar `Notify()`, la UI Blazor queda desfasada porque depende de `SnapshotChanged`.

2. **`controls.json` está cacheado**: `ModularControlProvider` lo carga una sola vez al arrancar. No esperar recarga en caliente tras editar el JSON.

3. **`foc_startup()` no es FOC**: El nombre es histórico. Es un arranque sinusoidal open-loop, no FOC con lazo de corriente. No agregar lógica de corriente ahí.

4. **No abrir caminos paralelos al HID**: La lectura de telemetría comparte el mismo `_ioLock` que los requests. No intentar leer HID desde múltiples puntos simultáneamente.

5. **Transporte se decide al boot**: El modo UART/USB se fija al arrancar leyendo PB8. No intentar cambiar de transporte dinámicamente.

6. **Flash: no escribir sin `SAVE_CONFIG`**: `SET_CONFIG` y `RESET_CONFIG` solo modifican RAM. La escritura a flash es explícita con `SAVE_CONFIG`.

7. **Consistencia de protocolo**: Si se agrega un opcode nuevo, debe implementarse en firmware (`comm_protocol.c`), en GUI (`CommConstants.cs`, `EscProtocol.cs`, `EscBridgeService.cs`) y documentarse en `COMM_PROTOCOL.md`.

8. **ScottPlot referenciado pero no usado**: El paquete `ScottPlot.Blazor` está en el `.csproj` pero el gráfico real es SVG manual. No intentar usar la API de ScottPlot para el gráfico actual.

9. **Temperatura y corriente son placeholders**: Los valores de telemetría de temperatura y corriente no son reales; no confiar en esos datos para lógica de protección.

10. **El timeout de `RUNNING` es independiente del stall**: Si `RUNNING` dura más de 1.5 s sin pasar a `READY`, se reintenta el arranque. Esto no depende del detector de stall.

11. **`ConfigParam.CurrentLimit` y `ConfigParam.TempLimit` existen en enums** de la GUI pero `EscProtocol.ConfigPayload()` no los soporta todavía. No asumir que están implementados.

12. **Coalescencia UDP en HIL**: El bridge trata los paquetes UDP como señales de último valor, no como cola FIFO. Si Simulink envía más rápido de lo que el HID puede procesar, solo se aplica el paquete más reciente.

13. **Después de cada comando nuevo en el bridge, decidir si necesita `refreshStatus`**: El bridge refresca estado después de la mayoría de los comandos. Si se agrega uno nuevo, evaluar si corresponde.

## 8. Cómo orientarse para cambios comunes

| Tipo de cambio | Archivos principales |
|----------------|---------------------|
| Botón o indicador visual | `Home.razor`, quizás `EscBridgeService.cs` |
| Control configurable nuevo desde JSON | `controls.json`, `ConfigParam`, `EscProtocol`, firmware `comm_protocol.c` |
| Comando HID nuevo | `CommOpcode`, `EscProtocol`, `EscBridgeService`, UI/endpoint, tests, firmware `comm_protocol.c` |
| Telemetría nueva | `LogParam`, `EscProtocol.DecodeTelemetry()`, `TelemetryStore`, UI |
| Cambio HIL/UDP | `HilUdpBridgeService.cs`, `HilModels.cs`, `EscProtocol` |
| Parámetro de config nuevo en firmware | `hard_config.h/.c`, `comm_protocol.c`, `flash_config.c`, `COMM_PROTOCOL.md` |
| Cambio en la máquina de estados | `state_machine.c`, `state_machine.h` |
| Cambio en conmutación | `bldc_driver.c`, `isr_callbacks.c` |
| Cambio en medición de velocidad | `speed_sensor.c`, `motor_control.c` |
