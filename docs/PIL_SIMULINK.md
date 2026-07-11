# PIL Simulink

Este flujo deja en Simulink la planta, el ESC y la conmutacion del BLDC. La planta usa su PI local; el MCU fisico recibe el mismo vector de velocidad para validar el controlador offline. En esta arquitectura el MCU no recibe cruces por cero ni conoce el paso de conmutacion: solo recibe velocidad simulada como realimentacion y devuelve el PWM logico calculado por el controlador.

El modelo selecciona el sector six-step a partir de la posicion mecanica ideal del rotor: `theta_e = mod(2 * theta_m, 2*pi)`. Los dos pares de polos coinciden con el bloque BLDC y el offset electrico inicial es cero. Es una abstraccion de simulacion; no representa una estrategia aplicable al STM32 sin sensor de posicion.

Simulink no abre el HID directamente. Envia UDP al bridge de la GUI en `127.0.0.1:5055`, y el bridge traduce esas entradas a frames binarios `HIL_*` hacia el MCU.

## Entrada minima desde Simulink

Formato recomendado:

```text
seq,speed_rpm,enable
```

Tambien se aceptan paquetes prefijados:

```text
PIL,speed_rpm,enable
PIL,seq,speed_rpm,enable
```

Campos:

| Campo | Descripcion |
| --- | --- |
| `seq` | Secuencia monotona para estadisticas de perdida. |
| `speed_rpm` | Velocidad simulada que usa el controlador del MCU como medicion. |
| `enable` | `0` detiene HIL, distinto de cero inicia/actualiza HIL. |

Para una validacion trazable, el replay usa:

```text
PILV,run_id,seq,speed_rpm,enable
```

`run_id` identifica una corrida y `seq` es un `uint32` monotono. El bridge fuerza una lectura fresca de salida para cada `PILV`, registra un JSONL append-only en `%LOCALAPPDATA%\EscGui\hil-validation`, y el MCU devuelve la secuencia que produjo el PWM.

Ejemplo:

```text
42,1800,1
```

## Compatibilidad

El bridge aun acepta el formato HIL anterior:

```text
seq,speed_rpm,reserved,load_torque,flags,enable
```

El campo `reserved` reemplaza al antiguo `zero_crossing_period`: se parsea por compatibilidad, pero firmware lo ignora y no genera ningun evento de conmutacion.

## Firmware

En `HIL_SIM`, las capturas fisicas de TIM2 no gobiernan nada. El firmware:

1. actualiza `hil_speed_rpm` con la velocidad simulada;
2. ejecuta el PI usando esa velocidad como feedback;
3. expone el PWM logico por `HIL_GET_OUTPUTS`;
4. detiene HIL si no llegan entradas por mas de 50 ms.

La simulacion de ESC/conmutacion queda completa del lado de Simulink.

## GUI

El dashboard principal mantiene el panel `HIL Simulink`. La pagina `/pil` agrega una vista de tramas con dos columnas:

| Columna | Contenido |
| --- | --- |
| `Simulink` | Paquetes UDP ASCII recibidos y respuestas enviadas al modelo. |
| `MCU PIL` | Frames HID binarios `HIL_START`, `HIL_STOP`, `HIL_SET_INPUTS` y `HIL_GET_OUTPUTS`. |

Esto permite comparar la velocidad enviada por Simulink con el PWM logico que devuelve el MCU durante una corrida PIL.

## Logger UDP externo

Si la GUI esta usando `127.0.0.1:5055` y se quiere corroborar exactamente que CSV sale de Simulink, se puede usar el proxy/logger:

```powershell
python .\simulacion_agitador\udp_hil_logger.py proxy --listen-port 5057 --target-port 5055 --log-file .\simulacion_agitador\hil_udp_log.jsonl
```

Luego el bloque `UDP Send` de Simulink debe enviar a `127.0.0.1:5057`.

En este repo, `simulacion_agitador/sim_motor.mdl` ya quedo configurado asi por defecto. Usa `LocalAddress = 0.0.0.0` y `LocalPort = 5058` para que el `UDP Receive` pueda leer la respuesta que vuelve al mismo puerto local.

El script:

1. registra el datagrama ASCII real que sale de Simulink;
2. indica si el bridge lo aceptaria con el parser actual;
3. reenvia el paquete a la GUI en `127.0.0.1:5055`;
4. registra tambien la respuesta del bridge.

Para escuchar sin reenviar, por ejemplo con la GUI cerrada:

```powershell
python .\simulacion_agitador\udp_hil_logger.py listen --listen-port 5055
```

## Recepcion UDP en Simulink

`simulacion_agitador/sim_motor.mdl` incluye un bloque `UDP Receive MCU` para leer la respuesta completa del bridge. El modelo se puede regenerar con:

```matlab
run("simulacion_agitador/configure_hil_udp_receive.m")
```

Configuracion actual:

| Bloque | Parametro | Valor |
| --- | --- | --- |
| `UDP Send` | `Port` | `5057` |
| `UDP Send` | `LocalPort` | `5058` |
| `UDP Receive MCU` | `Port` | `5057` |
| `UDP Receive MCU` | `LocalPort` | `5058` |
| `UDP Receive MCU` | `DataSize` | `[1, 256]` |
| `HIL PWM Full Scale` | `Value` | `2000` |
| `HIL Fallback Duty` | `Value` | `0.3` |

Esto asume el proxy/logger en `5057`. Para enviar directo a la GUI, cambiar el puerto destino de `5057` a `5055` en el script o en ambos bloques. `HIL PWM Full Scale` debe coincidir con `TIM1->ARR`; con la configuracion por defecto del firmware, `72 MHz / (2 * 18 kHz) = 2000`.

El parser `Parse MCU UDP PI` decodifica la respuesta base:

```text
ok,seq,tick_ms,app_state,mode,setpoint_rpm,measured_rpm,pwm_command,commutation_step,flags,timeout,rx_frames,lost_frames,effective_hz,avg_rtt_ms,jitter_ms
```

y guarda en workspace:

| Variable | Contenido |
| --- | --- |
| `hil_mcu_packet` | Vector `[valid, seq, tick_ms, app_state, mode, setpoint_rpm, measured_rpm, pwm_command, commutation_step, flags, timeout, rx_frames, lost_frames, effective_hz, avg_rtt_ms, jitter_ms]`. |
| `hil_mcu_duty` | `pwm_command / HIL PWM Full Scale`, saturado entre `0` y `1`. |
| `hil_sim_pi_duty` | Duty del PI local que acciona la planta. |
| `hil_plant_duty` | Duty efectivamente aplicado despues de `HIL Duty Delay`. |
| `hil_mcu_packet_valid` | `1` cuando se recibio un paquete `ok,...` completo. |
| `motor_rpm` | Velocidad mecanica real de la planta, en rpm. |
| `six_step_dsw` | Vector de seis comandos que entra al inversor promedio. |
| `six_step_sector` | Sector six-step activo (`1` a `6`); vale `0` cuando el drive esta deshabilitado. |

El duty que entra al conmutador ideal viene siempre del PI local de Simulink. `hil_mcu_duty` no controla la planta: es una observacion del controlador fisico. La senal local pasa por `HIL Duty Delay` con `Ts = 20 ms`, que representa un retardo discreto del actuador local y evita lazos algebraicos; no representa una latencia USB.

El interruptor manual del modelo alimenta `enable`. Por defecto selecciona `1` y habilita el drive local; al seleccionar `0`, el conmutador entrega `Dsw = [0 0 0 0 0 0]`. La rama BEMF de resistencias, sensores de fase y comparadores ya no participa en el control. Se conserva la medicion de tension de linea y sus scopes.

Importante: en esta arquitectura HIL el MCU solo devuelve el PWM logico del PI. La conmutacion/sector sigue quedando del lado de Simulink y no depende de `target_rpm`, del paquete UDP ni de cruces por cero. Si la velocidad aparece sinusoidal u oscilante, revisar primero `six_step_sector`, `six_step_dsw` y la alineacion entre el sector aplicado y el angulo del rotor.

## Validacion offline

No se sincroniza el step del solver con HID. Simscape conserva su solver variable y el vector HIL se toma cada 20 ms. Primero se ejecuta la planta localmente y se exporta un vector; luego se reproduce contra el MCU y se compara el PWM por identidad logica, no por hora de llegada.

```matlab
config = struct("targetRpm", 1000, "runId", uint32(1));
[vector, manifest, vectorPath] = export_hil_validation_vector(motor_rpm, config);
[responses, responsePath] = replay_hil_validation(vectorPath);
report = compare_hil_validation(vectorPath, responsePath, plot=true);
```

`firmware_pi_reference_step` reproduce el PI de firmware con paso de 2 ms. `compare_hil_validation` une `ExpectedPwm` con el `pwm_command` del MCU por `run_id` y `applied_source_seq`, elimina outputs cacheados por `output_generation` y reporta cobertura y error PWM.

### Contrato de importacion GUI

`export_hil_validation_vector` conserva `validationVector` y `manifest` para
los scripts MATLAB, y exporta adicionalmente `esc_validation_v1`. La GUI debe
importar exclusivamente esta estructura plana MAT v7:

| Campo | Tipo | Significado |
| --- | --- | --- |
| `schema_version` | `uint32` | Version del contrato, actualmente `1`. |
| `manifest_json` | `char` | Metadatos JSON: nombre, descripcion, periodo y configuracion PI. |
| `simulation_time_s` | `double[]` | Tiempo de simulacion de cada muestra. |
| `run_id` | `uint32[]` | Identificador de corrida, no nulo y uniforme. |
| `source_sequence` | `uint32[]` | Secuencia estrictamente ascendente. |
| `speed_rpm` | `uint16[]` | Entrada medida del controlador. |
| `enable` | `uint8[]` | `0` deshabilita, distinto de cero habilita. |
| `target_rpm` | `uint16[]` | Referencia usada por el controlador. |
| `expected_pwm` | `uint16[]` | Salida esperada en cuentas PWM logicas. |

`expected_pwm` se compara directamente contra `pwm_command` devuelto por el
MCU. Ambos estan en cuentas PWM, no en porcentaje. Los artefactos de
validacion deben conservar el MAT original junto con los resultados.

### Exportacion general desde `logsout`

Para un modelo nuevo, usar `export_simulink_validation_run` en vez de depender
del workspace base. El modelo debe habilitar Signal Logging y exponer en
`logsout` solo las senales declaradas por el llamador:

```matlab
spec = struct();
spec.validation = struct( ...
    "speedRpm", "validation_speed_rpm", ...
    "expectedPwm", "validation_pwm_counts", ...
    "enable", "validation_enable", ...
    "targetRpm", "validation_target_rpm");
spec.extraSignals = [ ...
    struct("name", "motor_rpm", "unit", "rpm")
    struct("name", "load_torque", "unit", "N*m")
    ];

config = struct( ...
    "experimentName", "Escalon 1000 RPM", ...
    "description", "PI discreto", ...
    "controllerPeriodSeconds", 0.002, ...
    "targetRpm", 1000, ...
    "kp", 0.75, "ki", 1.35, "kd", 0, ...
    "pwmFrequency", 18000, "pwmArr", 2000, "polePairs", 2);

[artifact, report, matPath] = export_simulink_validation_run( ...
    "sim_motor", 10, spec, config, "D:\runs\escalon_1000.mat");
```

El script carga el modelo, simula hasta el tiempo solicitado y no guarda ningun
cambio en el archivo del modelo. Si `config.runId` no esta definido, genera un
identificador `uint32` no nulo. Las cuatro senales de `spec.validation` deben
ser escalares y loguearse como `timeseries`. `expectedPwm` define la grilla de
tiempo discreta; debe tener periodo exacto `controllerPeriodSeconds`.

`speedRpm`, `enable` y `targetRpm` se alinean sobre esa grilla mediante
zero-order hold. No se usa interpolacion lineal. `targetRpm` debe permanecer
constante porque el protocolo MCU actual configura un unico setpoint por run.
Las extras declaradas se guardan como `experimentSignals` con su propio eje de
tiempo y unidad; la GUI actual conserva el artefacto aunque todavia no las
grafica.

Por defecto `replay_hil_validation` aplica `KP`, `KI`, `KD`, pares de polos y frecuencia PWM del manifiesto en RAM mediante la API local del bridge, luego cambia a `SimulinkControl`. No usa `SAVE_CONFIG`; la corrida no modifica la configuracion persistente. Se puede desactivar con `configureBridge=false` si esos parametros ya fueron preparados externamente.

## Verificacion rapida

Ejecutar una simulacion finita con el interruptor en `enable=1`. Sin respuesta UDP, `hil_mcu_packet_valid` puede quedar en cero, pero `hil_sim_pi_duty` y `hil_plant_duty` siguen controlando la planta. `six_step_sector` debe recorrer `1` a `6` y la velocidad debe salir de reposo. Al seleccionar `enable=0`, `six_step_dsw` debe ser nulo y el accionamiento debe detenerse. Una respuesta MCU valida solo actualiza `hil_mcu_packet` y `hil_mcu_duty` para validacion.
