# PIL Simulink

Este flujo deja en Simulink la planta, el ESC y la conmutacion del BLDC. El MCU fisico se usa para validar el controlador de velocidad y, mas adelante, el observador o generador de trayectorias. En esta arquitectura el MCU no recibe cruces por cero ni conoce el paso de conmutacion: solo recibe velocidad simulada como realimentacion y devuelve el PWM logico calculado por el controlador.

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

El parser `Parse MCU UDP PI` decodifica la respuesta:

```text
ok,seq,tick_ms,app_state,mode,setpoint_rpm,measured_rpm,pwm_command,commutation_step,flags,timeout,rx_frames,lost_frames,effective_hz,avg_rtt_ms,jitter_ms
```

y guarda en workspace:

| Variable | Contenido |
| --- | --- |
| `hil_mcu_packet` | Vector `[valid, seq, tick_ms, app_state, mode, setpoint_rpm, measured_rpm, pwm_command, commutation_step, flags, timeout, rx_frames, lost_frames, effective_hz, avg_rtt_ms, jitter_ms]`. |
| `hil_mcu_duty` | `pwm_command / HIL PWM Full Scale`, saturado entre `0` y `1`. |
| `hil_sim_pi_duty` | Duty calculado por un PI local simple, solo como referencia de comparacion. |
| `hil_mcu_packet_valid` | `1` cuando se recibio un paquete `ok,...` completo. |
| `six_step_dsw` | Vector de seis comandos que entra al inversor promedio. |
| `six_step_rpm_ref` | Rampa open-loop que usa el generador de sectores del modelo. |
| `six_step_fe_hz` | Frecuencia electrica de esa rampa. |
| `six_step_sector` | Sector six-step activo. Si queda clavado, las tensiones tambien quedan clavadas. |

El duty que entra al bloque de conmutacion six-step viene del PWM del MCU cuando el paquete es valido. Si no hay respuesta UDP valida, el modelo usa `HIL Fallback Duty` para no ocultar el problema con un PI local saturado. La senal pasa por `HIL Duty Delay` con `Ts = 20 ms`, que evita lazos algebraicos y representa el retardo discreto del ciclo Simulink-bridge-MCU.

Importante: en esta arquitectura HIL el MCU solo devuelve el PWM logico del PI. La conmutacion/sector sigue quedando del lado de Simulink. Por eso, si la velocidad aparece sinusoidal u oscilante, revisar primero `six_step_sector`, `six_step_dsw` y la alineacion entre el sector aplicado y el angulo del rotor.
