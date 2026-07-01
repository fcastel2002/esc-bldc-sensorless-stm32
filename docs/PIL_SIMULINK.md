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

En este repo, `simulacion_agitador/sim_motor.mdl` ya quedo configurado asi por defecto. Tambien usa `LocalAddress = 0.0.0.0` y `LocalPort = -1` para evitar choques de bind con un puerto local fijo.

El script:

1. registra el datagrama ASCII real que sale de Simulink;
2. indica si el bridge lo aceptaria con el parser actual;
3. reenvia el paquete a la GUI en `127.0.0.1:5055`;
4. registra tambien la respuesta del bridge.

Para escuchar sin reenviar, por ejemplo con la GUI cerrada:

```powershell
python .\simulacion_agitador\udp_hil_logger.py listen --listen-port 5055
```
