# Flujo actual del proyecto

Este documento resume como esta conectado el firmware actual del ESC BLDC/PMSM sensorless y donde mirar cada parte del codigo. La idea es que puedas volver al proyecto sin tener que reconstruir todo mentalmente desde cero.

## 1. Punto de entrada

El arranque empieza en [`main()`](../firmware/Core/Src/main.c#L67).

La secuencia actual es:

1. `HAL_Init()` inicializa HAL, SysTick y la base del micro.
2. [`SystemClock_Config()`](../firmware/Core/Src/main.c#L129) configura el STM32F103 con HSE + PLL a 72 MHz y deja preparado el clock USB.
3. Se inicializan perifericos generados por CubeMX:
   - [`MX_GPIO_Init()`](../firmware/Core/Src/gpio.c#L42)
   - [`MX_TIM4_Init()`](../firmware/Core/Src/tim.c#L225)
   - [`MX_TIM2_Init()`](../firmware/Core/Src/tim.c#L111)
   - [`MX_TIM1_Init()`](../firmware/Core/Src/tim.c#L33)
   - [`MX_CRC_Init()`](../firmware/Core/Src/crc.c#L30)
   - [`MX_TIM3_Init()`](../firmware/Core/Src/tim.c#L172)
4. Se ajustan prescalers de timers, se habilitan capturas por interrupcion en TIM2 y se llama a [`detect_motor()`](../firmware/Core/Src/motor_control.c#L172).
5. El `while (1)` ejecuta continuamente:
   - [`handleState()`](../firmware/Core/Src/state_machine.c#L16)
   - [`check_motor_status()`](../firmware/Core/Src/motor_control.c#L217)
   - parpadeo del LED de estado cada 200 ms usando `led_tick`.

En esta version, `main()` no inicializa UART ni USB directamente. Eso queda delegado a la capa de comunicacion cuando la maquina de estados llama a `commInit()`.

## 2. Maquina de estados

La variable global [`app_state`](../firmware/Core/Src/state_machine.c#L10) representa el estado de la aplicacion. Los estados estan definidos en [`App_States_t`](../firmware/Core/Inc/state_machine.h#L22):

| Estado | Rol actual |
| --- | --- |
| `IDLE` | Motor detenido, recibe comandos cada 50 ms. |
| `FOC_STARTUP` | Arranque sinusoidal open-loop. |
| `RUNNING` | Conmutacion six-step sensorless, esperando cruces por cero consistentes. |
| `READY` | Estado puente: configura TIM4 para ejecutar control PI. |
| `CLOSEDLOOP` | Control de velocidad en lazo cerrado. |
| `CONFIG` | Aplica parametros nuevos y actualiza dependencias cuando el cambio no se puede hacer en caliente. |
| `STOPPED` | Detiene interrupciones de control. |
| `HARD_ERROR` | Entra a `Error_Handler()`. |

[`handleState()`](../firmware/Core/Src/state_machine.c#L12) hace inicializaciones perezosas: primero llama a [`commInit()`](../firmware/Core/Src/comm.c#L30), despues carga configuracion persistente con [`flash_config_init()`](../firmware/Core/Src/flash_config.c#L157). Si no hay configuracion valida, se cargan defaults y se marcan cambios pendientes.

Cuando la configuracion esta lista, se aplican dos capas:

- [`update_all_esc()`](../firmware/Core/Src/hard_config.c#L43): actualiza parametros base del ESC, sobre todo `TIM1->ARR` segun frecuencia PWM.
- [`updateAllMotorControl()`](../firmware/Core/Src/motor_control.c#L105): recalcula limites PWM, setpoint inicial y ganancias del control de velocidad.

## 3. Comunicacion con el host

La fachada publica esta en [`comm.c`](../firmware/Core/Src/comm.c). Hoy ya no interpreta comandos ASCII: delega en el protocolo binario.

Flujo:

1. [`commInit()`](../firmware/Core/Src/comm.c#L30) limpia buffers y llama a [`comm_transport_init()`](../firmware/Core/Src/comm_transport.c#L32).
2. [`comm_transport_init()`](../firmware/Core/Src/comm_transport.c#L32) lee `PB8` (`COMM_MODE`):
   - `PB8` alto o abierto: USB FS Custom HID.
   - `PB8` bajo: UART por USART1.
3. En UART, [`HAL_UART_RxCpltCallback()`](../firmware/Core/Src/comm_transport.c#L119) llena un buffer circular byte a byte.
4. En USB, [`CustomHID_OutReport()`](../firmware/Core/Src/usbd_customhid_if.c#L42) entrega reports de 64 bytes a [`comm_transport_receive_usb_report()`](../firmware/Core/Src/comm_transport.c#L108).
5. [`processUartData()`](../firmware/Core/Src/comm.c#L39) llama a [`comm_transport_process()`](../firmware/Core/Src/comm_transport.c#L87), que arma frames y los pasa a [`comm_protocol_handle_frame()`](../firmware/Core/Src/comm_protocol.c#L374).

El protocolo esta definido en [`comm_protocol.h`](../firmware/Core/Inc/comm_protocol.h) y explicado en [`firmware/COMM_PROTOCOL.md`](../firmware/COMM_PROTOCOL.md). Cada request/response/event ocupa 64 bytes, con magic `0xEC 0xB1`, version, opcode, parametro, payload y CRC16.

Comandos principales implementados en [`execute_request()`](../firmware/Core/Src/comm_protocol.c#L293):

- `PING`: eco de payload.
- `GET_STATUS`: devuelve estado, transporte, flags del motor, setpoint, velocidad y `max_pwm`.
- `RUN`: llama a [`foc_startup()`](../firmware/Core/Src/startup.c#L94).
- `STOP` / `ESTOP`: llaman a [`stop_motor()`](../firmware/Core/Src/motor_control.c#L310). `ESTOP` corta PWM inmediatamente, no bloquea el procesamiento de comunicaciones y se acepta en cualquier estado operativo.
- `SET_SPEED_RPM`: cambia `speed_setpoint_rpm`.
- `SINE_DRIVE`: entra o actualiza el lazo abierto senoidal continuo con frecuencia electrica y amplitud atomicas; `STOP` sale del modo y un watchdog de 1.5 s cubre la perdida del host.
- `GET_CONFIG`, `SET_CONFIG`, `RESET_CONFIG`: leen, modifican o restauran parametros.
  En `CLOSEDLOOP`, `PWM_FREQ`, `KP`, `KI` y `KD` se aplican en caliente sin salir del lazo cerrado; el resto sigue pasando por `CONFIG`.
- `LOG_START`, `LOG_STOP`, `LOG_RATE`: controlan telemetria periodica.

Los scripts de prueba del host estan en [`gui/`](../gui):

- [`protocol.py`](../gui/protocol.py): arma y parsea frames.
- [`prueba_uart.py`](../gui/prueba_uart.py): prueba `PING` y `GET_STATUS` por COM.
- [`prueba_usb.py`](../gui/prueba_usb.py): prueba HID con VID/PID actual.

## 4. Configuracion persistente

Los parametros vivos del ESC estan en [`current_esc_params`](../firmware/Core/Src/hard_config.c#L21), con tipo [`ESCparams`](../firmware/Core/Inc/hard_config.h#L34).

Defaults principales en [`set_default_esc_params()`](../firmware/Core/Src/hard_config.c#L27):

- PWM: 18 kHz.
- PI RPM v2: `KP = 0.28`, `KI = 1.00`, `KD = 0.0`.
- velocidad maxima: 5400 RPM.
- velocidad minima: 200 RPM.
- pares de polos: 2.
- arranque: amplitud `20% -> 100%`, frecuencia electrica `2.09 -> 9.28 Hz`, duracion 3 s.

La persistencia esta en [`flash_config.c`](../firmware/Core/Src/flash_config.c). Usa dos paginas rotativas al final de Flash, definidas por [`FLASH_CONFIG_START`](../firmware/Core/Inc/flash_config.h#L13). [`flash_config_init()`](../firmware/Core/Src/flash_config.c) busca una pagina valida por firma y CRC; [`flash_config_save()`](../firmware/Core/Src/flash_config.c) escribe en la otra pagina para repartir desgaste. La configuracion V3 amplia el bloque con cinco parametros fisicos de arranque; bloques V1/V2 se validan con su layout original y se migran sin perder los campos existentes.

Cuando un comando modifica parametros, las funciones `set_*` de [`hard_config.c`](../firmware/Core/Src/hard_config.c) llaman a [`flash_config_parameter_changed()`](../firmware/Core/Src/flash_config.c#L257). Si el cambio llega durante `CLOSEDLOOP` y solo afecta `PWM_FREQ`, `KP`, `KI` o `KD`, el firmware recalcula runtime al instante y mantiene el motor girando. El resto de parametros sigue entrando por `CONFIG`, que fuerza recalculo de timers/control y deja el guardado pendiente hasta `SAVE_CONFIG`.

## 5. Arranque del motor

El arranque activo esta en [`startup.c`](../firmware/Core/Src/startup.c).

[`foc_startup()`](../firmware/Core/Src/startup.c#L94) no hace FOC completo con realimentacion de corriente; usa una rampa sinusoidal open-loop:

1. Genera tablas senoidales U/V/W con [`generate_sine_tables()`](../firmware/Core/Src/startup.c#L26).
2. Lee amplitud inicial/final, frecuencia electrica inicial/final y duracion desde la configuracion activa.
3. Configura TIM4 desde la frecuencia fisica inicial y fuerza la carga de PSC/ARR para no heredar el periodo anterior.
4. Habilita PWM en TIM1 y los enable de las fases.
5. Pone `app_state = FOC_STARTUP`.

Cada update de TIM4 llama a [`HAL_TIM_PeriodElapsedCallback()`](../firmware/Core/Src/isr_callbacks.c), que a su vez llama a [`update_pwm_startup_foc()`](../firmware/Core/Src/startup.c). Esa funcion avanza la fase de las tablas e interpola linealmente amplitud y frecuencia segun tiempo transcurrido. Al completar la duracion configurada detiene la senoidal y pasa a six-step.

El handoff detiene la senoidal, deja un duty inicial de 45%, prepara six-step y pasa a `RUNNING`.

Si se detecta `motor_stalled`, o si `RUNNING` dura mas de 4.5 s sin pasar a `READY`, [`handleState()`](../firmware/Core/Src/state_machine.c) corta el PWM durante 0.5 s y relanza [`foc_startup()`](../firmware/Core/Src/startup.c). Este timeout no depende del detector de stall; funciona como watchdog directo del enganche sensorless.

El mismo generador soporta `SINE_DRIVE`, un estado adicional sin handoff automatico. En ese estado aplica una frecuencia electrica fija de 2 a 10 Hz y una amplitud de 0 a 100%, ignora BEMF para conmutacion y exige que el bridge renueve el comando en un ciclo dedicado de 350 ms. Si pasan 1500 ms sin renovacion, el firmware deshabilita las tres fases y vuelve a `IDLE`.

## 6. Conmutacion sensorless

La conmutacion fisica esta en [`bldc_driver.c`](../firmware/Core/Src/bldc_driver.c).

[`bldc_commutate()`](../firmware/Core/Src/bldc_driver.c#L148) avanza o retrocede el paso segun `direction` y llama a la funcion interna `commutate(step)`. Cada paso:

- energiza dos fases;
- deja una fase flotante;
- configura la polaridad de captura en TIM2 para detectar el proximo cruce por cero;
- actualiza flags `floating_U`, `floating_V`, `floating_W`.

La ISR entra por [`TIM2_IRQHandler()`](../firmware/Core/Src/stm32f1xx_it.c#L207), HAL llama a [`HAL_TIM_IC_CaptureCallback()`](../firmware/Core/Src/isr_callbacks.c#L12), y esta funcion solo acepta el canal cuya fase esta flotante. Si corresponde, llama a [`zero_crossing_handler()`](../firmware/Core/Src/motor_control.c#L152).

[`zero_crossing_handler()`](../firmware/Core/Src/motor_control.c#L152) tiene dos tareas:

- conmutar al siguiente paso mediante la capa BLDC;
- alimentar el sensado de velocidad en [`speed_sensor.c`](../firmware/Core/Src/speed_sensor.c).

## 7. Medicion de velocidad

La medicion vive en [`speed_sensor.c`](../firmware/Core/Src/speed_sensor.c).

Hay dos niveles:

- Medicion por fase con [`speed_sensor_process_phase_measurement()`](../firmware/Core/Src/speed_sensor.c#L70): calcula periodos entre cruces, maneja overflow de timer de 16 bits y valida consistencia.
- Consenso trifasico con [`calculate_consensus_speed()`](../firmware/Core/Src/speed_sensor.c#L147): combina fases consistentes y marca si la medicion es confiable.

La fase W tiene tratamiento especial en [`speed_sensor_handle_W_measurement()`](../firmware/Core/Src/speed_sensor.c#L305), porque se usa para decidir si ya hay cruces por cero consistentes. Cuando el criterio se cumple, pone `consistent_zero_crossing = 1`, lo que permite pasar de `RUNNING` a `READY` en la maquina de estados.

En `CLOSEDLOOP`, [`speed_sensor_handle_consensus()`](../firmware/Core/Src/speed_sensor.c#L352) actualiza `filtered_speed`. Si el consenso no esta listo, usa un fallback con captura simple de TIM2 CH1.

Conversiones importantes:

- [`rpm_to_period()`](../firmware/Core/Src/speed_sensor.c#L229)
- [`period_to_rpm()`](../firmware/Core/Src/speed_sensor.c#L251)

## 8. Control de velocidad

Cuando hay cruces por cero consistentes, `handleState()` pasa por `READY`, configura TIM4 y entra a `CLOSEDLOOP`.

En `CLOSEDLOOP`, TIM4 CH1 dispara [`HAL_TIM_OC_DelayElapsedCallback()`](../firmware/Core/Src/isr_callbacks.c#L34). Si el canal activo es TIM4 CH1, se llama a [`pi_control()`](../firmware/Core/Src/motor_control.c#L274).

[`pi_control()`](../firmware/Core/Src/motor_control.c#L274):

1. obtiene RPM mecanicas sin limitarlas por el rango del setpoint;
2. calcula `error_rpm = setpoint_rpm - measured_rpm`;
3. ejecuta el PI trapezoidal con estado Q16.16 y ganancias en cuentas canonicas referidas a ARR=2000;
4. aplica anti-windup entre 100 y 2000 cuentas canonicas;
5. escala una sola vez al ARR activo y escribe el duty con [`bldc_set_pwm()`](../firmware/Core/Src/bldc_driver.c#L157).

`KD` permanece bloqueado en cero en algoritmo v2. Al entrar en `CLOSEDLOOP`,
el integrador se precarga para conservar el PWM de handoff. Si la medicion
fisica aun no es valida, el PI mantiene esa salida; en HIL espera la primera
entrada habilitada.

El motor se monitorea con [`check_motor_status()`](../firmware/Core/Src/motor_control.c#L217). Si en `RUNNING` o `CLOSEDLOOP` no llegan cruces por cero por mas de `TIMEOUT_MOTOR_STALL_MS`, se marca `motor_stalled`; la maquina de estados responde reintentando [`foc_startup()`](../firmware/Core/Src/startup.c#L94).

## 9. Mapa rapido de archivos

| Archivo | Responsabilidad |
| --- | --- |
| [`firmware/Core/Src/main.c`](../firmware/Core/Src/main.c) | Entrada, clocks, perifericos base, loop principal. |
| [`firmware/Core/Src/state_machine.c`](../firmware/Core/Src/state_machine.c) | Estados de alto nivel y coordinacion del firmware. |
| [`firmware/Core/Src/comm.c`](../firmware/Core/Src/comm.c) | Fachada de comunicacion y telemetria periodica. |
| [`firmware/Core/Src/comm_transport.c`](../firmware/Core/Src/comm_transport.c) | Seleccion UART/USB y transporte de frames. |
| [`firmware/Core/Src/comm_protocol.c`](../firmware/Core/Src/comm_protocol.c) | Protocolo binario, CRC, opcodes y efectos de comandos. |
| [`firmware/Core/Src/hard_config.c`](../firmware/Core/Src/hard_config.c) | Parametros del ESC, limites, getters/setters y actualizacion de TIM1. |
| [`firmware/Core/Src/flash_config.c`](../firmware/Core/Src/flash_config.c) | Persistencia en Flash con paginas rotativas y CRC. |
| [`firmware/Core/Src/startup.c`](../firmware/Core/Src/startup.c) | Generador SPWM comun, arranque configurable, modo senoidal continuo y transicion a six-step. |
| [`firmware/Core/Src/bldc_driver.c`](../firmware/Core/Src/bldc_driver.c) | Conmutacion de seis pasos y duty PWM aplicado. |
| [`firmware/Core/Src/motor_control.c`](../firmware/Core/Src/motor_control.c) | Integracion del PI RPM, manejo de stall y zero-crossing handler. |
| [`firmware/Core/Src/rpm_pi_controller.c`](../firmware/Core/Src/rpm_pi_controller.c) | Nucleo PI RPM Q16.16, anti-windup y escalado de ARR. |
| [`firmware/Core/Src/speed_sensor.c`](../firmware/Core/Src/speed_sensor.c) | Medicion de velocidad por cruces por cero y consenso trifasico. |
| [`firmware/Core/Src/isr_callbacks.c`](../firmware/Core/Src/isr_callbacks.c) | Callbacks HAL para captura, output compare y update de timers. |
| [`firmware/Core/Src/stm32f1xx_it.c`](../firmware/Core/Src/stm32f1xx_it.c) | Handlers de interrupcion generados y ruteo hacia HAL. |
| [`gui/protocol.py`](../gui/protocol.py) | Utilidades Python para construir, parsear y mostrar frames binarios. |

## 10. Camino mental completo

Una ejecucion normal queda asi:

```text
main()
  -> inicializa HAL, clocks, GPIO, timers, CRC
  -> habilita captura TIM2 para BEMF
  -> detect_motor()
  -> while:
       handleState()
         -> commInit()
         -> flash_config_init()
         -> update_all_esc()
         -> updateAllMotorControl()
         -> procesa comandos
         -> cambia estados
       check_motor_status()

Host RUN
  -> comm_transport_process()
  -> comm_protocol_handle_frame()
  -> execute_request(RUN)
  -> foc_startup()
  -> update_pwm_startup_foc() por TIM4
  -> executeTransition()
  -> RUNNING

Cruces por cero
  -> TIM2_IRQHandler()
  -> HAL_TIM_IC_CaptureCallback()
  -> zero_crossing_handler()
  -> bldc_commutate()
  -> speed_sensor_*
  -> consistent_zero_crossing = 1
  -> READY
  -> CLOSEDLOOP
  -> pi_control() por TIM4 CH1
  -> bldc_set_pwm()
```

## 11. Particularidades a recordar

- El modo de comunicacion se decide al boot con `PB8`; no cambia dinamicamente.
- El protocolo actual es binario de 64 bytes; `COMM_PROTOCOL.md` es la referencia del frame.
- `comm.c` conserva nombres historicos como `processUartData()`, pero por debajo tambien procesa USB.
- La configuracion se guarda en Flash solo cuando hay cambios pendientes.
- El arranque llamado `foc_startup()` es senoidal open-loop, no FOC completo con lazo de corriente.
- El pasaje a lazo cerrado depende de `consistent_zero_crossing`.
- El control PI actua ajustando duty, no cambiando directamente la secuencia de conmutacion.
- Temperatura y corriente en telemetria todavia son placeholders.
