# GUI Agent Context

Objetivo: dar contexto suficiente para modificar `EscGui` con la menor lectura posible. Este archivo prioriza arquitectura, ownership, invariantes y rutas de edicion. Para explicaciones pedagogicas largas, ver `GUI_BLAZOR_WALKTHROUGH.md` y `BRIDGE_WALKTHROUGH.md`.

## 1. Lectura minima sugerida

- La mayoria de los cambios de GUI se entienden leyendo solo:
  - `src/Esc.Web/Program.cs`
  - `src/Esc.Web/Components/Pages/Home.razor`
  - `src/Esc.Bridge/EscBridgeService.cs`
  - `src/Esc.Bridge/BridgeEndpointMapper.cs`
  - `src/Esc.Web/wwwroot/controls.json`
- Si el cambio toca HID o protocolo, sumar:
  - `src/Esc.Protocol/CommConstants.cs`
  - `src/Esc.Protocol/EscProtocol.cs`
  - `src/Esc.Transport/HidSharpEscTransport.cs`
- Si el cambio toca HIL/Simulink, sumar:
  - `src/Esc.Bridge/HilUdpBridgeService.cs`
  - `src/Esc.Protocol/HilModels.cs`
- Si el cambio toca grafico o telemetria, sumar:
  - `src/Esc.Web/Components/SpeedChart.razor`
  - `src/Esc.Bridge/TelemetryStore.cs`
  - `src/Esc.Protocol/TelemetrySample.cs`
- Se puede ignorar salvo tarea especifica:
  - `Components/Layout/ReconnectModal.*`
  - `Components/Layout/MainLayout.razor*`
  - referencia a `ScottPlot.Blazor` en `Esc.Web.csproj`

## 2. Mapa mental corto

```text
Browser UI
  -> Blazor Server components
  -> EscBridgeService
  -> EscProtocol
  -> HidSharpEscTransport
  -> USB HID
  -> MCU

Simulink
  -> UDP 127.0.0.1:5055
  -> HilUdpBridgeService
  -> EscBridgeService
  -> mismo camino HID
```

Regla central: solo `EscBridgeService` decide cuando y como se usa el HID. La UI no abre HID, y Simulink tampoco.

## 3. Ownership por proyecto

- `src/Esc.Web`
  - UI Blazor, layout, pagina principal, CSS, carga de `controls.json`.
  - Usa `EscBridgeService` por inyeccion de dependencias, no por HTTP.
- `src/Esc.Bridge`
  - Fuente de verdad de estado de la aplicacion.
  - Arbitraje entre GUI, telemetria, HTTP, WebSocket y UDP.
  - Expone `BridgeSnapshot` para la UI.
- `src/Esc.Transport`
  - Enumeracion HID y lectura/escritura de reports con HidSharp.
- `src/Esc.Protocol`
  - Frame binario de 64 bytes, CRC16, opcodes, payloads y decode.
- `tests/Esc.Tests`
  - Invariantes mas importantes del bridge y protocolo.

## 4. Arranque real

- `src/Esc.Web/Program.cs`:
  - registra Razor components interactivos del lado servidor.
  - llama `AddEscBridge()`.
  - registra `ModularControlProvider`.
  - habilita WebSockets y mapea endpoints del bridge.
- `src/Esc.Bridge/EscBridgeServiceCollectionExtensions.cs` registra:
  - `IEscDeviceEnumerator -> HidSharpDeviceEnumerator`
  - `IEscTransport -> HidSharpEscTransport`
  - `TelemetryStore`
  - `EscBridgeService`
  - `EscBridgeWorker`
  - `HilUdpBridgeService`

## 5. Fuente de verdad y sincronizacion

- `EscBridgeService` mantiene el estado mutable:
  - dispositivos detectados
  - dispositivo actual
  - `EscStatus`
  - `ControlMode`
  - errores
  - estado de log de velocidad
  - estadisticas HIL
- La UI consume `Bridge.Snapshot`, que es un `record` inmutable (`BridgeSnapshot`).
- La UI se refresca porque `EscBridgeService` emite `SnapshotChanged`.
- El acceso I/O al HID esta serializado con `_ioLock`.
- El armado del snapshot y el estado interno usan `_stateGate`.

Consecuencia: si un cambio afecta estado observable, casi seguro debe terminar actualizando `Snapshot` y llamando `Notify()`.

## 6. Flujo de la UI

- `App.razor` monta `Routes` y `ReconnectModal`.
- `Routes.razor` usa `MainLayout` y enruta a paginas Razor.
- Las paginas operativas principales son `Components/Pages/Home.razor`, `Startup.razor`, las validaciones y el monitor PIL.
- `Home.razor`:
  - inyecta `EscBridgeService` y `ModularControlProvider`.
  - se suscribe a `Bridge.SnapshotChanged`.
  - renderiza:
    - conexion HID
    - modo de control
    - estado del ESC
    - comandos RUN/STOP/ESTOP
    - controles modulares desde `controls.json`
    - telemetria de velocidad
    - lazo abierto sinusoidal trifasico
    - paneles minimizables con preferencias en `localStorage`
- `Startup.razor` muestra los cinco parametros fisicos de la rampa de arranque y separa Aplicar en RAM de Guardar en flash.

## 7. Worker de fondo

- `EscBridgeWorker` corre cada 500 ms.
- En cada iteracion:
  - `ScanAsync()`
  - `ReadTelemetryOnceAsync()`
  - `RefreshStatusAsync()` si hay conexion
- Un ciclo separado de 350 ms mantiene el `KEEPALIVE` de `SINE_DRIVE` sin depender del refresh general.
- `ReadTelemetryOnceAsync()` lee una trama cuando cualquier canal esta activo; un loop dedicado de 10 ms evita acumular eventos mientras el status conserva su ciclo de 500 ms.

Consecuencia: la telemetria no es un stream continuo separado; depende del worker y del log habilitado.

## 8. Modos y permisos

- `GuiControl`
  - la GUI puede enviar `Run`, `Stop`, `SetSpeedRpm`, `SetConfig`, `ResetConfig`, `SaveConfig`.
- `SimulinkControl`
  - la GUI queda bloqueada para control real.
  - `ESTOP` sigue permitido.
  - Simulink puede enviar `Run`, `Stop`, `SetSpeedRpm`.
- `MonitorOnly`
  - sin control desde GUI ni HIL.
  - solo monitoreo/adquisicion.

Detalles importantes:

- `SetSpeedRpmAsync()` valida `GuiControl`.
- `SetSpeedRpmFromSimulinkAsync()` valida `SimulinkControl`.
- `HilStartAsync()` y `HilSetInputsAsync()` validan `mode != MonitorOnly`.
- `EmergencyStopAsync()` no depende del modo.
- `SetSineDriveAsync()` solo admite `GuiControl`; `EscBridgeWorker` envia `KEEPALIVE` en un ciclo dedicado de 350 ms y el firmware corta tras 1500 ms sin renovacion. `KEEPALIVE` nunca puede iniciar el modo desde `IDLE`.
- El panel ofrece `Manual`, que conserva un borrador hasta pulsar Aplicar, y `Dinamico`, que usa `UpdateSineDriveAsync()` con `KEEPALIVE` y sin `GET_STATUS` adicional para enviar frecuencia en cada input valido y amplitud al soltar el slider. Al usar `KEEPALIVE`, una edicion dinamica nunca puede iniciar desde `IDLE`.
- `Dinamico` nunca inicia el motor por editar estando detenido, conserva solo el ultimo valor pendiente si el HID esta ocupado y persiste la seleccion en `localStorage`.
- Cambiar fuera de `GuiControl` o desconectar detiene primero el lazo abierto; `ESTOP` conserva su camino inmediato sin `GET_STATUS` posterior.

## 9. Controles modulares

- `controls.json` define los controles que aparecen en la UI.
- `ModularControlProvider` lo carga una sola vez y lo cachea en memoria.
- Si se cambia `controls.json`, normalmente hay que reiniciar la app para ver el cambio.
- Campos del schema actual:
  - `id`, `label`, `kind`, `command`, `param`, `min`, `max`, `step`, `unit`, `defaultValue`
- `Home.razor` reconoce estos `command`:
  - `set_speed_rpm`
  - `set_config`
  - `reset_config`
  - `log_speed_start`
  - `log_speed_stop`

La persistencia en flash no viene del JSON. La UI muestra el boton guardar solo si el control usa `set_config` o `reset_config`, y luego llama `SaveConfigAsync(parameter)`.

## 10. Telemetria

- El firmware emite `TelemetryEvent`.
- `EscBridgeService.HandleFrame()` decodifica el evento y lo guarda en `TelemetryStore`.
- `TelemetryStore` conserva hasta 2000 muestras por variable.
- `BridgeSnapshot.SpeedTelemetry` trae estadisticas agregadas.
- Los canales disponibles son velocidad, `IU`, `IV` y periodo/calidad BEMF. Las corrientes incluyen cuentas ADC, flags de validez y sector de conmutacion.
- `BridgeSnapshot.StartupConfiguration` y `BridgeSnapshot.SineDrive` exponen configuracion y readback del nuevo modo.
- `Bridge.SpeedSamples` expone las muestras de `speed`.
- `SpeedChart.razor` dibuja SVG manual.

Suposiciones actuales del chart:

- usa `TargetTickMs` como eje X.
- ordena muestras por `TargetTickMs`.
- solo grafica velocidad.

## 11. HIL / Simulink

- `HilUdpBridgeService` escucha en `127.0.0.1:5055`.
- Acepta comandos discretos y CSVs de entradas HIL.
- Drena paquetes pendientes y prioriza:
  - `ESTOP` por encima de todo
  - el ultimo setpoint o ultimo paquete HIL si son coalescibles
- Mantiene:
  - `RxFrames`
  - `LostFrames`
  - `EffectiveRateHz`
  - `AverageRoundTripMs`
  - `JitterMs`
  - `LastOutputs`

Formatos relevantes:

- setpoint:
  - `SETPOINT,rpm`
  - `SETPOINT,seq,rpm`
  - aliases: `SP`, `SPEED`
- discreto:
  - `RUN`, `MOTOR_RUN`
  - `MOTOR_STOP`, `REAL_STOP`
  - `ESTOP`
  - `HIL_START`, `START`
  - `HIL_STOP`, `STOP`
- entradas HIL:
  - recomendado: `seq,speed_rpm,enable`
  - prefijado: `PIL,speed_rpm,enable`
  - prefijado con secuencia: `PIL,seq,speed_rpm,enable`
  - compatibilidad legacy: `seq,speed_rpm,reserved,load_torque,flags,enable`
  - `reserved` se ignora; Simulink simula ESC/conmutacion y el MCU solo recibe velocidad

## 12. Superficie HTTP/WebSocket

Los endpoints viven en `BridgeEndpointMapper.cs`.

- snapshot / dispositivos:
  - `GET /api/bridge/snapshot`
  - `GET /api/bridge/devices`
  - `GET /api/bridge/telemetry/speed`
- conexion y modo:
  - `POST /api/bridge/connect`
  - `POST /api/bridge/disconnect`
  - `POST /api/bridge/mode`
- control:
  - `POST /api/bridge/run`
  - `POST /api/bridge/stop`
  - `POST /api/bridge/estop`
  - `POST /api/bridge/set-speed`
  - `POST /api/bridge/sine-drive`
- config:
  - `GET /api/bridge/config/{parameter}`
  - `POST /api/bridge/config/{parameter}`
  - `POST /api/bridge/config/{parameter}/reset`
  - `POST /api/bridge/config/{parameter}/save`
- log:
  - `POST /api/bridge/log/rate`
  - `POST /api/bridge/log/speed/start`
  - `POST /api/bridge/log/speed/stop`
- HIL:
  - `POST /api/bridge/hil/start`
  - `POST /api/bridge/hil/stop`
  - `POST /api/bridge/hil/inputs`
  - `GET /api/bridge/hil/outputs`
- WebSocket:
  - `GET /ws/bridge`
  - envia `bridge.Snapshot` cada 500 ms

Importante: la pagina `Home.razor` no usa estos endpoints. Llama al bridge en memoria. Los endpoints existen para clientes externos o integracion futura.

## 13. Protocolo e HID

- `CommConstants.FrameSize = 64`
- Header de 10 bytes, CRC16 al final.
- `CommOpcode` contiene lo soportado por GUI/bridge:
  - `Ping`, `GetStatus`
  - `Run`, `Stop`, `EmergencyStop`, `SetSpeedRpm`, `SetControlMode`
  - `GetConfig`, `SetConfig`, `ResetConfig`, `SaveConfig`
  - `LogStart`, `LogStop`, `LogRate`, `TelemetryEvent`
  - `HilStart`, `HilStop`, `HilSetInputs`, `HilGetOutputs`
  - `SineDrive`
- `HidSharpEscTransport`:
  - abre por VID/PID y `DevicePath`
  - agrega report ID 0 al escribir cuando Windows lo necesita
  - normaliza reports al leer

## 14. Cambios comunes

- Agregar un boton o indicador visual:
  - casi siempre basta con `Home.razor` y, si hace falta backend, `EscBridgeService.cs`
- Agregar un control configurable nuevo desde JSON:
  - editar `controls.json`
  - si es config nueva, tambien `ConfigParam`, `EscProtocol.ConfigPayload()`, `DecodeConfigValue()`, y tal vez firmware
- Agregar un comando HID nuevo:
  - `CommOpcode`
  - payload/decode en `EscProtocol`
  - metodo en `EscBridgeService`
  - UI o endpoint que lo invoque
  - tests
- El lazo abierto usa un payload atomico de frecuencia electrica en mHz y amplitud en permille. No se modela como dos controles independientes de `controls.json`.
- Agregar telemetria nueva:
  - `LogParam`
  - decode en `EscProtocol.DecodeTelemetry()`
  - consumo en `TelemetryStore` o nueva UI
- Cambiar HIL/UDP:
  - `HilUdpBridgeService.cs`
  - si cambia payload binario, tambien `HilModels.cs` y `EscProtocol`

## 15. Trampas reales del proyecto

- `controls.json` esta cacheado; no esperar recarga en caliente.
- El paquete `ScottPlot.Blazor` esta referenciado pero el grafico real usa SVG manual.
- `SaveConfig` es separado: `SetConfig` no persiste a flash por si solo.
- `ConfigParam.CurrentLimit` y `ConfigParam.TempLimit` existen en enum, pero `EscProtocol.ConfigPayload()` no los soporta todavia.
- `Home.razor` depende de `Bridge.SnapshotChanged`; si se saltea `Notify()`, la UI queda desfasada.
- El bridge refresca estado despues de la mayoria de los comandos. Si se agrega uno nuevo, decidir si necesita `refreshStatus`.
- La lectura de telemetria comparte el mismo lock que los requests. No abrir caminos paralelos al HID desde la UI.

## 16. Tests utiles

- `tests/Esc.Tests/BridgeTests.cs`
  - valida handshake de conexion
  - valida bloqueo por modo
  - valida que `ESTOP` siga permitido
  - valida buffering de telemetria
  - valida flujo `SetConfig` + `SaveConfig`
  - valida opcodes HIL
- `tests/Esc.Tests/ProtocolTests.cs`
  - valida framing, CRC y decode del protocolo

## 17. Comando de ejecucion

```powershell
dotnet run --project .\EscGui\src\Esc.Web\Esc.Web.csproj
```

URL local esperada: `http://localhost:5187`

## 18. Regla practica para futuros agentes

Si el cambio es visual, empezar por `Home.razor`. Si el cambio es de comportamiento, casi siempre el centro real es `EscBridgeService.cs`. Solo bajar a `EscProtocol` y `Esc.Transport` cuando el cambio altera bytes, opcodes o HID.
