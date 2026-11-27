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
- Si el cambio toca curvas de carga, sumar:
  - `src/Esc.Web/Components/Pages/LoadCurves.razor`
  - `src/Esc.Protocol/LoadSweepModels.cs`
  - `src/Esc.Bridge/LoadCurveAnalysis.cs`, `src/Esc.Bridge/LoadSweepCsvWriter.cs`
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
```

Regla central: solo `EscBridgeService` decide cuando y como se usa el HID. La UI no abre HID. El puente UDP/HIL con Simulink se elimino en la spec 002.

## 3. Ownership por proyecto

- `src/Esc.Web`
  - UI Blazor, layout, pagina principal, CSS, carga de `controls.json`.
  - Usa `EscBridgeService` por inyeccion de dependencias, no por HTTP.
- `src/Esc.Bridge`
  - Fuente de verdad de estado de la aplicacion.
  - Arbitraje entre GUI, telemetria, HTTP y WebSocket.
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

## 5. Fuente de verdad y sincronizacion

- `EscBridgeService` mantiene el estado mutable:
  - dispositivos detectados
  - dispositivo actual
  - `EscStatus`
  - `ControlMode`
  - errores
  - estado de log de velocidad
  - estado del barrido de carga (`LoadSweepSnapshot`) y la curva de tara fijada en el host
- La UI consume `Bridge.Snapshot`, que es un `record` inmutable (`BridgeSnapshot`).
- La UI se refresca porque `EscBridgeService` emite `SnapshotChanged`.
- El acceso I/O al HID esta serializado con `_ioLock`.
- El armado del snapshot y el estado interno usan `_stateGate`.

Consecuencia: si un cambio afecta estado observable, casi seguro debe terminar actualizando `Snapshot` y llamando `Notify()`.

## 6. Flujo de la UI

- `App.razor` monta `Routes` y `ReconnectModal`.
- `Routes.razor` usa `MainLayout` y enruta a paginas Razor.
- Las paginas operativas principales son `Components/Pages/Home.razor` (Dashboard), `Debug.razor`, `Startup.razor` y `LoadCurves.razor` (`/curvas`). El monitor PIL y la API `/api/validation/*` se eliminaron en la spec 002.
- `MainLayout.razor` es comun a todas las paginas y concentra conexion HID, selector de modo, `ESTOP` global y la tarjeta `CONFIG MCU`.
- El tema es oscuro y sus tokens viven en `wwwroot/industrial.css` (`--bg`, `--surface*`, `--accent`, `--plot-*`, mas alias legacy `--line`, `--cyan`, ... usados por los graficos).
- `Home.razor`:
  - inyecta `EscBridgeService` y `ModularControlProvider`.
  - se suscribe a `Bridge.SnapshotChanged` y lee la config activa al detectar una conexion nueva.
  - renderiza en dos columnas:
    - principal: velocidad (`SpeedChart`) y esfuerzo de control (canales `Duty` y `LoadCurrent`)
    - lateral: RUN/STOP + consigna, estado del ESC, controles modulares de `controls.json` con botones -/+ que usan `step`
    - paneles minimizables con preferencias en `localStorage`
- `Debug.razor`:
  - paso de conmutacion fijo (`STEP_TEST`): selector de los 6 pasos six-step (UV..WV: primera fase PWM, segunda a GND, tercera flotante) con duty 0-30 %. El firmware reutiliza `commutate()` via `bldc_apply_step()`.
  - lazo abierto sinusoidal trifasico (movido desde el Dashboard).
- `Startup.razor` muestra los cinco parametros fisicos de la rampa de arranque y separa Aplicar en RAM de Guardar en flash.
- `LoadCurves.razor` configura `LOAD_SWEEP` (escalera o rampa), superpone varias curvas (la actual, guardadas e importadas de CSV), grafica I bus, duty, potencia, dispersion de velocidad o corriente de carga contra una tara, y exporta CSV.

## 7. Worker de fondo

- `EscBridgeWorker` corre cada 500 ms.
- En cada iteracion:
  - `ScanAsync()`
  - `ReadTelemetryOnceAsync()`
  - `RefreshStatusAsync()` si hay conexion
- Un ciclo separado de 350 ms mantiene el `KEEPALIVE` de `SINE_DRIVE`/`STEP_TEST`, consulta el freewheel y sigue el barrido de carga (`PollLoadSweepAsync`) sin depender del refresh general.
- `ReadTelemetryOnceAsync()` lee una trama cuando cualquier canal esta activo; un loop dedicado de 10 ms evita acumular eventos mientras el status conserva su ciclo de 500 ms.

Consecuencia: la telemetria no es un stream continuo separado; depende del worker y del log habilitado.

## 8. Modos y permisos

- `GuiControl`
  - la GUI puede enviar `Run`, `Stop`, `SetSpeedRpm`, `SetConfig`, `ResetConfig`, `SaveConfig`.
- `MonitorOnly`
  - sin control desde GUI.
  - solo monitoreo/adquisicion; `ESTOP` y `ABORT` del barrido siguen permitidos.

Detalles importantes:

- `SetSpeedRpmAsync()` valida `GuiControl`.
- `StartLoadSweepAsync()` valida `GuiControl` y el plan (rango activo, ventana, maximo 128 puntos) antes de enviar; `AbortLoadSweepAsync()` no depende del modo.
- `EmergencyStopAsync()` no depende del modo.
- `SetSineDriveAsync()` solo admite `GuiControl`; `EscBridgeWorker` envia `KEEPALIVE` en un ciclo dedicado de 350 ms y el firmware corta tras 1500 ms sin renovacion. `KEEPALIVE` nunca puede iniciar el modo desde `IDLE`.
- El panel ofrece `Manual`, que conserva un borrador hasta pulsar Aplicar, y `Dinamico`, que usa `UpdateSineDriveAsync()` con `KEEPALIVE` y sin `GET_STATUS` adicional para enviar frecuencia en cada input valido y amplitud al soltar el slider. Al usar `KEEPALIVE`, una edicion dinamica nunca puede iniciar desde `IDLE`.
- `Dinamico` nunca inicia el motor por editar estando detenido, conserva solo el ultimo valor pendiente si el HID esta ocupado y persiste la seleccion en `localStorage`.
- Cambiar fuera de `GuiControl` o desconectar detiene primero el lazo abierto; `ESTOP` conserva su camino inmediato sin `GET_STATUS` posterior.
- `SetStepTestAsync()` sigue las mismas reglas (solo `GuiControl`, `KEEPALIVE` via `MaintainStepTestAsync()` en el mismo ciclo de 350 ms, watchdog de 1500 ms en firmware). Se detiene con `STOP`/`ESTOP`, que vuelven a `IDLE`.

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
- Los canales disponibles son velocidad, tension y corriente de bus (INA219) y duty (`DisplayValue` en %). No hay sensado de corriente por fase; el firmware solo reporta magnitudes medidas.
- `BridgeSnapshot.StartupConfiguration`, `BridgeSnapshot.SineDrive` y `BridgeSnapshot.StepTest` exponen configuracion y readback de los modos manuales.
- `Bridge.SpeedSamples` expone las muestras de `speed`; `Bridge.GetTelemetrySamples(LogParam)` el resto de los canales.
- `SpeedChart.razor` dibuja SVG manual (velocidad) y reutiliza `js/speed-chart.js` para cursores y descarga de CSV. `LoadCurves.razor` dibuja su propio SVG (rpm en X).

Suposiciones actuales del chart:

- usa `TargetTickMs` como eje X.
- ordena muestras por `TargetTickMs`.
- solo grafica velocidad.

## 11. Curvas de carga

- El firmware ejecuta el barrido (`LOAD_SWEEP`) y guarda hasta 128 puntos promediados en RAM; el bridge copia los puntos nuevos con `GET_POINTS` (3 por trama).
- `StartLoadSweepAsync()` borra los puntos del host; `SaveCurrentLoadCurve()` e `ImportLoadCurveCsv()` agregan curvas guardadas y `SetLoadCurveTare()` elige la tara (nada de esto viaja al MCU).
- `LoadCurveAnalysis.Subtract()` y `LoadCurrentVsTare()` interpolan la tara linealmente y nunca extrapolan fuera de su rango de rpm. La corriente de carga es `(I_bus − I_bus,tara)/d`.
- `LoadSweepCsvWriter` genera el CSV con una linea `#` de metadatos; `LoadSweepCsvReader` lo relee (tambien el formato viejo con `load_ma`).

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
  - `POST /api/bridge/step-test`
- config:
  - `GET /api/bridge/config/{parameter}`
  - `POST /api/bridge/config/{parameter}`
  - `POST /api/bridge/config/{parameter}/reset`
  - `POST /api/bridge/config/{parameter}/save`
- log:
  - `POST /api/bridge/log/rate`
  - `POST /api/bridge/log/speed/start`
  - `POST /api/bridge/log/speed/stop`
- curvas de carga:
  - `GET /api/bridge/load-sweep` (refresca estado y puntos)
  - `POST /api/bridge/load-sweep/start` (`LoadSweepRequest`)
  - `POST /api/bridge/load-sweep/abort`
  - `GET /api/bridge/load-sweep/csv`
- config del controlador:
  - `GET /api/bridge/controller-config`
- WebSocket:
  - `GET /ws/bridge`
  - envia `bridge.Snapshot` cada 500 ms

Importante: la pagina `Home.razor` no usa estos endpoints. Llama al bridge en memoria. Los endpoints existen para clientes externos o integracion futura.

## 13. Protocolo e HID

- `CommConstants.FrameSize = 64`
- Header de 10 bytes, CRC16 al final.
- `CommOpcode` contiene lo soportado por GUI/bridge:
  - `Ping`, `GetStatus`
  - `Run`, `Stop`, `EmergencyStop`, `SetSpeedRpm`
  - `GetConfig`, `SetConfig`, `ResetConfig`, `SaveConfig`
  - `LogStart`, `LogStop`, `LogRate`, `TelemetryEvent`
  - `SineDrive`, `StepTest`, `ZcDiag`
  - `LoadSweep`
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
- Cambiar el barrido de carga:
  - firmware `load_sweep.c` y `comm_protocol.c`
  - `LoadSweepModels.cs` y `EscProtocol` si cambian bytes
  - `EscBridgeService` (`StartLoadSweepAsync`, `RefreshLoadSweepAsync`) y `LoadCurves.razor`

## 15. Trampas reales del proyecto

- `controls.json` esta cacheado; no esperar recarga en caliente.
- El paquete `ScottPlot.Blazor` esta referenciado pero el grafico real usa SVG manual.
- `SaveConfig` es separado: `SetConfig` no persiste a flash por si solo.
- `ConfigParam.SpeedRamp` (0 = sin limite) se aplica en caliente; persistirla requiere `SaveConfig` como el resto.
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
  - valida el barrido de carga (payload, drenado de puntos, validacion local, tara)
- `tests/Esc.Tests/LoadCurveAnalysisTests.cs`
  - valida curvas, interpolacion de tara y CSV
- `tests/Esc.Tests/ProtocolTests.cs`
  - valida framing, CRC y decode del protocolo

## 17. Comando de ejecucion

```powershell
dotnet run --project .\EscGui\src\Esc.Web\Esc.Web.csproj
```

URL local esperada: `http://localhost:5187`

## 18. Regla practica para futuros agentes

Si el cambio es visual, empezar por `Home.razor`. Si el cambio es de comportamiento, casi siempre el centro real es `EscBridgeService.cs`. Solo bajar a `EscProtocol` y `Esc.Transport` cuando el cambio altera bytes, opcodes o HID.
