# Walkthrough del ESC Bridge en C#

Esta guía explica cómo funciona el Bridge de la GUI, asumiendo que no tenés experiencia previa en C#.

La idea no es solo decir "qué archivo hace qué", sino recorrer el código como si siguiéramos un paquete desde la GUI o Simulink hasta el microcontrolador.

## 1. Qué problema resuelve el Bridge

El microcontrolador se comunica por USB HID. Windows permite que varias aplicaciones vean el dispositivo, pero en la práctica no conviene que la GUI, Simulink, scripts de prueba y otras herramientas intenten abrir el HID al mismo tiempo.

Por eso se creó el Bridge:

```text
GUI Blazor
    |
    | HTTP / llamadas internas C#
    v
ESC Bridge
    |
    | USB HID
    v
MCU

Simulink
    |
    | UDP localhost:5055
    v
ESC Bridge
    |
    | USB HID
    v
MCU
```

El Bridge es el único dueño del USB HID. Todo lo demás le pide cosas al Bridge.

Esto deja varias ventajas:

- La GUI no necesita conocer detalles de HID.
- Simulink no compite contra la GUI por el dispositivo USB.
- El protocolo binario queda centralizado.
- Se puede agregar TCP, UDP, WebSocket o HTTP sin cambiar el firmware cada vez.
- Se puede aplicar lógica de seguridad: `ESTOP` siempre permitido, modos de control, bloqueo en `Monitor only`, etc.

## 2. Capas del proyecto

La GUI está dividida en capas:

```text
gui/EscGui/src/Esc.Protocol
    Define frames, opcodes, CRC, payloads y modelos del protocolo.

gui/EscGui/src/Esc.Transport
    Abre el USB HID usando HidSharp.

gui/EscGui/src/Esc.Bridge
    Coordina estado, comandos, telemetría, HTTP, UDP y arbitraje.

gui/EscGui/src/Esc.Web
    Interfaz Blazor visible en el navegador.
```

La regla de diseño fue:

```text
Esc.Web no habla HID.
Esc.Bridge decide qué hacer.
Esc.Transport abre el HID.
Esc.Protocol arma y parsea bytes.
```

## 3. Archivos principales del Bridge

Los archivos más importantes son:

```text
EscBridgeService.cs
```

Es el corazón del Bridge. Guarda el estado actual, conecta/desconecta el HID, manda comandos, recibe telemetría y protege el acceso al USB.

```text
HilUdpBridgeService.cs
```

Escucha UDP en `127.0.0.1:5055`. Es el camino que usa Simulink.

```text
BridgeEndpointMapper.cs
```

Expone endpoints HTTP locales como `/api/bridge/connect`, `/api/bridge/run`, `/api/bridge/set-speed`, etc.

```text
EscBridgeWorker.cs
```

Es una tarea de fondo que escanea el dispositivo, lee telemetría y refresca estado periódicamente.

```text
TelemetryStore.cs
```

Guarda muestras de telemetría para que la GUI pueda graficar velocidad.

```text
BridgeSnapshot.cs
```

Define una "foto" del estado actual: conexión, modo, último error, estado del motor, estadísticas HIL, telemetría, etc.

```text
CommandResult.cs
```

Representa el resultado de un comando: éxito, error, mensaje y status devuelto por el ESC.

## 4. Cómo arranca todo

El punto de entrada de la web es:

```text
gui/EscGui/src/Esc.Web/Program.cs
```

La línea clave es:

```csharp
builder.Services.AddEscBridge();
```

### Qué significa `builder.Services`

En aplicaciones ASP.NET / Blazor, `builder.Services` es el contenedor de servicios.

Un "servicio" es un objeto que la aplicación crea y reutiliza. En lugar de hacer:

```csharp
var bridge = new EscBridgeService(...);
```

en cada lugar donde lo necesitamos, se registra una vez y Blazor lo entrega automáticamente.

Esto se llama inyección de dependencias.

### Qué hace `AddEscBridge()`

Está en:

```text
gui/EscGui/src/Esc.Bridge/EscBridgeServiceCollectionExtensions.cs
```

El código registra:

```csharp
services.AddSingleton<IEscDeviceEnumerator, HidSharpDeviceEnumerator>();
services.AddSingleton<IEscTransport, HidSharpEscTransport>();
services.AddSingleton<TelemetryStore>();
services.AddSingleton<EscBridgeService>();
services.AddHostedService<EscBridgeWorker>();
services.AddHostedService<HilUdpBridgeService>();
```

Explicación:

- `AddSingleton<T>()`: crea una sola instancia para toda la aplicación.
- `IEscDeviceEnumerator`: interfaz para buscar dispositivos HID.
- `HidSharpDeviceEnumerator`: implementación real usando HidSharp.
- `IEscTransport`: interfaz para hablar con el HID.
- `HidSharpEscTransport`: implementación real del transporte HID.
- `EscBridgeService`: servicio central del Bridge.
- `AddHostedService`: registra una tarea que corre en segundo plano.

### Qué es una interfaz

Una interfaz en C# define un contrato.

Por ejemplo:

```csharp
public interface IEscTransport
{
    Task OpenAsync(...);
    Task CloseAsync(...);
    Task WriteFrameAsync(...);
    Task<EscFrame?> ReadFrameAsync(...);
}
```

Eso significa: "cualquier clase que diga que implementa `IEscTransport` debe tener estos métodos".

El Bridge usa la interfaz, no la clase concreta. Eso permite reemplazar el transporte real por uno falso en tests.

## 5. Glosario rápido de C# usado en el Bridge

### `using`

Ejemplo:

```csharp
using Esc.Protocol;
using Esc.Transport;
```

Le dice al archivo que vamos a usar tipos definidos en otros espacios de nombres. Es parecido a un `#include`, pero no copia código; solo permite escribir nombres más cortos.

### `namespace`

Ejemplo:

```csharp
namespace Esc.Bridge;
```

Agrupa clases. Sirve para ordenar el código y evitar choques de nombres.

### `public`, `private`, `protected`

Controlan quién puede usar algo.

- `public`: accesible desde otros archivos/proyectos.
- `private`: solo dentro de la misma clase.
- `protected`: accesible desde la clase y clases derivadas.

En el Bridge, casi todo lo interno es `private` para que otros componentes no puedan modificar el estado sin pasar por métodos controlados.

### `class`

Define un tipo con datos y comportamiento.

```csharp
public sealed class EscBridgeService
```

`EscBridgeService` es una clase porque tiene estado interno y métodos.

### `sealed`

```csharp
public sealed class EscBridgeService
```

Significa que nadie puede heredar de esa clase.

Lo usé porque no necesitamos extender estas clases por herencia. Eso simplifica el diseño.

### `record`

Ejemplo:

```csharp
public sealed record BridgeSnapshot(...);
```

Un `record` es ideal para modelos de datos. C# genera automáticamente constructor, comparación, propiedades y algunas utilidades.

Lo usé para objetos como `BridgeSnapshot`, `EscStatus`, `HilInputs` y `HilOutputs`, porque representan datos.

### `enum`

Ejemplo:

```csharp
public enum ControlMode
{
    GuiControl,
    SimulinkControl,
    MonitorOnly,
}
```

Un `enum` es una lista cerrada de valores posibles.

En lugar de usar números sueltos como `0`, `1`, `2`, el código usa nombres claros.

### `Task` y `async`

Ejemplo:

```csharp
public async Task<CommandResult> ConnectAsync(...)
```

`Task` representa una operación que puede tardar.

`async` permite usar `await` dentro del método.

Se usa mucho porque HID, UDP, HTTP y timers son operaciones de entrada/salida. No queremos congelar la GUI esperando una respuesta USB.

### `await`

Ejemplo:

```csharp
await _transport.OpenAsync(descriptor, cancellationToken);
```

Le dice a C#: "esperá a que esta operación termine, pero sin bloquear el hilo completo".

Esto es importante en una app web, porque puede haber más cosas ocurriendo al mismo tiempo.

### `CancellationToken`

Ejemplo:

```csharp
CancellationToken cancellationToken = default
```

Es un mecanismo para cancelar operaciones. Por ejemplo, si la app se está cerrando, el token avisa a tareas de fondo que deben terminar.

### `?`

Ejemplo:

```csharp
private HidDeviceDescriptor? _currentDevice;
```

El `?` indica que la variable puede ser `null`.

`null` significa "no hay valor".

En este caso, `_currentDevice` puede ser `null` cuando no hay dispositivo conectado.

### `??`

Ejemplo:

```csharp
return _stream ?? throw new InvalidOperationException(...);
```

Significa: si lo de la izquierda no es `null`, usalo. Si es `null`, usá lo de la derecha.

### `=>`

Ejemplo:

```csharp
public IReadOnlyList<TelemetrySample> SpeedSamples => _telemetryStore.GetSamples("speed");
```

Es una forma corta de escribir una propiedad o función simple.

### `lock`

Ejemplo:

```csharp
lock (_stateGate)
{
    _status = status;
}
```

Evita que dos hilos modifiquen el mismo estado al mismo tiempo.

El Bridge tiene tareas de fondo, GUI, HTTP y UDP. Varias partes pueden leer/escribir estado. `lock` protege esos datos.

### `SemaphoreSlim`

Ejemplo:

```csharp
private readonly SemaphoreSlim _ioLock = new(1, 1);
```

Es una especie de candado asíncrono.

El HID debe manejarse de a una transacción por vez. No queremos que mientras un método espera respuesta de `GET_STATUS`, otro mande `SET_SPEED_RPM` y mezcle respuestas.

Por eso se usa `_ioLock`.

### `try`, `catch`, `finally`

Ejemplo:

```csharp
try
{
    ...
}
catch (Exception ex)
{
    ...
}
finally
{
    _ioLock.Release();
}
```

- `try`: intentá ejecutar esto.
- `catch`: si hubo error, manejalo acá.
- `finally`: ejecutá esto siempre, haya error o no.

En el Bridge, `finally` es clave para liberar `_ioLock`.

### `ConfigureAwait(false)`

Ejemplo:

```csharp
await SendCommandAsync(...).ConfigureAwait(false);
```

Es una optimización/patrón común en librerías C#. Indica que después del `await` no hace falta volver al mismo contexto de ejecución.

No cambia la lógica del Bridge; solo evita dependencias innecesarias del contexto de UI.

### `with`

Ejemplo:

```csharp
_status = _status with { SpeedSetpointRpm = rpm };
```

Los `record` son inmutables por estilo. `with` crea una copia modificando solo algunas propiedades.

En este caso: "creá un nuevo `EscStatus` igual al anterior, pero con otro `SpeedSetpointRpm`".

## 6. El estado central: `BridgeSnapshot`

El Bridge no expone todas sus variables internas directamente. En cambio expone una foto de estado:

```csharp
public BridgeSnapshot Snapshot
{
    get
    {
        lock (_stateGate)
        {
            return BuildSnapshot();
        }
    }
}
```

Qué significa:

- `public`: la GUI puede pedirlo.
- `BridgeSnapshot`: el tipo que devuelve.
- `get`: esto es una propiedad de solo lectura.
- `lock (_stateGate)`: mientras arma la foto, nadie puede cambiar el estado.
- `BuildSnapshot()`: junta todo el estado actual en un solo objeto.

`BridgeSnapshot` contiene:

```csharp
DeviceConnectionState State
ControlMode Mode
IReadOnlyList<HidDeviceDescriptor> Devices
HidDeviceDescriptor? CurrentDevice
EscStatus? Status
string? LastError
TelemetryStats SpeedTelemetry
HilBridgeStats Hil
```

La GUI usa este objeto para mostrar:

- Si el HID está conectado.
- Qué dispositivo se detectó.
- Último error.
- Estado del motor.
- Setpoint.
- Velocidad medida.
- Estadísticas de HIL/Simulink.
- Muestras de telemetría.

## 7. Estados de conexión

Están definidos en:

```text
BridgeEnums.cs
```

```csharp
public enum DeviceConnectionState
{
    NotDetected,
    Detected,
    Connecting,
    Connected,
    Error,
    Disconnected,
}
```

La idea es separar varias situaciones:

- `NotDetected`: no se encontró ningún HID con VID/PID esperado.
- `Detected`: Windows ve el dispositivo, pero aún no se validó con `PING`.
- `Connecting`: se está intentando abrir.
- `Connected`: el HID abrió y respondió.
- `Error`: hubo un problema.
- `Disconnected`: estaba conectado pero desapareció.

Esto es importante porque "Windows ve un HID" no alcanza. El Bridge recién considera válido al MCU cuando responde `PING` y `GET_STATUS`.

## 8. Modos de control

También están en:

```text
BridgeEnums.cs
```

```csharp
public enum ControlMode
{
    GuiControl,
    SimulinkControl,
    MonitorOnly,
}
```

### `GuiControl`

La GUI puede mandar comandos como `RUN`, `STOP`, `SET_SPEED_RPM`, configs, etc.

### `SimulinkControl`

Simulink puede mandar comandos reales por UDP, por ejemplo:

```text
SETPOINT,1,1500
RUN
MOTOR_STOP
```

La GUI queda como monitor y no puede mandar comandos de control normales.

### `MonitorOnly`

Nadie debería mandar control desde la GUI ni desde Simulink. Es modo de observación.

`ESTOP` queda permitido porque es seguridad.

## 9. Escaneo HID: `ScanAsync`

Archivo:

```text
EscBridgeService.cs
```

Método:

```csharp
public async Task ScanAsync(CancellationToken cancellationToken = default)
```

Qué hace:

1. Le pide al enumerador HID una lista de dispositivos.
2. Guarda esa lista.
3. Si el dispositivo actual desapareció, marca desconectado.
4. Si apareció uno nuevo, marca detectado.
5. Notifica a la GUI.

Parte clave:

```csharp
IReadOnlyList<HidDeviceDescriptor> devices =
    await _deviceEnumerator.ListAsync(cancellationToken);
```

`_deviceEnumerator` es una interfaz. La implementación real es `HidSharpDeviceEnumerator`.

`IReadOnlyList<T>` significa "lista que se puede leer, pero no modificar desde afuera".

### Por qué escanear periódicamente

El usuario puede:

- Enchufar el MCU después de abrir la GUI.
- Desenchufarlo.
- Resetear el MCU.
- Cambiar de USB.

El Bridge debe enterarse sin reiniciar la aplicación.

Por eso `EscBridgeWorker` llama periódicamente a:

```csharp
await _bridge.ScanAsync(...)
```

## 10. Conexión real: `ConnectAsync`

`ConnectAsync` no solo abre el HID. Hace validación real.

Flujo:

```text
GUI pulsa Conectar
    |
    v
EscBridgeService.ConnectAsync()
    |
    v
ScanAsync()
    |
    v
OpenAsync() del transporte HID
    |
    v
PING
    |
    v
GET_STATUS
    |
    v
State = Connected
```

Parte importante:

```csharp
EscFrame ping = await SendRequestCoreAsync(
    CommOpcode.Ping,
    0,
    "ping"u8.ToArray(),
    cancellationToken);
```

Explicación:

- `CommOpcode.Ping`: comando `PING`.
- `0`: parámetro no usado.
- `"ping"u8.ToArray()`: crea bytes UTF-8 con el texto `ping`.
- `SendRequestCoreAsync`: manda frame y espera respuesta.

Luego:

```csharp
EscProtocol.EnsureOkResponse(ping, CommOpcode.Ping);
```

Esto verifica:

- La respuesta es tipo `Response`.
- El opcode devuelto es `Ping`.
- El status es `Ok`.

Después pide estado:

```csharp
EscFrame statusFrame = await SendRequestCoreAsync(
    CommOpcode.GetStatus,
    0,
    Array.Empty<byte>(),
    cancellationToken);

EscStatus status = EscProtocol.DecodeStatus(statusFrame);
```

`Array.Empty<byte>()` significa "payload vacío".

Si todo sale bien, guarda:

```csharp
_state = DeviceConnectionState.Connected;
_status = status;
```

## 11. Por qué hay dos candados: `_stateGate` y `_ioLock`

Hay dos problemas distintos:

### `_stateGate`

Protege variables de estado:

```csharp
private readonly object _stateGate = new();
```

Se usa con:

```csharp
lock (_stateGate)
{
    _status = status;
}
```

Sirve para evitar que la GUI lea `Snapshot` mientras otra tarea está cambiando `_status`, `_state`, `_devices`, etc.

### `_ioLock`

Protege el HID:

```csharp
private readonly SemaphoreSlim _ioLock = new(1, 1);
```

Se usa en:

```csharp
await _ioLock.WaitAsync(cancellationToken);
try
{
    return await SendRequestCoreAsync(...);
}
finally
{
    _ioLock.Release();
}
```

Sirve para que solo haya una transacción HID activa.

El protocolo usa `seq`, pero el stream HID es compartido. Si dos tareas mandaran frames a la vez, se podría leer la respuesta equivocada o mezclar telemetría con respuestas.

Por eso todo comando pasa por `_ioLock`.

## 12. Cómo se manda un comando al MCU

La mayoría de comandos terminan pasando por:

```csharp
private async Task<CommandResult> SendCommandAsync(
    CommOpcode opcode,
    byte parameter,
    byte[] payload,
    CancellationToken cancellationToken,
    bool refreshStatus = true)
```

Ejemplo: `SetSpeedRpmAsync`.

```csharp
payload = EscProtocol.UInt16Payload(rpm);
return await SendCommandAsync(CommOpcode.SetSpeedRpm, 0, payload, cancellationToken);
```

Flujo:

```text
SetSpeedRpmAsync(1500)
    |
    v
EscProtocol.UInt16Payload(1500)
    |
    v
SendCommandAsync(SET_SPEED_RPM)
    |
    v
SendRequestAsync()
    |
    v
_ioLock
    |
    v
SendRequestCoreAsync()
    |
    v
EscProtocol.BuildRequest()
    |
    v
_transport.WriteFrameAsync()
    |
    v
_transport.ReadFrameAsync()
```

`SendCommandAsync` devuelve un `CommandResult`.

Si el MCU respondió OK:

```csharp
CommandResult.Ok()
```

Si hubo error:

```csharp
CommandResult.Failed(...)
```

## 13. `SendRequestCoreAsync`: la transacción HID real

Este método es una de las partes más importantes:

```csharp
byte sequence = unchecked(++_sequence);
byte[] request = EscProtocol.BuildRequest(sequence, opcode, parameter, payload);
await _transport.WriteFrameAsync(request, cancellationToken);
```

Explicación:

- `_sequence`: contador de mensajes.
- `++_sequence`: incrementa antes de usar.
- `unchecked`: si el byte pasa de 255, vuelve a 0 sin lanzar error.
- `BuildRequest`: arma el frame binario de 64 bytes.
- `WriteFrameAsync`: lo manda por HID.

Después espera respuesta hasta 1 segundo:

```csharp
DateTimeOffset deadline = DateTimeOffset.UtcNow.AddMilliseconds(1_000);
while (DateTimeOffset.UtcNow < deadline)
{
    ...
}
```

Dentro del loop:

```csharp
EscFrame? frame = await _transport.ReadFrameAsync(remaining, cancellationToken);
```

Puede pasar que el MCU mande telemetría mientras esperamos una respuesta. Por eso:

```csharp
if (HandleFrame(frame))
{
    continue;
}
```

Si era telemetría, la guarda y sigue esperando la respuesta correcta.

La respuesta válida tiene que coincidir:

```csharp
if (frame.Type == CommFrameType.Response &&
    frame.Sequence == sequence &&
    frame.Opcode == opcode)
{
    return frame;
}
```

Esto evita aceptar una respuesta vieja o de otro comando.

Si no llega nada:

```csharp
throw new TimeoutException(...)
```

## 14. Protocolo binario: `Esc.Protocol`

El Bridge no arma bytes manualmente en cada comando. Usa:

```text
Esc.Protocol/EscProtocol.cs
```

El frame HID mide 64 bytes:

```text
magic, version, type, seq, opcode, param, status, payload_len, payload, crc
```

### `BuildFrame`

Arma el frame:

```csharp
var frame = new byte[CommConstants.FrameSize];
frame[0] = CommConstants.Magic0;
frame[1] = CommConstants.Magic1;
frame[2] = CommConstants.Version;
frame[3] = (byte)frameType;
frame[4] = sequence;
frame[5] = (byte)opcode;
...
```

Después escribe el CRC:

```csharp
ushort crc = ComputeCrc16(frame.AsSpan(0, CommConstants.CrcOffset));
```

### `Parse`

Cuando llega una respuesta, `Parse` verifica:

- Tamaño.
- Magic bytes.
- Versión.
- Longitud de payload.
- CRC.

Si algo está mal lanza `EscProtocolException`.

Esto es bueno: si el frame está corrupto, se detecta en una sola capa.

### Payloads

Ejemplo para setpoint:

```csharp
public static byte[] UInt16Payload(int value)
{
    var payload = new byte[2];
    BinaryPrimitives.WriteUInt16LittleEndian(payload, checked((ushort)value));
    return payload;
}
```

Explicación:

- `uint16` son 2 bytes.
- `LittleEndian` significa byte menos significativo primero.
- `checked((ushort)value)` valida que el valor entre en `0..65535`.

## 15. Transporte HID: `Esc.Transport`

El transporte real está en:

```text
HidSharpEscTransport.cs
```

El Bridge no conoce detalles de HidSharp. Solo llama:

```csharp
_transport.OpenAsync(...)
_transport.WriteFrameAsync(...)
_transport.ReadFrameAsync(...)
```

### Por qué hay un byte extra en Windows HID

En `WriteFrameAsync`:

```csharp
var report = new byte[reportLength];
frame.CopyTo(report, reportLength >= CommConstants.FrameSize + 1 ? 1 : 0);
```

Windows HID suele esperar un byte inicial de Report ID.

El firmware usa Report ID 0, entonces se deja un byte adelante y el frame real empieza en posición 1.

Por eso si el frame ESC es de 64 bytes, el reporte Windows puede ser de 65 bytes:

```text
[report_id=0][64 bytes ESC frame]
```

Al leer, se normaliza:

```csharp
private static ReadOnlySpan<byte> NormalizeReport(ReadOnlySpan<byte> report)
{
    if (report.Length >= CommConstants.FrameSize + 1 && report[0] == 0)
    {
        return report.Slice(1, CommConstants.FrameSize);
    }
    ...
}
```

## 16. HTTP local: `BridgeEndpointMapper`

Archivo:

```text
BridgeEndpointMapper.cs
```

Acá se definen endpoints para la GUI y para pruebas externas.

Ejemplo:

```csharp
group.MapPost("/run", async (EscBridgeService bridge, CancellationToken cancellationToken) =>
    await bridge.RunAsync(cancellationToken));
```

Esto crea:

```text
POST /api/bridge/run
```

Cuando llega un POST a esa ruta, ASP.NET llama a `bridge.RunAsync()`.

### Qué significa `group.MapPost`

Registra una ruta HTTP tipo POST.

También hay:

```csharp
group.MapGet("/snapshot", (EscBridgeService bridge) => bridge.Snapshot);
```

Esto crea:

```text
GET /api/bridge/snapshot
```

y devuelve el estado actual en JSON.

### Qué son los records al final

Al final del archivo:

```csharp
public sealed record SetSpeedRequest(int Rpm);
```

Esto define el formato esperado para el JSON.

Si enviás:

```json
{ "rpm": 1500 }
```

ASP.NET lo convierte automáticamente en:

```csharp
new SetSpeedRequest(1500)
```

## 17. WebSocket local

En `BridgeEndpointMapper` también está:

```csharp
endpoints.Map("/ws/bridge", HandleWebSocketAsync);
```

Un WebSocket es una conexión persistente. En lugar de hacer una petición HTTP cada vez, se mantiene abierta y el servidor puede mandar datos periódicamente.

El código manda `Snapshot` cada 500 ms:

```csharp
string json = JsonSerializer.Serialize(bridge.Snapshot, JsonOptions);
byte[] bytes = Encoding.UTF8.GetBytes(json);
await socket.SendAsync(...);
await Task.Delay(500, context.RequestAborted);
```

Actualmente la GUI Blazor usa principalmente el servicio interno, pero este WebSocket deja una puerta simple para clientes externos.

## 18. Worker de fondo: `EscBridgeWorker`

Archivo:

```text
EscBridgeWorker.cs
```

Hereda de:

```csharp
BackgroundService
```

Eso significa: "esta clase corre en segundo plano mientras la app esté viva".

El método importante es:

```csharp
protected override async Task ExecuteAsync(CancellationToken stoppingToken)
```

`override` significa que estamos reemplazando un método definido en la clase base `BackgroundService`.

El loop:

```csharp
while (!stoppingToken.IsCancellationRequested)
{
    await _bridge.ScanAsync(...);
    await _bridge.ReadTelemetryOnceAsync(...);

    if (_bridge.Snapshot.State == DeviceConnectionState.Connected)
    {
        await _bridge.RefreshStatusAsync(...);
    }

    await Task.Delay(500, stoppingToken);
}
```

Cada 500 ms:

1. Escanea HID.
2. Lee una muestra de telemetría si corresponde.
3. Refresca estado si está conectado.

Este loop es una razón por la que la GUI puede mostrar cambios sin que el usuario pulse "Actualizar" todo el tiempo.

## 19. Telemetría: `TelemetryStore`

Archivo:

```text
TelemetryStore.cs
```

Guarda muestras en memoria.

Estructura:

```csharp
private readonly Dictionary<string, Queue<TelemetrySample>> _samples = new(...);
```

Explicación:

- `Dictionary<TKey, TValue>` es un mapa clave-valor.
- La clave es el nombre de variable: `"speed"`, `"current"`, etc.
- El valor es una `Queue<TelemetrySample>`.
- Una `Queue` es una cola: entran muestras nuevas al final y salen viejas al principio.

Capacidad:

```csharp
private const int DefaultCapacity = 2_000;
```

Si la cola supera la capacidad:

```csharp
while (queue.Count > _capacity)
{
    queue.Dequeue();
    _droppedSamples[sample.Variable] = ...
}
```

Esto evita que la GUI consuma memoria infinita si la adquisición corre mucho tiempo.

## 20. Simulink por UDP: `HilUdpBridgeService`

Archivo:

```text
HilUdpBridgeService.cs
```

Aunque el nombre diga HIL, hoy este servicio hace dos cosas:

1. Recibe entradas HIL para simulación.
2. Recibe comandos reales desde Simulink, como `SETPOINT`.

Escucha en:

```csharp
public const int DefaultPort = 5055;
```

y crea el socket UDP:

```csharp
using var udp = new UdpClient(new IPEndPoint(IPAddress.Loopback, DefaultPort));
```

Explicación:

- `UdpClient`: clase de .NET para UDP.
- `IPAddress.Loopback`: `127.0.0.1`.
- `DefaultPort`: `5055`.
- `using var`: cuando el método termine, C# libera el recurso automáticamente.

El loop principal:

```csharp
while (!stoppingToken.IsCancellationRequested)
{
    UdpReceiveResult received = await udp.ReceiveAsync(stoppingToken);
    received = await ReceiveLatestPendingPacketAsync(udp, received, stoppingToken);
    string response = await HandlePacketAsync(received.Buffer, stoppingToken);
    byte[] bytes = Encoding.ASCII.GetBytes(response);
    await udp.SendAsync(bytes, received.RemoteEndPoint, stoppingToken);
}
```

Qué hace:

1. Espera un paquete UDP.
2. Revisa si hay paquetes acumulados.
3. Se queda con el paquete más relevante.
4. Procesa el texto.
5. Devuelve una respuesta ASCII.

## 21. Por qué UDP y no HTTP para Simulink

HTTP sirve bien para comandos humanos: conectar, desconectar, guardar config.

Pero para señales que cambian seguido, como setpoint o velocidad simulada, HTTP agrega demasiado overhead.

UDP es simple:

- Mando un paquete.
- Si se pierde, mando otro en el siguiente sample time.
- No se bloquea esperando conexión.
- Simulink lo soporta con bloques `UDP Send` / `UDP Receive`.

Por eso:

- GUI usa llamadas C# y HTTP local.
- Simulink usa UDP.
- Ambos terminan en el mismo `EscBridgeService`.

## 22. Latest value wins

Este cambio se agregó porque durante pruebas se vio que el motor tardaba en responder a cambios de setpoint desde Simulink.

El problema era conceptual:

```text
SETPOINT,1,1000
SETPOINT,2,1000
SETPOINT,3,1000
...
SETPOINT,80,2000
```

Si el HID tarda más que Simulink en enviar paquetes, se acumulan paquetes viejos. Procesarlos todos produce retardo.

Para señales continuas, no queremos reproducir el pasado. Queremos el último valor.

Por eso:

```csharp
private static async Task<UdpReceiveResult> ReceiveLatestPendingPacketAsync(...)
```

Este método drena paquetes pendientes del socket UDP y elige cuál procesar.

### Clasificación de paquetes

```csharp
private enum PacketKind
{
    Invalid,
    Discrete,
    EmergencyStop,
    Setpoint,
    HilInputs,
}
```

Tipos:

- `Setpoint`: señal continua, se puede coalescer.
- `HilInputs`: señal continua, se puede coalescer.
- `Discrete`: comando puntual como `RUN`.
- `EmergencyStop`: máxima prioridad.
- `Invalid`: se ignora si hay otro mejor.

`coalescer` significa juntar muchos eventos y quedarse con uno.

### Regla

```text
SETPOINT y HIL inputs:
    aplicar el último.

ESTOP:
    aplicar inmediatamente con prioridad.

RUN / STOP:
    tratarlos como comandos discretos.
```

La condición:

```csharp
if (selectedKind == PacketKind.Invalid ||
    nextKind == PacketKind.Discrete ||
    IsCoalescible(nextKind) && IsCoalescible(selectedKind))
{
    selected = next;
    selectedKind = nextKind;
}
```

Si el paquete nuevo y el seleccionado son señales continuas, se reemplaza por el más nuevo.

## 23. Comandos UDP reales desde Simulink

El Bridge reconoce:

```text
SETPOINT,1500
SETPOINT,1,1500
SP,1,1500
SPEED,1,1500
RUN
MOTOR_RUN
MOTOR_STOP
REAL_STOP
ESTOP
```

Para setpoint:

```csharp
if (command is "SETPOINT" or "SP" or "SPEED")
{
    return await HandleSetpointCommandAsync(...);
}
```

`or` en un `is` es pattern matching. Significa: si `command` coincide con cualquiera de esos textos.

`HandleSetpointCommandAsync` parsea:

```text
SETPOINT,rpm
SETPOINT,seq,rpm
```

Luego llama:

```csharp
_bridge.SetSpeedRpmFromSimulinkAsync(
    rpm,
    refreshStatus: false,
    cancellationToken: cancellationToken)
```

### Por qué `refreshStatus: false`

Antes, cada setpoint hacía:

```text
SET_SPEED_RPM
GET_STATUS
```

Eso duplicaba las transacciones HID.

Para una señal enviada desde Simulink a 100 Hz, eso genera cola y retardo.

Ahora hace:

```text
SET_SPEED_RPM
```

y actualiza el setpoint cacheado en el Bridge:

```csharp
UpdateCachedSpeedSetpoint((ushort)rpm);
```

La GUI igual refresca estado periódicamente con el worker.

## 24. Modos de seguridad para Simulink

Para mandar setpoint real desde Simulink:

```csharp
private bool AllowsSimulinkControl()
{
    lock (_stateGate)
    {
        return _mode == ControlMode.SimulinkControl;
    }
}
```

Es decir, la GUI debe estar en:

```text
Simulink control
```

Si no:

```csharp
return CommandResult.Failed("Set GUI mode to Simulink control...");
```

Esto evita que la GUI y Simulink peleen por el control.

`ESTOP` llama a:

```csharp
_bridge.EmergencyStopAsync(...)
```

Ese comando no usa `AllowsSimulinkControl`. La idea es que `ESTOP` esté siempre disponible.

## 25. HIL por UDP

El paquete PIL recomendado es:

```text
seq,speed_rpm,enable
```

Tambien se aceptan paquetes prefijados:

```text
PIL,speed_rpm,enable
PIL,seq,speed_rpm,enable
```

Ejemplo:

```text
1,1000,1
```

El formato HIL viejo sigue aceptado por compatibilidad:

```text
seq,speed_rpm,reserved,load_torque,flags,enable
```

El campo `reserved` se ignora; ya no se mandan cruces por cero desde Simulink. Simulink simula planta, ESC y conmutacion; el MCU solo recibe velocidad simulada.

Se parsea con:

```csharp
TryParseInputs(text, out uint sequence, out HilInputs inputs, out string? error)
```

`out` significa que el método devuelve valores adicionales por parámetros.

Si parsea bien:

```csharp
CommandResult result = await _bridge.HilSetInputsAsync(inputs, cancellationToken);
```

Eso manda al MCU:

```text
HIL_SET_INPUTS
```

### Poll HIL y validacion offline

Igual que con setpoint, si cada entrada HIL hiciera:

```text
HIL_SET_INPUTS
HIL_GET_OUTPUTS
```

la tasa efectiva se reduce.

Por eso:

```csharp
private const double HilOutputPollPeriodMs = 20;
```

El Bridge aplica entradas rapido y pide salidas HIL cada 20 ms para monitoreo.

El response UDP normal puede usar la ultima salida cacheada. El formato
`PILV,run_id,seq,speed_rpm,enable` se reserva para replay de validacion:
no se coalesce, fuerza una lectura `HIL_GET_OUTPUTS` fresca y registra la
proveniencia `run_id`/`source_seq`/`output_generation` para comparar offline.

## 26. Recorrido completo: Simulink cambia setpoint real

Supongamos que Simulink manda:

```text
SETPOINT,25,1800
```

Flujo:

```text
UDP Send de Simulink
    |
    v
HilUdpBridgeService.ReceiveAsync()
    |
    v
ReceiveLatestPendingPacketAsync()
    |
    v
HandlePacketAsync()
    |
    v
HandleSetpointCommandAsync()
    |
    v
EscBridgeService.SetSpeedRpmFromSimulinkAsync(1800, refreshStatus: false)
    |
    v
EscProtocol.UInt16Payload(1800)
    |
    v
SendCommandAsync(SET_SPEED_RPM)
    |
    v
SendRequestAsync() con _ioLock
    |
    v
EscProtocol.BuildRequest()
    |
    v
HidSharpEscTransport.WriteFrameAsync()
    |
    v
MCU
    |
    v
Respuesta OK
    |
    v
BuildCommandResponse()
    |
    v
UDP response a Simulink
```

Respuesta esperada:

```text
ok,setpoint,25,1800,actual_rpm,app_state
```

Si no está en modo `SimulinkControl`:

```text
err,setpoint,25,Set GUI mode to Simulink control before sending real motor commands.
```

## 27. Recorrido completo: GUI manda RUN

La GUI llama:

```csharp
Bridge.RunAsync()
```

Eso va a:

```csharp
public Task<CommandResult> RunAsync(...)
{
    return SendControlCommandAsync(CommOpcode.Run, cancellationToken);
}
```

Antes de mandar, verifica:

```csharp
if (!AllowsGuiControl())
{
    return CommandResult.Failed($"Control is locked by {_mode}.");
}
```

Solo si el modo es `GuiControl`, manda:

```text
RUN
```

al MCU.

Esto evita que la GUI mande `RUN` cuando Simulink tiene el control.

## 28. Recorrido completo: adquisición de velocidad

La GUI manda:

```csharp
StartSpeedLogAsync()
```

Eso envía:

```text
LOG_START, param SPEED
```

Cuando el MCU manda un evento de telemetría:

```csharp
if (frame.Type == CommFrameType.Event &&
    frame.Opcode == CommOpcode.TelemetryEvent)
{
    TelemetrySample sample = EscProtocol.DecodeTelemetry(frame, DateTimeOffset.UtcNow);
    _telemetryStore.Add(sample);
    Notify();
    return true;
}
```

`DecodeTelemetry` extrae:

- Variable.
- Valor.
- Unidad.
- Timestamp del MCU.
- Timestamp del host.

`TelemetryStore` guarda la muestra.

La GUI lee:

```csharp
Bridge.SpeedSamples
```

y el gráfico usa esas muestras.

## 29. `Notify()` y actualización de la GUI

En `EscBridgeService`:

```csharp
public event EventHandler? SnapshotChanged;
```

Un `event` es un mecanismo para avisar a otros objetos que algo pasó.

Cuando cambia el estado:

```csharp
private void Notify()
{
    SnapshotChanged?.Invoke(this, EventArgs.Empty);
}
```

El `?.` significa: "si `SnapshotChanged` no es null, llamalo".

La GUI se suscribe a este evento. Cuando el Bridge cambia, la página actualiza su `Snapshot`.

## 30. Por qué el Bridge no usa HID directo desde la página Blazor

Podríamos haber hecho que `Home.razor` abra HID y mande comandos directamente. No lo hice por varias razones:

1. La página debería saber demasiado del protocolo.
2. Simulink no podría compartir esa lógica.
3. Los locks de HID quedarían mezclados con código visual.
4. Sería más difícil testear.
5. Agregar TCP/UDP después sería duplicar código.

Separar el Bridge permite que la página diga:

```csharp
Bridge.SetSpeedRpmAsync(1500)
```

y no le importe si abajo hay HID, TCP, UDP, logs, locks o CRC.

## 31. Por qué hay `Esc.Protocol`

Otra opción mala habría sido construir bytes en cada comando:

```csharp
byte[] frame = ...
frame[5] = 0x13;
...
```

Eso se vuelve difícil de mantener.

Con `Esc.Protocol`:

```csharp
EscProtocol.BuildRequest(...)
EscProtocol.Parse(...)
EscProtocol.DecodeStatus(...)
EscProtocol.HilInputsPayload(...)
```

Todo el conocimiento del formato binario queda en un solo lugar.

Si el protocolo cambia, se toca una capa.

## 32. Por qué hay `Esc.Transport`

El transporte HID real usa HidSharp, pero el Bridge no debería depender de detalles de esa librería.

Por eso existe:

```csharp
public interface IEscTransport
```

En producción se usa:

```csharp
HidSharpEscTransport
```

En tests se puede usar:

```csharp
FakeEscTransport
```

Esto permite probar lógica del Bridge sin tener el MCU conectado.

## 33. Qué significa "single owner" del HID

El HID lo abre una sola clase:

```text
HidSharpEscTransport
```

Y esa clase es usada por una sola instancia:

```text
EscBridgeService
```

La GUI no abre HID.

Simulink no abre HID.

El worker no abre otro HID.

Todos pasan por:

```text
EscBridgeService
```

Este es el punto central de la arquitectura.

## 34. Cómo agregar un comando nuevo

Supongamos que agregás en firmware:

```text
SET_CURRENT_LIMIT
```

El camino sería:

### 1. Agregar opcode

En `CommConstants.cs`:

```csharp
public enum CommOpcode : byte
{
    ...
    SetCurrentLimit = 0x44,
}
```

### 2. Agregar payload

En `EscProtocol.cs`, si hace falta:

```csharp
public static byte[] CurrentLimitPayload(int milliamps)
{
    return UInt16Payload(milliamps);
}
```

### 3. Agregar método en el Bridge

En `EscBridgeService.cs`:

```csharp
public async Task<CommandResult> SetCurrentLimitAsync(int milliamps, CancellationToken cancellationToken = default)
{
    if (!AllowsGuiControl())
    {
        return CommandResult.Failed($"Control is locked by {_mode}.");
    }

    byte[] payload = EscProtocol.UInt16Payload(milliamps);
    return await SendCommandAsync(CommOpcode.SetCurrentLimit, 0, payload, cancellationToken);
}
```

### 4. Agregar endpoint HTTP si lo querés desde afuera

En `BridgeEndpointMapper.cs`:

```csharp
group.MapPost("/current-limit", async (CurrentLimitRequest request, EscBridgeService bridge, CancellationToken cancellationToken) =>
    await bridge.SetCurrentLimitAsync(request.Milliamps, cancellationToken));
```

### 5. Agregar control en la GUI

Podés agregarlo en `controls.json` o como control fijo.

### 6. Agregar test

En tests, verificar que el Bridge mande el opcode correcto.

## 35. Errores comunes al extender el Bridge

### Mandar comandos sin pasar por `_ioLock`

No hagas operaciones HID directas desde fuera de `SendRequestAsync`.

Si se saltea `_ioLock`, se pueden cruzar respuestas.

### Hacer `GET_STATUS` después de cada muestra rápida

Para comandos humanos está bien.

Para señales de Simulink no.

Por eso `SetSpeedRpmFromSimulinkAsync` usa:

```csharp
refreshStatus: false
```

### Usar la GUI y Simulink para controlar a la vez

Por eso existen los modos:

```text
GuiControl
SimulinkControl
MonitorOnly
```

### Guardar telemetría infinita

Por eso `TelemetryStore` tiene capacidad limitada.

### Tratar setpoint como cola

Para señales continuas, el último valor importa más que los valores viejos.

Por eso existe `ReceiveLatestPendingPacketAsync`.

## 36. Resumen mental del Bridge

Podés pensarlo así:

```text
EscBridgeService
    Es el cerebro.
    Decide estados, permisos y comandos.

Esc.Protocol
    Es el traductor binario.
    Convierte objetos C# en frames de 64 bytes y viceversa.

Esc.Transport
    Es el cable USB.
    Abre HID, escribe reportes y lee reportes.

HilUdpBridgeService
    Es la puerta para Simulink.
    Recibe texto UDP y lo traduce a llamadas al Bridge.

BridgeEndpointMapper
    Es la puerta HTTP.
    Expone comandos locales tipo API.

EscBridgeWorker
    Es el mantenimiento automático.
    Escanea, refresca estado y lee telemetría.

TelemetryStore
    Es el buffer para gráficos.
```

## 37. Lectura recomendada del código

Si querés entenderlo en orden, leelo así:

1. `Program.cs`
2. `EscBridgeServiceCollectionExtensions.cs`
3. `BridgeEnums.cs`
4. `BridgeSnapshot.cs`
5. `EscBridgeService.cs`
6. `HilUdpBridgeService.cs`
7. `BridgeEndpointMapper.cs`
8. `IEscTransport.cs`
9. `HidSharpEscTransport.cs`
10. `EscProtocol.cs`

Ese orden va desde "cómo arranca la app" hasta "cómo se arman los bytes".

## 38. Resumen del flujo Simulink actual

Para motor real:

```text
Simulink UDP Send:
    SETPOINT,seq,rpm

Bridge:
    descarta setpoints viejos acumulados
    exige modo SimulinkControl
    manda SET_SPEED_RPM por HID
    no hace GET_STATUS por cada paquete

MCU:
    actualiza setpoint
```

Para HIL:

```text
Simulink UDP Send:
    seq,speed_rpm,enable

Bridge:
    descarta entradas HIL viejas acumuladas
    manda HIL_SET_INPUTS
    pide HIL_GET_OUTPUTS solo cada cierto tiempo para monitoreo

MCU:
    usa la velocidad simulada como feedback
    devuelve PWM logico para que Simulink simule ESC/planta
```

## 39. Por qué el diseño sirve para el futuro

Este diseño deja preparadas varias extensiones:

- Agregar TCP para Simulink si UDP no alcanza.
- Agregar protocolo binario para Simulink en lugar de CSV ASCII.
- Agregar más variables HIL.
- Agregar corriente, temperatura, torque, flags de fault.
- Separar el Bridge como proceso independiente.
- Agregar logs a archivo.
- Agregar tests con transporte falso.
- Usar la misma GUI para otra aplicación del driver.

Lo importante es que el firmware y el HID siguen teniendo un único dueño lógico: el Bridge.
