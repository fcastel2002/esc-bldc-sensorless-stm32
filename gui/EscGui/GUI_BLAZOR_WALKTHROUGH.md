# Walkthrough completo de la GUI Blazor en C#

Esta guía explica la GUI completa del proyecto `EscGui`: tecnologías usadas, estructura de carpetas, cómo arranca la aplicación, cómo funciona Blazor, cómo se dibuja el dashboard, cómo se conectan los botones con el Bridge, cómo se cargan controles desde `controls.json`, cómo se grafica velocidad y cómo entra Simulink por UDP.

La escribí pensando en alguien que no conoce C# ni Blazor. Por eso, cuando aparece una palabra reservada, una sintaxis rara o una decisión de arquitectura, se explica en contexto.

El documento anterior [BRIDGE_WALKTHROUGH.md](BRIDGE_WALKTHROUGH.md) sigue siendo útil como zoom profundo del Bridge. Este documento mira la GUI completa.

## 1. Qué aplicación se construyó

La GUI es una aplicación web local escrita en C# con Blazor.

Cuando ejecutás:

```powershell
dotnet run --project gui\EscGui\src\Esc.Web\Esc.Web.csproj
```

se levanta un servidor local y abrís:

```text
http://localhost:5187
```

Aunque se abre en el navegador, no es una página web pública. Es una aplicación local que corre en tu PC y controla el ESC por USB HID.

La idea general:

```text
Navegador
    |
    | Blazor Server / eventos UI
    v
Esc.Web
    |
    | llamadas C# directas
    v
Esc.Bridge
    |
    | frames binarios de 64 bytes
    v
Esc.Transport
    |
    | HID usando HidSharp
    v
MCU STM32
```

Y para Simulink:

```text
Simulink
    |
    | UDP 127.0.0.1:5055
    v
HilUdpBridgeService
    |
    v
EscBridgeService
    |
    v
USB HID
    |
    v
MCU STM32
```

## 2. Tecnologías usadas

### .NET

.NET es la plataforma de Microsoft para ejecutar aplicaciones C#.

En este proyecto los `.csproj` usan:

```xml
<TargetFramework>net10.0</TargetFramework>
```

Eso significa que el proyecto compila para .NET 10. En tu máquina el SDK instalado es preview, por eso al compilar puede aparecer un mensaje tipo:

```text
NETSDK1057: Está usando una versión preliminar de .NET
```

No es un error. Es un aviso.

### C#

C# es el lenguaje principal de la GUI. Los archivos `.cs`, `.razor` y parte de los `.csproj` pertenecen al mundo .NET/C#.

### Blazor Web App

Blazor permite construir interfaces web usando C# y Razor en lugar de escribir toda la lógica en JavaScript.

En esta GUI se usa Blazor con componentes interactivos del lado servidor:

```csharp
builder.Services.AddRazorComponents()
    .AddInteractiveServerComponents();
```

Eso significa:

- El HTML se muestra en el navegador.
- La lógica C# corre en el proceso local del servidor.
- Cuando tocás un botón, el evento vuelve al servidor local y ejecuta C#.

Para una GUI de laboratorio/local esto es práctico porque el código que controla HID puede vivir del lado servidor, no en el navegador.

### Razor Components

Los archivos `.razor` mezclan HTML con C#.

Ejemplo:

```razor
<button @onclick="RunAsync">RUN</button>
```

Eso parece HTML, pero `@onclick="RunAsync"` conecta el botón con una función C#.

### ASP.NET Core

ASP.NET Core es la base web sobre la que corre Blazor.

Se usa para:

- Levantar el servidor local.
- Registrar servicios.
- Exponer endpoints HTTP.
- Manejar WebSockets.
- Ejecutar tareas de fondo.

### Minimal APIs

En `BridgeEndpointMapper.cs` se usan endpoints estilo Minimal API:

```csharp
group.MapPost("/run", async (...) => ...);
```

Eso crea rutas HTTP locales sin escribir controladores grandes.

### HidSharp

HidSharp es la librería C# usada para abrir dispositivos HID desde Windows.

Está declarada en:

```xml
<PackageReference Include="HidSharp" Version="2.6.4" />
```

Se usa en `Esc.Transport`.

### UDP

UDP se usa para recibir datos desde Simulink.

El Bridge escucha:

```text
127.0.0.1:5055
```

UDP se eligió porque es simple y sirve bien para señales periódicas como setpoint o variables HIL.

### JSON

Los controles modulares se definen en:

```text
gui/EscGui/src/Esc.Web/wwwroot/controls.json
```

La GUI lee ese archivo y crea sliders/inputs sin tener que programar cada control manualmente.

### CSS

El estilo visual está en:

```text
gui/EscGui/src/Esc.Web/wwwroot/app.css
```

Es CSS normal: colores, grillas, botones, paneles, layout responsive.

### SVG

El gráfico de velocidad actual usa SVG propio en:

```text
SpeedChart.razor
```

El proyecto tiene referencia a `ScottPlot.Blazor`, pero el gráfico que quedó funcionando está implementado manualmente con SVG para tener control directo sobre el eje horizontal de tiempo y eje vertical de velocidad.

### ScottPlot.Blazor

Está referenciado:

```xml
<PackageReference Include="ScottPlot.Blazor" Version="5.1.58" />
```

Pero el componente actual `SpeedChart.razor` no lo usa. Se puede volver a usar más adelante si conviene, pero el SVG manual es simple y suficiente para esta primera GUI.

## 3. Estructura de carpetas

La solución está en:

```text
gui/EscGui/EscGui.slnx
```

Dentro hay varios proyectos:

```text
gui/EscGui/src/Esc.Web
```

La aplicación Blazor. Contiene las páginas, componentes, CSS y `controls.json`.

```text
gui/EscGui/src/Esc.Bridge
```

El Bridge: estado, comandos, telemetría, HTTP, UDP, tareas de fondo.

```text
gui/EscGui/src/Esc.Transport
```

La capa HID usando HidSharp.

```text
gui/EscGui/src/Esc.Protocol
```

El protocolo binario: opcodes, frames, CRC, payloads.

```text
gui/EscGui/tests/Esc.Tests
```

Tests automáticos de protocolo y Bridge.

## 4. Qué es un `.csproj`

Cada proyecto C# tiene un archivo `.csproj`.

Por ejemplo:

```text
gui/EscGui/src/Esc.Web/Esc.Web.csproj
```

Este archivo dice:

- Qué SDK usa.
- Qué versión de .NET usa.
- Qué paquetes NuGet usa.
- Qué otros proyectos referencia.

Ejemplo:

```xml
<Project Sdk="Microsoft.NET.Sdk.Web">
```

`Microsoft.NET.Sdk.Web` indica que este proyecto es una aplicación web.

```xml
<ProjectReference Include="..\Esc.Bridge\Esc.Bridge.csproj" />
```

Esto significa que `Esc.Web` puede usar clases de `Esc.Bridge`.

```xml
<PackageReference Include="ScottPlot.Blazor" Version="5.1.58" />
```

Esto agrega un paquete externo.

## 5. Qué es NuGet

NuGet es el gestor de paquetes de .NET.

Es parecido a:

- `pip` en Python.
- `npm` en JavaScript.
- librerías descargadas por el IDE en otros lenguajes.

Cuando hacés:

```powershell
dotnet build
```

.NET descarga/restaura paquetes NuGet si hacen falta.

## 6. Cómo arranca la aplicación

El archivo de entrada es:

```text
gui/EscGui/src/Esc.Web/Program.cs
```

Código principal:

```csharp
var builder = WebApplication.CreateBuilder(args);
```

`var` significa que C# infiere el tipo automáticamente.

`WebApplication.CreateBuilder(args)` crea un objeto para configurar la app.

Después:

```csharp
builder.Services.AddRazorComponents()
    .AddInteractiveServerComponents();
```

Esto agrega Blazor.

```csharp
builder.Services.AddEscBridge();
```

Esto registra el Bridge y servicios relacionados.

```csharp
builder.Services.AddSingleton<ModularControlProvider>();
```

Esto registra el servicio que lee `controls.json`.

Luego:

```csharp
var app = builder.Build();
```

Construye la aplicación ya configurada.

Después se configuran piezas del servidor:

```csharp
app.UseWebSockets();
app.UseAntiforgery();
app.MapStaticAssets();
app.MapEscBridgeEndpoints();
app.MapRazorComponents<App>()
    .AddInteractiveServerRenderMode();
app.Run();
```

Resumen:

- `UseWebSockets()`: habilita WebSockets.
- `UseAntiforgery()`: protección normal de formularios web.
- `MapStaticAssets()`: sirve archivos estáticos como CSS y JSON.
- `MapEscBridgeEndpoints()`: agrega la API local del Bridge.
- `MapRazorComponents<App>()`: registra Blazor.
- `app.Run()`: arranca el servidor.

## 7. Qué es inyección de dependencias

En C# moderno, muchos objetos no se crean manualmente con `new`.

En vez de eso se registran servicios:

```csharp
builder.Services.AddSingleton<ModularControlProvider>();
```

Y luego Blazor los inyecta donde se necesitan:

```razor
@inject EscBridgeService Bridge
@inject ModularControlProvider ControlProvider
```

Eso significa:

```text
Cuando esta página necesite Bridge, dame la instancia registrada de EscBridgeService.
Cuando necesite ControlProvider, dame la instancia registrada de ModularControlProvider.
```

Esto evita pasar objetos manualmente de un archivo a otro.

## 8. Qué significa `AddSingleton`

`AddSingleton` crea una sola instancia para toda la aplicación.

Ejemplo:

```csharp
services.AddSingleton<EscBridgeService>();
```

Tiene sentido porque el Bridge debe ser único:

```text
un solo Bridge
un solo dueño del HID
un solo estado compartido
```

Si cada página creara su propio Bridge, podrían competir por el USB.

## 9. `App.razor`: el HTML base

Archivo:

```text
gui/EscGui/src/Esc.Web/Components/App.razor
```

Es la estructura HTML principal.

Incluye:

```html
<html>
<head>
...
</head>
<body>
...
</body>
</html>
```

Carga CSS:

```razor
<link rel="stylesheet" href="@Assets["app.css"]" />
```

Carga Blazor:

```razor
<script src="@Assets["_framework/blazor.web.js"]"></script>
```

Monta las rutas:

```razor
<Routes @rendermode="InteractiveServer" />
```

`InteractiveServer` significa que los componentes son interactivos y su lógica corre del lado servidor.

## 10. `Routes.razor`: el router

Archivo:

```text
gui/EscGui/src/Esc.Web/Components/Routes.razor
```

Código:

```razor
<Router AppAssembly="typeof(Program).Assembly" NotFoundPage="typeof(Pages.NotFound)">
```

El router decide qué componente mostrar según la URL.

Como `Home.razor` tiene:

```razor
@page "/"
```

entonces se muestra cuando entrás a:

```text
http://localhost:5187/
```

## 11. `_Imports.razor`

Archivo:

```text
gui/EscGui/src/Esc.Web/Components/_Imports.razor
```

Tiene líneas como:

```razor
@using Esc.Bridge
@using Esc.Protocol
@using Esc.Transport
@using Esc.Web.Services
```

`@using` funciona como `using` en C#: permite usar clases sin escribir el namespace completo.

Gracias a eso en `Home.razor` podemos escribir:

```csharp
BridgeSnapshot
ControlMode
CommandResult
DashboardControl
```

en vez de:

```csharp
Esc.Bridge.BridgeSnapshot
Esc.Bridge.ControlMode
...
```

## 12. `Home.razor`: la página principal

Archivo:

```text
gui/EscGui/src/Esc.Web/Components/Pages/Home.razor
```

Tiene dos partes grandes:

```razor
HTML/Razor visual

@code {
    lógica C#
}
```

La parte visual define lo que ves.

La parte `@code` define variables, funciones y eventos.

## 13. Directivas de Razor

Arriba del archivo:

```razor
@page "/"
@implements IDisposable
@inject EscBridgeService Bridge
@inject ModularControlProvider ControlProvider
```

### `@page`

```razor
@page "/"
```

Indica que este componente es una página y vive en la ruta raíz.

### `@implements`

```razor
@implements IDisposable
```

Dice que el componente implementa la interfaz `IDisposable`.

Eso obliga a tener:

```csharp
public void Dispose()
```

Se usa para limpiar la suscripción al evento del Bridge cuando la página deja de existir.

### `@inject`

```razor
@inject EscBridgeService Bridge
```

Pide a Blazor que inyecte el servicio `EscBridgeService`.

Después la página puede llamar:

```csharp
Bridge.ConnectAsync()
Bridge.RunAsync()
Bridge.SetSpeedRpmAsync(...)
```

## 14. HTML mezclado con C#

Ejemplo:

```razor
<span>@Snapshot.State</span>
```

El `@` indica "acá va C#".

Si `Snapshot.State` vale `Connected`, Blazor renderiza:

```html
<span>Connected</span>
```

Otro ejemplo:

```razor
<button disabled="@(!IsConnected)">Actualizar</button>
```

`disabled` se activa cuando `!IsConnected` es verdadero.

`!` significa negación.

## 15. Estado local de la página

En `@code`:

```csharp
private BridgeSnapshot Snapshot { get; set; } = default!;
private IReadOnlyList<DashboardControl> _controls = Array.Empty<DashboardControl>();
private IReadOnlyList<TelemetrySample> _speedSamples = Array.Empty<TelemetrySample>();
private readonly Dictionary<string, double> _controlValues = new(StringComparer.OrdinalIgnoreCase);
private string? _selectedDevicePath;
private ushort _logRateMs = 500;
private string _lastCommandResult = "Listo.";
```

### `private`

Solo se usa dentro de `Home.razor`.

### `BridgeSnapshot Snapshot`

Foto del estado global del Bridge.

### `IReadOnlyList<DashboardControl>`

Lista de controles cargados desde JSON.

### `Dictionary<string, double>`

Mapa donde la clave es el ID del control y el valor es el valor actual del input.

Ejemplo:

```text
"kp" -> 0.75
"ki" -> 1.35
"speed_setpoint" -> 1500
```

### `string?`

Puede contener texto o `null`.

### `ushort`

Entero sin signo de 16 bits. Rango:

```text
0..65535
```

Se usa porque muchas cosas del protocolo son `uint16`.

### `default!`

```csharp
private BridgeSnapshot Snapshot { get; set; } = default!;
```

Le dice al compilador: "sé que ahora parece no inicializado, pero lo voy a inicializar antes de usar".

En este caso se inicializa en:

```csharp
OnInitializedAsync()
```

## 16. Propiedades calculadas

Ejemplo:

```csharp
private bool IsConnected => Snapshot.State == DeviceConnectionState.Connected;
```

Esto no guarda un valor. Lo calcula cada vez que se usa.

Si `Snapshot.State` es `Connected`, entonces `IsConnected` vale `true`.

Otro ejemplo:

```csharp
private bool CanSendControl => IsConnected && Snapshot.Mode == ControlMode.GuiControl;
```

`&&` significa AND lógico.

La GUI solo puede mandar control si:

```text
está conectada
y
está en modo GuiControl
```

## 17. `switch` expression

Ejemplo:

```csharp
private string ModeHint => Snapshot.Mode switch
{
    ControlMode.GuiControl => "La GUI puede enviar comandos al ESC.",
    ControlMode.SimulinkControl => "La GUI monitorea; los comandos quedan reservados para Simulink salvo ESTOP.",
    ControlMode.MonitorOnly => "Solo adquisicion y monitoreo.",
    _ => string.Empty
};
```

Esto traduce un enum a texto.

`_` significa "cualquier otro caso".

## 18. Ciclo de vida: `OnInitializedAsync`

Blazor llama este método cuando crea la página:

```csharp
protected override async Task OnInitializedAsync()
```

Qué hace:

1. Copia el snapshot inicial del Bridge.
2. Lee `controls.json`.
3. Inicializa valores de controles.
4. Si ya está conectado, lee configs reales desde el MCU.
5. Copia la tasa de log.
6. Selecciona el primer dispositivo HID.
7. Se suscribe al evento `SnapshotChanged`.

Código:

```csharp
Snapshot = Bridge.Snapshot;
_controls = await ControlProvider.GetControlsAsync();
```

`await` espera una operación asíncrona.

Luego:

```csharp
Bridge.SnapshotChanged += HandleSnapshotChanged;
```

`+=` se usa para suscribirse a un evento.

Cuando el Bridge cambie, llamará a:

```csharp
HandleSnapshotChanged
```

## 19. Por qué existe `Dispose`

Al final:

```csharp
public void Dispose()
{
    Bridge.SnapshotChanged -= HandleSnapshotChanged;
}
```

`-=` desuscribe del evento.

Si no se hace, el Bridge podría seguir intentando actualizar una página que ya no existe. Eso puede causar fugas de memoria o comportamientos raros.

## 20. Actualización automática de la pantalla

Cuando el Bridge cambia:

```csharp
private void HandleSnapshotChanged(object? sender, EventArgs args)
{
    Snapshot = Bridge.Snapshot;
    _speedSamples = Bridge.SpeedSamples;
    PickDefaultDevice();
    _ = InvokeAsync(StateHasChanged);
}
```

### `StateHasChanged`

Le dice a Blazor:

```text
volvé a renderizar este componente
```

### `InvokeAsync`

Se usa porque el evento puede venir desde un hilo de fondo. Blazor necesita que el render se programe en el contexto correcto.

### `_ =`

Indica que intencionalmente ignoramos el `Task` devuelto por `InvokeAsync`.

## 21. Barra superior y estado de conexión

Parte visual:

```razor
<section class="topbar">
    ...
    <div class="connection-state @StateClass">
        <span>@Snapshot.State</span>
        <strong>@ConnectionLabel</strong>
    </div>
</section>
```

`class="connection-state @StateClass"` mezcla una clase fija con una clase calculada.

Si el estado es `Connected`, `StateClass` devuelve:

```text
connected
```

Entonces el HTML queda:

```html
<div class="connection-state connected">
```

El CSS usa eso:

```css
.connection-state.connected {
    border-color: #238636;
}
```

Por eso cambia el color del borde según el estado.

## 22. Selector de dispositivo HID

Código visual:

```razor
<select @bind="_selectedDevicePath" disabled="@(!Snapshot.Devices.Any())">
```

### `@bind`

Conecta el valor del `<select>` con la variable `_selectedDevicePath`.

Si el usuario cambia el selector, cambia la variable.

Si C# cambia la variable, cambia el selector.

### `Snapshot.Devices.Any()`

`Any()` devuelve `true` si la lista tiene al menos un elemento.

Si no hay dispositivos, el selector se deshabilita.

### `@foreach`

```razor
@foreach (var device in Snapshot.Devices)
{
    <option value="@device.DevicePath">@DeviceLabel(device)</option>
}
```

Recorre todos los dispositivos HID detectados y crea una opción por cada uno.

## 23. Botones de conexión

```razor
<button @onclick="RefreshDevicesAsync">Refrescar</button>
<button class="primary" @onclick="ConnectAsync" disabled="@(!Snapshot.Devices.Any() || Snapshot.State == DeviceConnectionState.Connected)">Conectar</button>
<button @onclick="DisconnectAsync" disabled="@(Snapshot.State != DeviceConnectionState.Connected)">Desconectar</button>
```

### `@onclick`

Conecta un click con una función C#.

Si el usuario pulsa "Conectar", Blazor ejecuta:

```csharp
ConnectAsync()
```

### `||`

Significa OR lógico.

El botón conectar se deshabilita si:

```text
no hay dispositivos
o
ya está conectado
```

## 24. Modo de control

Visual:

```razor
<select value="@Snapshot.Mode" @onchange="SetModeAsync">
    <option value="@ControlMode.GuiControl">GUI control</option>
    <option value="@ControlMode.SimulinkControl">Simulink control</option>
    <option value="@ControlMode.MonitorOnly">Monitor only</option>
</select>
```

Acá se usa `@onchange`, no `@bind`, porque queremos ejecutar código específico cuando cambia.

Función:

```csharp
private async Task SetModeAsync(ChangeEventArgs args)
{
    if (Enum.TryParse(args.Value?.ToString(), out ControlMode mode))
    {
        await Bridge.SetModeAsync(mode);
        Snapshot = Bridge.Snapshot;
    }
}
```

### `ChangeEventArgs`

Objeto que contiene información del cambio del `<select>`.

### `Enum.TryParse`

Convierte texto a enum.

Ejemplo:

```text
"SimulinkControl" -> ControlMode.SimulinkControl
```

### `out`

`out ControlMode mode` significa que `TryParse` devuelve un segundo valor a través de ese parámetro.

## 25. Panel de estado

Visual:

```razor
<span>Setpoint</span><strong>@(Snapshot.Status?.SpeedSetpointRpm.ToString() ?? "-") rpm</strong>
```

### `?.`

Operador null-safe.

Si `Snapshot.Status` es `null`, no intenta acceder a `SpeedSetpointRpm`.

### `??`

Si lo de la izquierda es `null`, usa lo de la derecha.

En castellano:

```text
si hay status, mostrar setpoint; si no, mostrar "-"
```

El panel muestra:

- App state.
- Transporte.
- Setpoint.
- Velocidad.
- Max PWM.
- Stall.
- Cruces consistentes.

Todo viene del Bridge, que a su vez lo obtiene con `GET_STATUS` del MCU.

## 26. Panel de control fijo

Visual:

```razor
<button class="run" @onclick="RunAsync" disabled="@(!CanSendControl)">RUN</button>
<button @onclick="StopAsync" disabled="@(!CanSendControl)">STOP</button>
<button class="estop" @onclick="EmergencyStopAsync" disabled="@(!IsConnected)">ESTOP</button>
```

`RUN` y `STOP` dependen de:

```csharp
CanSendControl
```

Eso obliga a estar en `GuiControl`.

`ESTOP` solo exige conexión:

```csharp
IsConnected
```

Esto es intencional: `ESTOP` debe estar disponible incluso si Simulink tiene el control.

Funciones:

```csharp
private async Task RunAsync() => await StoreResultAsync(Bridge.RunAsync());
private async Task StopAsync() => await StoreResultAsync(Bridge.StopAsync());
private async Task EmergencyStopAsync() => await StoreResultAsync(Bridge.EmergencyStopAsync());
```

`=>` es sintaxis corta.

Estas funciones llaman al Bridge y guardan el mensaje de resultado.

## 27. `StoreResultAsync`

```csharp
private async Task StoreResultAsync(Task<CommandResult> operation)
{
    CommandResult result = await operation;
    _lastCommandResult = result.Message;
    Snapshot = Bridge.Snapshot;
}
```

Esta función evita repetir código.

En vez de escribir en cada comando:

```csharp
CommandResult result = await ...
_lastCommandResult = result.Message;
Snapshot = Bridge.Snapshot;
```

se centraliza.

`CommandResult` tiene:

- `Success`.
- `DeviceStatus`.
- `Message`.

## 28. Controles modulares

La sección:

```razor
<section class="panel">
    <h2>Controles modulares</h2>
    ...
</section>
```

se arma a partir de:

```text
wwwroot/controls.json
```

Ejemplo:

```json
{
  "id": "kp",
  "label": "KP",
  "kind": "number",
  "command": "set_config",
  "param": "Kp",
  "min": 0,
  "max": 20,
  "step": 0.05,
  "unit": "",
  "defaultValue": 0.75
}
```

Eso genera un input numérico para `KP`.

## 29. Modelo `DashboardControl`

Archivo:

```text
gui/EscGui/src/Esc.Web/Services/DashboardControl.cs
```

Código:

```csharp
public sealed record DashboardControl
{
    public string Id { get; init; } = string.Empty;
    public string Label { get; init; } = string.Empty;
    public string Kind { get; init; } = "number";
    public string Command { get; init; } = string.Empty;
    public string? Param { get; init; }
    public double Min { get; init; }
    public double Max { get; init; }
    public double Step { get; init; } = 1;
    public string Unit { get; init; } = string.Empty;
    public double DefaultValue { get; init; }
}
```

### `record`

Es una clase orientada a datos.

### `{ get; init; }`

Significa que la propiedad se puede asignar al crear el objeto, pero después queda como solo lectura.

Esto sirve para datos cargados desde JSON: se cargan una vez y no deberían cambiar.

## 30. Carga de `controls.json`

Archivo:

```text
ModularControlProvider.cs
```

Función:

```csharp
public async Task<IReadOnlyList<DashboardControl>> GetControlsAsync(...)
```

Pasos:

1. Si ya se cargó antes, devuelve cache.
2. Arma el path a `controls.json`.
3. Si no existe, devuelve lista vacía.
4. Si existe, lo lee con `JsonSerializer`.
5. Si falla, loguea warning y devuelve lista vacía.

### `IWebHostEnvironment`

Se usa para saber dónde está `wwwroot`.

```csharp
string path = Path.Combine(_environment.WebRootPath, "controls.json");
```

`WebRootPath` apunta a:

```text
gui/EscGui/src/Esc.Web/wwwroot
```

### `JsonSerializer.DeserializeAsync`

Convierte JSON a objetos C#.

```csharp
DashboardControl[]? controls = await JsonSerializer.DeserializeAsync<DashboardControl[]>(...)
```

Si el JSON tiene campos como `defaultValue`, con `JsonSerializerDefaults.Web` se mapean correctamente a `DefaultValue`.

## 31. Render de controles modulares

En `Home.razor`:

```razor
@foreach (var control in _controls)
{
    <div class="control-item">
        <label for="@control.Id">@control.Label</label>
        ...
    </div>
}
```

Por cada control del JSON se genera un bloque visual.

Si es slider:

```razor
@if (control.Kind.Equals("slider", StringComparison.OrdinalIgnoreCase))
{
    <input type="range" ... />
}
```

Si es button:

```razor
else if (control.Kind.Equals("button", StringComparison.OrdinalIgnoreCase))
{
    <button ...>@control.Label</button>
}
```

Si no, usa number input:

```razor
else
{
    <input type="number" ... />
}
```

### `StringComparison.OrdinalIgnoreCase`

Hace comparación ignorando mayúsculas/minúsculas.

Así `"Slider"`, `"slider"` y `"SLIDER"` funcionan igual.

## 32. Valores de controles

Los valores se guardan en:

```csharp
private readonly Dictionary<string, double> _controlValues = new(...);
```

Para obtener el valor:

```csharp
private double ControlValue(DashboardControl control)
{
    return _controlValues.TryGetValue(control.Id, out double value)
        ? value
        : control.DefaultValue;
}
```

### Operador ternario `? :`

```csharp
condicion ? valorSiTrue : valorSiFalse
```

En este caso:

```text
si existe un valor guardado, usalo
si no, usá defaultValue
```

Para actualizar:

```csharp
private void UpdateControlValue(DashboardControl control, ChangeEventArgs args)
{
    if (double.TryParse(args.Value?.ToString(), out double value))
    {
        _controlValues[control.Id] = value;
    }
}
```

`double.TryParse` convierte texto a número decimal.

## 33. Aplicar un control modular

Función:

```csharp
private async Task ApplyControlAsync(DashboardControl control)
```

Usa un `switch` sobre `control.Command`:

```csharp
Task<CommandResult> operation = control.Command switch
{
    "set_speed_rpm" => Bridge.SetSpeedRpmAsync((int)Math.Round(value)),
    "set_config" when TryParseConfig(control.Param, out ConfigParam parameter) => Bridge.SetConfigAsync(parameter, value),
    "reset_config" when TryParseConfig(control.Param, out ConfigParam parameter) => Bridge.ResetConfigAsync(parameter),
    "log_speed_start" => Bridge.StartSpeedLogAsync(),
    "log_speed_stop" => Bridge.StopSpeedLogAsync(),
    _ => Task.FromResult(CommandResult.Failed($"Comando modular no soportado: {control.Command}."))
};
```

### `when`

Es una condición adicional dentro del `switch`.

Ejemplo:

```csharp
"set_config" when TryParseConfig(...)
```

Significa:

```text
si command es "set_config" y además se pudo parsear el parámetro
```

### `Math.Round`

Redondea el valor del slider o input.

Se usa para velocidad porque el comando espera entero.

## 34. Guardar en flash

Los controles de configuración tienen un botón pequeño de guardar.

Visual:

```razor
<button class="icon-button save-button" ... title="Guardar en flash">
    <svg ...>
        <path ... />
    </svg>
</button>
```

### Por qué SVG

El icono de guardar es un SVG embebido. Es liviano y no depende de imágenes externas.

### `title`

Muestra tooltip al pasar el mouse.

### `aria-label`

Texto para accesibilidad.

### Qué hace

`Aplicar` manda `SET_CONFIG`, pero no persiste.

El botón de guardar manda:

```csharp
Bridge.SaveConfigAsync(parameter)
```

Eso termina en `SAVE_CONFIG` hacia el MCU.

La separación es intencional:

```text
Aplicar = cambiar RAM/config activa
Guardar = persistir en flash
```

Así no se escribe flash cada vez que movés un slider.

## 35. Lectura inicial de configuraciones

Cuando conectás la GUI:

```csharp
if (result.Success)
{
    await LoadConfigValuesAsync();
}
```

`LoadConfigValuesAsync` recorre los controles persistibles:

```csharp
foreach (DashboardControl control in _controls)
{
    if (!CanPersistControl(control) || !TryParseConfig(control.Param, out ConfigParam parameter))
    {
        continue;
    }

    object? value = await Bridge.GetConfigAsync(parameter);
    ...
}
```

### `continue`

Salta al siguiente elemento del loop.

Si el control no es configuración persistible, no intenta leerlo del MCU.

### `object?`

`object` es el tipo base de casi todo en C#.

`GetConfigAsync` puede devolver distintos tipos:

- `ushort`.
- `byte`.
- `double`.
- `byte[]`.

Por eso se usa `object?`.

Luego:

```csharp
_controlValues[control.Id] = Convert.ToDouble(value);
```

Así los inputs muestran lo que realmente tiene el MCU.

## 36. Deshabilitar controles según modo

Función:

```csharp
private bool ControlDisabled(DashboardControl control)
{
    if (!IsConnected)
    {
        return true;
    }

    return control.Command is not ("log_speed_start" or "log_speed_stop")
        && Snapshot.Mode != ControlMode.GuiControl;
}
```

Reglas:

- Si no está conectado, todo deshabilitado.
- Si el control es de log, se permite aunque no sea `GuiControl`.
- Si es comando de control/config, solo en `GuiControl`.

Esto evita que la GUI mande comandos mientras Simulink tiene control.

## 37. Adquisición y gráfico

La sección de adquisición tiene:

```razor
<SpeedChart Samples="@_speedSamples" />
```

Eso usa el componente:

```text
gui/EscGui/src/Esc.Web/Components/SpeedChart.razor
```

### `[Parameter]`

En `SpeedChart.razor`:

```csharp
[Parameter]
public IReadOnlyList<TelemetrySample> Samples { get; set; } = Array.Empty<TelemetrySample>();
```

`[Parameter]` indica que el componente recibe ese valor desde afuera.

En este caso `Home.razor` le pasa `_speedSamples`.

## 38. Cómo funciona `SpeedChart.razor`

El gráfico es un SVG:

```razor
<svg class="speed-chart" viewBox="0 0 720 300" preserveAspectRatio="none">
```

### SVG

SVG es dibujo vectorial en HTML.

Permite dibujar:

- Líneas.
- Texto.
- Círculos.
- Polilíneas.
- Ejes.

El gráfico dibuja:

- Fondo.
- Grilla horizontal.
- Grilla vertical.
- Ejes.
- Curva de velocidad.
- Punto de última muestra.
- Etiquetas.

### Eje X: tiempo

Cada muestra tiene:

```csharp
TargetTickMs
```

Ese es el timestamp del MCU.

Se calcula:

```csharp
double x = PlotLeft + ((sample.TargetTickMs - minTick) / tickRange) * PlotWidth;
```

En castellano:

```text
posición X = margen izquierdo + porcentaje de tiempo transcurrido * ancho del gráfico
```

### Eje Y: velocidad

Cada muestra tiene:

```csharp
DisplayValue
```

Se calcula:

```csharp
double y = PlotBottom - ((sample.DisplayValue - yMin) / (yMax - yMin)) * PlotHeight;
```

Se resta desde `PlotBottom` porque en SVG el eje Y crece hacia abajo.

En matemática normal:

```text
mayor velocidad -> más arriba
```

En SVG:

```text
menor coordenada y -> más arriba
```

Por eso se usa `PlotBottom - ...`.

### Polyline

La curva se dibuja con:

```razor
<polyline points="@Points" class="chart-line" />
```

`Points` genera un string:

```text
x1,y1 x2,y2 x3,y3 ...
```

SVG lo interpreta como una línea que une todos esos puntos.

## 39. Rango Y automático

En `SpeedChart.razor`:

```csharp
private (double Min, double Max) YRange
```

Esto calcula el mínimo y máximo de velocidad visible.

Agrega margen:

```csharp
double margin = valueRange < 1
    ? Math.Max(50, Math.Abs(maxValue) * 0.02)
    : Math.Max(10, valueRange * 0.15);
```

Si la velocidad casi no cambia, igual deja margen para que la línea no quede pegada al borde.

Esto corrigió el problema donde la gráfica quedaba visualmente aplastada.

## 40. Panel HIL Simulink

En `Home.razor`:

```razor
<section class="panel hil-panel">
```

Muestra:

- Estado HIL.
- Frames RX.
- Perdidos.
- Rate.
- Jitter.
- RTT medio.
- PWM lógico.
- Timeout.

Todos esos datos vienen de:

```csharp
Snapshot.Hil
```

`Snapshot.Hil` es un `HilBridgeStats`.

El Bridge actualiza esas estadísticas cuando procesa paquetes UDP desde Simulink.

## 41. CSS de la GUI

Archivo:

```text
gui/EscGui/src/Esc.Web/wwwroot/app.css
```

El CSS define:

- Tipografía.
- Colores.
- Botones.
- Paneles.
- Grillas.
- Responsive para pantallas chicas.
- Estilo del gráfico SVG.

Ejemplo:

```css
.dashboard {
    width: min(1280px, calc(100vw - 32px));
    margin: 0 auto;
    padding: 24px 0 40px;
}
```

Significa:

- Máximo 1280 px de ancho.
- Si la pantalla es más chica, usa el ancho disponible menos 32 px.
- Centrado horizontal.

### Paneles

```css
.panel {
    margin-top: 14px;
    padding: 16px;
}
```

Todos los paneles comparten estilo.

### Grilla responsive

```css
.controls-grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(240px, 1fr));
    gap: 14px;
}
```

Esto hace que los controles se acomoden solos:

- Cada control mide al menos 240 px.
- Si hay espacio, entran varios por fila.
- Si no, bajan a otra fila.

### Media query

```css
@media (max-width: 820px) {
    ...
}
```

Aplica estilos distintos en pantallas angostas.

## 42. Por qué la GUI no es "fancy"

La prioridad fue:

- Claridad.
- Estado visible.
- Controles fáciles de agregar.
- Seguridad.
- Poco acoplamiento.
- Uso en laboratorio.

Por eso la UI usa:

- Paneles simples.
- Botones claros.
- Inputs nativos.
- Selectores nativos.
- CSS propio.
- Gráfico simple.

Evité una UI con demasiadas dependencias visuales porque todavía estás definiendo variables reales del driver y el flujo Simulink/HIL.

## 43. Cómo agregar un nuevo control sin tocar C#

Si el comando ya existe en `ApplyControlAsync`, podés agregar una entrada en:

```text
controls.json
```

Ejemplo:

```json
{
  "id": "max_speed",
  "label": "Velocidad maxima",
  "kind": "number",
  "command": "set_config",
  "param": "MaxSpeed",
  "min": 200,
  "max": 12000,
  "step": 50,
  "unit": "rpm",
  "defaultValue": 5400
}
```

Campos:

- `id`: identificador interno único.
- `label`: texto visible.
- `kind`: `slider`, `number` o `button`.
- `command`: acción que entiende la GUI.
- `param`: parámetro del firmware si aplica.
- `min`: mínimo.
- `max`: máximo.
- `step`: incremento.
- `unit`: unidad visible.
- `defaultValue`: valor inicial antes de leer el MCU.

## 44. Cómo agregar un comando modular nuevo

Si agregás en `controls.json`:

```json
"command": "set_current_limit"
```

también tenés que enseñar a `Home.razor` qué significa.

En:

```csharp
Task<CommandResult> operation = control.Command switch
{
    ...
};
```

agregarías:

```csharp
"set_current_limit" => Bridge.SetCurrentLimitAsync((int)Math.Round(value)),
```

Pero antes debe existir:

```csharp
SetCurrentLimitAsync
```

en `EscBridgeService`.

## 45. Cómo agregar una nueva variable al gráfico

Hoy el gráfico está pensado para velocidad.

Para agregar corriente o temperatura, haría falta:

1. Que el firmware mande telemetría real de esa variable.
2. Que `EscProtocol.DecodeTelemetry` la decodifique.
3. Que `TelemetryStore` la guarde.
4. Que `Home.razor` tenga otra lista de muestras.
5. Que se use otro `SpeedChart` o un gráfico más genérico.

El paso natural sería renombrar `SpeedChart` a algo tipo:

```text
TelemetryChart.razor
```

y pasarle:

```csharp
Samples
YAxisLabel
Unit
```

## 46. Cómo agregar una nueva página

Crear archivo:

```text
gui/EscGui/src/Esc.Web/Components/Pages/Config.razor
```

Con:

```razor
@page "/config"

<h1>Config</h1>
```

Blazor la detecta por `@page`.

Luego podrías navegar a:

```text
http://localhost:5187/config
```

Hoy la app usa una sola pantalla porque para laboratorio es más cómodo tener todo a la vista.

## 47. Cómo agregar un panel nuevo en la pantalla principal

En `Home.razor`, agregás:

```razor
<section class="panel">
    <div class="panel-title">
        <h2>Nuevo panel</h2>
    </div>
    ...
</section>
```

El CSS `.panel` ya le da estilo.

Si necesitás datos del Bridge, probablemente los tomes de:

```csharp
Snapshot
```

o agregues una propiedad nueva a `BridgeSnapshot`.

## 48. Cómo viaja un click de la GUI al MCU

Ejemplo: botón `RUN`.

```text
Usuario hace click RUN
    |
    v
Blazor ejecuta RunAsync()
    |
    v
Home.razor llama Bridge.RunAsync()
    |
    v
EscBridgeService valida modo GuiControl
    |
    v
EscProtocol arma frame RUN
    |
    v
Esc.Transport escribe HID
    |
    v
MCU recibe opcode RUN
    |
    v
MCU responde OK
    |
    v
Bridge devuelve CommandResult
    |
    v
Home.razor muestra "OK"
```

## 49. Cómo viaja un setpoint desde Simulink

```text
Simulink manda UDP:
SETPOINT,30,1800
    |
    v
HilUdpBridgeService recibe paquete
    |
    v
descarta setpoints viejos acumulados
    |
    v
llama Bridge.SetSpeedRpmFromSimulinkAsync(...)
    |
    v
Bridge exige modo SimulinkControl
    |
    v
manda SET_SPEED_RPM por HID
    |
    v
MCU responde OK
```

La GUI no manda nada en ese flujo. Solo monitorea.

## 50. Diferencia entre GUI control y Simulink control

La GUI tiene este concepto:

```text
modo = quién tiene permiso de mandar comandos de control
```

### GUI control

La GUI puede mandar:

- RUN.
- STOP.
- Setpoint.
- Configs.
- Save config.

### Simulink control

Simulink puede mandar por UDP:

- SETPOINT.
- RUN.
- MOTOR_STOP.
- ESTOP.

La GUI mira, pero no cambia control.

### Monitor only

Solo monitoreo.

`ESTOP` debe seguir disponible.

## 51. Tests

Los tests están en:

```text
gui/EscGui/tests/Esc.Tests
```

Se ejecutan con:

```powershell
dotnet test gui\EscGui\EscGui.slnx
```

Hay tests de:

- CRC/protocolo.
- Build/parse de frames.
- Bridge con transporte fake.
- Bloqueos por modo.
- HIL opcodes.
- Setpoint externo desde Simulink.

El transporte fake permite probar sin MCU físico.

## 52. Comandos útiles

Build:

```powershell
dotnet build gui\EscGui\EscGui.slnx
```

Tests:

```powershell
dotnet test gui\EscGui\EscGui.slnx
```

Run:

```powershell
dotnet run --project gui\EscGui\src\Esc.Web\Esc.Web.csproj
```

Abrir:

```text
http://localhost:5187
```

Probar snapshot HTTP:

```powershell
Invoke-WebRequest -UseBasicParsing http://localhost:5187/api/bridge/snapshot
```

Probar UDP setpoint:

```powershell
$udp = [System.Net.Sockets.UdpClient]::new()
$bytes = [Text.Encoding]::ASCII.GetBytes("SETPOINT,1,1500")
[void]$udp.Send($bytes,$bytes.Length,"127.0.0.1",5055)
$udp.Dispose()
```

## 53. Archivos que conviene leer en orden

Si querés aprender la GUI desde cero:

1. `gui/EscGui/src/Esc.Web/Program.cs`
2. `gui/EscGui/src/Esc.Web/Components/App.razor`
3. `gui/EscGui/src/Esc.Web/Components/Routes.razor`
4. `gui/EscGui/src/Esc.Web/Components/Pages/Home.razor`
5. `gui/EscGui/src/Esc.Web/Services/DashboardControl.cs`
6. `gui/EscGui/src/Esc.Web/Services/ModularControlProvider.cs`
7. `gui/EscGui/src/Esc.Web/wwwroot/controls.json`
8. `gui/EscGui/src/Esc.Web/Components/SpeedChart.razor`
9. `gui/EscGui/src/Esc.Web/wwwroot/app.css`
10. `gui/EscGui/BRIDGE_WALKTHROUGH.md`

Ese orden va de lo general a lo específico.

## 54. Idea mental final

Pensá la GUI así:

```text
Home.razor
    Es la pantalla.
    Muestra estado, botones, controles y gráfico.

ModularControlProvider
    Lee controls.json.

SpeedChart
    Dibuja muestras de velocidad.

app.css
    Define cómo se ve todo.

EscBridgeService
    Es el backend local.
    Decide permisos, estado y comandos.

EscProtocol
    Convierte comandos en bytes.

EscTransport
    Mueve bytes por HID.

HilUdpBridgeService
    Recibe paquetes desde Simulink.
```

La GUI no es solo una página: es una aplicación local con frontend Blazor y backend C# en el mismo proceso.
