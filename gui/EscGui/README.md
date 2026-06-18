# ESC GUI

Blazor Web App local para controlar y monitorear el ESC BLDC STM32 mediante un bridge C# que es el unico dueno del enlace USB HID.

## Ejecutar

```powershell
dotnet run --project gui\EscGui\src\Esc.Web\Esc.Web.csproj
```

URL local:

```text
http://localhost:5187
```

## Estructura

- `src/Esc.Protocol`: frames de 64 bytes, CRC16, opcodes, payloads y modelos de dominio.
- `src/Esc.Transport`: enumeracion HID y transporte USB con HidSharp.
- `src/Esc.Bridge`: servicio local, estado de conexion, modos de control, adquisicion y endpoints.
- `src/Esc.Web`: dashboard Blazor y controles modulares.
- `tests/Esc.Tests`: pruebas de protocolo y bridge con transporte falso.

## Endpoints locales

- `GET /api/bridge/snapshot`
- `GET /api/bridge/devices`
- `POST /api/bridge/connect`
- `POST /api/bridge/disconnect`
- `POST /api/bridge/mode`
- `POST /api/bridge/run`
- `POST /api/bridge/stop`
- `POST /api/bridge/estop`
- `POST /api/bridge/set-speed`
- `GET /api/bridge/telemetry/speed`
- `GET /ws/bridge`

La integracion futura con Simulink debe consumir el bridge por un endpoint local, evitando abrir el HID desde dos procesos a la vez.
