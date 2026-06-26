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

## Generar ejecutable

Para publicar un `.exe` de Windows que levante el servidor:

```powershell
powershell -ExecutionPolicy Bypass -File .\gui\EscGui\publish-gui-win-x64.ps1
```

El ejecutable queda en:

```text
gui\EscGui\publish\win-x64\Esc.Web.exe
```

Al abrir `Esc.Web.exe` se levanta el servidor local y se intenta abrir el navegador en `http://localhost:5187`.
Manten la carpeta `gui\EscGui\publish\win-x64` completa junto al ejecutable porque ahi quedan tambien los recursos web publicados.

## Estructura

- `src/Esc.Protocol`: frames de 64 bytes, CRC16, opcodes, payloads y modelos de dominio.
- `src/Esc.Transport`: enumeracion HID y transporte USB con HidSharp.
- `src/Esc.Bridge`: servicio local, estado de conexion, modos de control, adquisicion y endpoints.
- `src/Esc.Web`: dashboard Blazor y controles modulares.
- `tests/Esc.Tests`: pruebas de protocolo y bridge con transporte falso.

## Guias de lectura

- `GUI_AGENT_CONTEXT.md`: mapa compacto orientado a agentes para ubicar rapido arquitectura, ownership, invariantes y archivos a tocar.
- `GUI_BLAZOR_WALKTHROUGH.md`: recorrido completo de la GUI, Blazor, C#, servicios, controles modulares, grafico y tecnologias usadas.
- `BRIDGE_WALKTHROUGH.md`: recorrido detallado del Bridge, protocolo HID, UDP para Simulink y flujo hacia el MCU.

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
