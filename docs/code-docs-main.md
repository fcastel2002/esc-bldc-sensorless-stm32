# Documentacion del codigo {#mainpage}

Esta documentacion tecnica se genera a partir del codigo de aplicacion del firmware y los scripts de prueba del host.

Entradas incluidas:

- `firmware/Core/Inc`: cabeceras publicas del firmware.
- `firmware/Core/Src`: implementacion de aplicacion STM32, control, comunicacion y persistencia.
- `gui`: helpers Python para probar el protocolo binario desde UART o USB HID.
- `firmware/COMM_PROTOCOL.md`: especificacion manual del protocolo de comunicacion.

Entradas excluidas:

- `firmware/Drivers` y `firmware/Middlewares`, porque son codigo de proveedor o generado.
- `firmware/build`, porque contiene artefactos de compilacion.

## Generacion local

Desde `firmware/`, usando el mismo flujo de build:

```powershell
cmake --build --preset Debug --target code_docs
```

O desde la raiz del repositorio, si `doxygen` esta disponible en `PATH`:

```powershell
doxygen docs/Doxyfile
```

La salida HTML queda en:

```text
docs/api/html/index.html
```

El build CMake del firmware intenta ejecutar el mismo comando despues de compilar `firmware_esc_bldc_v1`, siempre que `doxygen` este instalado y disponible en `PATH`.
