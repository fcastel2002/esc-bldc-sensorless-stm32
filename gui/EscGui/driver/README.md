# ESC BLDC STM32 HID driver package

Este paquete no instala un driver propio. Solo asocia el hardware ID del ESC con
un nombre amigable y delega en el driver HID generico de Windows mediante
`input.inf`.

Hardware IDs esperados:

```text
USB\VID_3232&PID_EC32
HID\VID_3232&PID_EC32
```

Archivos del paquete:

```text
esc-bldc-stm32-hid.inf
esc-bldc-stm32-hid.cat
esc-bldc-stm32-test-signing.cer
```

## Verificacion

Herramientas usadas:

```text
C:\Program Files (x86)\Windows Kits\10\Tools\10.0.26100.0\x64\infverif.exe
C:\Program Files (x86)\Windows Kits\10\bin\10.0.26100.0\x86\Inf2Cat.exe
C:\Program Files (x86)\Windows Kits\10\bin\10.0.18362.0\x64\signtool.exe
```

Comandos:

```powershell
cd gui\EscGui\driver

& "C:\Program Files (x86)\Windows Kits\10\Tools\10.0.26100.0\x64\infverif.exe" /w .\esc-bldc-stm32-hid.inf

& "C:\Program Files (x86)\Windows Kits\10\bin\10.0.26100.0\x86\Inf2Cat.exe" /driver:. "/os:10_X64,10_RS3_ARM64,10_GE_X64,10_GE_ARM64" /verbose /uselocaltime

& "C:\Program Files (x86)\Windows Kits\10\bin\10.0.18362.0\x64\signtool.exe" verify /pa /c .\esc-bldc-stm32-hid.cat .\esc-bldc-stm32-hid.inf
```

Si `signtool verify` falla con `root certificate which is not trusted`, la firma
existe pero falta confiar el certificado de prueba.

## Confiar el certificado de prueba

Abrir PowerShell como administrador:

```powershell
cd D:\06. Proyectos\esc-bldc-stm32\gui\EscGui\driver
certutil -f -addstore Root .\esc-bldc-stm32-test-signing.cer
certutil -f -addstore TrustedPublisher .\esc-bldc-stm32-test-signing.cer
```

Luego verificar de nuevo:

```powershell
& "C:\Program Files (x86)\Windows Kits\10\bin\10.0.18362.0\x64\signtool.exe" verify /pa .\esc-bldc-stm32-hid.cat
& "C:\Program Files (x86)\Windows Kits\10\bin\10.0.18362.0\x64\signtool.exe" verify /pa /c .\esc-bldc-stm32-hid.cat .\esc-bldc-stm32-hid.inf
```

## Instalacion

PowerShell como administrador:

```powershell
cd D:\06. Proyectos\esc-bldc-stm32\gui\EscGui\driver
pnputil /add-driver .\esc-bldc-stm32-hid.inf /install
```

Si Windows sigue rechazando el paquete por politica de firma durante desarrollo,
habilitar modo de prueba y reiniciar:

```powershell
bcdedit /set testsigning on
```

Para volver al modo normal:

```powershell
bcdedit /set testsigning off
```

Para producto real se debe usar un VID/PID propio o asignado y firmar el paquete
con un certificado/flujo de firma de driver valido para distribucion.
