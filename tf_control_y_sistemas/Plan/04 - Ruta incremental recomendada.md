---
tags:
  - roadmap
  - decisión
  - implementación
---

# Ruta incremental recomendada

## Etapa 0 — Línea base

### Objetivo

Registrar el comportamiento actual antes de modificar hardware.

### Entregables

- RPM de consigna y medida.
- Duty PWM.
- Respuestas a escalones y rampas.
- Ensayos con y sin barra.
- Video o medición externa de la barra.

### Decisión

Confirmar que el comportamiento puede repetirse y que existe una referencia contra la cual comparar el sensado de corriente.

---

## Etapa 1 — Corriente y tensión de bus

### Objetivo

Obtener mediciones eléctricas confiables sin alterar la conmutación existente.

### Entregables

- Esquemático del front-end.
- Selección de shunt/sensor y rango.
- ADC sincronizado por TIM1.
- Calibración de offset y ganancia.
- Telemetría de $I_{bus}$ y $V_{bus}$.
- Corriente media, RMS, pico y potencia.

### Puerta de decisión

- Si la corriente y la potencia cambian de manera repetible con la carga, continuar al detector de barra.
- Si la medición está dominada por ruido o ventanas inválidas, corregir topología antes de ampliar el algoritmo.

---

## Etapa 2 — Caracterización de carga y barra

### Objetivo

Determinar qué variables eléctricas distinguen acoplamiento, sobrecarga y desacople.

### Entregables

- Mapa viscosidad–RPM–duty–corriente–potencia.
- Máxima velocidad estable por condición.
- Firma temporal de un desacople.
- Comparación contra medición externa de barra.

### Puerta de decisión

- Si existe una firma clara, desarrollar detector y reenganche.
- Si las variables eléctricas no distinguen estados, evaluar un sensor adicional o modificar el caso de estudio.

---

## Etapa 3 — Supervisor de desacople

### Objetivo

Detectar pérdida de barra y ejecutar una recuperación segura.

### Entradas candidatas

- Corriente media/RMS.
- Potencia.
- Duty.
- Error de velocidad.
- Derivadas o cambios relativos.
- Consistencia de BEMF.

### Salidas

- Estado `acoplado`, `carga alta`, `desacoplado` o `incierto`.
- Reducción automática de consigna.
- Rampa de reenganche.
- Detención segura si falla la recuperación.

### Métricas

- Tiempo de detección.
- Falsos positivos y negativos.
- Porcentaje de reenganches exitosos.
- Pérdida máxima de velocidad de barra.

---

## Etapa 4 — Extensión opcional: observador

### Objetivo

Agregar dos corrientes de fase y estimar BEMF/velocidad mediante un modelo.

### Condición para iniciarla

La Etapa 1 debe estar calibrada y estable. Debe existir tiempo suficiente para identificación de parámetros, muestreo rápido y validación.

### Resultado esperado

- Velocidad estimada comparada contra los cruces por cero.
- Rango de velocidad en que el observador es confiable.
- Análisis de sensibilidad a $R$, $L$, ruido y caídas del inversor.

---

## Etapa 5 — Revisión del alcance

### Continuar con el agitador si

- Existe una frontera de desacople reproducible.
- Corriente/potencia permiten estimar carga.
- Puede demostrarse una mejora mediante supervisión o control.
- La medición de barra valida el planteo.

### Considerar el identificador BLDC si

- La planta está sobredimensionada y no aparecen fenómenos relevantes.
- No puede observarse la barra de manera útil.
- El interés principal se desplazó al accionamiento eléctrico.
- Existe tiempo para cambiar el objetivo y validar parámetros.

