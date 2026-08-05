---
tags:
  - ensayos
  - validación
  - métricas
---

# Plan experimental

## Objetivo general

Determinar si las variables del ESC permiten describir la carga viscosa y detectar la pérdida de acoplamiento de la barra.

## Variables controladas

- Recipiente y geometría.
- Barra y orientación de imanes.
- Separación vertical.
- Volumen.
- Temperatura.
- Viscosidad o concentración de la mezcla.
- Velocidad de consigna.
- Perfil de aceleración.

La temperatura debe registrarse porque modifica fuertemente la viscosidad.

## Variables medidas

- RPM del motor.
- RPM de la barra mediante referencia externa.
- Duty PWM.
- Corriente de bus media, RMS y pico.
- Tensión de bus.
- Potencia eléctrica aproximada.
- Estado de saturación del PI.
- Calidad o consistencia de BEMF.
- Eventos de desacople y reenganche.

## Matriz mínima de ensayos

| Factor | Niveles iniciales sugeridos |
|---|---|
| Viscosidad | 3–4 niveles |
| Velocidad | Baja, media y alta |
| Volumen | 1–2 niveles |
| Barra | Una geometría fija inicialmente |
| Perturbación | Ninguna, descentrado y aumento de separación |

Cada punto debe repetirse para evaluar dispersión. No es necesario ampliar todos los factores simultáneamente: primero se fija geometría, volumen y temperatura para aislar la viscosidad.

## Ensayos

### 1. Línea base sin barra

- Barrido lento de RPM.
- Obtener corriente/potencia propia del motor e imán impulsor.
- Repetir a distintas temperaturas del accionamiento.

### 2. Barra en fluido de baja viscosidad

- Barrido de RPM con rampa lenta.
- Comparar motor y barra.
- Determinar corriente adicional respecto de la línea base.

### 3. Variación de viscosidad

- Repetir el barrido para cada nivel.
- Registrar la máxima velocidad estable.
- Identificar aumento de corriente, duty y desfase.

### 4. Desacople provocado

- Aumentar lentamente velocidad o separación.
- Registrar la transición completa.
- Etiquetar el instante real mediante video o sensor externo.

### 5. Reenganche

- Reducir velocidad tras detectar desacople.
- Aplicar una rampa controlada.
- Medir éxito, tiempo y repetibilidad.

### 6. Cambios de consigna

- Escalón pequeño.
- Escalón grande.
- Rampa lenta.
- Rampa rápida.

Comparar respuesta del PI, corriente pico y probabilidad de desacople.

## Métricas

- Error estacionario de motor y barra.
- Sobreimpulso.
- Tiempo de establecimiento.
- IAE o ISE de velocidad.
- Corriente y potencia media.
- Corriente pico.
- Tiempo en saturación.
- Máxima RPM estable.
- Tiempo de detección de desacople.
- Falsos positivos/negativos.
- Tasa de reenganche exitoso.

## Resultado mínimo esperado

Un mapa que permita responder:

1. ¿Cuándo la velocidad del motor representa la velocidad de la barra?
2. ¿Cómo cambia la carga eléctrica con la viscosidad?
3. ¿Qué firma produce el desacople?
4. ¿Puede detectarse antes o inmediatamente después de ocurrir?
5. ¿Puede recuperarse mediante una estrategia automática?

