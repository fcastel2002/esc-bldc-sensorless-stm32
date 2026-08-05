---
tags:
  - trabajo-final
  - alcance
  - observabilidad
---

# Problema y criterio de suficiencia

## El problema no es que el PI sea simple

El firmware actual ya contiene varios elementos relevantes:

- Conmutación six-step sensorless.
- Detección de cruces por cero de BEMF.
- Estimación de velocidad del rotor.
- Control PI discreto cada 2 ms.
- Implementación en punto fijo.
- Integración trapezoidal y anti-windup.
- Transición sin salto al cerrar el lazo.
- Máquina de estados y soporte HIL.

El término derivativo está deshabilitado, por lo que el controlador actual es un **PI**, no un PID completo. Agregar una derivada, LQR o Kalman sin una necesidad concreta no aumentaría por sí mismo la calidad académica.

## Tres variables diferentes

No debe suponerse que las siguientes variables son siempre equivalentes:

| Variable | Significado |
|---|---|
| Velocidad del motor | Velocidad del imán impulsor medida por BEMF |
| Velocidad de la barra | Movimiento real del elemento dentro del fluido |
| Agitación | Movimiento y homogeneización efectiva del fluido |

El PI puede mantener correctamente la velocidad del motor aunque la barra oscile, pierda sincronismo o deje de producir una mezcla adecuada.

## Qué sería insuficiente

- Mostrar solamente que el agitador gira con agua.
- Presentar el bracket y la cápsula como contribución principal.
- Mostrar únicamente la respuesta de RPM del motor.
- Suponer que funcionará con mayor viscosidad sin ensayos.
- Agregar sensores o algoritmos sin vincularlos con una pregunta de control.

## Qué sería suficiente y defendible

- Definir claramente la variable que representa la agitación.
- Construir un modelo agregado de motor, acoplamiento y carga viscosa.
- Caracterizar experimentalmente el rango estable de operación.
- Medir la barra de manera independiente durante la validación.
- Comparar motor, barra, duty, corriente y potencia.
- Analizar saturación, ruido y pérdida de acoplamiento.
- Validar el controlador discreto en simulación y maqueta.
- Presentar métricas y límites de validez.

## Modelo conceptual mínimo

Puede emplearse un modelo concentrado, sin necesidad de CFD:

$$
J_m\dot{\omega}_m = \tau_e - \tau_{acople} - B_m\omega_m
$$

$$
J_b\dot{\omega}_b = \tau_{acople} - \tau_{viscosa}
$$

$$
\dot{\delta}=\omega_m-\omega_b
$$

La viscosidad modifica el par resistente y el acoplamiento magnético posee un par máximo. Al superar ese límite puede producirse el desacople.

## Criterio académico

Los contenidos mínimos de la materia describen su campo, no una lista obligatoria para el trabajo final. Es preferible resolver de forma rigurosa un problema acotado de modelado, control discreto, estimación y validación antes que agregar muchas técnicas superficialmente.

