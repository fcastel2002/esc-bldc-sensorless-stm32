---
tags:
  - trabajo-final
  - control-y-sistemas
  - agitador-magnetico
aliases:
  - Inicio TF Control y Sistemas
---

# Trabajo Final de Control y Sistemas
 
## Conclusión actual

La recomendación es **no cambiar todavía de proyecto**. El agitador magnético puede ser un trabajo final suficiente si el aporte deja de ser solamente “regular las RPM del motor” y pasa a estudiar la relación entre:

1. Velocidad del motor.
2. Velocidad real de la barra.
3. Carga viscosa y calidad de la agitación.
4. Pérdida de acoplamiento magnético.

El próximo paso recomendado es agregar **sensado de corriente de bus y tensión de bus**. Esto permite estudiar carga, potencia y desacople sin obligar desde el inicio a medir las tres corrientes de fase ni a migrar a FOC.

> [!important] Decisión provisional
> Implementar primero `Ibus + Vbus`, caracterizar el agitador y decidir después si conviene desarrollar un detector de desacople, un observador de velocidad o migrar al identificador de parámetros del motor.

## Navegación

- [[01 - Problema y criterio de suficiencia]]
- [[Alternativas/02 - Alternativas comparadas]]
- [[Sensado/03 - Sensado de corriente y sincronización del ADC]]
- [[Plan/04 - Ruta incremental recomendada]]
- [[Plan/05 - Plan experimental]]
- [[06 - Matriz de decisión]]
- [[Referencias/07 - Fuentes y archivos relevantes]]

## Documentos originales

- [[Anteproyecto_TF_CyS_2026_CASTEL_13784.pdf]]
- [[scope_control_y_sistemas]]

## Pregunta rectora sugerida

> ¿Es posible mantener y verificar una condición de agitación estable frente a cambios de viscosidad, estimando la carga y detectando la pérdida de acoplamiento mediante velocidad, corriente y potencia del accionamiento?

