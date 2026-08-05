---
tags:
  - alternativas
  - decisión
  - alcance
---

# Alternativas comparadas

## A. Continuar el agitador sin agregar sensado eléctrico

### Objetivo

Fortalecer el proyecto mediante modelado y validación de la barra, manteniendo el PI y la medición de velocidad actuales.

### Necesita

**Hardware**

- Maqueta actual.
- Medición externa y temporal de la barra: video, estroboscopio, sensor Hall o sensor óptico.
- Fluidos de prueba y temperatura registrada.

**Software y control**

- Modelo agregado motor-acoplamiento-fluido.
- Identificación de modelos para distintas viscosidades.
- Simulación del PI, saturación y perturbaciones.
- Registro de RPM, duty y estado del controlador.

**Validación**

- Comparar RPM del motor y RPM de la barra.
- Determinar la frontera de desacople.
- Ensayar distintas velocidades, viscosidades, volúmenes y separaciones.

### Pros

- Menor costo y tiempo.
- Reutiliza todo lo implementado.
- Mantiene alineación total con el anteproyecto.
- Puede ser suficiente si la validación es rigurosa.

### Contras

- Poca información interna sobre esfuerzo y carga.
- Detección de desacople difícil sin un sensor permanente.
- Menor contenido de instrumentación y procesamiento de señales.
- Puede continuar percibiéndose como simple si los ensayos son poco exigentes.

### Cuándo elegirla

Si el plazo es corto o si la medición externa demuestra una dinámica rica y suficiente para modelar y validar.

---

## B. Agitador con corriente y tensión de bus

### Objetivo

Estimar esfuerzo, potencia y carga para detectar cambios de viscosidad y pérdida de acoplamiento.

### Necesita

**Hardware**

- Un shunt o sensor de corriente de bus.
- Amplificador de corriente apropiado.
- Conexiones Kelvin y protección de la entrada ADC.
- Divisor resistivo para medir $V_{bus}$.
- Un canal ADC para corriente y otro para tensión.
- Protección de sobrecorriente independiente para fallas destructivas.

**Software y control**

- ADC sincronizado con TIM1.
- Calibración de offset y ganancia.
- Corriente media, RMS y pico.
- Potencia aproximada $P_{in}=V_{bus}I_{bus}$.
- Telemetría y registro junto con RPM y duty.
- Detector de desacople basado en múltiples variables.

**Validación**

- Medición externa de la barra como verdad de referencia.
- Ensayos con barra, sin barra, distintas viscosidades y desacoples provocados.
- Evaluar falsos positivos, tiempo de detección y repetibilidad.

### Pros

- Mejor relación entre complejidad y beneficio.
- Agrega ADC, acondicionamiento, calibración y DSP.
- Permite estudiar carga y potencia.
- Puede habilitar protección y limitación de corriente.
- Es suficiente para comenzar a detectar la barra.
- No exige rediseñar toda la estrategia sensorless.

### Contras

- La corriente de bus no identifica directamente cada corriente de fase.
- Tiene ventanas ciegas durante vectores cero y duties extremos.
- El desacople se detecta indirectamente y requiere calibración.
- La corriente cambia también por temperatura, tensión, fricción y errores de conmutación.

### Cuándo elegirla

Es la **alternativa recomendada como próximo paso**.

---

## C. Dos corrientes de fase y observador de velocidad/BEMF

### Objetivo

Estimar BEMF, posición y velocidad a partir del modelo eléctrico, usando los cruces por cero actuales como referencia de validación.

### Necesita

**Hardware**

- Dos shunts de fase, dos shunts de low-side o dos sensores inline.
- Dos cadenas de amplificación y protección.
- Dos canales ADC sincronizados; idealmente conversiones simultáneas.
- Medición de $V_{bus}$.
- Ancho de banda y settling compatibles con el PWM.

La tercera corriente se reconstruye mediante:

$$
i_U+i_V+i_W=0
$$

**Software y control**

- Reconstrucción de tensiones de fase usando $V_{bus}$, duty y sector.
- Parámetros aproximados $R$, $L$, $K_e$ y pares de polos.
- Observador de BEMF, Luenberger, sliding mode o equivalente.
- PLL o método de cálculo de ángulo y velocidad.
- Evaluación de confiabilidad a baja velocidad.
- Arranque open-loop y transición al observador.

**Validación**

- Comparación contra velocidad BEMF actual.
- Ensayos de transitorios, baja velocidad y cambios de carga.
- Análisis de error, ruido y pérdida de observabilidad.

### Pros

- Gran profundidad en modelado, estimación y control discreto.
- Permite comparar dos métodos sensorless.
- Puede mejorar diagnóstico y control de par.
- Conecta naturalmente con identificación de parámetros.

### Contras

- Mucho más complejo que medir carga.
- Corriente sola no alcanza: también se necesita tensión y modelo.
- La estimación basada en BEMF pierde calidad cerca de cero RPM.
- La derivada de corriente amplifica ruido.
- Las caídas del L298 y los diodos introducen errores importantes.
- Puede desviar el trabajo desde el agitador hacia el ESC.

### Cuándo elegirla

Como segunda etapa, luego de validar correctamente el sensado de corriente. No es necesaria para detectar inicialmente la barra.

---

## D. Tres shunts y control avanzado de corrientes/FOC

### Objetivo

Obtener máxima observabilidad de las corrientes y habilitar control vectorial o diagnóstico completo por fase.

### Necesita

**Hardware**

- Tres shunts y tres cadenas analógicas.
- ADC sincronizado y estrategia para seleccionar muestras válidas.
- Nueva etapa de potencia adecuada para FOC.
- Drivers con salidas complementarias y dead time real.
- Medición precisa de $V_{bus}$.
- Protección rápida de sobrecorriente.

**Software y control**

- Clarke y Park.
- Lazos de corriente $i_d/i_q$.
- Estimador de posición o sensor.
- SVPWM o modulación compatible.
- Identificación de parámetros eléctricos.
- Gestión precisa de tiempos y saturaciones.

### Pros

- Máxima profundidad técnica.
- Control directo de par.
- Mayor flexibilidad para observadores y diagnósticos.
- Puede producir un ESC considerablemente más avanzado.

### Contras

- Tres corrientes no son matemáticamente necesarias: dos bastan para reconstruir la tercera.
- Alto riesgo de expansión de alcance.
- La etapa actual con L298 no es una base ideal para FOC.
- Requiere mucho hardware, firmware y validación.
- Puede hacer que el agitador quede relegado a una aplicación secundaria.

### Cuándo elegirla

Solo si se decide explícitamente que el eje del trabajo será el accionamiento del motor y existe tiempo para rediseñar la etapa de potencia.

---

## E. Migrar a un identificador de parámetros BLDC/PMSM

### Objetivo

Identificar parámetros como $R$, $L$, $K_e/K_t$, $J$ y $B$ mediante ensayos e inyección de señales.

### Necesita

**Hardware**

- Dos corrientes de fase o una estrategia equivalente bien justificada.
- Medición de $V_{bus}$ y reconstrucción de tensión aplicada.
- Protección de sobrecorriente.
- Posiblemente bloqueo mecánico, sensor de velocidad externo o banco de carga.
- Etapa de potencia suficientemente conocida y repetible.

**Software y control**

- Secuencias seguras de inyección de tensión.
- Identificación offline u online.
- Compensación de caídas del inversor y temperatura del bobinado.
- Estimación de incertidumbre y repetibilidad.
- Uso posterior de parámetros en un modelo, observador o controlador.

**Validación**

- Comparar parámetros con ensayos alternativos.
- Validar predicción de corriente, velocidad o respuesta dinámica.
- Mostrar que los parámetros mejoran un controlador u observador.

### Pros

- Problema académico claro de identificación de sistemas.
- Justifica fuertemente el sensado de corriente.
- Produce parámetros útiles para simulación y observadores.
- Puede transformarse en una herramienta reutilizable para distintos motores.

### Contras

- No es automáticamente más propio de Control y Sistemas.
- Si los parámetros no se usan después, puede quedar como instrumentación aislada.
- Alta sensibilidad a temperatura, caídas del L298 y errores de tensión.
- Requiere cambiar el objetivo y buena parte del anteproyecto.
- Mayor riesgo y menor reutilización del agitador ya construido.

### Cuándo elegirla

Si los ensayos muestran que el agitador no produce un problema de carga observable, o si se decide deliberadamente concentrar el trabajo en identificación y control de motores.

