---
tags:
  - adc
  - corriente
  - pwm
  - stm32
---

# Sensado de corriente y sincronización del ADC

## Regla principal

El ADC debe sincronizarse con el PWM, pero **no debe muestrear exactamente sobre un flanco**.

La secuencia correcta es:

```text
flanco de conmutación → blanking → señal estable → adquisición ADC
```

El tiempo de blanking debe determinarse con osciloscopio. Depende del L298, diodos, amplificador de corriente, filtros y layout.

## Qué sucede en un paso six-step

Ejemplo `POS_UV`:

- U = PWM.
- V = LOW.
- W = flotante.

```text
                 conmutación       ventana estable       conmutación
PWM U        ____|‾‾‾‾‾‾‾‾‾‾‾‾‾‾|________________
                 ↑ ruido             ↑ muestra          ↑ ruido

VUV ideal        ≈ +Vbus                  ≈ 0
Corriente fase   aumenta/rizado           recircula
Corriente bus    ≈ corriente fase         ≈ 0
```

Al apagar el PWM, las dos fases energizadas quedan idealmente en LOW. La tensión línea-línea impuesta se aproxima a cero, pero la corriente inductiva sigue circulando por transistores y diodos.

> [!warning] Confusión habitual
> El intervalo PWM OFF puede ser apropiado para medir la BEMF de la fase flotante, pero un shunt de bus normalmente debe medirse durante el intervalo PWM ON. Durante el vector cero puede existir corriente de fase y simultáneamente corriente de bus casi nula.

## Topologías posibles

| Topología | Canales | Ventaja | Limitación principal |
|---|---:|---|---|
| Shunt de bus | 1 | Simple y suficiente para carga/potencia | Ventanas ciegas y reconstrucción limitada |
| Dos shunts de fase | 2 | Reconstruye las tres corrientes | Muestreo dependiente del estado de los switches |
| Tres shunts de low-side | 3 | Más opciones de muestreo y diagnóstico | Mayor hardware y calibración |
| Sensores inline/Hall | 2 o 3 | Corriente observable durante más estados | Costo, offset y ancho de banda |

## Selección recomendada para la primera etapa

### Señales

- $I_{bus}$.
- $V_{bus}$.
- RPM por BEMF.
- Duty PWM.
- Sector de conmutación.
- Calidad de los cruces por cero.

### Magnitudes calculadas

$$
P_{in}\approx V_{bus}I_{bus}
$$

- Corriente media.
- Corriente RMS.
- Corriente pico.
- Potencia media.
- Duty requerido a velocidad constante.
- Variaciones rápidas asociadas a desacople.

## Disparo con TIM1

TIM1 utiliza actualmente los canales 1, 2 y 3 para el PWM center-aligned. El canal 4 puede emplearse como canal interno de temporización:

1. Configurar TIM1_CH4 como compare interno.
2. Ubicar `CCR4` dentro de la ventana válida.
3. Usar el evento para disparar conversiones inyectadas del ADC.
4. Leer el resultado por interrupción o mediante la estrategia DMA disponible.
5. Reubicar el punto cuando cambie el duty.

No es necesario sacar TIM1_CH4 a un pin físico.

## Ventanas extremas

Para 18 kHz:

$$
T_{PWM}\approx55.6\ \mu s
$$

Con un duty de 5 %:

$$
T_{ON}\approx2.8\ \mu s
$$

Esta ventana puede ser menor que la suma de:

- Retardo de conmutación.
- Blanking.
- Settling del amplificador.
- Adquisición y conversión ADC.

Las muestras inválidas deben detectarse y descartarse. Para telemetría pueden filtrarse varios ciclos; para un observador rápido puede ser necesario limitar duty, cambiar topología o usar sensores de fase.

## Estimar BEMF desde corrientes

El modelo eléctrico es:

$$
v=Ri+L\frac{di}{dt}+e
$$

Por lo tanto:

$$
\hat e=v-Ri-L\frac{di}{dt}
$$

Durante un vector cero puede aproximarse $v\approx0$, pero siguen existiendo caídas en transistores y diodos. Además, una única muestra no permite conocer $di/dt$.

Para un estimador real se necesitan:

- Dos corrientes de fase.
- $V_{bus}$ y estados PWM.
- $R$, $L$ y pares de polos.
- Observador o filtrado robusto.
- Transición desde arranque open-loop.
- Comparación contra la medición BEMF existente.

La corriente por sí sola no entrega directamente velocidad. La frecuencia de conmutación visible en la corriente puede ser la frecuencia impuesta por el controlador y no la velocidad mecánica real durante una desincronización.

## Requisitos analógicos mínimos

- Shunt dimensionado por caída y disipación:

$$
V_{shunt}=I_{max}R_{shunt}
$$

$$
P_{shunt}=I_{RMS}^{2}R_{shunt}
$$

- Amplificador con rango de modo común y ancho de banda apropiados.
- Salida limitada al rango 0–3.3 V del ADC.
- Protección ante transitorios y posibles excursiones negativas.
- Conexiones Kelvin.
- Masa analógica cuidada.
- Calibración de cero y ganancia.
- Fusible o protección rápida independiente del firmware.

