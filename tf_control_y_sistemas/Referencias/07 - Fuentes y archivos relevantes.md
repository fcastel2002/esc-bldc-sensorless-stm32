---
tags:
  - referencias
  - documentación
---

# Fuentes y archivos relevantes

## Dentro de esta vault

- [[Anteproyecto_TF_CyS_2026_CASTEL_13784.pdf]] — propuesta original del trabajo final.
- [[scope_control_y_sistemas]] — contenidos mínimos y alcance general de la materia.

## Documentación del repositorio

Estas rutas están fuera de la vault, en el repositorio principal:

- `docs/PROJECT_FLOW.md` — arquitectura del firmware, control PI y medición de velocidad.
- `firmware/COMM_PROTOCOL.md` — protocolo, telemetría y HIL.
- `firmware/Core/Src/bldc_driver.c` — secuencia six-step y PWM por fase.
- `firmware/Core/Src/rpm_pi_controller.c` — PI discreto Q16.16 y anti-windup.
- `firmware/Core/Src/speed_sensor.c` — medición de velocidad por BEMF.
- `firmware/Core/Src/tim.c` — TIM1 center-aligned y canales PWM.
- `firmware/firmware_esc_bldc_v1.ioc` — asignación de pines y periféricos.
- `INFORME/ESC_Bldc___paper_congreso_utn___en_revision.pdf` — informe técnico previo del ESC.
- `INFORME/ESC_Bldc___paper_congreso_utn___en_revision/imagenes_circuitos/esquematico_2zc.pdf` — comparadores de BEMF.

## Documentación técnica externa

### Sensado de corriente

- [ST AN5397 — Current sensing in motion control applications](https://www.st.com/resource/en/application_note/dm00653792-current-sensing-in-motion-control-applications-stmicroelectronics.pdf)
- [ST AN4076 — Two or three shunt resistor based current sensing](https://www.st.com/resource/en/application_note/dm00051150-two-or-three-shunt-resistor-based-current-sensing-circuit-design-in-3phase-inverters-stmicroelectronics.pdf)
- [ST UM1052 — STM32 PMSM FOC SDK](https://www.st.com/resource/en/user_manual/um1052-stm32f-pmsm-singledual-foc-sdk-v43-stmicroelectronics.pdf)
- [ST L298 — página de producto y sensado](https://www.st.com/en/motor-drivers/l298.html)
- [ST L298 — datasheet](https://www.st.com/resource/en/datasheet/l298.pdf)

### ADC y temporizadores

- [ST RM0008 — STM32F101/102/103 Reference Manual](https://www.st.com/resource/en/reference_manual/rm0008-stm32f101xx-stm32f102xx-stm32f103xx-stm32f105xx-and-stm32f107xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)

### Control sensorless

- [ST — 6-step Firmware Algorithm](https://wiki.st.com/stm32mcu/wiki/STM32MotorControl%3A6-step_Firmware_Algorithm)
- [Microchip — Rotor Position and Speed Estimation](https://onlinedocs.microchip.com/oxy/GUID-69278EBA-9F00-4CEE-8103-2998236856BD-en-US-2/GUID-8F3C253A-0421-4746-963B-8E4175B44C67.html)
- [Microchip — Sensorless FOC using Sliding Mode Observer](https://www.microchip.com/content/dam/mchp/documents/MCU32/ApplicationNotes/ApplicationNotes/Sensorless-Field-Oriented-Control-for-a-Permanent-Magnet-Synchronous-Motor-Using-Sliding-Mode-DS00004398.pdf)

### Agitación y estimación de viscosidad

- [Real-time rheological monitoring with the smart stirrer](https://wrap.warwick.ac.uk/id/eprint/184482/)
- [IKA — agitador con detección de pérdida de barra](https://www.ika.com/en/Products-LabEq/Magnetic-Stirrers-pg188/I-MAG-Industry-stirrer-20014226/Technical-Data-cptd.html)

## Afirmaciones que deben validarse experimentalmente

- La corriente aumenta de forma monotónica con la viscosidad en la maqueta concreta.
- El desacople produce una caída suficientemente clara de corriente o potencia.
- La corriente de bus permite distinguir desacople de una perturbación normal.
- La velocidad del motor representa adecuadamente la velocidad de la barra dentro del rango elegido.
- El front-end posee suficiente ancho de banda y relación señal-ruido para un observador.

