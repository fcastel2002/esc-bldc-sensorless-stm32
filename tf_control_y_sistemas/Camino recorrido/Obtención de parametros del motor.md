
Como una primera iteración de este problema voy a obtener los parametros con un multimetro y un osciloscopio.
Lo ideal es que en un futuro la calibración (obtención de parametros) se haga de forma automatica con una tecnica llamada High Freq Injection.


**Problemas con los que encontre**

simplemente al graficar en el tiempo la bemf de mi motor haciendolo girar con un taladro electrico, se ve una forma de onda bastante particular![[DSOBMP0003.bmp]]
No es ni sinusoidal ni trapezoidal pura. Por lo que es complicado obtener el valor que pide para el bloque de "Angulo electrico durante el cual la BEMF es constante".

Ya pude medir la R_s y la L_s. Ahora seria interesante pasar esos valores al sistema dq. 

Todavía no se muy bien que estrategia voy a seguir pero creo que será util tener todos los parametros bien identificados aunque luego no los use. Además la idea es trabajar principalmente con processor in the loop.
