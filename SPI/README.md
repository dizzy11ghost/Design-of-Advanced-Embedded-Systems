*Miguel Alonso De La Rosa Zamora A01646106 - Sophia Leñero Gómez A01639462 - Gerardo Maximiliano López Herrera A01614489*
# Laboratorio de interfaces seriales
## Parte 2 - SPI

### 1. Ejemplos
*Descripción*
Implementación del ejemplo de display de 7 segmentos usando el driver MAX7219 con la comunicación SPI. El código hace funcionar 4 digitos en total. Se verificó el correcto funcionamiento probando diferentes valores en el display y asegurando que cada dígito fuera controlado de manera adecuada.

*Diagrama de conexiones para sólo 2 displays*

<img width="618" height="511" alt="image" src="https://github.com/user-attachments/assets/d4e7598d-56d8-4d94-b090-dbaeaf88441a" />

*Evidencia de funcionamiento*

<img width="1600" height="900" alt="image" src="https://github.com/user-attachments/assets/93b905a6-bb54-4c3c-b406-2f8ce1bd555e" />


### 2. Contador de 4 digitos
*Descripción*
Utilizando el código de la primera parte, se realizó la implementación de un contador de cuatro dígitos usando el MAX7219. El contador funciona utilizando switches y tiene la capacidad de realizar lo siguiente;
-Inicio / pausado del contador
-Incremento / decremento
-Ajuste manual mientras el contador yace pausado
-Reinicio

*Evidencia de funcionamiento*
https://github.com/user-attachments/assets/57e6635f-afee-48d9-ba04-7f18ae4cc40c


### 3. Aplicación - sistema de monitoreo
*Descripción*
Para esta parte de la práctica se desarrolló un sistema de monitoreo capaz de recibir un valor usando un ADC y que muestra en display de 7 segmentos 4 digitos. El sistema permite la configuración de un umbral usando el keypad. El led RGB indica el modo de operació, y en modo normal, indica cuando el valor medido sobrepasa el rango establecido. El sistema además mantiene registro de los valores mínimo y máximo del ADC medidos desde que el sistema empezó o desde que los min/max fueron reseteados.
Los modos de operación fueron los siguientes (permitiendo al usuario cambiar entre ellos por medio de un botón):
-Modo normal: el display muestra la lectura del ADC, el led se enciende cuando el valor leido es mayor o igual al umbral.
-Modo mínimo: el display muestra el valor más bajo grabado hasta ahora y el LED usa otro color para indicar el modo mínimo.
-Modo máximo: el display muestra el valor más alto grabado hasta ahora y el LED usa otro color para indicar el modo máximo.
-Modo de umbral: Le permite al usuario ingresar y configurar el umbral usando el keypad.
Al principio, los primeros datos recibidos del ADC representan tanto el valor máximo como el mínimo.
Hay un pushbutton que sirve para resetear el máximo y mínimo, y la comparasión hacia el umbral tiene que ser actualizada periódicamente sin que se bloquee la operación y uso del keypad o de los pushbuttons.
La implementación por lo tanto evita usar delays largos bloqueantes que le impidan al usuario interactuar con el sistema.


*Evidencia de funcionamiento*
https://github.com/user-attachments/assets/11b33cb0-075b-416c-ab18-9b82c3aacfd4



