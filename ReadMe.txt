Este proyecto implementa un sistema utilizando FreeRTOS con un eficiente mecanismo de sincronización entre tareas. Se han utilizado 2 mutex, 1 flag y 3 colas de mensajes para coordinar y garantizar la integridad de las operaciones entre las distintas tareas.

La placa STM32F411RE. 

Integración con Linux:
El sistema se comunica con un programa implementado para Linux, permitiendo su control mediante la terminal. Desde esta interfaz no gráfica (ui), se puede definir el tiempo de ejecución del programa, brindando flexibilidad en su gestión.


Configuración del Entorno de Desarrollo:

1. Abre el proyecto en la aplicación STM32 en la PC (Windows o Linux).
Carga del Binario en la Placa:

2. Actualiza la placa utilizando la aplicación JTAG.
Compilación del Programa Host:

3. Compila el programa host en una máquina Linux.
Conexión entre el Host y la Placa:

4. Conecta la placa al proyecto a través de un puerto serie en Linux y envíale el retardo que deseas que aplique. Si no deseas más datos, envía el código 00.


Pablo Mendoza y Asier Insausti
