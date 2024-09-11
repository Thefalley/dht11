# Sistema con FreeRTOS y STM32F411RE
[![Linux Version](https://img.shields.io/pypi/pyversions/nd2.svg?color=green)]([https://python.org](https://www.linux.org/))

Este proyecto implementa un sistema mediante FreeRTOS con eficientes mecanismos de sincronización entre tareas. Se han empleado 2 mutex, 1 flag y 3 colas de mensajes para coordinar y garantizar la integridad de las operaciones entre tareas.

## Placa Utilizada
- STM32F411RE.

## Integración con Linux
El sistema se comunica con un programa para Linux, permitiendo su control desde la terminal. Esta interfaz no gráfica (ui) facilita la definición del tiempo de ejecución del programa, brindando flexibilidad en su gestión.

## Configuración del Entorno de Desarrollo

1. Abre el proyecto en la aplicación STM32 en la PC (Windows o Linux).

## Carga del Binario en la Placa

2. Actualiza la placa utilizando la aplicación JTAG.

## Compilación del Programa Host

3. Compila el programa host en una máquina Linux.

## Conexión entre el Host y la Placa

4. Conecta la placa al proyecto mediante un puerto serie en Linux. Envía el retardo deseado; si no deseas más datos, envía el código 00.

---

**Desarrolladores:**
- Pablo Mendoza
- Asier Insausti

