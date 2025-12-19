# Simulación del Bug de Aceleración - EDU-CIAA

Este directorio contiene una simulación para PC del error de "Aceleración pegada" detectado en el proyecto.

## Archivos
- **main_sim.c**: El programa principal que ejecuta la prueba. Simula enviar aceleración válida y luego "ruido".
- **sim_core.c**: La lógica exacta del firmware (portada de `app.c`), incluyendo el bug.
- **mock_os.c**: Simulación del sistema operativo (FreeRTOS) y hardware.

## Cómo Compilar y Correr

### Opción 1: Si tienes GCC en tu PATH (Comando estándar)
1. Abre una terminal en esta carpeta (`f:\proyectos\car\TallerdeProyecto1`).
2. Compila:
   ```cmd
   gcc simulation/main_sim.c simulation/sim_core.c simulation/mock_os.c -o simulation/bug_sim.exe
   ```
3. Ejecuta:
   ```cmd
   simulation/bug_sim.exe
   ```

### Opción 2: Usando la ruta absoluta (Si el comando 'gcc' falla)
Detectamos que tienes MinGW instalado en `C:\MinGW`. Usa este comando si el anterior no funciona:

1. Compila:
   ```cmd
   C:\MinGW\bin\gcc.exe simulation/main_sim.c simulation/sim_core.c simulation/mock_os.c -o simulation/bug_sim.exe
   ```
2. Ejecuta:
   ```cmd
   simulation/bug_sim.exe
   ```

## Interpretación de Resultados

- **FAIL**: Si ves `Throttle=1020` (o cualquier valor alto constant) mientras dice "Injecting NOISE", el bug persiste.
- **SUCCESS**: Si ves `Throttle=0` poco después de que empiece el ruido, y luego recupera valor con "VALID Data", el fix funciona.
