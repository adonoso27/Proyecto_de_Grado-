# INSTALL.md  
# Instructivo de instalación y ejecución del sistema UR30 + OAK-D-Lite + Modbus/TCP

Este documento explica, a modo de **tutorial paso a paso**, cómo reproducir el sistema de visión
y control robótico del proyecto de grado:

**“Desarrollo de un sistema inteligente de control para un robot UR30”**  
Autor: **Alejandro José Donoso Pérez – Universidad de los Andes (2024–2025)**

El objetivo es que cualquier usuario pueda:

- Conectar su PC con el UR30 mediante Modbus/TCP  
- Configurar el cliente Modbus en PolyScope  
- Ejecutar el servidor Modbus (`modbus.py`)  
- Usar la cámara OAK-D-Lite como sensor de visión con `dados.py`  
- Cargar el programa URScript en el UR30  
- Completar el ciclo percepción → decisión → acción con cubos de colores  


---

## 🚀 Pasos a seguir (vista general)

1. Preparar el entorno de Python en el PC.  
2. Configurar la red entre el PC y el UR30 (Modbus/TCP).  
3. Configurar el cliente Modbus en PolyScope.  
4. Ejecutar el servidor Modbus (`modbus.py`) en el PC.  
5. Comprobar la cámara OAK-D-Lite.  
6. Ejecutar el pipeline de visión (`dados.py`).  
7. Cargar y ejecutar el programa URScript en el UR30.  
8. Colocar un cubo y observar la clasificación automática.  


---

## 📌 1. Requisitos de hardware

- Robot colaborativo **UR30**  
- **Control Box UR** con puerto Ethernet  
- Cámara **OAK-D-Lite RGB-D**  
- PC con **Ubuntu 22.04.5 LTS** (o equivalente Linux)  
- Cable **Ethernet directo** PC ↔ Control Box  
- Cable **USB-C** para conectar la OAK-D-Lite al PC  


---

## 📌 2. Preparación del entorno Python en el PC

1. Abrir una terminal en el directorio del proyecto.  
2. Crear y activar un entorno virtual de Python:

   ```bash
   python3 -m venv venv
   source venv/bin/activate
