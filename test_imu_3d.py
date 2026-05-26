import serial
import serial.tools.list_ports
import time
import math
import open3d as o3d
import numpy as np
import threading
import sys

# Variables globales para los ángulos
current_yaw = 0.0
current_pitch = 0.0
current_roll = 0.0
is_running = True

def read_serial_data(port_name):
    global current_yaw, current_pitch, current_roll, is_running
    try:
        ser = serial.Serial(port_name, 115200, timeout=1)
        print(f"Conectado a {port_name}")
        
        # Enviar comando de calibración al inicio
        print("\n--- INICIANDO CALIBRACIÓN PREVIA ---")
        print("MANTEN LA PLACA COMPLETAMENTE PLANA Y QUIETA.")
        ser.write(b'c')
        
        while is_running:
            try:
                line = ser.readline().decode('utf-8').strip()
                if line.startswith("YAW:"):
                    # Ejemplo: YAW:10.5,PITCH:2.1,ROLL:-1.2
                    parts = line.split(',')
                    if len(parts) == 3:
                        yaw = float(parts[0].split(':')[1])
                        pitch = float(parts[1].split(':')[1])
                        roll = float(parts[2].split(':')[1])
                        
                        current_yaw = yaw
                        current_pitch = pitch
                        current_roll = roll
                elif line:
                    print(f"[ESP32]: {line}")
            except Exception as e:
                pass
    except Exception as e:
        print(f"Error al abrir puerto {port_name}: {e}")
        is_running = False

def create_arrow(color, translation=[0, 0, 0]):
    arrow = o3d.geometry.TriangleMesh.create_arrow(cylinder_radius=0.05, cone_radius=0.1, cylinder_height=0.6, cone_height=0.4)
    arrow.compute_vertex_normals()
    arrow.paint_uniform_color(color)
    arrow.translate(translation)
    return arrow

def main():
    global is_running
    print("Buscando puertos...")
    ports = serial.tools.list_ports.comports()
    target_port = None
    for p in ports:
        if "CH340" in p.description or "Serial Port" in p.description or "COM" in p.device:
            target_port = p.device
            break

    if target_port is None:
        print("No se encontró el ESP32. Asegúrese de que está conectado.")
        return

    # Iniciar hilo de lectura del serial
    t = threading.Thread(target=read_serial_data, args=(target_port,), daemon=True)
    t.start()

    # Visualización Open3D
    print("Iniciando visor 3D. Cierra la ventana para salir.")
    
    # Ejes base de referencia
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
    
    # Objeto que va a rotar con el IMU
    box = o3d.geometry.TriangleMesh.create_box(width=1.0, height=0.2, depth=0.6)
    box.translate([-0.5, -0.1, -0.3])
    box.paint_uniform_color([0.8, 0.2, 0.2])
    box.compute_vertex_normals()

    vis = o3d.visualization.Visualizer()
    vis.create_window("Prueba IMU - Posicion 3D", width=800, height=600)
    vis.add_geometry(coord_frame)
    vis.add_geometry(box)

    # Variables para deshacer la rotación de la iteración anterior
    last_R = np.identity(3)

    try:
        while is_running:
            # Obtener rotación en radianes
            r_rad = math.radians(current_roll)
            p_rad = math.radians(current_pitch)
            y_rad = math.radians(current_yaw)

            # Convertir euler (Open3D usa ZYX por defecto, aquí podemos crear la matriz manualmente)
            R = box.get_rotation_matrix_from_xyz((p_rad, y_rad, r_rad))

            # Aplicar la matriz relativa para el paso actual
            # Primero deshacemos la rotación anterior y luego aplicamos la nueva
            box.rotate(np.linalg.inv(last_R), center=(0,0,0))
            box.rotate(R, center=(0,0,0))
            last_R = R.copy()

            vis.update_geometry(box)
            if not vis.poll_events():
                break
            vis.update_renderer()
            
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("Saliendo...")
    
    is_running = False
    vis.destroy_window()

if __name__ == "__main__":
    main()
