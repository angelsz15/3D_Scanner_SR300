import pyrealsense2 as rs

def check_devices():
    print("Iniciando escaneo de dispositivos RealSense...")
    try:
        ctx = rs.context()
        devices = ctx.query_devices()
        print(f"Cámaras RealSense detectadas por la librería Python: {len(devices)}")
        if len(devices) == 0:
             print("Aviso: La librería pyrealsense2 no ve ninguna cámara.")
        else:
            for i, dev in enumerate(devices):
                print(f"\n[{i}] Dispositivo encontrado:")
                print(f"  - Nombre: {dev.get_info(rs.camera_info.name)}")
                print(f"  - Serial: {dev.get_info(rs.camera_info.serial_number)}")
                print(f"  - Firmware: {dev.get_info(rs.camera_info.firmware_version)}")
                print(f"  - Producto: {dev.get_info(rs.camera_info.product_line)}")
    except Exception as e:
        print(f"Error durante el escaneo: {e}")

if __name__ == "__main__":
    check_devices()
