import serial
import serial.tools.list_ports
import threading
import time

class IMUReader:
    def __init__(self, baudrate=115200):
        self.baudrate = baudrate
        self.serial_port = None
        self.is_running = False
        self.thread = None
        
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.lock = threading.Lock()
        
        self.connect()

    def connect(self):
        ports = serial.tools.list_ports.comports()
        for p in ports:
            if "CH340" in p.description or "Serial Port" in p.description or "COM" in p.device:
                try:
                    self.serial_port = serial.Serial(p.device, self.baudrate, timeout=1)
                    print(f"[IMU] Conectado exitosamente al puerto {p.device}")
                    break
                except Exception as e:
                    print(f"Ignorando puerto {p.device}: {e}")
                    
        if not self.serial_port:
            print("[IMU] ADVERTENCIA: No se encontró ningún dispositivo IMU. El escáner funcionará sin estabilización.")

    def start(self):
        if self.serial_port:
            self.is_running = True
            self.thread = threading.Thread(target=self._read_loop, daemon=True)
            self.thread.start()
            
            # Autocalibrar al iniciar
            print("[IMU] Solicitando calibración inicial...")
            self.send_command('c')

    def stop(self):
        self.is_running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        if self.serial_port:
            self.serial_port.close()

    def send_command(self, cmd):
        if self.serial_port:
            try:
                self.serial_port.write(cmd.encode())
            except Exception as e:
                pass

    def _read_loop(self):
        while self.is_running:
            try:
                if self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                    if line.startswith("YAW:"):
                        parts = line.split(',')
                        if len(parts) == 3:
                            yaw_val = float(parts[0].split(':')[1])
                            pitch_val = float(parts[1].split(':')[1])
                            roll_val = float(parts[2].split(':')[1])
                            
                            with self.lock:
                                self.yaw = yaw_val
                                self.pitch = pitch_val
                                self.roll = roll_val
                else:
                    # Prevenir que el hilo ahogue al procesador
                    time.sleep(0.005)
            except Exception as e:
                time.sleep(0.01)

    def get_pose(self):
        """Devuelve (yaw, pitch, roll) en grados"""
        with self.lock:
            return self.yaw, self.pitch, self.roll
