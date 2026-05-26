#include <Wire.h>

const int MPU = 0x68; // Dirección I2C del MPU6050

// Variables para los datos RAW
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

// Offsets de calibración
float gyro_x_offset = 0, gyro_y_offset = 0, gyro_z_offset = 0;
float accel_x_offset = 0, accel_y_offset = 0;

// Ángulos calculados
float roll = 0, pitch = 0, yaw = 0;

unsigned long tiempo_previo = 0;

void setup() {
  Serial.begin(115200);
  
  // En muchos ESP32-C3 los pines I2C por defecto son SDA=8, SCL=9. 
  // Si tu placa usa otros (ej SDA=4, SCL=5), cámbialo a Wire.begin(4, 5);
  Wire.begin();
  
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // Registro PWR_MGMT_1
  Wire.write(0);     // Resetea a 0 para encender el MPU
  Wire.endTransmission(true);

  // Filtro pasa bajas digital (DLPF) para reducir ruido
  Wire.beginTransmission(MPU);
  Wire.write(0x1A); // CONFIG
  // 0x05 recorta todo por encima de 10Hz. Esto ELIMINARÁ la VIBRACIÓN casi por completo.
  Wire.write(0x05); 
  Wire.endTransmission(true);

  // Escala del Giroscopio a +/- 500 °/s
  Wire.beginTransmission(MPU);
  Wire.write(0x1B); // GYRO_CONFIG
  Wire.write(0x08); 
  Wire.endTransmission(true);

  delay(1000);
  calibrar_imu();
  tiempo_previo = millis();
}

void calibrar_imu() {
  Serial.println("CALIBRANDO IMU. NO MOVER...");
  long sum_gx = 0, sum_gy = 0, sum_gz = 0;
  long sum_ax = 0, sum_ay = 0;
  int num_muestras = 500;
  
  for(int i = 0; i < num_muestras; i++){
    Wire.beginTransmission(MPU);
    Wire.write(0x3B); // Acelerómetro
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 14, true);
    
    int16_t ax = Wire.read() << 8 | Wire.read();
    int16_t ay = Wire.read() << 8 | Wire.read();
    int16_t az = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read(); // ignorar temp
    int16_t gx = Wire.read() << 8 | Wire.read();
    int16_t gy = Wire.read() << 8 | Wire.read();
    int16_t gz = Wire.read() << 8 | Wire.read();

    sum_ax += ax;
    sum_ay += ay;
    sum_gx += gx;
    sum_gy += gy;
    sum_gz += gz;
    delay(3);
  }
  
  accel_x_offset = (float)sum_ax / num_muestras;
  accel_y_offset = (float)sum_ay / num_muestras;
  
  gyro_x_offset = (float)sum_gx / num_muestras;
  gyro_y_offset = (float)sum_gy / num_muestras;
  gyro_z_offset = (float)sum_gz / num_muestras;
  
  Serial.println("CALIBRACION FINALIZADA.");
  tiempo_previo = micros();
}

void loop() {
  // Comandos seriales
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'c' || c == 'C') {
      calibrar_imu();
      yaw = 0;
      roll = 0;
      pitch = 0;
      tiempo_previo = micros();
    }
  }

  // Leer datos
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // Empieza con el registro de Aceleración
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);

  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Tmp = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();

  unsigned long tiempo_actual = micros();
  float dt = (tiempo_actual - tiempo_previo) / 1000000.0;
  tiempo_previo = tiempo_actual;

  // Si hubo un salto extraño o dt escandaloso, ignorar
  if (dt > 0.1) dt = 0.01;

  // Convertir a grados/s
  // Rango de 500 deg/s -> divisor es 65.5
  float raw_gx = (GyX - gyro_x_offset) / 65.5;
  float raw_gy = (GyY - gyro_y_offset) / 65.5;
  float raw_gz = (GyZ - gyro_z_offset) / 65.5;

  // Calcular fuerza g usando el acelerómetro (eliminando el offset de reposo)
  float raw_ax = (AcX - accel_x_offset) / 16384.0;
  float raw_ay = (AcY - accel_y_offset) / 16384.0;
  float raw_az = AcZ / 16384.0; // En reposo z debe ser 1g, no requiere centrado al 0 para rotación simple
  
  // ======================================================================
  // ¡AQUÍ PUEDES INTERCAMBIAR TUS EJES SI SOLDAS LA IMU GIRADA!
  // Configuración actual: Montaje Vertical según las pruebas reales
  // ======================================================================
  float gx = -raw_gz;   
  float gy = -raw_gy;   
  float gz = raw_gx;   

  float ax = -raw_ay;   
  float ay = -raw_ax;   
  float az = raw_az;   
  // ======================================================================

  // Deadband (Zona Muerta): Si el giro es menor a 0.5 grados por segundo, considérarlo cero
  // Esto ELIMINA por completo el "Drift" (que pierda referencia) cuando está quieto
  if (abs(gx) < 0.5) gx = 0;
  if (abs(gy) < 0.5) gy = 0;
  if (abs(gz) < 0.5) gz = 0;

  float accel_pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;
  float accel_roll = atan2(ay, az) * 180 / PI;

  // Filtro Complementario (99% gyro y 1% accel). Eliminará cualquier resto de vibración.
  roll = 0.99 * (roll + gx * dt) + 0.01 * accel_roll;
  pitch = 0.99 * (pitch + gy * dt) + 0.01 * accel_pitch;
  
  // Para yaw solo usamos el giroscopio
  yaw += gz * dt; 

  Serial.print("YAW:");
  Serial.print(yaw);
  Serial.print(",PITCH:");
  Serial.print(pitch);
  Serial.print(",ROLL:");
  Serial.println(roll);

  // Leer lo mas rapido posible para mejorar la precision de integracion (aprox 100Hz+)
  delay(5);
}
