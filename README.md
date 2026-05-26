# 🔴 Escáner 3D Continuo con Intel RealSense SR300

Programa en Python para escanear objetos en 3D de forma continua, usando la cámara **Intel RealSense SR300**. 
El usuario graba un vídeo dando la vuelta al objeto, y el sistema reconstruye automáticamente una nube de puntos 3D completa fusionando todos los fotogramas.

---

## 📋 Requisitos

### Hardware
- Intel RealSense SR300 conectada por **USB 3.0** (puerto azul)
- Módulo IMU (MPU6050 + ESP32-C3) conectado por USB (Opcional pero altamente recomendado para evitar derivas al girar).
- El Intel RealSense Viewer debe estar **cerrado** al ejecutar el script

### Software
El proyecto usa un entorno virtual basado en Python 3.9 para compatibilidad con la SR300:

```bash
# Instalar dependencias (si no está ya creado el entorno env_sr300)
pip install uv
uv venv --python 3.9 env_sr300
uv pip install --python env_sr300 pyrealsense2==2.50.0.3812 opencv-python open3d numpy
```

---

## 🚀 Cómo Usar

```bash
.\env_sr300\Scripts\python.exe escanner_3d.py
```

### Controles

| Tecla | Acción |
|-------|--------|
| **C** | Calibrar el IMU (dejar el escáner sobre la mesa sin mover) |
| **R** | Empezar / Parar grabación 3D |
| **Q / ESC** | Salir del programa |

### Flujo de trabajo

1. Coloca el objeto sobre una superficie y **apunta la cámara** a menos de 60 cm.
2. Pulsa **R** para empezar a grabar (¡aparecerá un punto rojo en pantalla!).
3. Muévete **lentamente** alrededor del objeto (~30 segundos para un escaneo 360°).
4. Pulsa **R** de nuevo para parar y construir el modelo.
5. El archivo `.ply` se guardará en la carpeta `escaneos/`.
6. Se abrirá un visor 3D interactivo para ver el resultado.

> **Tip:** Cuantas más texturas y detalles tenga el objeto (colores, marcas, bordes), mejor funcionará el algoritmo de alineación.

---

## 📁 Estructura del Proyecto

```
3D_Scanner/
├── escanner_3d.py      # Script principal de escaneo
├── imu_reader.py       # Hilo en segundo plano para leer datos del ESP32
├── test_imu_3d.py      # Script de prueba visual para verificar la orientación del IMU
├── check_camera.py     # Script de diagnóstico de la cámara
├── imu_esp32/
│   └── imu_esp32.ino   # Firmware para el ESP32-C3 y el MPU6050 (Filtro complementario)
├── env_sr300/          # Entorno virtual Python 3.9 (pyrealsense2==2.50)
└── escaneos/           # Carpeta donde se guardan los resultados .ply
```

---

## ⚙️ Arquitectura y Funciones Internas

### `main()` — Bucle principal de captura

Es el punto de entrada del programa. Se encarga de:

1. **Inicializar el pipeline de RealSense** (`rs.pipeline`): Configura los streams de color (640×480, BGR8, 30 fps) y profundidad (640×480, Z16, 30 fps).

2. **Alineación color↔profundidad** (`rs.align`): El sensor RGB y el sensor de infrarrojos tienen ópticas separadas físicamente en la cámara. Los fotogramas crudos no están en el mismo espacio. La clase `rs.align` reproyecta el mapa de profundidad sobre el plano de la cámara de color usando la calibración de fábrica (extrínseca entre sensores), de modo que cada píxel RGB tiene exactamente su correspondiente valor de profundidad en metros.

3. **Submuestreo temporal**: Para no saturar la RAM y acelerar el proceso de ICP posterior, solo se guarda **1 de cada 5 frames** (de 30 fps a ~6 muestras/sec), controlado por `record_timer` y `record_interval = 5`.

4. **Construcción de la nube de puntos RGBD** (`o3d.geometry.RGBDImage`): Siendo `z` la profundidad en metros leída del sensor, cada píxel `(u, v)` de la imagen se convierte a un punto 3D `(X, Y, Z)` usando el modelo de cámara pinhole:

$$
X = \frac{(u - c_x) \cdot z}{f_x}, \quad Y = \frac{(v - c_y) \cdot z}{f_y}, \quad Z = z
$$

   donde `fx, fy` son las distancias focales y `cx, cy` el punto principal, todos extraídos de los intrínsecos de la cámara con `get_intrinsics()`.

5. **Volteo de coordenadas**: Open3D y RealSense usan ejes Y y Z contrarios. Se aplica la transformación:
   ```python
   pcd.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])
   ```

6. **Pre-filtrado ligero** (`voxel_down_sample` con `voxel_size=0.005`): Antes de guardar en memoria, cada nube se reduce a un grid de 5 mm para ganar velocidad sin perder forma.

---

### `align_and_merge(scans)` — Fusión de nubes de puntos con Colored ICP

Esta es la función más importante matemáticamente. Toma la lista de nubes capturadas y devuelve una sola nube 3D fusionada.

#### Paso 1: Estimación de Normales

Para cada par **(source, target)**, se estiman las normales superficiales de cada punto usando una búsqueda KD-Tree:

```python
pcd.estimate_normals(KDTreeSearchParamHybrid(radius=0.03, max_nn=30))
```

Un KD-Tree permite encontrar los 30 vecinos más cercanos en un radio de 3 cm con complejidad `O(log N)`. Con esa vecindad local se ajusta un plano por mínimos cuadrados y se extrae su vector normal unitario `n̂`.

#### Paso 2: Pre-alineación IMU (Tracking del Yaw)

Para evitar que el Colored ICP se pierda al rotar superficies muy simétricas, se inyecta el ángulo absoluto de guiñada (`Yaw`) recolectado por el ESP32-C3 como inicialización:

```python
delta_yaw_rad = np.radians(scans[i-1]['yaw'] - scans[i]['yaw'])
init_trans = np.identity(4)
# Matriz de rotación en el eje Y
```
Esta pequeña ayuda alinea las nubes un 90% antes del ICP, permitiendo escaneos 360º casi perfectos.

#### Paso 3: Colored ICP (Iterative Closest Point con Color)

El algoritmo ICP clásico busca la transformación rígida **T** = {R, t} (rotación `R` ∈ SO(3) y traslación `t` ∈ ℝ³) que minimiza la distancia entre dos nubes de puntos. La función de coste es:

$$
E_{geo}(T) = \sum_{(p_i, q_i) \in \mathcal{K}} \left( (T p_i - q_i) \cdot \hat{n}_{q_i} \right)^2
$$

donde $\mathcal{K}$ es el conjunto de pares de correspondencias más cercanas y `n̂_q` es la normal del punto destino. Esto es la variante **Point-to-Plane**.

El **Colored ICP** (Park et al., 2017) añade una restricción de color al coste:

$$
E(T) = (1 - \lambda) \cdot E_{geo}(T) + \lambda \cdot E_{color}(T)
$$

donde $E_{color}$ penaliza la diferencia de intensidad de color entre el punto transformado y su correspondencia en la imagen del target:

$$
E_{color}(T) = \sum_{(p_i, q_i) \in \mathcal{K}} \left( C_{source}(p_i) - C_{target}(T p_i) \right)^2
$$

Esto es crucial para escaneos 360° porque evita el **drift** (deriva acumulativa): en objetos con zonas curvas y simétricas, el ICP geométrico puro puede confundir correspondencias y "resbalar". El anclaje en las texturas de color rompe esa ambigüedad.

La solución iterativa utiliza el método de Gauss-Newton, resolviendo en cada iteración un sistema lineal de la forma:

$$
J^T J \cdot \delta\xi = -J^T r
$$

donde `ξ` ∈ se(3) es la actualización infinitesimal de la pose en algebra de Lie, `J` es el Jacobiano del error y `r` es el vector de residuos. La transformación acumulada `T_global` se actualiza como:

```python
current_transform = current_transform @ result_icp.transformation
```

#### Paso 4: Post-procesado (Multi-fase)

Tras acumular todas las nubes en la nube maestra `merged_pcd`, se aplica un filtro de 4 fases para garantizar una malla nítida:

1. **Voxel Down-Sample (2mm)**: Discretiza el espacio en voxels de 2 mm. Todos los puntos que caen en el mismo voxel se reemplazan por su centroide, homogeneizando la densidad y eliminando puntos solapados.
2. **Statistical Outlier Removal (Amplio)**: Filtro agresivo (`std_ratio=2.0`) para eliminar manchas de ruido o fantasmas muy lejanos a las superficies reales.
3. **Statistical Outlier Removal (Fino)**: Segunda pasada más estricta (`std_ratio=1.2`) para alisar el ruido microscópico superficial intrínseco del sensor infrarrojo de la RealSense.
4. **Radius Outlier Removal**: Por último, cualquier punto que no posea al menos 10 vecinos en un radio esférico de 15mm es purgado, borrando así pequeñas "motas de polvo" flotantes.

---

## 📦 Librerías Utilizadas

| Librería | Versión | Uso |
|----------|---------|-----|
| `pyrealsense2` | 2.50.0 | Comunicación con el hardware SR300 |
| `opencv-python` | 4.x | Visualización y manipulación de imágenes 2D |
| `open3d` | 0.19 | Nubes de puntos, Colored ICP, exportación PLY |
| `numpy` | 2.x | Operaciones matriciales y de arrays |
| `pyserial` | 3.x | Comunicación con el microcontrolador ESP32 |

> ⚠️ **Nota de compatibilidad**: La SR300 es una cámara legacy. Las versiones de `pyrealsense2` superiores a la 2.53 no incluyen soporte para este hardware en sus wheels de Python para Windows. Por este motivo se utiliza el entorno `env_sr300` con Python 3.9 y `pyrealsense2==2.50.0.3812`.

---

## 📖 Referencias

- Park, J., Zhou, Q. Y., & Koltun, V. (2017). *Colored Point Cloud Registration Revisited*. ICCV 2017. 
- Besl, P.J. & McKay, N.D. (1992). *A Method for Registration of 3-D Shapes*. IEEE TPAMI 14(2).
- Intel RealSense SDK 2.0: https://github.com/IntelRealSense/librealsense
- Open3D: http://www.open3d.org/docs/release/
