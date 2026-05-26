import pyrealsense2 as rs
import numpy as np
import cv2
import open3d as o3d
import copy
import time
import os
from imu_reader import IMUReader

def align_and_merge(scans):
    if len(scans) == 0:
        return None
    if len(scans) == 1:
        return scans[0]
        
    print(f"\nProcesando el video 3D... Uniendo {len(scans)} fotogramas capturados...")
    
    merged_pcd = o3d.geometry.PointCloud()
    merged_pcd += scans[0]['pcd']
    
    current_transform = np.identity(4)
    
    for i in range(1, len(scans)):
        source = scans[i]['pcd']
        target = scans[i-1]['pcd']
        
        if i % 5 == 0:
            print(f"  Alineando fotograma {i}/{len(scans)}...")
            
        source.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.03, max_nn=30))
        target.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.03, max_nn=30))
        
        # Tolerancia muy baja porque es video contínuo (los frames están muy pegados)
        threshold = 0.05
        
        # Pre-alineación basada en la IMU para Yaw (eje de rotación)
        # Asumiendo que el giro del escáner ocurre predominantemente horizontal (Y)
        delta_yaw_deg = scans[i-1]['yaw'] - scans[i]['yaw']
        delta_yaw_rad = np.radians(delta_yaw_deg)
        
        init_trans = np.identity(4)
        init_trans[0, 0] = np.cos(delta_yaw_rad)
        init_trans[0, 2] = np.sin(delta_yaw_rad)
        init_trans[2, 0] = -np.sin(delta_yaw_rad)
        init_trans[2, 2] = np.cos(delta_yaw_rad)
        
        result_icp = o3d.pipelines.registration.registration_colored_icp(
            source, target, threshold, init_trans,
            o3d.pipelines.registration.TransformationEstimationForColoredICP(),
            o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
                                                              relative_rmse=1e-6,
                                                              max_iteration=50) # Menos iteraciones porque está muy cerca
        )
        
        current_transform = current_transform @ result_icp.transformation
        
        source_aligned = copy.deepcopy(source)
        source_aligned.transform(current_transform)
        merged_pcd += source_aligned

    print("\n¡Grabación unida! Limpiando nube de puntos en múltiples fases...")

    # Fase 1: Voxel downsample para unificar densidad
    print("  [1/4] Voxel downsample...")
    merged_pcd = merged_pcd.voxel_down_sample(voxel_size=0.002)

    # Fase 2: Primera pasada estadística amplia - elimina puntos muy alejados del grupo
    print("  [2/4] Eliminando puntos aislados (pasada amplia)...")
    merged_pcd, _ = merged_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

    # Fase 3: Segunda pasada estadística fina - elimina el ruido fino restante
    print("  [3/4] Apretando la limpieza (pasada fina)...")
    merged_pcd, _ = merged_pcd.remove_statistical_outlier(nb_neighbors=30, std_ratio=1.2)

    # Fase 4: Filtro de radio - elimina "nubes" de puntos flotantes sin vecinos cercanos
    print("  [4/4] Eliminando puntos flotantes (radio filter)...")
    merged_pcd, _ = merged_pcd.remove_radius_outlier(nb_points=10, radius=0.015)

    print(f"  ✓ Nube limpia: {len(merged_pcd.points)} puntos finales.")
    return merged_pcd

def main():
    print("Iniciando conexión con la cámara RealSense SR300...")
    
    pipeline = rs.pipeline()
    config = rs.config()

    try:
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    except Exception as e:
        config.enable_stream(rs.stream.depth)
        config.enable_stream(rs.stream.color)

    try:
        profile = pipeline.start(config)
    except Exception as e:
        print(f"Error al iniciar cámara: {e}")
        return

    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    align_to = rs.stream.color
    align = rs.align(align_to)

    imu = IMUReader(baudrate=115200)
    imu.start()

    scans = []
    os.makedirs("escaneos", exist_ok=True)

    print("\n================ TUS CONTROLES ==================")
    print("1. Enpunta al objeto de cerca (aislado).")
    print("2. Presiona 'C' para CALIBRAR EL IMU.")
    print("3. Presiona 'R' para EMPEZAR A GRABAR en 3D.")
    print("4. Muévete muy lentamente alrededor del objeto.")
    print("5. Vuelve a presionar 'R' para PARAR Y CONSTRUIR.")
    print("6. Presiona 'Q' o 'ESC' para SALIR.")
    print("=================================================\n")

    is_recording = False
    record_timer = 0
    # Guardaremos 1 de cada 5 frames visuales (~6 fotogramas por segundo) para no colapsar la RAM
    record_interval = 5 

    try:
        while True:
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)

            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not aligned_depth_frame or not color_frame:
                continue

            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            display_color = color_image.copy()
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            
            # Lógica de Captura Continua
            if is_recording:
                record_timer += 1
                if record_timer >= record_interval:
                    record_timer = 0
                    
                    depth_trunc = 0.6 # Ignorar fondo lejano
                    depth_o3d = o3d.geometry.Image(depth_image)
                    color_o3d = o3d.geometry.Image(cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB))
                    
                    intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
                    pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
                        intrinsics.width, intrinsics.height, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy)

                    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
                        color_o3d, depth_o3d, depth_scale=1.0/depth_scale, depth_trunc=depth_trunc, convert_rgb_to_intensity=False)
                    
                    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, pinhole_camera_intrinsic)
                    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
                    
                    # Filtro ultra rápido
                    pcd = pcd.voxel_down_sample(voxel_size=0.005)
                    
                    # Guardamos la pose junto al PointCloud
                    imu_yaw, imu_pitch, imu_roll = imu.get_pose()
                    scans.append({'pcd': pcd, 'yaw': imu_yaw})

            # Textos en pantalla
            cv2.putText(display_color, f"Fotogramas 3D: {len(scans)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # IMU UI
            imu_yaw, imu_pitch, imu_roll = imu.get_pose()
            cv2.putText(display_color, f"IMU: Y:{imu_yaw:.1f} P:{imu_pitch:.1f} R:{imu_roll:.1f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(display_color, "R: Grabar | C: Calibrar IMU | Q: Salir", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            if is_recording:
                cv2.circle(display_color, (30, 125), 10, (0, 0, 255), -1)
                cv2.putText(display_color, "GRABANDO...", (50, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            images = np.hstack((display_color, depth_colormap))
            cv2.imshow('Scanner 3D CONTINUO - SR300', images)
            
            key = cv2.waitKey(1)
            
            if key & 0xFF == ord('q') or key == 27:
                break
                
            elif key & 0xFF == ord('c'):
                print("\n[!] Solicitando calibración del IMU...")
                imu.send_command('c')
                
            elif key & 0xFF == ord('r'):
                # Si estaba grabando, detener y fusionar
                if is_recording:
                    is_recording = False
                    print(f"\n[!] Grabación detenida. Total de fotogramas 3D en la cinta: {len(scans)}.")
                    
                    if len(scans) < 2:
                        print("Muy pocos fotogramas para unir. Moviéndonos y graba más rato.")
                        scans = []
                        continue
                        
                    merged_result = align_and_merge(scans)
                    
                    filename = os.path.join("escaneos", f"video_mesh_{int(time.time())}.ply")
                    o3d.io.write_point_cloud(filename, merged_result)
                    print(f"\n¡Éxito! Grabación 3D unida y guardada en: {filename}")
                    
                    print("Cierra el visor 3D para continuar...")
                    o3d.visualization.draw_geometries([merged_result], window_name="Malla Grabada")
                    
                    scans = [] # Reiniciar cinta
                    record_timer = 0
                else: # Empezar a grabar
                    is_recording = True
                    scans = []
                    record_timer = 0
                    print("\n[REC] ¡Grabando en 3D! Mueve la cámara lentamente...")

    finally:
        imu.stop()
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
