import pyrealsense2 as rs
import numpy as np
import cv2
import open3d as o3d
import time
import os

def align_and_merge_tsdf(rgbd_images, intrinsic):
    if len(rgbd_images) == 0:
        return None
        
    print(f"\n[+] Iniciando motor de alta precisión TSDF con {len(rgbd_images)} fotogramas...")
    
    # Volumen TSDF: Integra matemáticamente el ruido y genera una sola superficie pulida.
    # voxel_length = 2mm de precisión, sdf_trunc = tolerancia 1cm.
    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=0.002,
        sdf_trunc=0.01,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)
        
    # El primer fotograma es nuestro punto de origen 0,0,0
    current_pose = np.identity(4)
    volume.integrate(rgbd_images[0], intrinsic, current_pose)
    
    # Odometría Híbrida: usa profundidad + color para tracking (previene que el tracking resbale)
    odo_criteria = o3d.pipelines.odometry.OdometryOption()
    
    successful_frames = 1
    
    for i in range(1, len(rgbd_images)):
        source = rgbd_images[i]
        target = rgbd_images[i-1]
        
        # Calcular cuánto se movió la cámara usando Odometría RGBD
        success, trans, info = o3d.pipelines.odometry.compute_rgbd_odometry(
            source, target, intrinsic, np.identity(4),
            o3d.pipelines.odometry.RGBDOdometryJacobianFromHybridTerm(),
            odo_criteria)
            
        if success:
            # Multiplicamos la pose actual por el cambio (trans)
            # odo_trans mapea source -> target, por ende pose_actual = trans * pose_anterior
            current_pose = np.dot(current_pose, trans)
            
            # Integramos matemáticamente los píxeles al grid volumétrico 3D usando su pose exacta
            # Esto elimina las "capas dobles" y promedia el ruido de las mallas
            volume.integrate(source, intrinsic, np.linalg.inv(current_pose))
            
            successful_frames += 1
            if i % 10 == 0:
                print(f"  [Odometría] Fotograma {i}/{len(rgbd_images)} alineado e integrado con precisión.")
        else:
            print(f"  [Aviso] Pérdida de Odometría en fotograma {i}. Ignorando frame.")

    print(f"\n[+] Extrayendo malla 3D continua de {successful_frames} fotogramas útiles...")
    
    # Extraer Mesh Continua Triangulada
    mesh = volume.extract_triangle_mesh()
    mesh.compute_vertex_normals()
    
    return mesh

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

    raw_frames = []
    os.makedirs("escaneos", exist_ok=True)

    print("\n================ TUS CONTROLES ==================")
    print("1. Enpunta al objeto de cerca (aislado del fondo principal).")
    print("2. Presiona 'R' para EMPEZAR A GRABAR en 3D.")
    print("3. Muévete LENTO Y CONSTANTE alrededor del objeto.")
    print("4. Vuelve a presionar 'R' para PARAR Y FUSIONAR.")
    print("5. Presiona 'Q' o 'ESC' para SALIR.")
    print("=================================================\n")

    is_recording = False
    record_timer = 0
    # Guardamos 1 de cada 4 fotogramas para tener superposiciones ricas en detalles pero procesar rápido 
    record_interval = 4 

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
            
            # ---- OBTENER PARÁMETROS INTRÍNSECOS DE CÁMARA (Lente) ----
            intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            pinhole_intrinsic = o3d.camera.PinholeCameraIntrinsic(
                intrinsics.width, intrinsics.height, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy)

            # Lógica de Captura 
            if is_recording:
                record_timer += 1
                if record_timer >= record_interval:
                    record_timer = 0
                    
                    depth_trunc = 0.6  # Recorte de fondo para mejor odometría
                    
                    depth_o3d = o3d.geometry.Image(depth_image)
                    color_o3d = o3d.geometry.Image(cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB))
                    
                    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                        color_o3d, depth_o3d, depth_scale=1.0/depth_scale, depth_trunc=depth_trunc, convert_rgb_to_intensity=False)
                    
                    raw_frames.append(rgbd)

            # --- HUD VISUAL ---
            cv2.putText(display_color, f"Frames Grabados: {len(raw_frames)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(display_color, "R: Grabar | Q: Salir", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            if is_recording:
                cv2.circle(display_color, (30, 95), 10, (0, 0, 255), -1)
                cv2.putText(display_color, "GRABANDO...", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            images = np.hstack((display_color, depth_colormap))
            cv2.imshow('Scanner 3D TSDF - SR300', images)
            
            key = cv2.waitKey(1)
            
            if key & 0xFF == ord('q') or key == 27:
                break
                
            elif key & 0xFF == ord('r'):
                # Parar y Reconstruir
                if is_recording:
                    is_recording = False
                    print(f"\n[!] Grabación detenida. Total frames: {len(raw_frames)}.")
                    
                    if len(raw_frames) < 3:
                        print("Muy pocos fotogramas. Graba durante al menos un par de segundos.")
                        raw_frames = []
                        continue
                        
                    # LLAMADA AL MOTOR TSDF
                    merged_mesh = align_and_merge_tsdf(raw_frames, pinhole_intrinsic)
                    
                    if merged_mesh is not None:
                        # Invertir 180 grads en Y y Z para visualización correcta (Open3D vs Sensor)
                        merged_mesh.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
                        
                        filename = os.path.join("escaneos", f"supermesh_{int(time.time())}.ply")
                        o3d.io.write_triangle_mesh(filename, merged_mesh)
                        print(f"\n¡Éxito Absoluto! Malla de alta definición guardada en: {filename}")
                        
                        print("Cierra el visor 3D para continuar...")
                        o3d.visualization.draw_geometries([merged_mesh], window_name="Malla Volumétrica TSDF Alta Precisión")
                    
                    raw_frames = [] # Reset
                    record_timer = 0
                else:
                    # Comenzar a grabar
                    is_recording = True
                    raw_frames = []
                    record_timer = 0
                    print("\n[REC] Empezando Captura Volumétrica... Muévete estable y con suavidad.")

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
