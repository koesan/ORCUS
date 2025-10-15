"""TepeGöz - Ana Flask Uygulaması

Web tabanlı multi-drone kontrol arayüzü. ROS ile entegre çalışır.
RESTful API ile çoklu drone bağlantısı, paralel görev yönetimi ve canlı video stream'i sağlar.
Her dron bağımsız olarak kendi görevini yürütür.
"""

from flask import Flask, render_template, request, jsonify, Response
import threading
import os
import rospy
from modules.drone_manager import DroneManager
from modules.collision_mission_controller import CollisionMissionController
import time
from config import APP_HOST, APP_PORT

app = Flask(__name__, template_folder='templates')

# Ana drone manager
drone_manager = DroneManager()
# Not: Mission controller'lar artık her dron için ayrı ayrı oluşturuluyor

@app.route('/')
def index():
    """Ana sayfayı render eder."""
    return render_template('index.html')

@app.route('/connect_drone', methods=['POST'])
def connect_drone():
    """Drona bağlanma isteğini işler."""
    data = request.json
    connection_string = data.get('connection_string')
    return jsonify(drone_manager.start_connection(connection_string))

def select_drone():
    """Kontrol edilecek dronu seçer."""
    port = request.json.get('port')
    return jsonify(drone_manager.select_drone(port))

@app.route('/set_area', methods=['POST'])
def set_area():
    """Tüm dronlar için görev alanını belirler - Akıllı grid paylaşımı."""
    data = request.json
    coordinates = data.get('coordinates')

    if not coordinates:
        return jsonify({'status': 'error', 'message': 'Koordinatlar belirtilmedi'})

    try:
        # Akıllı grid paylaşımı: Her drone'a özel alt-grid
        drone_assignments = drone_manager.partition_grid_intelligently(coordinates)

        if not drone_assignments:
            return jsonify({'status': 'error', 'message': 'Bağlı dron yok veya grid oluşturulamadı'})

        # Her drone için kendi alt-grid'ini ayarla
        results = []
        for port, drone_cells in drone_assignments.items():
            controller = drone_manager.get_or_create_controller(port)
            result = controller.set_area(drone_cells)
            if result['status'] == 'ok':
                results.append(f"Dron {port} ({len(drone_cells)} hücre)")
        
        if results:
            message = f"Alan akıllıca paylaştırıldı: {', '.join(results)}"
            return jsonify({'status': 'ok', 'message': message})
        else:
            return jsonify({'status': 'error', 'message': 'Hiçbir drone\'a alan atanamadı'})
    except Exception as e:
        print(f"set_area hatası: {e}")
        import traceback
        traceback.print_exc()
        return jsonify({'status': 'error', 'message': str(e)})

@app.route('/start_mission', methods=['POST'])
def start_mission():
    """Tüm dronlar için görevi başlatır."""
    data = request.json
    mission_type = data.get('mission_type')
    
    if not mission_type:
        return jsonify({'status': 'error', 'message': 'Görev tipi belirtilmedi'})
    
    try:
        started_drones = []
        failed_drones = []
        
        for port, vehicle in drone_manager.drones.items():
            if vehicle == "connecting":
                continue
            
            try:
                controller = drone_manager.get_or_create_controller(port)
                
                # Dronu geçici olarak aktif yap
                original_active = drone_manager.active_drone
                original_port = drone_manager.active_drone_port
                
                drone_manager.active_drone = vehicle
                drone_manager.active_drone_port = port
                
                result = controller.start_mission(mission_type)
                
                # Orijinal durumu geri yükle
                if original_active:
                    drone_manager.active_drone = original_active
                    drone_manager.active_drone_port = original_port
                
                if result['status'] == 'ok':
                    started_drones.append(str(port))
                else:
                    failed_drones.append(f"{port}: {result.get('message', 'Hata')}")
            except Exception as e:
                failed_drones.append(f"{port}: {str(e)}")
        
        if started_drones:
            message = f"{len(started_drones)} dron görevine başladı (Port: {', '.join(started_drones)})"
            if failed_drones:
                message += f" | Başarısız: {'; '.join(failed_drones)}"
            return jsonify({'status': 'ok', 'message': message})
        else:
            return jsonify({'status': 'error', 'message': 'Hiçbir dron başlatılamadı'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)})

@app.route('/stop_mission', methods=['POST'])
def stop_mission():
    """Tüm dronlar için görevi durdurur (stop_all ile aynı)."""
    return stop_all()

@app.route('/status')
def status():
    """Tüm dronların durumunu döndürür."""
    return jsonify(drone_manager.get_status())

@app.route('/pause_all', methods=['POST'])
def pause_all():
    """Tüm dronların görevlerini duraklatır."""
    results = []
    for port, controller in drone_manager.drone_controllers.items():
        if controller.is_mission_active:
            controller.is_mission_active = False
            results.append(f"Dron {port} duraklatıldı")
    
    if results:
        return jsonify({'status': 'ok', 'message': '; '.join(results)})
    else:
        return jsonify({'status': 'error', 'message': 'Aktif görev yok'})

@app.route('/resume_all', methods=['POST'])
def resume_all():
    """Tüm dronların görevlerini devam ettirir."""
    results = []
    for port, controller in drone_manager.drone_controllers.items():
        if not controller.is_mission_active and controller._mission_thread and controller._mission_thread.is_alive():
            controller.is_mission_active = True
            results.append(f"Dron {port} devam ediyor")
    
    if results:
        return jsonify({'status': 'ok', 'message': '; '.join(results)})
    else:
        return jsonify({'status': 'error', 'message': 'Duraklatılmış görev yok'})

@app.route('/stop_all', methods=['POST'])
def stop_all():
    """Tüm dronların görevlerini durdurur."""
    results = []
    for port in list(drone_manager.drone_controllers.keys()):
        try:
            controller = drone_manager.drone_controllers[port]
            if port in drone_manager.drones:
                vehicle = drone_manager.drones[port]
                if vehicle != "connecting":
                    original_active = drone_manager.active_drone
                    original_port = drone_manager.active_drone_port
                    
                    drone_manager.active_drone = vehicle
                    drone_manager.active_drone_port = port
                    
                    result = controller.stop_mission()
                    
                    if original_active and original_port != port:
                        drone_manager.active_drone = original_active
                        drone_manager.active_drone_port = original_port
                    
                    if result['status'] == 'ok':
                        results.append(f"Dron {port} durduruldu")
        except Exception as e:
            print(f"Dron {port} durdurulurken hata: {e}")
    
    if results:
        return jsonify({'status': 'ok', 'message': '; '.join(results)})
    else:
        return jsonify({'status': 'error', 'message': 'Durdurulan dron yok'})

@app.route('/camera_feed')
def camera_feed():
    """Seçili dron kamerasından canlı video akışı sağlar."""
    port = request.args.get('port', drone_manager.active_drone_port)
    
    if port is None:
        return "Kamera beslemesi yok", 404
    
    try:
        port = int(port)
    except:
        return "Geçersiz port", 400
    
    if port not in drone_manager.drones:
        return "Dron bağlı değil", 404
    
    return Response(drone_manager.camera_handler.generate_frames(port),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/select_camera', methods=['POST'])
def select_camera():
    """Görüntülenecek kamerayı seçer."""
    data = request.json
    port = data.get('port')
    
    if not port:
        return jsonify({'status': 'error', 'message': 'Port belirtilmedi'})
    
    try:
        port = int(port)
        if port not in drone_manager.drones:
            return jsonify({'status': 'error', 'message': f'Dron {port} bağlı değil'})
        
        # Thread-safe olarak port değiştir
        with drone_manager.lock:
            drone_manager.active_drone_port = port
            drone_manager.active_drone = drone_manager.drones[port]
        
        # Kısa bir bekleme - yeni kamera feed'inin başlaması için
        time.sleep(0.1)
        
        return jsonify({'status': 'ok', 'message': f'Kamera {port} seçildi'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)})

if __name__ == '__main__':
    try:
        # ROS'u ayrı bir thread'de başlat
        ros_thread = threading.Thread(target=rospy.spin)
        ros_thread.daemon = True
        
        # ROS düğümünü ve thread'i başlat
        drone_manager.camera_handler.init_ros_node()
        ros_thread.start()

        print("="*80)
        print("TepeGöz Multi-Drone Sistemi Başlatıldı")
        print("="*80)
        print(f"🌐 Web Arayüzü: http://{APP_HOST}:{APP_PORT}")
        print("✈️  Çoklu dron desteği aktif - Her dron bağımsız çalışır")
        print("="*80)

        app.run(host=APP_HOST, port=APP_PORT, threaded=True, debug=True, use_reloader=False)

    except rospy.ROSInterruptException:
        print("ROS düğümü durduruldu.")
    except Exception as e:
        print(f"Uygulama başlatılırken bir hata oluştu: {e}")