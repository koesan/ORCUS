"""ORCUS Swarm Kamikaze Drone Control Hub.

Web-based swarm control interface with ROS integration.
Provides RESTful API for swarm connection, parallel mission management, and live video streaming.
"""

from flask import Flask, render_template, request, jsonify, Response
import threading
import logging
import rospy
from modules.core.fleet_manager import DroneManager
import time
from config import APP_HOST, APP_PORT

app = Flask(__name__, template_folder='templates')

# Main Drone Manager
drone_manager = DroneManager()

@app.route('/')
def index():
    """Renders the main dashboard."""
    return render_template('index.html')

@app.route('/connect_drone', methods=['POST'])
def connect_drone():
    """Handles drone connection requests."""
    data = request.json
    connection_string = data.get('connection_string')
    if not connection_string:
         return jsonify({'status': 'error', 'message': 'No connection string provided'})
    return jsonify(drone_manager.start_connection(connection_string))

@app.route('/select_drone', methods=['POST'])
def select_drone():
    """Selects the active drone for control/viewing."""
    port = request.json.get('port')
    return jsonify(drone_manager.select_drone(port))

@app.route('/set_area', methods=['POST'])
def set_area():
    """Sets the mission area for all drones with intelligent grid partitioning."""
    data = request.json
    coordinates = data.get('coordinates')

    if not coordinates:
        return jsonify({'status': 'error', 'message': 'No coordinates provided'})

    try:
        drone_assignments = drone_manager.partition_grid_intelligently(coordinates)

        if not drone_assignments:
            return jsonify({'status': 'error', 'message': 'No drones connected or grid creation failed'})

        results = []
        for port, drone_cells in drone_assignments.items():
            controller = drone_manager.get_or_create_controller(port)
            result = controller.set_area(drone_cells)
            if result['status'] == 'ok':
                results.append(f"Drone {port} ({len(drone_cells)} cells)")
        
        if results:
            return jsonify({'status': 'ok', 'message': f"Area partitioned: {', '.join(results)}"})
        return jsonify({'status': 'error', 'message': 'Could not assign area to any drone'})
    except Exception as e:
        logging.error(f"set_area error: {e}", exc_info=True)
        return jsonify({'status': 'error', 'message': str(e)})

@app.route('/start_mission', methods=['POST'])
def start_mission():
    """Starts the mission for all connected drones."""
    data = request.json
    mission_type = data.get('mission_type')
    
    if not mission_type:
        return jsonify({'status': 'error', 'message': 'Mission type not specified'})
    
    try:
        started_drones = []
        failed_drones = []
        
        for port, vehicle in drone_manager.drones.items():
            if vehicle == "connecting":
                continue
            
            try:
                controller = drone_manager.get_or_create_controller(port)
                with drone_manager.drone_context(port, vehicle):
                    result = controller.start_mission(mission_type)
                
                if result['status'] == 'ok':
                    started_drones.append(str(port))
                else:
                    failed_drones.append(f"{port}: {result.get('message', 'Error')}")
            except Exception as e:
                failed_drones.append(f"{port}: {str(e)}")
        
        if started_drones:
            message = f"{len(started_drones)} drones started (Ports: {', '.join(started_drones)})"
            if failed_drones:
                message += f" | Failed: {'; '.join(failed_drones)}"
            return jsonify({'status': 'ok', 'message': message})
        return jsonify({'status': 'error', 'message': 'No drones could be started'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)})

@app.route('/stop_mission', methods=['POST'])
def stop_mission():
    """Stops missions for all drones (alias for stop_all)."""
    return stop_all()

@app.route('/status')
def status():
    """Returns the status of all drones."""
    return jsonify(drone_manager.get_status())

@app.route('/pause_all', methods=['POST'])
def pause_all():
    """Pauses missions for all drones - sends zero velocity before pausing."""
    results = []
    for port, controller in drone_manager.drone_controllers.items():
        if controller.is_mission_active:
            try:
                vehicle = drone_manager.drones.get(port)
                if vehicle and vehicle != "connecting":
                    controller.send_ned_velocity(vehicle, 0, 0, 0)
            except Exception:
                pass
            controller.is_mission_active = False
            results.append(f"Drone {port} paused")
    
    if results:
        return jsonify({'status': 'ok', 'message': '; '.join(results)})
    else:
        return jsonify({'status': 'error', 'message': 'No active missions'})

@app.route('/resume_all', methods=['POST'])
def resume_all():
    """Resumes missions for all drones."""
    results = []
    for port, controller in drone_manager.drone_controllers.items():
        # Check if thread is alive but flag is False
        if not controller.is_mission_active and controller._mission_thread and controller._mission_thread.is_alive():
            controller.is_mission_active = True
            results.append(f"Drone {port} resuming")
    
    if results:
        return jsonify({'status': 'ok', 'message': '; '.join(results)})
    else:
        return jsonify({'status': 'error', 'message': 'No paused missions found'})

@app.route('/stop_all', methods=['POST'])
def stop_all():
    """Stops missions for all drones."""
    results = []
    for port in list(drone_manager.drone_controllers.keys()):
        try:
            controller = drone_manager.drone_controllers[port]
            if port in drone_manager.drones:
                vehicle = drone_manager.drones[port]
                if vehicle != "connecting":
                    with drone_manager.drone_context(port, vehicle):
                        result = controller.stop_mission()
                    
                    if result['status'] == 'ok':
                        results.append(f"Drone {port} stopped")
        except Exception as e:
            logging.error(f"Error stopping Drone {port}: {e}")
    
    if results:
        return jsonify({'status': 'ok', 'message': '; '.join(results)})
    return jsonify({'status': 'error', 'message': 'No drones stopped'})

@app.route('/camera_feed')
def camera_feed():
    """Provides live video stream from selected drone camera."""
    port_arg = request.args.get('port')
    
    # Default to active drone port if not specified
    target_port = drone_manager.active_drone_port
    if port_arg:
        try:
            target_port = int(port_arg)
        except:
             pass
    
    if target_port is None:
        return "No camera feed selected", 404
    
    if target_port not in drone_manager.drones:
        return "Drone not connected", 404
    
    return Response(drone_manager.camera_handler.generate_frames(target_port),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/select_camera', methods=['POST'])
def select_camera():
    """Selects the camera to view."""
    data = request.json
    port = data.get('port')
    
    if not port:
        return jsonify({'status': 'error', 'message': 'Port not specified'})
    
    try:
        port = int(port)
        if port not in drone_manager.drones:
            return jsonify({'status': 'error', 'message': f'Drone {port} not connected'})
        
        with drone_manager.lock:
            drone_manager.active_drone_port = port
            drone_manager.active_drone = drone_manager.drones[port]
        
        time.sleep(0.1)
        return jsonify({'status': 'ok', 'message': f'Camera {port} selected'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)})

@app.route('/swarm_data')
def swarm_data():
    """Returns swarm and target data in JSON format."""
    if not drone_manager.swarm_manager:
         return jsonify({})
         
    data = drone_manager.swarm_manager.get_battlespace_state()
    data['raw_targets'] = drone_manager.swarm_manager.get_drone_targets_buffer()
    return jsonify(data)

@app.route('/approve_attack/<target_id>')
def approve_attack(target_id):
    """Approves attack on a specific target."""
    if drone_manager.swarm_manager.approve_attack(target_id):
        return jsonify({'status': 'ok', 'message': f'{target_id} Approved'})
    return jsonify({'status': 'error', 'message': 'Target not found'})

@app.route('/approve_all', methods=['POST'])
def approve_all():
    """Approves attack for ALL pending targets."""
    if not drone_manager.swarm_manager:
        return jsonify({'status': 'error', 'message': 'Swarm Manager inactive'})
        
    count = drone_manager.swarm_manager.approve_all_targets()
    return jsonify({'status': 'ok', 'message': f'Bulk approval for {count} targets'})

if __name__ == '__main__':
    try:
        def start_ros_node():
            try:
                drone_manager.camera_handler.init_ros_node()
                rospy.spin()
            except Exception as e:
                logging.error(f"ROS Thread Error: {e}")

        ros_thread = threading.Thread(target=start_ros_node)
        ros_thread.daemon = True
        ros_thread.start()

        logging.info("="*60)
        logging.info("ORCUS Swarm Kamikaze System Started")
        logging.info("="*60)
        logging.info(f"Web Interface: http://{APP_HOST}:{APP_PORT}")
        logging.info("Battlespace Swarm Intelligence Active")
        logging.info("="*60)

        app.run(host=APP_HOST, port=APP_PORT, threaded=True, debug=False, use_reloader=False)

    except rospy.ROSInterruptException:
        logging.info("ROS node stopped.")
    except Exception as e:
        logging.error(f"Application start error: {e}")