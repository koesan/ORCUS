"""ORCUS Swarm Kamikaze Drone Control Hub.

Web-based swarm control interface with ROS integration.
Provides RESTful API for swarm connection, parallel mission management, and live video streaming.
"""

from flask import Flask, render_template, request, jsonify, Response
import atexit
import threading
import logging
import rospy
from modules.core.fleet_manager import DroneManager
from modules.core.logger import SwarmLogger
import time
from config import APP_HOST, APP_PORT

app = Flask(__name__, template_folder='templates')

# Main Drone Manager
drone_manager = DroneManager()
atexit.register(drone_manager.shutdown)
_mission_command_lock = threading.Lock()
_rtl_monitor_lock = threading.Lock()
_rtl_monitors = {}


def _monitor_rtl_completion(controllers_to_watch):
    """Background watcher for post-stop RTL completion."""
    for port, controller in controllers_to_watch:
        try:
            vehicle = drone_manager.drones.get(port)
            if vehicle and vehicle != "connecting":
                with drone_manager.drone_context(port, vehicle):
                    controller.await_rtl_completion()
        except Exception:
            logging.exception("RTL monitor error for Drone %s", port)
    with _rtl_monitor_lock:
        for port, _controller in controllers_to_watch:
            _rtl_monitors.pop(port, None)


def _json_payload():
    """Safely parse JSON request bodies."""
    return request.get_json(silent=True) or {}


def _json_safe(value):
    """Recursively convert numpy-like values to plain JSON-safe types."""
    if isinstance(value, dict):
        return {str(k): _json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [_json_safe(v) for v in value]
    if hasattr(value, "tolist"):
        try:
            return value.tolist()
        except Exception:
            pass
    if hasattr(value, "item"):
        try:
            return value.item()
        except Exception:
            pass
    return value

@app.route('/')
def index():
    """Renders the main dashboard."""
    return render_template('index.html')

@app.route('/connect_drone', methods=['POST'])
def connect_drone():
    """Handles drone connection requests."""
    data = _json_payload()
    connection_string = data.get('connection_string')
    if not connection_string:
         return jsonify({'status': 'error', 'message': 'No connection string provided'})
    return jsonify(drone_manager.start_connection(connection_string))

@app.route('/select_drone', methods=['POST'])
def select_drone():
    """Selects the active drone for control/viewing."""
    port = _json_payload().get('port')
    return jsonify(drone_manager.select_drone(port))

@app.route('/set_area', methods=['POST'])
def set_area():
    """Sets the mission area for all drones with intelligent grid partitioning."""
    data = _json_payload()
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
    if not _mission_command_lock.acquire(blocking=False):
        return jsonify({'status': 'error', 'message': 'Another mission command is already in progress'})
    data = _json_payload()
    mission_type = data.get('mission_type')
    
    if not mission_type:
        return jsonify({'status': 'error', 'message': 'Mission type not specified'})
    
    try:
        with drone_manager.lock:
            ready_ports = [port for port, vehicle in drone_manager.drones.items() if vehicle != "connecting"]
        if not ready_ports:
            return jsonify({'status': 'error', 'message': 'No connected drones available'})

        missing_area = []
        for port in ready_ports:
            controller = drone_manager.get_or_create_controller(port)
            if not getattr(controller, "mission_coordinates", None):
                missing_area.append(str(port))
        if missing_area:
            return jsonify({
                'status': 'error',
                'message': f'Area/cell assignment missing for drones: {", ".join(missing_area)}'
            })

        started_drones = []
        resumed_drones = []
        failed_drones = []
        
        with drone_manager.lock:
            drones_snapshot = list(drone_manager.drones.items())
        for port, vehicle in drones_snapshot:
            if vehicle == "connecting":
                continue
            
            try:
                controller = drone_manager.get_or_create_controller(port)
                with drone_manager.drone_context(port, vehicle):
                    was_paused = bool(getattr(controller, "is_paused", False))
                    if was_paused:
                        result = controller.resume_mission()
                    else:
                        result = controller.start_mission(mission_type)
                
                if result['status'] == 'ok':
                    if was_paused:
                        resumed_drones.append(str(port))
                    elif controller.is_mission_active:
                        started_drones.append(str(port))
                    else:
                        started_drones.append(str(port))
                else:
                    failed_drones.append(f"{port}: {result.get('message', 'Error')}")
            except Exception as e:
                failed_drones.append(f"{port}: {str(e)}")
        
        if started_drones or resumed_drones:
            parts = []
            if started_drones:
                parts.append(f"{len(started_drones)} drones started (Ports: {', '.join(started_drones)})")
            if resumed_drones:
                parts.append(f"{len(resumed_drones)} drones resumed (Ports: {', '.join(resumed_drones)})")
            message = " | ".join(parts)
            if failed_drones:
                message += f" | Failed: {'; '.join(failed_drones)}"
            return jsonify({'status': 'ok', 'message': message})
        return jsonify({'status': 'error', 'message': 'No drones could be started'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)})
    finally:
        _mission_command_lock.release()

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
    """Pauses missions for all drones and preserves mission state."""
    if not _mission_command_lock.acquire(blocking=False):
        return jsonify({'status': 'error', 'message': 'Another mission command is already in progress'})
    results = []
    try:
        with drone_manager.lock:
            controllers_snapshot = list(drone_manager.drone_controllers.items())
        for port, controller in controllers_snapshot:
            result = controller.pause_mission()
            if result.get('status') == 'ok':
                results.append(f"Drone {port} paused")
        
        if results:
            return jsonify({'status': 'ok', 'message': '; '.join(results)})
        else:
            return jsonify({'status': 'error', 'message': 'No active missions'})
    finally:
        _mission_command_lock.release()

@app.route('/resume_all', methods=['POST'])
def resume_all():
    """Resumes missions for all drones."""
    results = []
    with drone_manager.lock:
        controllers_snapshot = list(drone_manager.drone_controllers.items())
    for port, controller in controllers_snapshot:
        result = controller.resume_mission()
        if result.get('status') == 'ok':
            results.append(f"Drone {port} resuming")
    
    if results:
        return jsonify({'status': 'ok', 'message': '; '.join(results)})
    else:
        return jsonify({'status': 'error', 'message': 'No paused missions found'})

@app.route('/stop_all', methods=['POST'])
def stop_all():
    """Stop active missions and always clear volatile mission runtime state."""
    if not _mission_command_lock.acquire(blocking=False):
        return jsonify({'status': 'error', 'message': 'Another mission command is already in progress'})
    results = []
    controllers_to_watch = []
    try:
        with drone_manager.lock:
            controller_items = list(drone_manager.drone_controllers.items())

        for port, controller in controller_items:
            try:
                vehicle = drone_manager.drones.get(port)
                if vehicle == "connecting" or vehicle is None:
                    continue
                if controller.is_running() or getattr(controller, "is_paused", False):
                    with drone_manager.drone_context(port, vehicle):
                        result = controller.stop_mission()
                    if result.get('status') == 'ok':
                        results.append(f"Drone {port} RTL started")
                        controllers_to_watch.append((port, controller))
                controller.soft_reset_state()
            except Exception as e:
                logging.error(f"Error stopping Drone {port}: {e}", exc_info=True)

        if drone_manager.swarm_manager:
            drone_manager.swarm_manager.reset_runtime_state()

        with _rtl_monitor_lock:
            pending = [(port, controller) for port, controller in controllers_to_watch if port not in _rtl_monitors]
            for port, controller in pending:
                _rtl_monitors[port] = time.time()
        if pending:
            watcher = threading.Thread(target=_monitor_rtl_completion, args=(pending,), daemon=True)
            watcher.start()

        if results:
            message = '; '.join(results) + ' | Soft reset completed, RTL continues in background'
        else:
            message = 'Soft reset completed. No active missions were running.'
        return jsonify({'status': 'ok', 'message': message})
    finally:
        _mission_command_lock.release()

@app.route('/camera_feed')
def camera_feed():
    """Provides live video stream from selected drone camera."""
    port_arg = request.args.get('port')
    
    # Default to active drone port if not specified
    target_port = drone_manager.active_drone_port
    if port_arg:
        try:
            target_port = int(port_arg)
        except Exception:
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
    data = _json_payload()
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
        SwarmLogger.log("INFO", "WEB_UI", f"Camera selected | port={port}", "UI")
        
        time.sleep(0.1)
        return jsonify({'status': 'ok', 'message': f'Camera {port} selected'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)})

@app.route('/swarm_data')
def swarm_data():
    """Returns swarm and target data in JSON format."""
    if not drone_manager.swarm_manager:
         return jsonify({})

    payload = {'targets': {}, 'drones': {}, 'raw_targets': {}}
    try:
        payload.update(drone_manager.swarm_manager.get_battlespace_state() or {})
    except Exception:
        logging.exception("swarm_data battlespace error")
    try:
        payload['raw_targets'] = drone_manager.swarm_manager.get_drone_targets_buffer()
    except Exception:
        logging.exception("swarm_data raw-target buffer error")
    try:
        targets = payload.get("targets", {}) or {}
        drones = payload.get("drones", {}) or {}
        raw_targets = payload.get("raw_targets", {}) or {}
        target_preview = []
        for tid, item in list(targets.items())[:4]:
            target_preview.append(
                f"{tid}:{item.get('track_state')}/{item.get('presentation_state') or item.get('status')} "
                f"grp={int(bool(item.get('is_group')))}"
            )
        drone_preview = []
        for port, item in list(drones.items())[:4]:
            drone_preview.append(
                f"D{port}:{item.get('execution_state') or item.get('action')} "
                f"tgt={item.get('current_target_id') or '-'} "
                f"link={item.get('session_phase') or '-'}"
            )
        raw_preview = []
        for port, items in list(raw_targets.items())[:4]:
            raw_preview.append(f"D{port}:{len(items or [])}")
        SwarmLogger.log_throttled(
            "UI_STATE",
            "LEADER",
            f"/swarm_data targets={len(targets)} drones={len(drones)} raw_buffers={len(raw_targets)} "
            f"target_preview={' | '.join(target_preview) if target_preview else '-'} "
            f"drone_preview={' | '.join(drone_preview) if drone_preview else '-'} "
            f"raw_preview={' | '.join(raw_preview) if raw_preview else '-'}",
            "UI",
            interval_s=5.0,
        )
    except Exception:
        logging.exception("swarm_data summary log error")
    return jsonify(_json_safe(payload))

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
