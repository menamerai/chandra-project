import os
import socket
import json
import time
import threading
import subprocess
from loguru import logger

class GhostRobotClientWrapper:
    _daemon_process = None
    _daemon_lock = threading.Lock()
    
    def __init__(self, sim=True):
        """Initialize connection to robot or simulator via the daemon"""
        logger.info(f"Initializing GhostRobotClientWrapper with sim={sim}")
        self.sim = sim
        self._start_daemon_if_needed()
        
        # Test connection
        response = self._send_command({'action': 'ping'})
        if not response.get('success'):
            logger.error("Failed to connect to mavlink daemon")
            raise RuntimeError("Failed to connect to mavlink daemon")
    
    @classmethod
    def _start_daemon_if_needed(cls):
        """Start the daemon process if it's not already running"""
        with cls._daemon_lock:
            # First try to ping the existing daemon
            if os.path.exists("/tmp/mavlink_socket"):
                try:
                    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                    sock.settimeout(2.0)
                    sock.connect("/tmp/mavlink_socket")
                    sock.sendall(json.dumps({'action': 'ping'}).encode('utf-8') + b'\n')  # Add newline
                    data = sock.recv(1024)
                    sock.close()
                    response = json.loads(data.decode('utf-8'))
                    if response.get('success'):
                        logger.info("Connected to existing mavlink daemon")
                        return
                except Exception as e:
                    logger.warning(f"Failed to connect to existing socket: {str(e)}")
                    try:
                        os.unlink("/tmp/mavlink_socket")
                    except:
                        pass
            
            # Start the daemon process
            if cls._daemon_process is None or cls._daemon_process.poll() is not None:
                logger.info("Starting mavlink daemon process")
                python38_path = "/usr/bin/python3.8"
                daemon_script = os.path.join(os.path.dirname(__file__), "mavlink_daemon.py")
                
                # Start daemon as a background process
                cls._daemon_process = subprocess.Popen(
                    [python38_path, daemon_script],
                    # No stdin/stdout/stderr capture to avoid blocking
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL
                )
                
                # Wait for socket to be created
                timeout = 10
                while timeout > 0 and not os.path.exists("/tmp/mavlink_socket"):
                    time.sleep(0.5)
                    timeout -= 0.5
                
                if not os.path.exists("/tmp/mavlink_socket"):
                    raise RuntimeError("Failed to start mavlink daemon (socket not created)")
                
                # Give a bit more time for the daemon to initialize
                time.sleep(1)
    
    def _send_command(self, command):
        """Send a command to the daemon and get the response"""
        try:
            # Ensure daemon is running
            self._start_daemon_if_needed()
            
            # Add sim parameter to command
            command['sim'] = self.sim
            
            # Connect to socket
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            sock.settimeout(10.0)  # 10 second timeout
            sock.connect("/tmp/mavlink_socket")
            
            # Send command
            sock.sendall(json.dumps(command).encode('utf-8') + b'\n')  # Add newline as message delimiter
            
            # Receive response - don't shut down write end until we're done
            data = b""
            while True:
                chunk = sock.recv(4096)
                if not chunk:
                    break
                data += chunk
            
            # Close socket
            sock.close()
            
            # Parse and return response
            return json.loads(data.decode('utf-8'))
            
        except Exception as e:
            logger.error(f"Error sending command to daemon: {str(e)}")
            return {'success': False, 'error': str(e)}
    
    def set_action_mode(self, mode):
        """Set robot action mode: 0=sit, 1=stand, 2=walk"""
        response = self._send_command({
            'action': 'set_action_mode',
            'mode': mode
        })
        return response.get('current_mode', -1)
    
    def get_action_mode(self):
        """Get current action mode"""
        response = self._send_command({
            'action': 'get_action_mode'
        })
        return response.get('current_mode', -1)
    
    def move_robot(self, x=0.0, y=0.0, z=0.0, r=0.0, duration=0.0):
        """Move the robot based on current action mode"""
        # First check if we're in a valid mode
        mode = self.get_action_mode()
        if mode <= 0:
            logger.warning(f"Cannot move in mode {mode}. Setting to stand mode first.")
            self.set_action_mode(2)  # Set to stand mode
            time.sleep(3)  # Wait for mode change
        
        # Now move
        response = self._send_command({
            'action': 'move_robot',
            'x': x,
            'y': y,
            'z': z,
            'r': r,
            'duration': duration
        })
        return response.get('success', False)
    
    def stop_movement(self):
        """Stop all robot movement"""
        response = self._send_command({
            'action': 'stop_movement'
        })
        return response.get('success', False)
    
    def get_status(self):
        """Get robot status"""
        response = self._send_command({
            'action': 'get_status'
        })
        return response.get('status', {})
    
    def emergency_stop(self):
        """Trigger emergency stop"""
        response = self._send_command({
            'action': 'emergency_stop'
        })
        return response.get('success', False)
    
    def take_over(self):
        """Take over control"""
        response = self._send_command({
            'action': 'take_over'
        })
        return response.get('success', False)
    
    def close(self):
        """Close connection (but keep daemon running)"""
        return True
    
    @classmethod
    def shutdown_daemon(cls):
        """Shutdown the daemon (call this when your application exits)"""
        try:
            # Try to send shutdown command
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            sock.connect("/tmp/mavlink_socket")
            sock.sendall(json.dumps({'action': 'shutdown'}).encode('utf-8'))
            sock.close()
            
            # Wait for daemon to exit
            if cls._daemon_process:
                cls._daemon_process.wait(timeout=5)
                cls._daemon_process = None
                
            # Clean up socket
            if os.path.exists("/tmp/mavlink_socket"):
                os.unlink("/tmp/mavlink_socket")
                
            return True
        except Exception as e:
            logger.error(f"Error shutting down daemon: {str(e)}")
            return False