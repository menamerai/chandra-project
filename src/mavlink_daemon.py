import os
import sys
import json
import socket
import threading
import logging
import time
import signal


# Configure logging to output to both file and console with filename in format
def setup_logger():
    """Set up logger to output to both file and console"""
    # Create logger
    logger = logging.getLogger("mavlink_daemon")
    logger.setLevel(logging.INFO)

    # Create formatter with filename
    formatter = logging.Formatter(
        "%(asctime)s - %(name)s - %(filename)s:%(lineno)d - %(levelname)s - %(message)s"
    )

    # Create file handler
    file_handler = logging.FileHandler("/tmp/mavlink_daemon.log", mode="a")
    file_handler.setFormatter(formatter)

    # Create console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setFormatter(formatter)

    # Add handlers to logger
    logger.addHandler(file_handler)
    logger.addHandler(console_handler)

    return logger


# Set up the logger
logger = setup_logger()

# Import mavlink module
from mavlink import GhostRobotClient

# Socket path
SOCKET_PATH = "/tmp/mavlink_socket"


class MavlinkDaemon:
    def __init__(self, sim=True):
        """Initialize the daemon with a persistent GhostRobotClient"""
        logger.info(f"Starting Mavlink Daemon (sim={sim})")

        # Create the robot client (only once for the entire daemon lifetime)
        self.client = GhostRobotClient(sim=sim)
        self.running = True
        self.lock = threading.Lock()

        # Set up signal handlers
        signal.signal(signal.SIGINT, self.handle_shutdown)
        signal.signal(signal.SIGTERM, self.handle_shutdown)

        # Clean up socket if it exists
        if os.path.exists(SOCKET_PATH):
            os.unlink(SOCKET_PATH)

        # Create socket server
        self.server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.server.bind(SOCKET_PATH)
        self.server.listen(5)
        self.server.settimeout(1.0)  # Allow checking shutdown condition

    def handle_shutdown(self, signum, frame):
        """Handle shutdown signals"""
        logger.info(f"Shutdown signal received ({signum})")
        self.running = False

    def handle_client(self, conn):
        """Handle a client connection and process command"""
        try:
            # Receive data
            data = b""
            while True:
                chunk = conn.recv(4096)
                if not chunk:
                    break
                data += chunk
                if b"\n" in chunk:  # Look for message delimiter
                    break

            # Process command if we have data
            if data:
                try:
                    with self.lock:  # Thread safety for the robot client
                        command = json.loads(data.strip().decode("utf-8"))
                        response = self.process_command(command)
                        conn.sendall(json.dumps(response).encode("utf-8"))
                except Exception as e:
                    logger.error(f"Error processing command: {str(e)}")
                    conn.sendall(
                        json.dumps({"success": False, "error": str(e)}).encode("utf-8")
                    )

        except Exception as e:
            logger.error(f"Connection handling error: {str(e)}")
        finally:
            conn.close()

    def process_command(self, command):
        """Process a command received from a client"""
        action = command.get("action")

        # Common response structure
        response = {"success": False}

        try:
            if action == "ping":
                response = {"success": True, "message": "pong"}

            elif action == "set_action_mode":
                mode = command.get("mode", 0)
                result = self.client.set_action_mode(mode)
                response = {"success": True, "current_mode": result}

            elif action == "move_robot":
                x = command.get("x", 0.0)
                y = command.get("y", 0.0)
                z = command.get("z", 0.0)
                r = command.get("r", 0.0)
                duration = command.get("duration", 0.0)
                result = self.client.move_robot(x, y, z, r, duration)
                response = {"success": result}

            elif action == "stop_movement":
                result = self.client.stop_movement()
                response = {"success": result}

            elif action == "get_status":
                status = self.client.get_status()
                # Convert numpy arrays to lists for JSON serialization
                clean_status = {}
                for key, value in status.items():
                    if hasattr(value, "tolist"):
                        clean_status[key] = value.tolist()
                    else:
                        clean_status[key] = value
                response = {"success": True, "status": clean_status}

            elif action == "emergency_stop":
                result = self.client.emergency_stop()
                response = {"success": result}

            elif action == "take_over":
                result = self.client.take_over()
                response = {"success": result}

            elif action == "get_action_mode":
                result = self.client.get_action_mode()
                response = {"success": True, "current_mode": result}

            elif action == "roll_over":
                mode = command.get("mode", 1)
                logger.info(f"Daemon: executing roll_over with mode={mode}")
                result = self.client.roll_over(mode)
                response = {"success": result}

            elif action == "shutdown":
                logger.info("Shutdown command received")
                response = {"success": True, "message": "Shutting down"}
                self.running = False

            else:
                response = {"success": False, "error": f"Unknown action: {action}"}

        except Exception as e:
            logger.error(f"Error processing command {action}: {str(e)}")
            response = {"success": False, "error": str(e)}

        return response

    def run(self):
        """Run the daemon main loop"""
        logger.info(f"Daemon running on {SOCKET_PATH}")

        while self.running:
            try:
                # Accept connections with timeout to allow checking shutdown flag
                try:
                    conn, addr = self.server.accept()
                    threading.Thread(target=self.handle_client, args=(conn,)).start()
                except socket.timeout:
                    continue

            except Exception as e:
                logger.error(f"Socket error: {str(e)}")
                if self.running:
                    time.sleep(1)  # Avoid busy loop in case of persistent errors

        # Cleanup
        logger.info("Daemon shutting down...")
        try:
            self.client.close()
            self.server.close()
            if os.path.exists(SOCKET_PATH):
                os.unlink(SOCKET_PATH)
        except Exception as e:
            logger.error(f"Error during shutdown: {str(e)}")

        logger.info("Daemon stopped")


if __name__ == "__main__":
    # Run the daemon with sim=True by default
    sim = True
    if len(sys.argv) > 1 and sys.argv[1] == "real":
        sim = False

    daemon = MavlinkDaemon(sim=sim)
    daemon.run()
