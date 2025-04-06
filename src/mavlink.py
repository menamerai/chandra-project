import os
import importlib.util
import numpy as np
import time
import logging
import sys

# Configure logging to output to both file and console with filename in format
def setup_logger():
    """Set up logger to output to both file and console"""
    # Create logger
    logger = logging.getLogger("mavlink")
    logger.setLevel(logging.INFO)
    
    # Create formatter with filename
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(filename)s:%(lineno)d - %(levelname)s - %(message)s'
    )
    
    # Create file handler
    file_handler = logging.FileHandler('/tmp/mavlink.log', mode='a')
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

# Set up paths to Ghost SDK
ghost_sdk_path = os.path.expanduser("~/ghost_sim/GhostSDK")
mblink_path = os.path.join(ghost_sdk_path, "share/gr/mblink")
examples_path = os.path.join(mblink_path, "examples")
grpy_path = os.path.join(examples_path, "grpy")

# Point directly to the .so file we know exists
grmblinkpy_path = os.path.join(grpy_path, "grmblinkpy.cpython-38-x86_64-linux-gnu.so")
savepkz_path = os.path.join(grpy_path, "savePKZ.py")

# Set environment variables
os.environ["GHOSTSIM_DIR"] = os.path.join(ghost_sdk_path, "bin")
os.environ["LD_LIBRARY_PATH"] = f"{os.path.join(ghost_sdk_path, 'lib')}:{os.environ.get('LD_LIBRARY_PATH', '')}"
os.environ["CONTROLS_INSTALL_DIR"] = ghost_sdk_path
os.environ["ROS_DOMAIN_ID"] = "124"  # For simulation

# Load grmblinkpy module directly
spec = importlib.util.spec_from_file_location("grmblinkpy", grmblinkpy_path)
grmblinkpy = importlib.util.module_from_spec(spec)
spec.loader.exec_module(grmblinkpy)
MBLink = grmblinkpy.MBLink
Frame = grmblinkpy.Frame
BArm = getattr(grmblinkpy, 'BArm', None)

# Load savePKZ module
spec = importlib.util.spec_from_file_location("savePKZ", savepkz_path)
savePKZ = importlib.util.module_from_spec(spec)
spec.loader.exec_module(savePKZ)

# Create our own MB80v2 class without relative imports
class MB80v2(MBLink):
    diskList = []
    diskSentCount = 0
    steppable_list = []
    steppable_sent_count = 0

    def __init__(self, sim=False, verbose=True, log=False, port=0, use_blocking_get=True):
        MBLink.__init__(self)
        self.log = log
        if log:
            self.startLog()
        self.start("127.0.0.1" if sim else "192.168.168.105", verbose, port, use_blocking_get)
        self.rxstart()
        self.floatArraySendCounter = 0
        self.ext_data = []
    
    def _finalizeLog(self, accum_data):
        # first turn the lists of arrays to 2D arrays
        for key in accum_data:
            if(str(key) != "param_value"):
                accum_data[key] = np.asanyarray(accum_data[key])
        # t lumped with y for mb data
        accum_data['t'] = accum_data['y'][:,-1]
        accum_data['y'] = accum_data['y'][:-1]
        if len(self.ext_data)>0:
            accum_data['ext_data'] = np.asanyarray(self.ext_data)
            print("Appended extra data to log; size=", accum_data['ext_data'].shape)
        dirname = os.path.dirname(__file__)
        log_dir = os.path.join(examples_path, 'logs')
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        fname = savePKZ.savePKZ(log_dir, accum_data)
        return fname
    
    def get(self):
        res = super(MB80v2, self).get()
        res['t'] = res['y'][-1]
        res['y'] = res['y'][:-1]
        return res

    def stopLog(self, save=True):
        accum_data = super(MB80v2, self).stopLog()
        if self.log and save:
            return self._finalizeLog(accum_data)

    def rxstop(self, save=True):
        accum_data = super(MB80v2, self).rxstop()
        if self.log and save:
            return self._finalizeLog(accum_data)


# Create a simplified client for your FastAPI server
class GhostRobotClient:
    def __init__(self, sim=True):
        """Initialize connection to robot or simulator"""
        logger.info(f"Initializing GhostRobotClient with sim={sim}")
        self.mb = MB80v2(sim=sim, verbose=False, log=False, use_blocking_get=False)
        logger.info("MB80v2 initialization complete")
        
        # Verify connection with ping
        try:
            self.mb.sendPing()
            status = self.mb.getLatest()
            logger.info(f"Initial status check: {list(status.keys())}")
        except Exception as e:
            logger.error(f"Error during initialization: {e}")
        
        # Initialize twist values for movement commands
        self.twist_values = np.zeros(4, dtype=np.float32)  # x, y, z, r (forward, lateral, rotation, height)
        
        # Track internal action mode state - ignore what the robot reports
        self._action_mode = 0
        self.set_action_mode(0)  # Default to sit mode
        logger.info(f"Default action mode set to {self._action_mode}")
        logger.info("GhostRobotClient initialization complete")
        
    def set_action_mode(self, mode):
        """Set robot action mode: 0=sit, 1=stand, 2=walk"""
        logger.info(f"Setting action mode to {mode}")
        try:
            # Send the command to the robot
            self.mb.setMode("action", mode)
            time.sleep(1.5)  # Give time for mode change to take effect
            
            # Update internal state (ignore what robot reports)
            self._action_mode = mode
            
            # Still try to get the actual mode for logging purposes
            actual_mode = self.mb.getMode("action")
            logger.info(f"Robot reports action mode: {actual_mode}, internal tracking: {self._action_mode}")
            
            # Return our tracked mode, not what the robot reports
            return self._action_mode
        except Exception as e:
            logger.error(f"Error in set_action_mode: {str(e)}")
            return -1
        
    def get_action_mode(self):
        """Get current action mode (from internal tracking)"""
        try:
            # For debugging, get the actual robot mode
            actual_mode = self.mb.getMode("action")
            if actual_mode != self._action_mode:
                logger.debug(f"Mode discrepancy - robot:{actual_mode}, internal:{self._action_mode}")
            
            # Return our tracked mode, not what the robot reports
            return self._action_mode
        except Exception as e:
            logger.error(f"Error in get_action_mode: {str(e)}")
            return self._action_mode  # Fall back to internal tracking on error
    
    def move_robot(self, x=0.0, y=0.0, z=0.0, r=0.0, duration=0.0):
        """
        Move the robot based on current action mode
        x: forward/backward (-1.0 to 1.0)
        y: left/right (-1.0 to 1.0)
        z: rotation (-1.0 to 1.0)
        r: height adjustment (-1.0 to 1.0) - only in arm mode
        duration: seconds to run the command (if 0, single command)
        """
        logger.info(f"move_robot called with x={x}, y={y}, z={z}, r={r}, duration={duration}")
        
        try:
            # Get action mode from our internal tracking
            action_mode = self._action_mode
            logger.info(f"Using internal action mode: {action_mode}")
            
            # Apply filtering like mavgui.py
            filt_coeff = 0.1
            target_twist = np.array([x, y, z, r], dtype=np.float32)
            self.twist_values = self.twist_values + filt_coeff * (target_twist - self.twist_values)
            
            if duration <= 0:
                # Direct command
                if action_mode == 1:  # STAND mode
                    self.mb.sendPose([0, 0, 1], [self.twist_values[1], self.twist_values[0], self.twist_values[2]])
                elif action_mode == 2:  # WALK mode
                    self.mb.sendManualTwist(self.twist_values)
                else:
                    logger.error(f"Cannot move in action mode {action_mode}")
                    return False
                return True
            else:
                # Timed movement with smooth acceleration/deceleration
                start_time = time.time()
                end_time = start_time + duration
                accel_time = min(1.0, duration / 3)
                
                try:
                    while time.time() < end_time:
                        current_time = time.time()
                        elapsed = current_time - start_time
                        
                        # Calculate interpolation factor
                        if elapsed < accel_time:
                            factor = np.sin((elapsed / accel_time) * (np.pi / 2))
                        elif elapsed > (duration - accel_time):
                            factor = np.sin(((duration - elapsed) / accel_time) * (np.pi / 2))
                        else:
                            factor = 1.0
                        
                        # Calculate current twist values
                        current_twist = factor * target_twist
                        self.twist_values = current_twist
                        
                        # Send command based on mode
                        if action_mode == 1:  # STAND mode
                            self.mb.sendPose([0, 0, 1], [self.twist_values[1], self.twist_values[0], self.twist_values[2]])
                        elif action_mode == 2:  # WALK mode
                            self.mb.sendManualTwist(self.twist_values)
                        else:
                            logger.error(f"Cannot move in action mode {action_mode}")
                            return False
                        
                        time.sleep(0.01)
                    
                    # Final stop command
                    self.stop_movement()
                    return True
                    
                except Exception as e:
                    logger.error(f"Error during timed movement: {e}")
                    self.stop_movement()
                    return False
                
        except Exception as e:
            logger.error(f"Error in move_robot: {str(e)}")
            return False
            
    def stop_movement(self):
        """Stop all robot movement"""
        logger.info("Stopping movement")
        try:
            self.twist_values = np.zeros(4, dtype=np.float32)
            action_mode = self.get_action_mode()
            if action_mode == 1:  # STAND mode
                self.mb.sendPose([0, 0, 1], [0, 0, 0])
            elif action_mode == 2:  # WALK mode
                self.mb.sendManualTwist(self.twist_values)
            return True
        except Exception as e:
            logger.error(f"Error stopping movement: {str(e)}")
            return False
        
    def roll_over(self, mode=1):
        """Trigger roll over action"""
        logger.info("Rolling over")
        try:
            logger.info(f"Setting roll over mode from {self.mb.getMode('roll_over')} to {mode}")
            self.mb.setMode("roll_over", mode)
            time.sleep(8.0)  # Wait for roll over to complete
            return True
        except Exception as e:
            logger.error(f"Error during roll over: {str(e)}")
            return False
        
    def get_status(self):
        """Get robot status"""
        return self.mb.getLatest()
        
    def emergency_stop(self):
        """Trigger emergency stop"""
        logger.info("Emergency stop triggered")
        self.mb.ensureMode("soft_estop", 1, 100)
        return True
        
    def take_over(self):
        """Take over control (like ESC key in mavgui)"""
        logger.info("Taking over control")
        self.mb.requestMode("take_over", 1)
        return True
        
    def close(self):
        """Close connection"""
        logger.info("Closing connection to robot")
        self.mb.rxstop(save=False)
        return True


if __name__ == "__main__":
    # Example usage
    client = GhostRobotClient(sim=True)
    try:
        client.set_action_mode(2)  # Set to WALK mode
        client.move_robot(x=0.0, y=0.5, z=0.0, r=0.0, duration=2.0)  # Move forward for 5 seconds
        time.sleep(2)  # Wait for 2 seconds
        client.stop_movement()  # Stop movement
    finally:
        client.close()  # Ensure connection is closed