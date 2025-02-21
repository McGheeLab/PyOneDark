from multiprocessing import Process, Queue
import threading
import queue
import sys
import time
import math

from SupportClasses.DeviceInterface import ZPStageManager, XYStageManager


from qt_core import *


# manages command processing in a separate thread usable by any device.
class Processor:
    def __init__(self):
        # Thread-safe queue for commands; each command is a tuple: (command_name, args, kwargs)
        self._queue = queue.Queue()
        # Dictionary mapping event names to lists of subscriber callbacks
        self._subscribers = {}
        self._running = True
        # Start the processing thread
        self._thread = threading.Thread(target=self._process_loop, name="ProcessorThread", daemon=True)
        self._thread.start()

    def register_handler(self, command_name, handler):
        """
        Register a callback for a specific command.
        :param command_name: str - the name of the command to subscribe to.
        :param handler: callable - a function to call when the command is processed.
        """
        if command_name not in self._subscribers:
            self._subscribers[command_name] = []
        self._subscribers[command_name].append(handler)

    def unregister_handler(self, command_name, handler):
        """
        Unregister a previously registered handler.
        """
        if command_name in self._subscribers:
            self._subscribers[command_name].remove(handler)
            if not self._subscribers[command_name]:
                del self._subscribers[command_name]

    def add_command(self, command_name, *args, **kwargs):
        """
        Add a new command to the processor's queue.
        :param command_name: str - the identifier for the command.
        :param args: positional arguments for the handler.
        :param kwargs: keyword arguments for the handler.
        """
        self._queue.put((command_name, args, kwargs))

    def _process_loop(self):
        """
        Internal method run in a separate thread. Processes commands as they arrive.
        """
        while self._running:
            try:
                # Wait for a command; timeout allows checking self._running periodically.
                command_name, args, kwargs = self._queue.get(timeout=1)
            except queue.Empty:
                continue

            # Dispatch command to all subscribers registered for that command name.
            handlers = self._subscribers.get(command_name, [])
            if not handlers:
                # Optionally, log or handle unregistered commands.
                print(f"[Processor] No handler registered for command: {command_name}")
            else:
                for handler in handlers:
                    try:
                        handler(*args, **kwargs)
                    except Exception as e:
                        # Exception handling for a misbehaving handler.
                        print(f"[Processor] Error in handler {handler} for command '{command_name}': {e}")
            self._queue.task_done()

    def stop(self):
        """
        Signal the processor to stop and wait for the thread to finish.
        """
        self._running = False
        self._thread.join()


class StageHandler:
    def __init__(self, processor, zp_stage, xy_stage):
        self.XYUPDATE_INTERVAL = 1.0
        self.ZUPDATE_INTERVAL = 0.5
        self.POS_UPDATE_INTERVAL = 0.5623  # Polling interval for updating positions

        self.processor = processor
        self.zp_stage = zp_stage
        self.xy_stage = xy_stage
        self._running = True
        # Stages are off by default.
        self._zp_running = False
        self._xy_running = False

        # State dictionaries storing full information for each axis.
        self.zp_state = {
            "Z": {"position": 0.0, "velocity": 0.0, "active": False, "user_deactivated": False},
            "P1": {"position": 0.0, "velocity": 0.0, "active": False, "user_deactivated": False},
            "P2": {"position": 0.0, "velocity": 0.0, "active": False, "user_deactivated": False},
            "P3": {"position": 0.0, "velocity": 0.0, "active": False, "user_deactivated": False},
        }
        
        self.axes_mapping = {
            "Z": "X",
            "P1": "Y",
            "P2": "Z",
            "P3": "E"
        }
        
        self.xy_state = {
            "x": {"position": 0.0, "velocity": 0.0, "active": False, "user_deactivated": False},
            "y": {"position": 0.0, "velocity": 0.0, "active": False, "user_deactivated": False},
            "f": {"position": 0.0, "velocity": 0.0, "active": False, "user_deactivated": False},
        }

        # Register command handlers.
        self.processor.register_handler("move_axis_1mm", self.handle_move_axis_1mm)
        self.processor.register_handler("set_axis_deactivation", self.handle_set_axis_deactivation)
        self.processor.register_handler("move_z_at_velocity", self.update_z_velocity)
        self.processor.register_handler("move_p1_at_velocity", self.update_p1_velocity)
        self.processor.register_handler("move_p2_at_velocity", self.update_p2_velocity)
        self.processor.register_handler("move_p3_at_velocity", self.update_p3_velocity)
        self.processor.register_handler("move_stage_at_velocity", self.update_xy_velocity)
        self.processor.register_handler("control_stage", self.handle_control_stage)

        # Start background threads.
        self.zp_thread = threading.Thread(target=self._zp_stage_loop, name="ZPStageHandlerThread", daemon=True)
        self.xy_thread = threading.Thread(target=self._xy_stage_loop, name="XYStageHandlerThread", daemon=True)
        self.pos_thread = threading.Thread(target=self._update_positions_loop, name="StagePositionUpdateThread", daemon=True)
        self.zp_thread.start()
        self.xy_thread.start()
        self.pos_thread.start()
    
    # ----- basic move to location commands for both stages -----
    def move_abs_z(self, z_value):
        pass
    
    def move_abs_xy(self, x_value, y_value):
        pass
    
    def move_rel_z(self, z_value):
        pass
    
    def move_rel_xy(self, x_value, y_value):
        pass  
    
    # ----- Velocity Update Methods for ZP Stage -----
    def update_z_velocity(self, *args, **kwargs):
        v1, v2 = self._extract_velocity(*args, **kwargs)
        # find non-zero velocity
        velocity = next((v for v in [v1, v2] if v != 0), 0.0)
        self.zp_state["Z"]["velocity"] = velocity
        self.zp_state["Z"]["active"] = (velocity != 0)

    def update_p1_velocity(self, *args, **kwargs):
        v1, v2 = self._extract_velocity(*args, **kwargs)
        # find non-zero velocity
        velocity = next((v for v in [v1, v2] if v != 0), 0.0)
        self.zp_state["P1"]["velocity"] = velocity
        self.zp_state["P1"]["active"] = (velocity != 0)

    def update_p2_velocity(self, *args, **kwargs):
        v1, v2 = self._extract_velocity(*args, **kwargs)
        # find non-zero velocity
        velocity = next((v for v in [v1, v2] if v != 0), 0.0)
        self.zp_state["P2"]["velocity"] = velocity
        self.zp_state["P2"]["active"] = (velocity != 0)

    def update_p3_velocity(self, *args, **kwargs):
        v1, v2 = self._extract_velocity(*args, **kwargs)
        # find non-zero velocity
        velocity = next((v for v in [v1, v2] if v != 0), 0.0)
        self.zp_state["P3"]["velocity"] = velocity
        self.zp_state["P3"]["active"] = (velocity != 0)

    # ----- Velocity Update for XY Stage -----
    def update_xy_velocity(self, *args, **kwargs):
        vx, vy = self._extract_velocity(*args, **kwargs)
        
        self.xy_state["x"]["velocity"] = vx
        self.xy_state["y"]["velocity"] = vy
        self.xy_state["x"]["active"] = (vx != 0)
        self.xy_state["y"]["active"] = (vy != 0)


    def _extract_velocity(self, *args, **kwargs):
        # Check if average is passed as a keyword argument.
        if "average" in kwargs:
            velocity = kwargs["average"]
        else:
            velocity = 0.0

        # check if velocity is a tuple or single value
        if isinstance(velocity, (list, tuple)):
            v1, v2 = velocity[0], velocity[1]
        else:
            v1, v2 = velocity, 0.0
        return v1, v2

    # ----- Stage Move Loops -----
    def _zp_stage_loop(self):
        while self._running:
            if self._zp_running:
                if (self.zp_state["Z"]["velocity"] != 0 or 
                    self.zp_state["P1"]["velocity"] != 0 or 
                    self.zp_state["P2"]["velocity"] != 0 or 
                    self.zp_state["P3"]["velocity"] != 0):
                    self.send_zp_move_command()
            time.sleep(self.ZUPDATE_INTERVAL)

    def _xy_stage_loop(self):
        last_xy_velocity = (None, None, None)
        while self._running:
            if self._xy_running:
                vx = self.xy_state["x"]["velocity"]
                vy = self.xy_state["y"]["velocity"]
                vz = self.xy_state["f"]["velocity"]
                current_xy_velocity = (vx, vy, vz)
                if current_xy_velocity != last_xy_velocity:
                    self.xy_stage.move_stage_at_velocity(vx, vy)
                    last_xy_velocity = current_xy_velocity
            time.sleep(self.XYUPDATE_INTERVAL)

    def send_zp_move_command(self):
        print("Sending ZP move command")
        dt = self.ZUPDATE_INTERVAL
        vz = self.zp_state["Z"]["velocity"]
        vp1 = self.zp_state["P1"]["velocity"]
        vp2 = self.zp_state["P2"]["velocity"]
        vp3 = self.zp_state["P3"]["velocity"]
        dz = vz * dt
        dp1 = vp1 * dt
        dp2 = vp2 * dt
        dp3 = vp3 * dt
        distance = math.sqrt(dz**2 + dp1**2 + dp2**2 + dp3**2)
        if distance == 0:
            return
        feedrate = (distance / dt) * 60
        axes = {'X': dz, 'Y': dp1, 'Z': dp2, 'E': dp3} # Mapping to contoller axes
        print(f"axes {axes} at feedrate {feedrate}")
        self.zp_stage.movecommand(axes, feedrate)

    # ----- Position Polling and State Update -----
    def _update_positions_loop(self):
        while self._running:
            zp_positions = self.zp_stage.get_current_position()  # e.g., (z, p1, p2, p3)
            if zp_positions and len(zp_positions) >= 4:
                self.zp_state["Z"]["position"] = zp_positions[0]
                self.zp_state["P1"]["position"] = zp_positions[1]
                self.zp_state["P2"]["position"] = zp_positions[2]
                self.zp_state["P3"]["position"] = zp_positions[3]
            xy_positions = self.xy_stage.get_current_position()  # e.g., (x, y, f)
            if xy_positions and len(xy_positions) >= 3:
                self.xy_state["x"]["position"] = xy_positions[0]
                self.xy_state["y"]["position"] = xy_positions[1]
                self.xy_state["f"]["position"] = xy_positions[2]
            time.sleep(self.POS_UPDATE_INTERVAL)


    # ----- Command Handlers for GUI Commands -----
    def handle_move_axis_1mm(self, **kwargs):
        stage = kwargs.get("stage")
        axis = kwargs.get("axis")
        distance = kwargs.get("distance", 1)
        if stage == "ZP":
            if axis in self.zp_state:
                mapped_axis = self.axes_mapping.get(axis)
                axes = {mapped_axis: distance}
                self.zp_stage.movecommand(axes, feedrate=60)
                print(f"Moving {axis} axis of ZP stage by {distance}mm")
        elif stage == "XY":
            if axis in self.xy_state:
                if axis == "x":
                    self.xy_stage.move_stage_at_velocity(distance, 0)
                elif axis == "y":
                    self.xy_stage.move_stage_at_velocity(0, distance)
                elif axis == "f":
                    print("f axis move not supported yet for XY stage")
                print(f"Moving {axis} axis of XY stage by {distance}mm")
        else:
            print(f"Unknown stage: {stage}")

    def handle_set_axis_deactivation(self, **kwargs):
        stage = kwargs.get("stage")
        axis = kwargs.get("axis")
        deactivated = kwargs.get("deactivated", False)
        if stage == "ZP":
            if axis in self.zp_state:
                self.zp_state[axis]["user_deactivated"] = deactivated
                print(f"ZP stage {axis} deactivated: {deactivated}")
        elif stage == "XY":
            if axis in self.xy_state:
                self.xy_state[axis]["user_deactivated"] = deactivated
                print(f"XY stage {axis} deactivated: {deactivated}")

    def handle_control_stage(self, **kwargs):
        stage = kwargs.get("stage")
        action = kwargs.get("action")
        if stage == "ZP":
            if action == "start":
                self._zp_running = True
                print("ZP stage started")
            elif action == "stop":
                self._zp_running = False
                print("ZP stage stopped")
        elif stage == "XY":
            if action == "start":
                self._xy_running = True
                print("XY stage started")
            elif action == "stop":
                self._xy_running = False
                print("XY stage stopped")
        else:
            print(f"Unknown stage for control: {stage}")

    def get_stage_info(self):
        return {"ZP": self.zp_state.copy(), "XY": self.xy_state.copy()}

    def set_stage_info(self, stage, axis, info):
        if stage == "ZP" and axis in self.zp_state:
            self.zp_state[axis].update(info)
        elif stage == "XY" and axis in self.xy_state:
            self.xy_state[axis].update(info)
        

    def stop(self):
        self._running = False
        self.zp_thread.join()
        self.xy_thread.join()
        self.pos_thread.join()


class AppController:
    def __init__(self):
        self.processor = Processor()
        # Initialize device attributes as None.
        self.xbox_interface = None  # Not used now because we use a process.
        self.zp_stage = None
        self.xy_stage = None
        self.stage_handler = None
        
        # Create a Queue for Xbox polling messages.
        self.xbox_queue = Queue()
        self.xbox_process = None
        self.xbox_timer = None  # QTimer to poll the queue.
        
        
        self.processor.register_handler("control_xbox", self.handle_control_xbox)# Register control handler for Xbox commands.
        self.processor.register_handler("control_stage", self.handle_control_stages)# Register a handle to start the stage devices.
        self.processor.register_handler("debug", self.debug_handler)# Register a simple debug handler that prints debug messages.
        
        
    def debug_handler(self, *args, **kwargs):
        message = kwargs.get("message", "")
        print("DEBUG:", message)
    
    # --- Xbox Process Control ---
    def start_xbox_interface(self):
        if self.xbox_process is None:
            # Start the Xbox polling process.
            from SupportClasses.XboxControl import xbox_polling_worker  # import here if needed
            self.xbox_process = Process(target=xbox_polling_worker, args=(self.xbox_queue,))
            self.xbox_process.start()
            print("Xbox polling process started.")
        else:
            print("Xbox polling process already running.")
    
    def stop_xbox_interface(self):
        if self.xbox_process:
            self.xbox_process.terminate()
            self.xbox_process.join()
            self.xbox_process = None
            print("Xbox polling process stopped.")
        else:
            print("Xbox polling process not running.")
    
    # --- Stage Devices Control (unchanged from previous example) ---
    def start_stage_devices(self):
        if self.zp_stage is None:
            self.zp_stage = ZPStageManager(simulate=False)
            print("ZP stage started.")
        else:
            print("ZP stage already running.")
        if self.xy_stage is None:
            self.xy_stage = XYStageManager(simulate=False)
            print("XY stage started.")
        else:
            print("XY stage already running.")
        if self.stage_handler is None:
            self.stage_handler = StageHandler(self.processor, self.zp_stage, self.xy_stage)
            print("StageHandler started.")
        else:
            print("StageHandler already running.")
    
    def stop_stage_devices(self):
        if self.stage_handler:
            try:
                self.stage_handler.stop()
                print("StageHandler stopped.")
            except Exception as e:
                print("Error stopping StageHandler:", e)
            finally:
                self.stage_handler = None
        else:
            print("StageHandler not running.")
        if self.zp_stage:
            try:
                self.zp_stage.stop()
                print("ZP stage stopped.")
            except Exception as e:
                print("Error stopping ZP stage:", e)
            finally:
                self.zp_stage = None
        else:
            print("ZP stage not running.")
        if self.xy_stage:
            try:
                self.xy_stage.stop()
                print("XY stage stopped.")
            except Exception as e:
                print("Error stopping XY stage:", e)
            finally:
                self.xy_stage = None
        else:
            print("XY stage not running.")
    
    def restart_stage_devices(self):
        self.stop_stage_devices()
        self.start_stage_devices()
        print("Stage devices restarted successfully.")
        
    def get_stage_info(self):
        if self.stage_handler is not None:
            return self.stage_handler.get_stage_info()
        else:
            print("StageHandler is not running.")
            return {"ZP": {}, "XY": {}}
    
    # --- Control functions for stages (as before) ---
    def handle_control_xbox(self, **kwargs):
        action = kwargs.get("action")
        if action == "start":
            self.start_xbox_interface()
        elif action == "stop":
            self.stop_xbox_interface()
        else:
            print(f"Unknown action for Xbox control: {action}")
    
    # --- Stage Devices Control ---
    def handle_control_stages(self, **kwargs):
        action = kwargs.get("action")
        if action == "start":
            self.start_stage_devices()
        elif action == "stop":
            self.stop_stage_devices()
        else:
            print(f"Unknown action for Stage control: {action}")
    
    def start_stage_devices(self):
        if self.zp_stage is None:
            self.zp_stage = ZPStageManager(simulate=False)
            print("ZP stage started.")
        else:
            print("ZP stage already running.")
        if self.xy_stage is None:
            self.xy_stage = XYStageManager(simulate=False)
            print("XY stage started.")
        else:
            print("XY stage already running.")
        if self.stage_handler is None:
            self.stage_handler = StageHandler(self.processor, self.zp_stage, self.xy_stage)
            print("StageHandler started.")
        else:
            print("StageHandler already running.")

    def stop_stage_devices(self):
        if self.stage_handler:
            try:
                self.stage_handler.stop()
                print("StageHandler stopped.")
            except Exception as e:
                print("Error stopping StageHandler:", e)
            finally:
                self.stage_handler = None
        else:
            print("StageHandler not running.")
        if self.zp_stage:
            try:
                self.zp_stage.stop()
                print("ZP stage stopped.")
            except Exception as e:
                print("Error stopping ZP stage:", e)
            finally:
                self.zp_stage = None
        else:
            print("ZP stage not running.")
        if self.xy_stage:
            try:
                self.xy_stage.stop()
                print("XY stage stopped.")
            except Exception as e:
                print("Error stopping XY stage:", e)
            finally:
                self.xy_stage = None
        else:
            print("XY stage not running.")

    # --- requests ---
    def get_stage_info(self):
        if self.stage_handler is not None:
            return self.stage_handler.get_stage_info()
        else:
            print("StageHandler is not running.")
            return {"ZP": {}, "XY": {}}

    # --- Application Lifecycle ---
    def start(self):
        self.app = QApplication(sys.argv)
        # Do not automatically start Xbox or stage devices since toggles are off.
        exit_code = self.app.exec()
        self.shutdown()
        sys.exit(exit_code)

    def shutdown(self):
        print("AppController: Shutting down application.")
        if self.xbox_interface:
            self.stop_xbox_interface()
        if self.stage_handler or self.zp_stage or self.xy_stage:
            self.stop_stage_devices()
        self.processor.stop()


class PrintManager:
    def __init__(self, processor, stage_handler):
        """
        This class manages a print job queue and executes print jobs.
        :param processor: The Processor instance to register commands with.
        :param stage_handler: The StageHandler instance used to control stage movement.
        """
        self.processor = processor
        self.stage_handler = stage_handler

        # Global values for location and movement.
        self.fastmove_z = 10.0  # Global fast-move Z value; can be set dynamically.
        self.xyz_zero = (0.0, 0.0, 0.0)  # Global zero position for XY(Z) stages.
        
        # Well queue: list of dict entries.
        # Each entry should have:
        #   "well_id": identifier (string)
        #   "target": tuple (target_x, target_y, target_z)
        #   "print_instance": an object holding all print instructions
        self.well_queue = []  # Well queue: list of dict entries to be printed.

        # Flags for print job control.
        self._pause_flag = threading.Event()  # When set, printing is paused.
        self._stop_flag = threading.Event()   # When set, printing stops and resets.
        self._pause_flag.clear()
        self._stop_flag.clear()
        
        # Thread for print job processing.
        self._print_thread = None  

    
    
    

class PrintManager2:
    def __init__(self, processor, stage_handler):
        """
        :param processor: The Processor instance to register commands with.
        :param stage_handler: The StageHandler instance used to control stage movement.
        """
        self.processor = processor
        self.stage_handler = stage_handler
        
        # Global fast-move Z value; can be set dynamically.
        self.fastmove_z = 10.0
        
        # Well queue: list of dict entries.
        # Each entry should have:
        #   "well_id": identifier (string)
        #   "target": tuple (target_x, target_y, target_z)
        #   "print_file": associated print file name (string)
        self.well_queue = []
        
        # Flags for print job control.
        self._pause_flag = threading.Event()  # When set, printing is paused.
        self._stop_flag = threading.Event()   # When set, printing stops and resets.
        self._pause_flag.clear()
        self._stop_flag.clear()
        
        self._print_thread = None

        # Register command handlers.
        self.processor.register_handler("start_print", self.start_print_job)
        self.processor.register_handler("pause_print", self.pause_print)
        self.processor.register_handler("resume_print", self.resume_print)
        self.processor.register_handler("stop_print", self.stop_print)
        self.processor.register_handler("load_print_job", self.load_print_job_file)
        self.processor.register_handler("save_print_job", self.save_print_job)
        self.processor.register_handler("set_fastmove_z", self.set_fastmove_z)
        self.processor.register_handler("list_print_jobs", self.list_print_jobs)
        self.processor.register_handler("goto", self.handle_goto)
        self.processor.register_handler("calibrate", self.handle_calibrate)

    # -------- print setup --------
    def calibrate(self):# needs revision
        """
        CALIBRATE command: when called, the current x,y,z position is set as 0,0,0.
        This is a blocking command that waits for an external signal to complete.
        """
        pass
        
        
    def set_fastmove_z(self, z_value):
        """
        Set the global fast-move Z height.
        Example usage:
            processor.add_command("set_fastmove_z", 15.0)
        """
        try:
            self.fastmove_z = float(z_value)
            print(f"[PrintManager] Global fast-move Z set to {self.fastmove_z}")
        except Exception as e:
            print(f"[PrintManager] Error setting fast-move Z: {e}")
    
    # -------- Well Queue Management --------
    def list_print_jobs(self):
        """
        Prints and returns a list of pending print jobs as tuples:
          (well_id, print_file)
        """
        jobs = [(job["well_id"], job["print_file"]) for job in self.well_queue]
        print("[PrintManager] Pending print jobs:")
        for well_id, print_file in jobs:
            print(f"   Well: {well_id}, Print File: {print_file}")
        return jobs

    def get_next_job(self):
        """
        Pop and return the next print job from the queue.
        """
        if self.well_queue:
            job = self.well_queue.pop(0)
            print(f"[PrintManager] Next job dequeued: {job['well_id']}")
            return job
        else:
            print("[PrintManager] No jobs in queue.")
            return None

    # -------- File I/O for Print Jobs --------
    def load_print_job_file(self, file_path):
        """
        Load a print job file that contains well jobs.
        Each line in the file should be a CSV line in the format:
          well_id, target_x, target_y, target_z, print_file
        This method clears the current queue before loading.
        """
        try:
            with open(file_path, 'r') as f:
                lines = [line.strip() for line in f if line.strip()]
            # Clear current queue.
            self.well_queue.clear()
            for line in lines:
                parts = line.split(',')
                if len(parts) != 5:
                    print(f"[PrintManager] Skipping invalid line: {line}")
                    continue
                well_id = parts[0].strip()
                try:
                    target_x = float(parts[1].strip())
                    target_y = float(parts[2].strip())
                    target_z = float(parts[3].strip())
                except Exception as e:
                    print(f"[PrintManager] Error parsing coordinates in line '{line}': {e}")
                    continue
                print_file = parts[4].strip()
                job = {
                    "well_id": well_id,
                    "target": (target_x, target_y, target_z),
                    "print_file": print_file
                }
                self.well_queue.append(job)
            print(f"[PrintManager] Loaded {len(self.well_queue)} print job(s) from {file_path}")
        except Exception as e:
            print(f"[PrintManager] Error loading print job file {file_path}: {e}")

    def save_print_job(self, file_path):
        """
        Save the current print job queue to a file in CSV format:
          well_id, target_x, target_y, target_z, print_file
        """
        try:
            with open(file_path, 'w') as f:
                for job in self.well_queue:
                    target_x, target_y, target_z = job["target"]
                    line = f"{job['well_id']},{target_x},{target_y},{target_z},{job['print_file']}\n"
                    f.write(line)
            print(f"[PrintManager] Saved {len(self.well_queue)} print job(s) to {file_path}")
        except Exception as e:
            print(f"[PrintManager] Error saving print job file {file_path}: {e}")

    # -------- Print Job Processing --------
    def start_print_job(self, file_path=None):
        """
        Starts processing the print job queue.
        If a file_path is provided, load that file first (which clears the queue).
        """
        # Optional: load new job file.
        if file_path:
            self.load_print_job_file(file_path)
        self._stop_flag.clear()
        self._pause_flag.clear()
        if self._print_thread is None or not self._print_thread.is_alive():
            self._print_thread = threading.Thread(target=self._process_print_jobs, daemon=True)
            self._print_thread.start()
            print("[PrintManager] Print job processing thread started.")
        else:
            print("[PrintManager] Print job processing already running.")

    # needs revision We should load in all the lines and run them sequentially
    def _process_print_jobs(self):
        """
        Process each print job in the queue sequentially.
        """
        while not self._stop_flag.is_set() and self.well_queue:
            # Pause check.
            if self._pause_flag.is_set():
                time.sleep(0.1)
                continue

            job = self.get_next_job()
            if job is None:
                break

            # Execute GOTO for the job.
            self._goto_job(job)
            
            # If there is a print file associated, execute its instructions.
            if job["print_file"]:
                self._execute_print_file(job["print_file"])

    # needs revision this function should just run one line 
    def _execute_print_file(self, print_file):
        """
        Reads a file where each line has comma-separated fields:
          x,y,z,p1,p2,p3,time
        and processes each line as a printing instruction.
        """
        try:
            with open(print_file, 'r') as pf:
                lines = [line.strip() for line in pf if line.strip()]
            print(f"[PrintManager] Executing PRINT instructions from {print_file} with {len(lines)} steps.")
        except Exception as e:
            print(f"[PrintManager] Error reading print file {print_file}: {e}")
            return

        for line in lines:
            if self._stop_flag.is_set():
                print("[PrintManager] Stop flag set; resetting print job processing.")
                return

            while self._pause_flag.is_set() and not self._stop_flag.is_set():
                time.sleep(0.1)

            parts = line.split(',')
            if len(parts) < 7:
                print(f"[PrintManager] Invalid PRINT instruction: {line}")
                continue

            try:
                x = float(parts[0])
                y = float(parts[1])
                z = float(parts[2])
                p1 = float(parts[3])
                p2 = float(parts[4])
                p3 = float(parts[5])
                duration = float(parts[6])
            except Exception as e:
                print(f"[PrintManager] Error parsing instruction '{line}': {e}")
                continue

            # Move Z fast, then XY, then final Z move.
            self._move_z(self.fastmove_z)
            self._move_xy(x, y)
            self._move_z(z)
            print(f"[PrintManager] Printing step: additional axes p1={p1}, p2={p2}, p3={p3}, dwell for {duration}s")
            time.sleep(duration)

    # -------- Command Handlers for Movement --------
    def handle_goto(self, location):
        """
        Moves to a well location.
        If location is "NEXT" (case-insensitive), the next job in the queue is popped.
        Otherwise, searches for a matching job (without removing it).
        """
        job = None
        if location.upper() == "NEXT":
            job = self.get_next_job()
            if job is None:
                return
        else:
            # Search without removing.
            for j in self.well_queue:
                if j["well_id"] == location:
                    job = j
                    break
            if job is None:
                print(f"[PrintManager] Job with well '{location}' not found in queue.")
                return
        
        self._goto_job(job)

    def _goto_job(self, job):
        """
        Move stages to the job's target location.
        Uses the global fast-move Z before XY move, then sets Z to the target.
        """
        target_x, target_y, target_z = job["target"]
        print(f"[PrintManager] GOTO job '{job['well_id']}': Target ({target_x}, {target_y}, {target_z}) with fastmove Z={self.fastmove_z}")
        # First, move Z to fast-move height.
        self._move_z(self.fastmove_z)
        # Then, move XY to target X, Y.
        self._move_xy(target_x, target_y)
        # Finally, move Z to the target Z.
        self._move_z(target_z)

    def _move_xy(self, x, y):
        """Issue a command to move the XY stage to (x, y)."""
        print(f"[PrintManager] Moving XY stage to ({x}, {y})")
        try:
            self.stage_handler.move_abs_xy(x, y)
        except AttributeError:
            print("[PrintManager] XYStageManager missing move_to; implement alternative move method.")

    def _move_z(self, z):
        """Issue a command to move the Z stage to z."""
        print(f"[PrintManager] Moving Z stage to {z}")
        try:
            self.stage_handler.move_abs_z(z)
        except AttributeError:
            print("[PrintManager] ZPStageManager missing move_to; implement alternative move method.")

    # -------- Print Job Control --------
    def pause_print(self):
        """Pause the current print job processing."""
        self._pause_flag.set()
        print("[PrintManager] Print job processing paused.")

    def resume_print(self):
        """Resume a paused print job."""
        self._pause_flag.clear()
        print("[PrintManager] Print job processing resumed.")

    def stop_print(self):
        """
        Stop the current print job processing.
        Any ongoing PRINT file processing will exit.
        """
        self._stop_flag.set()
        self._pause_flag.clear()
        print("[PrintManager] Print job processing stopped and reset.")
