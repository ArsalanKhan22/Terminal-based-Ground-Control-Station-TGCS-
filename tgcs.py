#!/usr/bin/env python3
"""
Terminal‑based GCS for PX4 + MAVROS
==================================
A terminal-based Ground Control Station (GCS) for controlling PX4 drones via MAVROS.

Features:
- Command-line interface for drone control and telemetry
- Supports arming/disarming, mode switching, takeoff, landing, altitude hold, waypoint navigation, and more
- Real-time status and telemetry display
- Command logging for session review
- Keyboard-based manual control

Commands
--------
status [sec]                 – Telemetry once or every 1s for [sec]
arm / disarm                 – Arm or disarm the drone
mode <MODE>                  – Set PX4 flight mode
alt <z>                      – Hold relative altitude
takeoff <rel_alt>            – Takeoff to relative altitude
land                         – Land at current location
user_control                 – Move drone using arrow keys
speed <m/s>                  – Set desired horizontal speed
heading <deg>                – Set desired heading (yaw)
goto <lat> <lon> <rel_alt>   – Fly to GPS coordinate with relative altitude
wp_add <lat> <lon> <rel_alt> – Add a waypoint to the queue
wp_start                     – Begin navigation through all waypoints
wp_list                      – Show all waypoints
wp_clear                     – Clear all waypoints
quit / exit                  – Exit the terminal
help / ?                     – Show help
"""
import math, time, threading, readline, sys, tty, termios, rospy, os
from datetime import datetime
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion
from sensor_msgs.msg   import BatteryState, NavSatFix
from mavros_msgs.msg   import State, HomePosition
from mavros_msgs.srv   import CommandBool, SetMode, CommandTOL, CommandHome
from scipy.spatial.transform import Rotation as R
import pymap3d as pm

STREAM_HZ     = 10
ARRIVE_THRESH = 0.8
MOVE_STEP     = 0.5

class TGCS:
    """
    Terminal-based Ground Control Station (GCS) for PX4 drones using MAVROS.

    This class provides a command-line interface for controlling and monitoring PX4-based drones via MAVROS.
    It supports arming/disarming, mode switching, takeoff, landing, altitude hold, waypoint navigation, and manual control.

    Attributes
    ----------
    state : State
        Current MAVROS state of the drone.
    pose : PoseStamped
        Current local position of the drone.
    global_pose : NavSatFix
        Current global GPS position of the drone.
    home : HomePosition
        Home position (reference for relative navigation).
    batt : BatteryState
        Battery status.
    vel : TwistStamped
        Current velocity.
    waypoints : list
        List of waypoints (lat, lon, rel_alt) for navigation.
    target_local : PoseStamped or None
        Current local target position for navigation.
    target_lock : threading.Lock
        Lock for synchronizing access to target_local.
    desired_mode : str or None
        Desired PX4 flight mode.
    set_heading : float or None
        Desired heading (yaw) in degrees.
    set_speed : float
        Desired horizontal speed in m/s.
    log_path : str
        Path to the command log file.

    Methods
    -------
    cli_loop()
        Starts the command-line interface loop.
    _dispatch(cmd, args)
        Dispatches a command to the appropriate handler.
    _prepare_offboard()
        Prepares the drone for OFFBOARD mode and arms it if necessary.
    _set_home(args)
        Sets the home position.
    _goto(args)
        Navigates to a specified GPS coordinate.
    _wp_add(args)
        Adds a waypoint to the queue.
    _wp_start()
        Begins navigation through all waypoints.
    _keyboard_control()
        Enables manual control using keyboard arrow keys.
    ...
    """
    def __init__(self):
        """
        Initialize the TGCS class, set up ROS subscribers, publishers, and service proxies.
        Also initializes logging and starts the setpoint streaming thread.
        """
        self.state, self.pose = State(), PoseStamped()
        self.global_pose, self.home = NavSatFix(), HomePosition()
        self.batt = BatteryState()
        self.vel = TwistStamped()
        self.waypoints = []

        self.target_local, self.target_lock = None, threading.Lock()
        self.desired_mode = None
        self.set_heading = None
        self.set_speed = 1.0

        now = datetime.now()
        log_name = now.strftime("%Y-%m-%d_%H-%M-%S_TGCS.log")
        self.log_path = os.path.join(os.path.expanduser("/home/ars/catkin_ws_terminal/src/terminal_gcs/scripts"), log_name)
        with open(self.log_path, "w") as f:
            f.write(f"TGCS Command Log - Started at {now.strftime('%Y-%m-%d %H:%M:%S')}\n")

        rospy.Subscriber("/mavros/state", State, lambda m: setattr(self,'state',m))
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, lambda m: setattr(self,'pose',m))
        rospy.Subscriber("/mavros/global_position/global", NavSatFix, lambda m: setattr(self,'global_pose',m))
        rospy.Subscriber("/mavros/home_position/home", HomePosition, lambda m: setattr(self,'home',m))
        rospy.Subscriber("/mavros/battery", BatteryState, lambda m: setattr(self,'batt',m))
        rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, lambda m: setattr(self,'vel',m))

        self.local_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        rospy.wait_for_service("/mavros/cmd/arming")
        self.arm_srv  = rospy.ServiceProxy("/mavros/cmd/arming",  CommandBool)
        self.mode_srv = rospy.ServiceProxy("/mavros/set_mode",    SetMode)
        self.tko_srv  = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)
        self.land_srv = rospy.ServiceProxy("/mavros/cmd/land",    CommandTOL)

        threading.Thread(target=self._stream_loop, daemon=True).start()
        print("TGCS ready. Type 'help' for commands.")

    def _log_command(self, cmd, args):
        """
        Log a command and its arguments to the session log file with a timestamp.

        Parameters
        ----------
        cmd : str
            The command name.
        args : list
            List of command arguments.
        """
        timestamp = datetime.now().strftime("[%Y-%m-%d %H:%M:%S]")
        command_str = f"{cmd} {' '.join(args)}"
        with open(self.log_path, "a") as f:
            f.write(f"{timestamp} Command: {command_str}\n")

    def _prepare_offboard(self):
        """
        Prepare the drone for OFFBOARD mode and arm it if not already armed.
        Publishes dummy setpoints to enable OFFBOARD mode.
        """
        if self.state.mode != "OFFBOARD":
            print("Switching to OFFBOARD mode...")
            dummy = PoseStamped()
            dummy.header.frame_id = "map"
            dummy.pose = self.pose.pose
            for _ in range(20):
                dummy.header.stamp = rospy.Time.now()
                self.local_pub.publish(dummy)
                rospy.sleep(0.1)
            self.mode_srv(0, "OFFBOARD")
            time.sleep(1)

        if not self.state.armed:
            print("Arming drone...")
            self.arm_srv(True)
            time.sleep(1)

    def _yaw_to_quaternion(self, yaw_deg):
        """
        Convert a yaw angle in degrees to a Quaternion message.

        Parameters
        ----------
        yaw_deg : float
            Yaw angle in degrees.

        Returns
        -------
        Quaternion
            Quaternion representing the yaw rotation.
        """
        r = R.from_euler('z', yaw_deg, degrees=True)
        q = r.as_quat()
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    def _stream_loop(self):
        """
        Continuously publish the current or target setpoint to the drone at STREAM_HZ rate.
        Handles arrival detection and target clearing.
        """
        rate = rospy.Rate(STREAM_HZ)
        while not rospy.is_shutdown():
            with self.target_lock:
                tgt = self.target_local
            if self.state.mode == "OFFBOARD":
                if tgt:
                    tgt.header.stamp = rospy.Time.now()
                    self.local_pub.publish(tgt)
                    dx = self.pose.pose.position.x - tgt.pose.position.x
                    dy = self.pose.pose.position.y - tgt.pose.position.y
                    dz = self.pose.pose.position.z - tgt.pose.position.z
                    
                    dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                    if dist < ARRIVE_THRESH:
                        print("Reached target.")
                        with self.target_lock: self.target_local = None
                else:
                    cur = PoseStamped()
                    cur.header.stamp = rospy.Time.now(); cur.header.frame_id = "map"
                    cur.pose = self.pose.pose
                    self.local_pub.publish(cur)
            rate.sleep()

    def _set_target(self, pose):
        """
        Set the current navigation target (local position).

        Parameters
        ----------
        pose : PoseStamped
            The target pose to set.
        """
        with self.target_lock: self.target_local = pose

    def _set_home(self, args):
        """
        Set the home position for the drone, either to the current GPS or specified coordinates.

        Parameters
        ----------
        args : list
            Either ["current"] or [lat, lon, alt].
        """
        if args == ["current"]:
            lat  = self.global_pose.latitude
            lon  = self.global_pose.longitude
            alt  = self.global_pose.altitude
        elif len(args) == 3:
            lat, lon, alt = map(float, args)
        else:
            raise ValueError("set_home current | <lat> <lon> <alt>")

        self.home.geo.latitude  = lat
        self.home.geo.longitude = lon
        self.home.geo.altitude  = alt
        print(f"Home set to: Lat {lat:.6f}, Lon {lon:.6f}, Alt {alt:.1f}")

        try:
            if not hasattr(self, "set_home_srv"):
                rospy.wait_for_service("/mavros/cmd/set_home")
                self.set_home_srv = rospy.ServiceProxy("/mavros/cmd/set_home", CommandHome)
            res = self.set_home_srv(current_gps=False, yaw=0.0,
                                    latitude=lat, longitude=lon, altitude=alt)
            if res.success:
                print("PX4 home location updated.")
            else:
                print("Failed to set home in PX4.")
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def cli_loop(self):
        """
        Start the command-line interface loop, accepting and dispatching user commands.
        """
        while not rospy.is_shutdown():
            try:
                tokens = input("TGCS> ").strip().split()
            except (EOFError, KeyboardInterrupt):
                print("\nExit."); break
            if tokens: self._dispatch(tokens[0].lower(), tokens[1:])

    def _dispatch(self, cmd, args):
        """
        Dispatch a command to the appropriate handler based on the command string.

        Parameters
        ----------
        cmd : str
            The command name.
        args : list
            List of command arguments.
        """
        try:
            self._log_command(cmd, args)
            if cmd in ("help", "?"): print(__doc__)
            elif cmd == "status": self._status_loop(args)
            elif cmd == "arm": self.arm_srv(True)
            elif cmd == "disarm": self.arm_srv(False)
            elif cmd == "mode": self._set_mode(args)
            elif cmd == "set_home": self._set_home(args)
            elif cmd == "takeoff": self._takeoff(args)
            elif cmd == "land": self._land()
            elif cmd == "alt": self._hold_alt(args)
            elif cmd == "goto": self._goto(args)
            elif cmd == "wp_add": self._wp_add(args)
            elif cmd == "wp_list": self._wp_list()
            elif cmd == "wp_clear": self._wp_clear()
            elif cmd == "wp_start": self._wp_start()
            elif cmd == "speed": self._set_speed(args)
            elif cmd == "heading": self._set_heading(args)
            elif cmd == "user_control": self._keyboard_control()
            elif cmd in ("quit", "exit"): rospy.signal_shutdown("bye")
            else: print("Unknown—type 'help'.")
        except Exception as e:
            print("Error:", e)

    def _status_loop(self, args):
        """
        Print telemetry status once or every second for a specified number of times.

        Parameters
        ----------
        args : list
            Optional: [count] for number of status prints.
        """
        count = int(args[0]) if args else 1
        for _ in range(count):
            self._status()
            if count > 1: time.sleep(1)

    def _status(self):
        """
        Print the current drone status, including mode, position, speed, heading, GPS, and battery.
        """
        p = self.pose.pose.position
        v = self.vel.twist.linear
        spd = math.sqrt(v.x**2 + v.y**2 + v.z**2)
        heading = self._quat_to_yaw(self.pose.pose.orientation)
        print(f"Mode: {self.state.mode:<10} Armed: {self.state.armed}")
        print(f"Local→ X:{p.x:+6.1f} Y:{p.y:+6.1f} Z:{p.z:+6.1f} Speed:{spd:.2f} m/s Heading:{heading:+5.1f}°")
        if self.global_pose.status.status >= 0:
            print(f"Global→ Lat:{self.global_pose.latitude:.6f} Lon:{self.global_pose.longitude:.6f} Alt:{self.global_pose.altitude:.1f} Sats:{self.global_pose.status.service}")
        else:
            print("Global→ waiting GPS…")
        vb = self.batt.voltage if self.batt.voltage > 1e-3 else float('nan')
        print(f"Battery: {vb:.2f} V")

    def _quat_to_yaw(self, q):
        """
        Convert a Quaternion to a yaw angle in degrees.

        Parameters
        ----------
        q : Quaternion
            The orientation quaternion.

        Returns
        -------
        float
            Yaw angle in degrees.
        """
        s = 2*(q.w*q.z + q.x*q.y); c = 1 - 2*(q.y*q.y + q.z*q.z)
        return math.degrees(math.atan2(s, c))

    def _set_mode(self, args):
        """
        Set the PX4 flight mode.

        Parameters
        ----------
        args : list
            [mode] to set.
        """
        if not args: raise ValueError("Provide mode")
        mode = args[0].upper()
        self.desired_mode = mode
        self.mode_srv(0, mode)

    def _takeoff(self, args):
        """
        Command the drone to take off to a specified relative altitude.

        Parameters
        ----------
        args : list
            [rel_alt] relative altitude in meters.
        """
        rel = float(args[0]) if args else 5.0
        if self.home.geo.latitude == 0:
            print("Home not set"); return
        abs_alt = self.home.geo.altitude + rel
        self.tko_srv(0, 0, self.home.geo.latitude, self.home.geo.longitude, abs_alt)
        print(f"Takeoff {rel} m AGL")

    def _land(self):
        """
        Command the drone to land at the current location and clear the navigation target.
        """
        self.land_srv(0, 0, self.global_pose.latitude, self.global_pose.longitude, self.home.geo.altitude)
        self._set_target(None)
        print("Landing …")

    def _hold_alt(self, args):
        """
        Command the drone to hold a specified relative altitude at its current XY position.

        Parameters
        ----------
        args : list
            [rel_alt] relative altitude in meters.
        """
        rel = float(args[0])
        tgt = PoseStamped(); tgt.header.frame_id = "map"
        tgt.pose.position.x = self.pose.pose.position.x
        tgt.pose.position.y = self.pose.pose.position.y
        tgt.pose.position.z = rel
        tgt.pose.orientation = self.pose.pose.orientation
        self._set_target(tgt); print(f"Hold {rel} m AGL")

    def _goto(self, args):
        """
        Navigate the drone to a specified GPS coordinate and relative altitude.

        Parameters
        ----------
        args : list
            [lat, lon, rel_alt] target GPS and altitude.
        """
        if len(args) != 3:
            raise ValueError("goto <lat> <lon> <rel_alt>")
        
        lat, lon, rel_alt = map(float, args)

        if self.home.geo.latitude == 0:
            print("No home GPS lock."); return

        # Convert GPS target to local ENU
        x, y, _ = pm.geodetic2enu(
            lat, lon, self.home.geo.altitude + rel_alt,
            self.home.geo.latitude, self.home.geo.longitude, self.home.geo.altitude
        )

        dx = x - self.pose.pose.position.x
        dy = y - self.pose.pose.position.y
        yaw_deg = math.degrees(math.atan2(dy, dx))
        quat = self._yaw_to_quaternion(yaw_deg)

        # Step 1: climb to required altitude
        climb_pose = PoseStamped()
        climb_pose.header.frame_id = "map"
        climb_pose.pose.position.x = self.pose.pose.position.x
        climb_pose.pose.position.y = self.pose.pose.position.y
        climb_pose.pose.position.z = rel_alt
        climb_pose.pose.orientation = quat

        # Step 2: go to target
        goto_pose = PoseStamped()
        goto_pose.header.frame_id = "map"
        goto_pose.pose.position.x = x
        goto_pose.pose.position.y = y
        goto_pose.pose.position.z = rel_alt
        goto_pose.pose.orientation = quat

        # Send 20 dummy setpoints first (required by PX4 to start OFFBOARD mode)
        print("Sending warm-up setpoints …")
        for _ in range(20):
            self.local_pub.publish(climb_pose)
            rospy.sleep(0.05)

        # Arm and switch to OFFBOARD
        self.arm_srv(True)
        result = self.mode_srv(0, "OFFBOARD")
        if not result.mode_sent:
            print("Failed to enter OFFBOARD"); return
        print("OFFBOARD mode set")

        # Set target to climb
        self._set_target(climb_pose)
        print(f"Climbing to {rel_alt:.1f} m AGL …")

        # Wait until altitude is reached
        while not rospy.is_shutdown():
            current_z = self.pose.pose.position.z
            if abs(current_z - rel_alt) < 0.5:
                break
            rospy.sleep(0.5)

        # Set target to GPS location
        self._set_target(goto_pose)
        print(f"→ Navigating to {lat:.6f}, {lon:.6f} at {rel_alt:.1f} m (yaw {yaw_deg:.1f}°)")

    def _set_speed(self, args):
        """
        Set the desired horizontal speed for navigation.

        Parameters
        ----------
        args : list
            [speed] in m/s.
        """
        if not args: raise ValueError("Provide speed")
        self.set_speed = float(args[0])
        print(f"Set horizontal speed to {self.set_speed} m/s")

    def _set_heading(self, args):
        """
        Set the desired heading (yaw) for the drone.

        Parameters
        ----------
        args : list
            [heading] in degrees.
        """
        if not args: raise ValueError("Provide heading")
        self.set_heading = float(args[0])
        print(f"Set yaw to {self.set_heading:.1f}° (feature pending)")

    def _wp_add(self, args):
        """
        Add a waypoint (lat, lon, rel_alt) to the navigation queue.

        Parameters
        ----------
        args : list
            [lat, lon, rel_alt] waypoint coordinates.
        """
        if len(args) != 3: raise ValueError("wp_add <lat> <lon> <rel_alt>")
        self.waypoints.append(tuple(map(float, args)))
        print("Waypoint added.")

    def _wp_list(self):
        """
        Print the list of all waypoints in the queue.
        """
        if not self.waypoints:
            print("No waypoints.")
        else:
            for i, wp in enumerate(self.waypoints):
                print(f"{i+1}: lat={wp[0]:.6f}, lon={wp[1]:.6f}, rel_alt={wp[2]:.1f}")

    def _wp_clear(self):
        """
        Clear all waypoints from the navigation queue.
        """
        self.waypoints.clear()
        print("Waypoint queue cleared.")

    def _wp_start(self):
        """
        Begin navigation through all waypoints in the queue.
        """
        if not self.waypoints:
            print("No waypoints to navigate.")
            return

        self._prepare_offboard()

        for lat, lon, rel in self.waypoints:
            x, y, _ = pm.geodetic2enu(lat, lon, self.home.geo.altitude + rel,
                                      self.home.geo.latitude, self.home.geo.longitude, self.home.geo.altitude)
            dx = x - self.pose.pose.position.x
            dy = y - self.pose.pose.position.y
            yaw_deg = math.degrees(math.atan2(dy, dx))
            quat = self._yaw_to_quaternion(yaw_deg)
            # Step 1: climb to required altitude
            climb_pose = PoseStamped()
            climb_pose.header.frame_id = "map"
            climb_pose.pose.position.x = self.pose.pose.position.x
            climb_pose.pose.position.y = self.pose.pose.position.y
            climb_pose.pose.position.z = rel
            climb_pose.pose.orientation = quat

            self._set_target(climb_pose)
            print(f"Climbing to {rel:.1f} m AGL before waypoint …")
            
            # Wait until altitude is reached
            while not rospy.is_shutdown():
                current_z = self.pose.pose.position.z
                if abs(current_z - rel) <= 1:
                    print((current_z - rel))
                    break
                rospy.sleep(0.5)
            # Step 2: fly to waypoint
            pose = PoseStamped(); pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = rel
            pose.pose.orientation = quat

            self._set_target(pose)
            print(f"Navigating to waypoint: lat={lat:.6f}, lon={lon:.6f}, alt={rel:.1f} (yaw {yaw_deg:.1f}°)")

            # Wait until position is reached
            while not rospy.is_shutdown():
                time.sleep(1)
                with self.target_lock:
                    if self.target_local is None:
                        break


    def _keyboard_control(self):
        """
        Enable manual drone control using keyboard arrow keys for local movement.
        ESC exits manual control mode.
        """
        print("Arrow keys to move ±0.5 m. ESC to exit.")
        fd = sys.stdin.fileno(); old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            pose = self.pose.pose.position; tgt = PoseStamped(); tgt.header.frame_id = "map"
            tgt.pose.position.x = pose.x; tgt.pose.position.y = pose.y; tgt.pose.position.z = pose.z
            tgt.pose.orientation = self.pose.pose.orientation
            self._set_target(tgt)
            while not rospy.is_shutdown():
                ch = sys.stdin.read(1)
                if ch == '\x1b':
                    nxt = sys.stdin.read(1)
                    if nxt == '[':
                        arrow = sys.stdin.read(1)
                        if arrow == 'A': tgt.pose.position.x += MOVE_STEP
                        elif arrow == 'B': tgt.pose.position.x -= MOVE_STEP
                        elif arrow == 'C': tgt.pose.position.y -= MOVE_STEP
                        elif arrow == 'D': tgt.pose.position.y += MOVE_STEP
                        self._set_target(tgt)
                    else: break
                else: break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

if __name__ == "__main__":
    rospy.init_node("tgcs_cli", anonymous=True)
    TGCS().cli_loop()
