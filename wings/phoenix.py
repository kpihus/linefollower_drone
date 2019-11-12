from dronekit import connect, Command, LocationGlobal, VehicleMode
from simple_pid import PID
from models.data import FlightData
from pymavlink import mavutil
import time, sys, argparse, math, threading

TARGET_ALTITUDE = 1.5
UPDATE_INTERVAL = 0.01

def to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]


class Phoenix:
    def __init__(self, flight_params=None, flight_commands=None):
        self.connection_string = '127.0.0.1:14540'
        self.vehicle = None
        self.last_heartbeat = None
        self.fpq = flight_params
        self.fcq = flight_commands
        self.last_flight_commands = time.time()
        self.flightData = FlightData()
        self.thrust = 0.0
        self.yaw_drift = 0.0
        self.roll_drift = 0.0

        self.thrust_pid = None
        self.roll_pid = None
        self.yaw_pid = None

        # default angles 0, 0 and current yaw
        self.roll_angle = -999
        self.pitch_angle = 0
        self.yaw_angle = -999

        # setup PIDs
        self.setup_pids()

    def loop(self):
        self.connect()
        self.vehicle.mode = VehicleMode("LOITER")

        while not self.vehicle.armed:
            print(" Waiting for arming..." + str(self.vehicle.armed))
            self.vehicle.armed = True
            time.sleep(1)

        self.vehicle.mode = VehicleMode("OFFBOARD")
        print("Starting infinite loop")

        # setup PIDs
        self.setup_pids()

        while self.vehicle.mode != "LAND":
            start = time.time()
            if not self.fcq.empty():
                flight_commands = self.fcq.get()
                if flight_commands and flight_commands.timestamp > self.last_flight_commands:
                    self.yaw_drift = flight_commands.yaw_drift
                    self.roll_drift = flight_commands.roll_drift

            self.gather_info()

            self.altitude_holder()
            self.yaw_holder()
            self.roll_holder()

            self.set_attitude(self.roll_angle, self.pitch_angle, self.yaw_angle, 0.0, False)
            process_time = time.time() - start
            time.sleep(UPDATE_INTERVAL)

    def connect(self):
        print("Connecting to Bird")
        vehicle = connect(self.connection_string, wait_ready=True)
        print(" Type: %s" % vehicle._vehicle_type)
        print(" Armed: %s" % vehicle.armed)
        print(" System status: %s" % vehicle.system_status.state)
        self.vehicle = vehicle

    def setup_pids(self):
        # ------------------- THRUST
        # Setup PID parameters (NOTE: These are tested values, do not randomly change these)
        thrust_kp = 0.35
        thrust_ki = 0.2
        thrust_kd = 0.35

        # Setup PID
        self.thrust_pid = PID(thrust_kp, thrust_ki, thrust_kd, setpoint=TARGET_ALTITUDE)
        self.thrust_pid.sample_time = UPDATE_INTERVAL  # we're currently updating this 0.01
        self.thrust_pid.output_limits = (0, 1)

        # ------------------- ROLL

        roll_kp = 0.2
        roll_ki = 0.2
        roll_kd = 0.8

        # PID for roll
        self.roll_pid = PID(roll_kp, roll_ki, roll_kd, setpoint=0)
        self.roll_pid.sample_time = UPDATE_INTERVAL  # we're currently updating this 0.01
        self.roll_pid.output_limits = (-5, 5)  # roll angle should only be between 5 / -5

        # ------------------- YAW
        yaw_kp = 0.2
        yaw_ki = 0.2
        yaw_kd = 0.8

        # PID for yaw
        self.yaw_pid = PID(yaw_kp, yaw_ki, yaw_kd, setpoint=0)
        self.yaw_pid.sample_time = UPDATE_INTERVAL  # we're currently updating this 0.01
        self.yaw_pid.output_limits = (-180, 180)

    def gather_info(self):
        altitude = self.vehicle.location.local_frame.down * -1
        pitch = math.degrees(self.vehicle.attitude.pitch)
        roll = math.degrees(self.vehicle.attitude.roll)
        yaw = math.degrees(self.vehicle.attitude.yaw)
        north = self.vehicle.location.local_frame.north
        east = self.vehicle.location.local_frame.east
        speed = self.vehicle.groundspeed
        self.flightData = FlightData(time.time(), altitude, pitch, roll, yaw, north, east, speed)
        self.fpq.put(self.flightData)

    def altitude_holder(self):
        self.thrust = self.thrust_pid(self.flightData.altitude)
        #print("new thrust", self.thrust)

    def roll_holder(self):
        print("calculating roll angle")
        self.roll_angle = self.roll_pid(math.degrees(self.vehicle.attitude.roll) - self.roll_drift)
        #print("new roll angle", self.roll_angle)

    def yaw_holder(self):
        self.yaw_angle = self.yaw_pid(math.degrees(self.vehicle.attitude.yaw) - self.yaw_drift)

    def land(self):
        vehicle = self.vehicle
        vehicle.mode = VehicleMode("LAND")

    def set_attitude(self, roll_angle=0.0, pitch_angle=0.0,
                     yaw_angle=None, yaw_rate=0.0, use_yaw_rate=True):
        """
        Note that from AC3.3 the message should be re-sent more often than every
        second, as an ATTITUDE_TARGET order has a timeout of 1s.
        In AC3.2.1 and earlier the specified attitude persists until it is canceled.
        The code below should work on either version.
        Sending the message multiple times is the recommended way.
        """

        # if any of the angles are defined as -1, keep whatever vehicle currently has
        current_attitude = self.vehicle.attitude
        roll_angle = math.degrees(current_attitude.roll) if roll_angle == -999 else roll_angle
        pitch_angle = math.degrees(current_attitude.pitch) if pitch_angle == -999 else pitch_angle
        yaw_angle = math.degrees(current_attitude.yaw) if yaw_angle == -999 else yaw_angle

        self.send_attitude_target(roll_angle, pitch_angle, yaw_angle, yaw_rate, use_yaw_rate)

    def send_attitude_target(self, roll_angle=0.0, pitch_angle=0.0,
                             yaw_angle=None, yaw_rate=0.0, use_yaw_rate=True):
        vehicle = self.vehicle
        """
        use_yaw_rate: the yaw can be controlled using yaw_angle OR yaw_rate.
                      When one is used, the other is ignored by Ardupilot.
        thrust: 0 <= thrust <= 1, as a fraction of maximum vertical thrust.
                Note that as of Copter 3.5, thrust = 0.5 triggers a special case in
                the code for maintaining current altitude.
        """

        if not use_yaw_rate and yaw_angle is None:
            yaw_angle = vehicle.attitude.yaw

        if yaw_angle is None:
            yaw_angle = 0.0

        print("New attitude", roll_angle, pitch_angle, yaw_angle)

        # Thrust >  0.5: Ascend
        # Thrust == 0.5: Hold the altitude
        # Thrust <  0.5: Descend
        msg = vehicle.message_factory.set_attitude_target_encode(
            0,  # time_boot_ms
            1,  # Target system
            1,  # Target component
            0b00000000 if use_yaw_rate else 0b00000100,
            to_quaternion(roll_angle, pitch_angle, yaw_angle),  # Quaternion
            0,  # Body roll rate in radian
            0,  # Body pitch rate in radian
            math.radians(yaw_rate),  # Body yaw rate in radian/second
            self.thrust  # Thrust
        )
        vehicle.send_mavlink(msg)


if __name__ == "__main__":

    p = Phoenix()
    p.connect()
