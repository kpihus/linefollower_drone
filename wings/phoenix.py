from dronekit import connect, Command, LocationGlobal, VehicleMode
from simple_pid import PID
from pymavlink import mavutil
import time, sys, argparse, math, threading

TARGET_ALTITUDE = 1.5
THRUST_UPDATE_INTERVAL = 0.01

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
        self.current_thrust = 0
        self.last_heartbeat = None
        self.fpq = flight_params
        self.fcq = flight_commands

        # controls drone speed
        self.current_pitch = 0

        self.current_yaw = 0
        self.current_roll = 0


    def infinite_loop(self):
        while True:
            if 8 == 9:
                return
            time.sleep(0.1)

    def connect(self):
        print("Connecting to Bird")
        vehicle = connect(self.connection_string, wait_ready=True)
        print(" Type: %s" % vehicle._vehicle_type)
        print(" Armed: %s" % vehicle.armed)
        print(" System status: %s" % vehicle.system_status.state)
        self.vehicle = vehicle
        self.arm_and_takeoff()


    def arm_and_takeoff(self):
        vehicle = self.vehicle

        vehicle.mode = VehicleMode("LOITER")
        print(" System status: %s" % vehicle.system_status.state)
        print("Starting to arm...")

        while not vehicle.armed:
            print(" Waiting for arming..." + str(vehicle.armed))
            vehicle.armed = True
            time.sleep(1)

        print("Armed: " + str(vehicle.armed))
        vehicle.mode = VehicleMode("OFFBOARD")

        # set starting yaw and roll values
        self.current_yaw = math.degrees(vehicle.attitude.yaw)
        self.current_roll = math.degrees(vehicle.attitude.roll)

        # set starting speed to 0
        self.current_pitch = 0

        # Start the altitude holder, this will take the vehicle to required altitude
        self.altitude_holder(TARGET_ALTITUDE)

    def rad_to_degree (self, value):
        return value * 180/math.pi

    def altitude_holder(self, target_altitude):
        vehicle = self.vehicle

        print("Altitude holder started")

        # Setup PID parameters (NOTE: These are tested values, do not randomly change these)
        kp = 0.35
        ki = 0.2
        kd = 0.35

        # Setup PID
        pid = PID(kp, ki, kd, setpoint=target_altitude)
        pid.sample_time = THRUST_UPDATE_INTERVAL # we're currently updating this 0.01
        pid.output_limits = (0, 1) # thrust can only be 0 to 1

        while (vehicle.mode != "LAND"):
            # get current altitude
            current_altitude = vehicle.location.local_frame.down * -1

            # calculate necessary thrust to be at target_altitude
            thrust = pid(current_altitude)
            self.current_thrust = thrust

            # print("Current thrust: " + str(self.current_thrust) + "Alt  - " + str(current_altitude))
            self.fly()
            time.sleep(THRUST_UPDATE_INTERVAL)

    def land(self):
        vehicle = self.vehicle
        vehicle.mode = VehicleMode("LAND")

    def fly(self):
        self.send_attitude_target(self.current_roll, self.current_pitch, self.current_yaw, 0.0, False)

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

        print("Attitude " + str(vehicle.attitude))

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
            self.current_thrust  # Thrust
        )
        vehicle.send_mavlink(msg)


if __name__ == "__main__":

    p = Phoenix()
    p.connect()
