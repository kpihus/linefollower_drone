from dronekit import connect, Command, LocationGlobal, VehicleMode
from pymavlink import mavutil
import time, sys, argparse, math, threading

STABLE_THRUST = 0.588
TAKEOFF_THRUST = 0.6

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
    def __init__(self):
        self.connection_string = '127.0.0.1:14540'
        self.vehicle = None
        self.current_thrust = 0
        self.last_heartbeat = None

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
        vehicle.add_message_listener('HEARTBEAT', self.heart)
        vehicle.add_message_listener('ATTITUDE', self.attitude_manager)
        self.vehicle = vehicle

    def heart(self, name, msg, a):
        self.last_heartbeat = time.time()

    def attitude_manager(self, vehicle, name, msg, ):
        pitch = math.degrees(msg.pitch)
        roll = math.degrees(msg.roll)
        # print("Roll: " + str(round(roll, 2)) + " Pitch: " + str(round(pitch, 2)))

    def arm_and_takeoff(self, target_altitude):
        vehicle = self.vehicle


        # while not vehicle.is_armable:
        #     print(" Waiting for vehicle to initialise...")
        #     time.sleep(1)



        vehicle.mode = VehicleMode("LOITER")
        while not vehicle.armed:
            print(" Waiting for arming...")
            vehicle.armed = True
            time.sleep(1)
        print("Armed: " + str(vehicle.armed))
        current_altitude = vehicle.location.local_frame.down * -1
        vehicle.mode = VehicleMode("OFFBOARD")
        while current_altitude < target_altitude:
            current_altitude = vehicle.location.local_frame.down * -1
            self.current_thrust = TAKEOFF_THRUST
            self.set_attitude()
        print('Takeoff done' + str(current_altitude))
        self.current_thrust = STABLE_THRUST
        self.set_attitude()
        self.altitude_holder(1.5)

    def altitude_holder(self, target_altitude):
        vehicle = self.vehicle
        ACCEPTABLE_ALTITUDE_ERROR = 0.15

        print("Altitdude holer started")

        kp = 0.001
        ki = 0.5
        kd = 0

        last_error = 0
        errors_sum = 0

        while (vehicle.mode != "LAND"):
            current_altitude = vehicle.location.local_frame.down * -1
            # print(" Altitude: %f Target Altitude: %f " % (current_altitude, target_altitude))
            # print
            # " Attitude: %s" % vehicle.attitude

            # print " Velocity: %s" % vehicle.velocity
            # print " Groundspeed: %s" % vehicle.groundspeed    # settable

            error = target_altitude - current_altitude

            if current_altitude < target_altitude + 0.1 or current_altitude > target_altitude - 0.1:
                errors_sum = 0

            thrust = STABLE_THRUST + error * kp + errors_sum * ki + (last_error - error) * kd

            if thrust > 1:
                thrust = 1
            if thrust < 0:
                thrust = 0

            self.current_thrust = thrust

            print("Current thrust: " + str(self.current_thrust) + "Alt  - " + str(current_altitude))
            self.set_attitude()
            last_error = error
            errors_sum += error
            time.sleep(0.01)

    def land(self):
        vehicle = self.vehicle
        vehicle.mode = VehicleMode("LAND")

    def drive(self):
        # todo read new attitude params from queue
        self.set_attitude()


    def set_attitude(self, roll_angle=0.0, pitch_angle=0.0,
                     yaw_angle=None, yaw_rate=0.0, use_yaw_rate=True,
                     duration=0):
        """
        Note that from AC3.3 the message should be re-sent more often than every
        second, as an ATTITUDE_TARGET order has a timeout of 1s.
        In AC3.2.1 and earlier the specified attitude persists until it is canceled.
        The code below should work on either version.
        Sending the message multiple times is the recommended way.
        """

        self.send_attitude_target(roll_angle, pitch_angle,
                             yaw_angle, yaw_rate, use_yaw_rate)
        start = time.time()
        while time.time() - start < duration:
            self.send_attitude_target(roll_angle, pitch_angle,
                                 yaw_angle, yaw_rate, use_yaw_rate)
            time.sleep(0.01)
        # Reset attitude, or it will persist for 1s more due to the timeout
        self.send_attitude_target(0, 0,
                             0, 0, True)

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

        global current_thrust

        if not use_yaw_rate and yaw_angle is None:
            yaw_angle = vehicle.attitude.yaw

        if yaw_angle is None:
            yaw_angle = 0.0

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

p = Phoenix()
p.connect()
p.arm_and_takeoff(1)