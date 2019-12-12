from dronekit import connect, Command, LocationGlobal, VehicleMode
from simple_pid import PID
from models.data import FlightData
from pymavlink import mavutil
import time, sys, argparse, math, threading, os

TARGET_ALTITUDE = 1.35
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
        self.platform = os.getenv('PLATFORM')
        self.connection_string = os.getenv('CONNECTION_STRING')
        self.vehicle = None
        self.last_heartbeat = None
        self.fpq = flight_params
        self.fcq = flight_commands
        self.last_flight_commands = time.time()
        self.flightData = FlightData()
        self.thrust = 0.1
        self.yaw_drift = 0.0
        self.roll_drift = 0.0
        self.pitch_drift = 0.0
        self.opt_flow_speed = 0.0
        self.state = "INIT"

        self.altch_pid = None
        self.thrust_pid = None
        self.pitch_pid = None
        self.roll_pid = None
        self.yaw_pid = None

        # default angles 0, 0 and current yaw
        self.roll_angle = 0
        self.pitch_angle = 0
        self.yaw_angle = -999

        # setup PIDs
        self.setup_pids()

        # stage lengths in seconds
        self.takeoff_stage_time = 1
        self.start_moving_time = 1

        #print("Current platform:", self.platform)

    def loop(self):
        """
            The loop will go through few stages,
            firstly we will make sure the vehicle is armed and in correct mode to accept attitude
            then we will enter into "get vehicle into air" mode, this will enable thrust PID and get vehicle to
            TARGET_ALTITUDE

            after takeoff stage, we will modify vehicle pitch so it would start moving forward
            after start moving stage, we will enable yaw and pitch PIDs to actually keep the vehicle on expect track

            stages are switched using time constraints for simplifications
        :return:
        """
        self.connect()
        if self.platform == 'SIMU':
            self.vehicle.mode = VehicleMode("LOITER")

        self.gather_info()
        hold_north = self.flightData.north
        hold_east = self.flightData.east

        # setup PIDs
        self.setup_pids()

        #print(time.time(), "Starting stage ONE 'arming'")
        while self.flightData.altitude < TARGET_ALTITUDE - 0.2:
            self.gather_info()

            msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
                0,  # time_boot_ms (not used)
                0, 0,  # target system, target component
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
                0b0000111111111000,  # type_mask (only positions enabled)
                hold_north, hold_east, -TARGET_ALTITUDE + 0.2,
                0, 0, 0,  # x, y, z velocity in m/s  (not used)
                0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
                0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
            # send command to vehic         le
            self.vehicle.send_mavlink(msg)

            if self.flightData.altitude > 0.1:
                self.altitude_holder()

            #print("Pre takeoff thrust", self.thrust)

            time.sleep(UPDATE_INTERVAL)

        self.pitch_angle = -float(os.getenv('SPEED'))
        print("Starting stage FOUR 'AI'", time.time())
        while self.vehicle.mode != "LAND":
            self.state = "FLY"
            start = time.time()

            self.gather_info()
            self.altitude_holder()
            print("Speed:", self.vehicle.groundspeed)
            #self.pitch_holder()

            if not self.fcq.empty():
                flight_commands = self.fcq.get()
                if flight_commands and flight_commands.timestamp > self.last_flight_commands:
                    self.yaw_drift = flight_commands.yaw_drift
                    self.roll_drift = flight_commands.roll_drift
                    self.opt_flow_speed = flight_commands.opt_flow_speed
                    #print("Drifts", self.roll_drift, self.yaw_drift)
                    #print("Opt flow speed", self.opt_flow_speed)
                    self.yaw_holder()
                    self.roll_holder()
                    self.last_flight_commands = flight_commands.timestamp

                    if flight_commands.number_of_lines == 0:
                        self.pitch_angle = 0
                    else:
                        self.pitch_angle = -float(os.getenv('SPEED'))

                        if self.vehicle.groundspeed > 0.5:
                            self.pitch_angle = 1

            self.set_attitude(self.roll_angle, self.pitch_angle, self.yaw_angle, 0.0, False)
            process_time = time.time() - start
            time.sleep(UPDATE_INTERVAL)

    def connect(self):
        #print("Connecting to Bird")
        vehicle = connect(self.connection_string, wait_ready=False, baud=int(os.getenv('BAUD')))
        vehicle.wait_ready('system_status', 'mode', 'armed', 'attitude')
        #print(" Type: %s" % vehicle._vehicle_type)
        #print(" Armed: %s" % vehicle.armed)
        #print(" System status: %s" % vehicle.system_status.state)

        # 823: ATTITUDE {time_boot_ms : 1360740, roll : 0.026306351646780968, pitch : 0.04570530727505684, yaw : -0.462706595659256, rollspeed : 0.0037141498178243637, pitchspeed : 0.0011765370145440102, yawspeed : -0.0026037555653601885}
        # 248: LOCAL_POSITION_NED {time_boot_ms : 1360712, x : 0.0006204088567756116, y : -0.00553098926320672, z : 0.0022805137559771538, vx : -0.0015985831851139665, vy : 0.004514497239142656, vz : 0.004841562360525131}
        # 82: DISTANCE_SENSOR {time_boot_ms: 1360657, min_distance: 30, max_distance: 1200, current_distance: 16, type: 0, id: 0, orientation: 25, covariance: 0, horizontal_fov: 0.0, vertical_fov: 0.0, quaternion: [0.0, 0.0, 0.0, 0.0]}
        vehicle.add_attribute_listener('attitude', self.process_data)
        self.vehicle = vehicle

    def process_data(self, attr_name, value, g):
        print(" CALLBACK: (%s):" % (attr_name))

    def setup_pids(self):
        # Altitude by channel override
        # altch_kp = float(os.getenv('ALTCH_KP'))
        # altch_ki = float(os.getenv('ALTCH_KI'))
        # altch_kd = float(os.getenv('ALTCH_KD'))
        #
        # self.altch_pid = PID(altch_kp, altch_ki, altch_kd, setpoint=TARGET_ALTITUDE)
        # self.altch_pid.sample_time = UPDATE_INTERVAL
        # self.altch_pid.output_limits = (1300, 2000)


        # ------------------- THRUST
        # Setup PID parameters (NOTE: These are tested values, do not randomly change these)
        thrust_kp = float(os.getenv('THRUST_KP'))
        thrust_ki = float(os.getenv('THRUST_KI'))
        thrust_kd = float(os.getenv('THRUST_KD'))

        # Setup PID
        self.thrust_pid = PID(thrust_kp, thrust_ki, thrust_kd, setpoint=TARGET_ALTITUDE)
        self.thrust_pid.sample_time = UPDATE_INTERVAL  # we're currently updating this 0.01
        self.thrust_pid.output_limits = (0.39, 1)

        # ------------------- ROLL

        roll_kp = float(os.getenv('ROLL_KP'))
        roll_ki = float(os.getenv('ROLL_KI'))
        roll_kd = float(os.getenv('ROLL_KD'))

        # PID for roll
        self.roll_pid = PID(roll_kp, roll_ki, roll_kd, setpoint=0)
        self.roll_pid.sample_time = UPDATE_INTERVAL  # we're currently updating this 0.01
        self.roll_pid.output_limits = (-5, 5)  # roll angle should only be between 5 / -5

        # ------------------- YAW
        yaw_kp = float(os.getenv('YAW_KP'))
        yaw_ki = float(os.getenv('YAW_KI'))
        yaw_kd = float(os.getenv('YAW_KD'))

        # PID for yaw
        self.yaw_pid = PID(yaw_kp, yaw_ki, yaw_kd, setpoint=0)
        self.yaw_pid.sample_time = UPDATE_INTERVAL  # we're currently updating this 0.01
        self.yaw_pid.output_limits = (-60, 60)

        # ------------------- PITCH
        pitch_kp = float(os.getenv('PITCH_KP'))
        pitch_ki = float(os.getenv('PITCH_KI'))
        pitch_kd = float(os.getenv('PITCH_KD'))

        # PID for yaw
        self.pitch_pid = PID(pitch_kp, pitch_ki, pitch_kd, setpoint=0.25)
        self.pitch_pid.sample_time = UPDATE_INTERVAL  # we're currently updating this 0.01
        self.pitch_pid.output_limits = (-5, 1.7)

    def gather_info(self):
        altitude = self.vehicle.location.local_frame.down * -1
        pitch = math.degrees(self.vehicle.attitude.pitch)
        roll = math.degrees(self.vehicle.attitude.roll)
        yaw = math.degrees(self.vehicle.attitude.yaw)
        north = self.vehicle.location.local_frame.north
        east = self.vehicle.location.local_frame.east
        #print("North/east", north, east)
        speed = self.vehicle.groundspeed
        thres_val = self.vehicle.channels['7']
        thres_max = self.vehicle.channels['8']
        state = self.state
        self.flightData = FlightData(time.time(), altitude, pitch, roll, yaw, north, east, speed, thres_val, thres_max, state)
        self.fpq.put(self.flightData)

    def altitude_holder(self):
        self.thrust = self.thrust_pid(self.flightData.altitude)
        #print("new thrust", self.thrust, " from altitude ", self.flightData.altitude)
        pass

    def roll_holder(self):
        self.roll_angle = self.roll_pid(self.roll_drift)
        ##print("new roll angle", self.roll_angle)
        pass

    def yaw_holder(self):
        self.yaw_angle = math.degrees(self.vehicle.attitude.yaw) - self.yaw_pid(self.yaw_drift)
        pass

    def pitch_holder(self):
        self.pitch_angle = -self.pitch_pid(self.vehicle.groundspeed)
        pass

    def land(self):
        vehicle = self.vehicle
        vehicle.mode = VehicleMode("LAND")

    def set_attitude(self, roll_angle=0.0, pitch_angle=0.0,
                     yaw_angle=None, yaw_rate=0.0, use_yaw_rate=True):

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

        print("New attitude", roll_angle, pitch_angle, yaw_angle, self.thrust, self.flightData.altitude)

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
