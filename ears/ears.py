import math
import time

from dronekit import connect

from ears.data import Data

class Ears:
    def __init__(self, fligth_params):
        self.connection_string = '127.0.0.1:14540'
        self.vehicle = None
        self.queue = fligth_params
        self.last_heartbeat = None

    def connect(self):
        print("Connecting to ears Bird")
        vehicle = connect(self.connection_string, wait_ready=True)
        vehicle.add_message_listener('HEARTBEAT', self.heart)
        self.vehicle = vehicle
        while True:
            altitude = vehicle.location.local_frame.down * -1
            pitch = math.degrees(vehicle.attitude.pitch)
            roll = math.degrees(vehicle.attitude.roll)
            yaw = math.degrees(vehicle.attitude.yaw)
            north = vehicle.location.local_frame.north
            east = vehicle.location.local_frame.east
            speed = vehicle.groundspeed
            d = Data(time.time(), altitude, pitch, roll, yaw, north, east, speed)
            self.queue.put(d)
            time.sleep(0.001)

    def heart(self, name, msg, a):
        self.last_heartbeat = time.time()

# e = Ears()
# e.connect()