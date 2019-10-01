
class FlightData:
    def __init__(self, timestamp=0.0, altitude=0.0, pitch=0.0, roll=0.0, yaw=0.0, north=0.0, east=0.0, speed=0.0):
        self.timestamp = timestamp
        self.altitude = altitude
        self.pitch = pitch
        self.roll = roll
        self.yaw = yaw
        self.north = north
        self.east = east
        self.speed = speed


class FlightCommands:
    def __init__(self, timestamp, yaw_drift, roll_drift):
        self.timestamp = timestamp
        self.yaw_drift = yaw_drift
        self.roll_drift = roll_drift