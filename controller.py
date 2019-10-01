from vision.eyes import Eyes
from wings.phoenix import Phoenix
from vision.eyes import Eyes
from ears.ears import Ears

from multiprocessing import Process
from multiprocessing.managers import BaseManager
from queue import LifoQueue


class QueueManager(BaseManager):
    pass


QueueManager.register('LifoQueue', LifoQueue)

if __name__ == "__main__":

    manager = QueueManager()
    manager.start()

    flight_params = manager.LifoQueue()
    flight_commands = manager.LifoQueue()

    phoenix = Phoenix(flight_params, flight_commands)
    eyes = Eyes(flight_params, flight_commands)

    visual = Process(target=eyes.start_capture)
    physical = Process(target=phoenix.loop)

    visual.start()
    physical.start()

    visual.join()
    physical.join()
