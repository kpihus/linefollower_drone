from dotenv import load_dotenv
load_dotenv()
from vision.eyes import Eyes
from wings.phoenix import Phoenix
from vision.eyes import Eyes
from ears.ears import Ears

from multiprocessing import Process
from multiprocessing.managers import BaseManager
from queue import LifoQueue
import zmq
import time

context = zmq.Context()
footage_socket = context.socket(zmq.PUB)
footage_socket.connect('tcp://localhost:5555')


class QueueManager(BaseManager):
    pass


QueueManager.register('LifoQueue', LifoQueue)

if __name__ == "__main__":

    manager = QueueManager()
    manager.start()

    flight_params = manager.LifoQueue()
    flight_commands = manager.LifoQueue()
    image_queue = manager.LifoQueue()

    phoenix = Phoenix(flight_params, flight_commands)
    eyes = Eyes(flight_params, flight_commands, image_queue)

    visual = Process(target=eyes.start_capture)
    physical = Process(target=phoenix.loop)
    print("Start proc")
    visual.start()
    physical.start()
    print("Join proc visual")
    #visual.join()
    print("Join proc physical")
    #physical.join()
    print("Starg image rec")
    while True:
        if not image_queue.empty():
            image_string = image_queue.get()
            
            footage_socket.send(image_string)
        time.sleep(0.01)
