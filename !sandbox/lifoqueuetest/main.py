from multiprocessing import Process
from multiprocessing.managers import BaseManager
from queue import LifoQueue

from sender import Sender
from receiver import Receiver

class QueueManager(BaseManager):
    pass


QueueManager.register('LifoQueue', LifoQueue)

if __name__ == "__main__":
    manager = QueueManager()
    manager.start()

    lifo = manager.LifoQueue()

    s = Sender(lifo)
    r = Receiver(lifo)

    p1 = Process(target=s.process)
    p1.start()

    p2 = Process(target=r.process)
    p2.start()
    p1.join()
    p2.join()
