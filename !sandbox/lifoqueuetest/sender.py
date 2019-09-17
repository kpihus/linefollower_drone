import time
from data import Data


class Sender:
    def __init__(self, q):
        self.q = q

    def process(self):
        while True:
            a = Data(time.time())
            print("Sender working" + str(a))
            self.q.put(a)
            time.sleep(1)
