import time


class Receiver:
    def __init__(self, q):
        self.q = q

    def process(self):
        while True:
            t = self.q.get()
            print("Receiver working" + str(t.t))
            time.sleep(3)
