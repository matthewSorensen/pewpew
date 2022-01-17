from os.path import join
import os
import threading
import queue
import serial
from pewpew.parser import ProtocolParser
from pewpew.worker_thread import WorkerSignals, worker_loop


def find_usbtty(prefix = 'tty.usbmodem'):

    npref = len(prefix)
    good = []
    
    for x in os.listdir('/dev'):
        if prefix == x[0:npref]:
            good.append(os.path.join('/dev',x))

    ngood = len(good)

    if ngood == 0:
        print(f"""Couldn't find a serial device with prefix '{prefix}'""")
        return None
    if ngood == 1:
        print(f"""Found device {good[0]}""")
        return good[0]

    print(f"""Found multiple devices with prefix '{prefix}' - please select one""")
    for i,d in enumerate(good):
        print(f"""    ({i}) {d}""")
    i = int(input())
    if i < 0 or ngood <= i:
        print("Invalid selection")
        return None
    return good[i]



class MachineConnection:

    def __init__(self, serial_path):
        self.serial_path = serial_path
        self.signals = WorkerSignals()
        self.worker = None
        
    def __enter__(self):
        self.worker = threading.Thread(target = worker_loop, args = (self.serial_path, self.signals))
        self.worker.start()
        self.signals.initialized.wait()
        self.busy = self.signals.busy
        self.idle = self.signals.idle
        return self

    def __exit__(self, type, value, tb):
        self.signals.die.set()
        self.worker.join()

    def realtime_message(self, message):
        self.signals.immediate.put(message)

    def buffered_messages(self, messages, done = True, start = True):
        # Directly forward this call to the other thread...
        self.busy.clear()
        self.signals.buffered.put((messages, start, done))

    def wait_until_idle(self):
        self.busy.wait()
        self.busy.clear()
        self.idle.wait()
        
    def status(self):
        self.signals.status_lock.acquire()
        stat = self.signals.status
        self.signals.status_lock.release()
        return stat
