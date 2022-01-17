from os.path import join
from itertools import islice
import time
import threading
import queue
import serial
from pewpew.definitions import MessageType, StatusFlag
from pewpew.parser import ProtocolParser
import pewpew.definitions as defs



def queue_taker(q):
    old = None
    start = False
    end = False
    def f(n):
        nonlocal old
        nonlocal start
        nonlocal end
        i,chunks = 0,[]
        while n > 0:
            if old is None:
                try:
                    old,new_start,new_end = q.get(False)
                    end |= new_end
                    start |= new_start
                except queue.Empty:
                    old_end = end
                    end = False
                    return i,chunks, start, old_end
            m = len(old)
            if m > n:
                chunks.append(old[0:n])
                old = old[n:]
                i += n
                n = 0
            else:
                n -= m
                i += m
                chunks.append(old)
                old = None
        old_start = start
        start = False
        return i,chunks, old_start, (end and old is None)
    return f



class WorkerSignals:
    def __init__(self):
        # Has the worker thread completed the handshake?
        self.initialized = threading.Event()
        # Is the other thread trying to kill the worker thread?
        self.die = threading.Event()
        # A work queue for messages that should be sent immediately, regardless of buffer state
        self.immediate = queue.Queue()
        # Events that need the buffer managment protocol
        self.buffered = queue.Queue()

        self.status_lock = threading.Lock()
        self.status = None
        self.peripheral = None
        

        self.busy = threading.Event()
        self.idle = threading.Event()
    
        
def worker_loop(port_path, signals):

    ser = serial.Serial(port_path, timeout = 1.0)
    parser = ProtocolParser.connect_to_port(ser)
    signals.initialized.set()

    taker = queue_taker(signals.buffered)
    parser.request_status()

    while True:
        if signals.die.is_set():
            ser.close()
            return

        while not signals.immediate.empty():
            parser.send_messages([signals.immediate.get()])


        if not parser.has_valid_request():
            parser.request_status()

        for message in parser.poll():
            can_send = None # If we had moves, how many could we send?
            
            if isinstance(message, defs.Status):
                flag = message.status_flag
                # Has anything happened between requesting this status and receiving it that would
                # invalidate the buffer size count?
                if message.request_counter == parser.status_number:
                    if flag == StatusFlag.BUSY or flag == StatusFlag.IDLE or flag == StatusFlag.HALT:
                        if message.free_space == 0:
                            parser.invalidate_request()
                            time.sleep(0.05)
                        else:
                            can_send = message.free_space
                    elif flag == StatusFlag.HOMING:
                        parser.invalidate_request()
                        time.sleep(0.025)

                # Either way, the rest of the info is still good - we just might not be able
                # to use it for buffer management
                signals.status_lock.acquire()
                
                signals.status = message
                if flag == StatusFlag.BUSY or flag == StatusFlag.HOMING:
                    signals.busy.set()
                    signals.idle.clear()
                else:
                    signals.idle.set()
                    
                signals.status_lock.release()
                
            elif isinstance(message, defs.BufferMessage):
                if message.request_counter == parser.buffer_number:
                    if message.spaces == 0:
                        parser.invalidate_request()
                    else:
                        can_send = message.spaces
            elif isinstance(message,defs.PeripheralStatus):
                signals.status_lock.acquire()
                signals.peripheral = message
                signals.status_lock.release()
            else:
                print(message)

            if can_send:
                n, chunks, start, done = taker(can_send)
                if n == 0:
                    parser.invalidate_request()
                    time.sleep(0.05)
                else:
                    parser.send_segments(n, chunks, start = start, done = done)
