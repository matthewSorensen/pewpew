from enum import Enum, auto
import struct
from collections import namedtuple
from os.path import join
from itertools import islice
import time

class MessageType(Enum):
    # Host sends an INQUIRE message, and waits for a DESCRIBE message. This may contain sundry data,
    # but most critically, it contains the number of axes the system is compiled for, which determines
    # the size and layout of the STATUS, SEGMENT, and EVENT messages.
    INQUIRE = auto()
    DESCRIBE = auto()

    # Normal system status call/response - the client may send status messages at arbitrary times, however
    ASK = auto()      # Host requests a status update asap
    STATUS = auto()   # Client sends a full status update
    
    # Control flow for keeping the buffer full, but not too full.
    BUFFER = auto() # Client sends the number of free execution buffer slots
    EXPECT = auto() # Host tells client to expect a given number of segments, and don't bother sending buffer size updates
    DONE = auto()   # Host tells client that the no more moves will be sent, so running to the end isn't an underflow error
    
    # And the actual messages we care about - motion segments, immediate segments, and homing moves
    SEGMENT = auto()  # Host sends a segment to go into the execution buffer
    IMMEDIATE = auto() # Host sends a segment that gets executed instantly, and isn't buffered
    HOME = auto()     # Host starts a homing cycle

    # Start executing moves from the execution buffer
    START = auto()

    # and a super generic error message that just signals that things are fatally broken, and
    # any debug info should be sought elsewhere
    ERROR = auto()

    @staticmethod
    def to_enum(obj):
        try:
            return MessageType(int(obj))
        except:
            return None


class HomingCyclePhase(Enum):
    APPROACH = auto()
    BACKOFF = auto()
    DONE  = auto()

class StatusFlag(Enum):
    IDLE = auto() # Nothing in the execution buffer, not doing anything.
    BUSY = auto() # Currently executing from the buffer
    HOMING = auto() # Waiting on a homing cycle to finish
    DEAD = auto() # Some sort of fatal error happened.
    BUFFER_UNDERFLOW = auto() # Ran out of moves without getting a DONE message

class SystemDescription(namedtuple("SystemDescription", "version axis_count magic buffer_size")):
    __slots__ = ()
    @staticmethod
    def table_entry():
        return struct.Struct("<LLLL"), None, SystemDescription

class StatusMessage(namedtuple("StatusMessage","request_counter status_flag free_space move_number position")):
    __slots__ = ()

    @staticmethod
    def unpack(*args):    
        return StatusMessage(args[0], StatusFlag(args[1]), args[2], args[3], args[4:])
    
    @staticmethod
    def table_entry(axes):
        return struct.Struct(f"<LLLL{axes}l"), None, StatusMessage.unpack

    
class BufferMessage(namedtuple("BufferMessage", "request_counter free_space")):
    
    __slots__ = ()

    @staticmethod
    def unpack(*args):    
        return BufferMessage(*args)
    
    @staticmethod
    def table_entry():
        return struct.Struct(f"<LL"), None, BufferMessage.unpack

    
def initial_structs():
    """ Structs for the messages with fixed sizes - we can always parse these,
    even before the INQUIRE/DESCRIBE handshake is done. """
    d = {}

    # These take no parameters
    for m in [MessageType.INQUIRE, MessageType.ERROR, MessageType.START, MessageType.DONE]:
        d[m] = struct.Struct("<"), None, None

    d[MessageType.DESCRIBE] = SystemDescription.table_entry() 
    d[MessageType.BUFFER] = BufferMessage.table_entry()
    d[MessageType.ASK] = struct.Struct("<L"), None, None
    d[MessageType.EXPECT] = struct.Struct("<LL"), None, None
    # We have under 32 axes, so homing commands are also fixed format
    d[MessageType.HOME] = struct.Struct("<LLd"), None, None # axis bitmask, homing mode, max speed
    return d

def variable_structs(d,axes):
    """ Once we've completed the INQUIRE/DESCRIBE handshake, we can parse/pack everything. Build
    the rest of the structs, with the right axis size """

    d[MessageType.STATUS] = StatusMessage.table_entry(axes)
    d[MessageType.SEGMENT] = struct.Struct(f"<LL{axes + 2}d"), None, None
    d[MessageType.IMMEDIATE] = struct.Struct(f"<LL{axes + 2}d"), None, None
    return d


def protocol_handshake(serial_port):
    """ Make contact with a device - send an INQUIRE message, and wait
    for a DESCRIBE message. Returns a fully-initialized set of structs,
    the number of axes, and the magic number. """
    
    structs = initial_structs()
    desc, _, dser = structs[MessageType.DESCRIBE]
    serial_port.write(MessageType.INQUIRE.value.to_bytes(1, byteorder='little'))
    response = serial_port.read(desc.size + 1)
    
    if int(response[0]) != MessageType.DESCRIBE.value or len(response) < desc.size + 1:
        print("Did not receive DESCRIBE packet - instead, got ", response)
        return None

    d = dser(*desc.unpack(response[1:]))
    print(f"Found protocol version {d.version}, with {d.axis_count} motion axes and magic number {d.magic}. Motion buffer is {d.buffer_size} segments.")
    return d, variable_structs(structs, d.axis_count)

class ProtocolParser:

    PROTOCOL_VERSION = 1

    @staticmethod
    def connect_to_port(serial):
        # Change the timeout to a large value, try to complete the handshake, and
        # change the timeout back to normal mode.
        serial.timeout = 1
        handshake = protocol_handshake(serial)

        if handshake is None or handshake[0].version != ProtocolParser.PROTOCOL_VERSION:
            print("Handshake failed, or protocol version is incompatible")
            return None
        
        serial.timeout = 0
        desc, structs = handshake

        return ProtocolParser(serial, desc, structs)

    def __init__(self,serial, desc, structs, send_buff = 1024):

        self.serial = serial
        self.structs = structs
        self.buff = bytearray(max(0 if x is None else x.size for _,(x,_,_) in structs.items()))
        self.desc = desc
        self.motion_buffer_size = desc.buffer_size
        self.message_type = None
        self.remaining_chars = 0
        self.head = 0
        self.error = False

        self.send_buff = bytearray(send_buff)

    def send_messages(self, messages):
        buff = self.send_buff
        buff_size = len(buff)
        i = 0

        for t, m in messages:
            s,f,_ = self.structs[t]
            n = 1 + s.size

            if buff_size < i + n:
                self.serial.write(buff[0:i])
                i = 0
            
            buff[i] = t.value
            if f:
                s.pack_into(buff, i+1, *f(*m))
            else:
                s.pack_into(buff, i+1, *m)
            i += n

        if i != 0:
            self.serial.write(buff[0:i])

    
    def poll(self):

        data = self.serial.read(1024) # Or some other chunk size
        if len(data) == 0:
            return
        
        if self.error:
            yield MessageType.ERROR, data
            
        for i,b in enumerate(data):
            if self.message_type is None:

                self.message_type = MessageType.to_enum(b)
                if self.message_type is None:
                    print("Invalid message type - ", b)
                    exit()

                self.head = 0
                struct,_,_ = self.structs[self.message_type]
                self.remaining_chars = struct.size

                if self.message_type == MessageType.ERROR:
                    yield MessageType.ERROR, data[i:]
                    self.error = True
                    return
            else:
                self.buff[self.head] = b
                self.head += 1
                self.remaining_chars -= 1

            if self.remaining_chars == 0:
                struct,_,f = self.structs[self.message_type]
                if f:
                    yield self.message_type, f(*struct.unpack(self.buff[0:struct.size]))
                else:
                    yield self.message_type, struct.unpack(self.buff[0:struct.size])
    
                self.message_type = None
def sequence_number():
    """ Yields an infinite, repeating stream of integers ranging from 1 to 2^32 - 1 """
    i = 0
    biggest = 2**32
    while True:
        i = (i + 1) % biggest
        if i != 0:
            yield i

class Rechunker:
    def __init__(self, iterator):
        try:
            self.next = next(iterator)
            self.iter = iterator
            self.done = False
        except StopIteration:
            self.done = True
            
    def take(self, n):
        if self.done:
            return
        if n > 0:
            yield self.next
            n -= 1
        if n > 0:
            yield from islice(self.iter, n)
        try:
            self.next = next(self.iter)
            self.done = False
        except StopIteration:
            self.done = True
            

def execute_segments(parser, segments):

    buffer_gen, status_gen = sequence_number(), sequence_number()    
    status_request_number, buffer_request_number = None, None
    segments = Rechunker(segments)
    # Prime the buffer...
    start = list(segments.take(parser.motion_buffer_size))
    buffer_request_number = next(buffer_gen)
    done = [(MessageType.DONE,())] if segments.done else []
    
    parser.send_messages([(MessageType.EXPECT,(buffer_request_number,len(start)))] + start + done + [(MessageType.START,())])
    

    done = False
    while not done:
        if not (status_request_number or buffer_request_number):
            status_request_number = next(buffer_gen)
            parser.send_messages([(MessageType.ASK,(status_request_number,))])

            
        for message, blob in parser.poll():

            can_send = None
            
            if message == MessageType.STATUS:
                yield MessageType.STATUS,blob
                if blob.request_counter == status_request_number:
                    if blob.status_flag == StatusFlag.BUSY:
                        if blob.free_space == 0 or segments.done:
                            status_request_number = None
                            time.sleep(0.05)
                        else:
                            can_send = blob.free_space
                    else:
                        done = True
            elif message == MessageType.BUFFER:
                if blob.request_counter == buffer_request_number:
                    if blob.free_space == 0 or segments.done:
                        buffer_request_number = None
                        status_request_number = None
                    else:
                        can_send = blob.free_space
            else:
                yield message, blob

            if can_send:
                chunk = list(segments.take(can_send))    
                buffer_request_number = next(buffer_gen)
                status_request_number = None
                buffer_done = [(MessageType.DONE,())] if segments.done else []
                parser.send_messages([(MessageType.EXPECT,(buffer_request_number, len(chunk)))] +  chunk + buffer_done)
                


def enum_to_cdecl(enum_class, prefix, type_name):
    fields = ",\n".join([f"""    {prefix}{x.name} = {x.value}""" for x in enum_class])
    return f"typedef enum {type_name} {{\n{fields}\n}} {type_name};"


def generate_c_interface(mod_name, directory_path):

    d0 = variable_structs(initial_structs(), 0)
    d1 = variable_structs(initial_structs(), 1)   

    sizes = []
    max_fixed, max_delta, max_message = 0,0,0
    for m in MessageType:
        if d0[m] is None:
            sizes.append('0')
        else:
            fixed = d0[m][0].size
            delta = d1[m][0].size - fixed
            if delta != 0:
                sizes.append(f"{fixed} + {delta} * NUM_AXIS")
            else:
                sizes.append(str(fixed))
            max_fixed = max(fixed, max_fixed)
            max_delta = max(delta, max_delta)
        max_message = max(m.value, max_message)

    with open(join(directory_path,f"{mod_name}.h"),"w") as f:
        
        f.write("// Auto-generated file containing enum definitions shared with python client. Do not edit directly!\n")
        f.write(f"#ifndef {mod_name}_h \n#define {mod_name}_h\n\n")
        f.write('#include <stdint.h>\n')
        f.write('#include "pin_maps.h"\n\n')
        f.write(f"#define MAX_MESSAGE {max_message}\n\n")
        f.write(enum_to_cdecl(MessageType, "MESSAGE_", "message_type_t") + "\n\n")
        f.write(enum_to_cdecl(HomingCyclePhase, "HOMING_", "homing_phase_t") + "\n\n")
        f.write(enum_to_cdecl(StatusFlag, "STATUS_", "status_flag_t") + "\n\n")
        f.write(f"extern const uint32_t message_sizes[{len(d0)}];\n")
        f.write(f"extern uint8_t message_buffer[{max_fixed} + {max_delta} * NUM_AXIS];\n")
        f.write("#endif\n")

                
    with open(join(directory_path,f"{mod_name}.cpp"),"w") as f:
        f.write("// Auto-generated file containing enum definitions shared with python client. Do not edit directly!\n")
        f.write(f"""#include "{mod_name}.h"\n""")
        f.write(f"const uint32_t message_sizes[{len(d0)}] = {{{', '.join(sizes)}}};\n\n")
        f.write(f"uint8_t message_buffer[{max_fixed} + {max_delta} * NUM_AXIS];\n\n")

