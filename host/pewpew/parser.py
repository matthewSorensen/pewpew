from itertools import islice
import serial
from pewpew.definitions import MessageType, StatusFlag, initial_structs, variable_structs
from pewpew.definitions import SystemDescription, BufferMessage, Ask
from pewpew.structmagic import TableEntry

def protocol_handshake(serial_port):
    """ Make contact with a device - send an INQUIRE message, and wait
    for a DESCRIBE message. Returns a fully-initialized set of structs,
    the number of axes, and the magic number. """

    structs = initial_structs()
    entry = structs[0][SystemDescription]
    
    serial_port.write(MessageType.INQUIRE.value.to_bytes(1, byteorder='little'))
    response = serial_port.read(entry.size + 1)
    
    if MessageType(response[0]) != SystemDescription.tag or len(response) < entry.size + 1:
        print("Did not receive DESCRIBE packet - instead, got ", response)
        return None

    d = entry.decode(response[1:])
    print(f"Found protocol version {d.version}, with {d.axis_count} motion axes and magic number {d.magic}. Motion buffer is {d.buffer_size} segments.")
    return d, variable_structs(structs, d.axis_count)


def sequence_number():
    """ Yields an infinite, repeating stream of integers ranging from 1 to 2^32 - 1 """
    i = 0
    biggest = 2**32
    while True:
        i = (i + 1) % biggest
        if i != 0:
            yield i

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
        self.buff = bytearray(max(x.size for _,x in structs[0].items()))
        self.desc = desc
        self.motion_buffer_size = desc.buffer_size
        self.message_type = None
        self.remaining_chars = 0
        self.head = 0
        self.error = False

        self.send_buff = bytearray(send_buff)

        self.message_sequence = sequence_number()
        self.status_number = None
        self.buffer_number = None

        self.request_status()
        

    def send_messages(self, *messages):
        buff = self.send_buff
        buff_size = len(buff)
        i = 0

        for chunk in messages:
            for message in chunk:
                size, fmt, obj, tag = 0, None, None, None
                if isinstance(message,MessageType):
                    size = 1
                    tag = message.value
                else:
                    t = type(message)
                    if t in self.structs[0]:
                        fmt, obj = self.structs[0][t].encode(message)
                    else:
                        print("parser: adding entry for", t, "dynamically")
                        entry = TableEntry.make_entry(t, {"NUM_AXIS" : self.desc.axis_count})
                        self.structs[0][t] = entry
                        fmt, obj = entry.encode(message)
                        
                    size = fmt.size + 1
                    tag = message.tag.value
                    
                if buff_size < i + size:
                    self.serial.write(buff[0:i])
                    i = 0

                buff[i] = tag
                if fmt is not None:
                    fmt.pack_into(buff, i + 1, *obj)
                i += size

        if i != 0:
            self.serial.write(buff[0:i])

    def send_segments(self, n, segments, done = False, start = False):
        # Invalidate any outstanding status requests and get a new buffer message sequence number
        self.status_number = None
        self.buffer_number = next(self.message_sequence)
        # Are we done sending moves, and should the controller start executing the moves?
        post = []
        if done:
            post.append(MessageType.DONE)
        if start:
            post.append(MessageType.START)
        # Send everything over - a BUFFER message to surpress BUFFER responses for n segments, the n segments, and the done/start messages
        segments.append(post)
        self.send_messages([BufferMessage(self.buffer_number, n)], *segments)

    def request_status(self):
        """ Invalidate all outstanding status requests and send a new one """
        self.status_number = next(self.message_sequence)
        self.send_messages([Ask(self.status_number)])

    def has_valid_request(self):
        return self.status_number is not None or self.buffer_number is not None

    def invalidate_request(self):
        self.status_number = None
        self.buffer_number = None
    
    def poll(self):

        data = self.serial.read(1024) # Or some other chunk size
        if len(data) == 0:
            return
        
        if self.error:
            yield MessageType.ERROR, data
            
        for i,b in enumerate(data):
            if self.message_type is None:

                decode = self.structs[1]
                self.message_type = MessageType.to_enum(b)
                if self.message_type is None:
                    print("Invalid message type - ", b)
                    exit()

                if self.message_type == MessageType.ERROR:
                    yield MessageType.ERROR, data[i:]
                    self.error = True
                    return
                elif self.message_type not in decode:
                    yield self.message_type
                    self.message_type = None
                else:
                    self.head = 0
                    self.remaining_chars = decode[self.message_type].size
                    
            else:
                self.buff[self.head] = b
                self.head += 1
                self.remaining_chars -= 1

            if self.message_type is not None and self.remaining_chars == 0:
                decode = self.structs[1][self.message_type]
                message = decode.decode(self.buff[0:decode.size])
                yield message
                self.message_type = None
