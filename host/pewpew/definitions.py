from enum import Enum, auto
import struct
from collections import namedtuple
from os.path import join
from itertools import islice
import time

from pewpew.structmagic import CParam, TableEntry
import numpy as np
from dataclasses import dataclass

# At the moment, our structs only have one size parameter
NUM_AXIS = CParam("NUM_AXIS")

class MessageType(Enum):
    # Host sends an INQUIRE message, and waits for a DESCRIBE message. This may contain sundry data,
    # but most critically, it contains the number of axes the system is compiled for, which is used to
    # set the NUM_AXIS parameter and determine all struct sizes.
    INQUIRE = auto()
    DESCRIBE = auto()

    # Normal system status call/response - the client may send status messages at arbitrary times, however
    ASK = auto()      # Host requests a status update asap
    STATUS = auto()   # Client sends a full status update
    
    # Control flow for keeping the buffer full, but not too full.
    # When a BUFFER message is sent from the client to the host, it contains the number of currently free buffer slots
    # When a BUFFER message is sent from the host to the client, it tells client to expect a given number of segments, and don't bother sending buffer size updates.
    BUFFER = auto()
    DONE = auto()   # Host tells client that the no more moves will be sent, so running to the end isn't an underflow error
    
    # And the actual messages we care about - motion segments, immediate segments, and homing moves
    SEGMENT = auto()  # Host sends a segment to go into the execution buffer
    IMMEDIATE = auto() # Host sends a segment that gets executed instantly, and isn't buffered
    HOME = auto()     # Host starts a homing cycle

    # Start executing moves from the execution buffer
    START = auto()
    # Realtime feedrate override - also the mechanism for implementing pauses.
    OVERRIDE = auto()
    # and a super generic error message that just signals that things are fatally broken, and
    # any debug info should be sought elsewhere
    ERROR = auto()

    @staticmethod
    def to_enum(obj):
        try:
            return MessageType(int(obj))
        except:
            return None


# INQUIRE is a single byte
# DONE is a single byte
# START is a single byte
# ERROR is a single byte

@dataclass
class SystemDescription:
    tag = MessageType.DESCRIBE
    
    version: np.uint32  # Current protocol version
    axis_count: np.uint32 # The number of axes in the system - used to set NUM_AXIS for other messages
    magic: np.uint32 # A random magic number. Intended for ID'ing particular machines?
    buffer_size: np.uint32  # How many slots are there in the motion/event buffer

@dataclass
class Ask:
    tag = MessageType.ASK
    request_counter: np.uint32
    
class StatusFlag(Enum):
    IDLE = auto() # Nothing in the execution buffer, not doing anything.
    BUSY = auto() # Currently executing from the buffer
    HALT = auto() # A real-time override set the speed to 0; save position but require an entirely new buffer.
    HOMING = auto() # Waiting on a homing cycle to finish
    DEAD = auto() # Some sort of fatal error happened.
    BUFFER_UNDERFLOW = auto() # Ran out of moves without getting a DONE message

@dataclass
class Status:
    tag = MessageType.STATUS

    request_counter: np.uint32
    status_flag: StatusFlag
    free_space: np.uint32
    move_number: np.uint32
    override: float
    position: (np.int32, NUM_AXIS)
    
@dataclass    
class BufferMessage:
    tag = MessageType.BUFFER
    
    request_counter: np.uint32
    spaces: np.uint32

@dataclass
class Segment:
    tag = MessageType.SEGMENT
    
    move_id : np.uint32
    move_flag : np.uint32
    start_velocity : float
    end_velocity : float
    coords: (float, NUM_AXIS)

@dataclass
class SpecialEvent:
    tag = MessageType.SEGMENT
    
    move_id : np.uint32
    move_flag : np.uint32
    coords: (float, 2 + NUM_AXIS)

@dataclass
class Immediate:
    tag = MessageType.SEGMENT
    
    move_id : np.uint32
    move_flag : np.uint32
    coords: (float, 2 + NUM_AXIS)

class HomingCyclePhase(Enum):
    APPROACH = auto()
    BACKOFF = auto()
    DONE  = auto()

@dataclass
class HomingMessage:
    tag = MessageType.HOME

    axis_bitmask: np.uint32
    phase: HomingCyclePhase
    speed: float

@dataclass
class OverrideMessage:
    tag = MessageType.OVERRIDE
    
    override : float
    override_velocity : float
    
def initial_structs():
    """ Structs for the messages with fixed sizes - we can always parse these,
    even before the INQUIRE/DESCRIBE handshake is done. """

    encode, decode = {},{}

    for cls in [SystemDescription, Ask, BufferMessage, HomingMessage, OverrideMessage]:
        entry = TableEntry.make_entry(cls, {})
        encode[cls] = entry
        decode[cls.tag] = entry

    return encode, decode


def variable_structs(d,axes):
    """ Once we've completed the INQUIRE/DESCRIBE handshake, we can parse/pack everything. Build
    the rest of the structs, with the right axis size """
    
    env = {"NUM_AXIS" : axes}
    encode, decode = d
    
    for cls in [SpecialEvent,Status, Segment, Immediate]:
        entry = TableEntry.make_entry(cls, env)
        encode[cls] = entry
        decode[cls.tag] = entry

    return d
