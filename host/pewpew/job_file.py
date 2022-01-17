from dataclasses import dataclass
from pewpew.definitions import SpecialEvent, Segment
from pewpew.laser_events import LaserEvent, GalvoLocation, GalvoLine, GalvoCircle
from pewpew.planner import MotionPlanner
import pickle

@dataclass
class LaserFileHeader:

    job_name : str # What should we report when running this?
    repeat : int # How many times do we repeat this file?
    preview : bool # Is this file just a preview?

def write_file(filepath, events, name = '', repeat = 1, preview = False):
    with open(filepath, 'wb') as f:
        pickle.dump((LaserFileHeader(name, repeat, preview),events),f)
    
def load_file(filepath, planner):
    # Make a copy of the planner, so we don't mess with its position
    planner = MotionPlanner(planner.kl, planner.microsteps, planner.position)
    start = planner.position
    # Load the events
    with open(filepath,'rb') as f:
        header, events = pickle.load(f)

    move_chunk = []
    out = []
    
    for i,e in enumerate(events):
        e.move_id = i

        if isinstance(e,Segment) and e.move_flag == 0:
            move_chunk.append(e)
        else:
            if move_chunk:
                for x in planner.plan_segments(move_chunk, offset = start, adjust_velocity = True):
                    out.append(x)
                move_chunk = []
            out.append(e)
        
    if move_chunk:
        for x in planner.plan_segments(move_chunk, offset = start, adjust_velocity = True):
            out.append(x)

    for x in planner.goto(*start):
        out.append(x)
            
    return header, out
