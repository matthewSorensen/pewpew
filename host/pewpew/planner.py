import numpy as np
import math
from collections import namedtuple
from pewpew.definitions import Segment
from dataclasses import dataclass

@dataclass
class FirstOrder:
    """ Represent a velocity profile with constant acceleration - stores
    initial velocity (v0), final velocity (v), acceleration (a), time duration (t),
    and length (x). Any three of the five are needed to uniquely determine a segment,
    so the relations v = v0 + a * t and x = t * (v0 + v) / 2 must be satisfied. """

    __slots__ = ('v0','v','a','t','x')

    v0: float
    v:  float
    a:  float
    t:  float
    x:  float

    
    @classmethod
    def normalize(_, v0 = None, v = None, a = None, t = None, x = None):
        """ Convert exactly three specified values to a full tuple. This is a 
        mess, but should be an interesting way to put all the calculations in one place."""
        # We don't want to normalize overspecified tuples
        if sum(x is None for x in [v0,v,a,t,x]) != 2:
            return None
        # There are only 10 cases! So we more or less directly go through them
        if v0 is None:
            if v is None:
                v0 = x / t - a * t / 2
                return FirstOrder(v0, v0 + a * t, a, t, x)
            elif a is None:
                v0 = 2 * x / t - v
                return FirstOrder(v0, v, (v - v0) / t, t, x)
            elif t is None:
                v0 = math.sqrt(v**2 - 2 * a * x)
                return FirstOrder(v0, v, a, (v - v0) / a, x)
            elif x is None:
                v0 = v - a * t
                return FirstOrder(v0, v, a, t, t * (v0 + v)/2)
        if v is None:
            # We know v0 is not none...
            if a is None:
                v = 2 * x / t - v0
                return FirstOrder(v0, v, (v - v0) / t, t, x)
            elif t is None:
                v = math.sqrt(v0**2 + 2 * a * x)
                return FirstOrder(v0, v, a, (v - v0) / a, x)
            else: # x is none
                v = v0 + a * t
                return FirstOrder(v0, v, a, t, t * (v0 + v)/2)
        if a is None:
            # v0, v are not none
            if t is None:
                t = 2 * x / (v0 + v)
                return FirstOrder(v0, v, (v - v0) / t, t, x)
            elif x is None:
                a = (v - v0) / t
                return FirstOrder(v0, v, a, t, t * (v0 + v)/ 2)
        t = (v - v0) / a
        return FirstOrder(v0, v, a, t, t * (v0 + v)/ 2)
    
    def reverse(self):
        return FirstOrder(self.v, self.v0, 0 - self.a, self.t, self.x)

    def valid(self,epsilon = 1e-20):
        a = abs(self.v0 + self.a * self.t - self.v) < epsilon
        return a and abs(self.t * (self.v + self.v0) - 2 * self.x) < epsilon

    @classmethod
    def minimum_speed(_,v0, amax, x):
        # If we start at v0, and decelerate at amax, can we
        # come to a complete stop before we travel x distance (returning 0),
        # or how fast are we going when we hit the end? 
        if 0.5 * v0**2 / amax <= x:
            return 0.0
        return math.sqrt(v0**2 - 2 * amax * x)
    
@dataclass
class KinematicLimits:
    v_max: float
    a_max: float
    junction_speed: float
    junction_deviation: float
    
def limit_vector(v, l):
    """ Returns a, such that f v such that |a v'| <= l component wise, and ||a v'|| is maximal. """
    return 1 / np.max(np.abs(v) / l)

@dataclass
class LineSegment:
    __slots__ = ('parent','start','end','unit','profile','amax')

    parent: int # Numerical tag to associate this line segment with higher-level data - preserved on subdivision
    start: np.ndarray # Segment starting point...
    end: np.ndarray #...ending point...
    unit: np.ndarray #...and the unit vector pointing from start to end.
    profile: FirstOrder # A velocity profile for the line segment
    amax: float # 
    
    @staticmethod
    def from_geo(parent, v0, v1, start, end,kl):
        v0, v1 = abs(v0), abs(v1)
        delta = end - start
        length = math.sqrt(delta.dot(delta))
        profile = FirstOrder.normalize(v0 = v0, v = v1, x = length)
        unit = delta  / length
        return LineSegment(parent, start, end, unit, profile, limit_vector(unit, kl.a_max))
    
class OtherEvent(namedtuple("OtherEvent",["v","stuff"])):
    __slots__ = ()

def limit_value_by_axis(limit, vector):
    limit_value = 1e19
    for i,x in enumerate(vector):
        if x != 0:
            limit_value = min(limit_value, abs(limit[i] / x))
            
    return limit_value

def compute_junction_velocity(p, s, limits):
    """ This is more or less a direct implementation of the grbl version"""
    junction_cos = -1 * s.unit.dot(p.unit)
   
    if junction_cos > 0.9999:
        # Extremely sharp corner - segments are directly opposite
        return limits.junction_speed
    elif junction_cos < -0.9999:
        # Extremely shallow corner - go as fast as possible
        return None
    else:
        junction_vect = s.unit - p.unit
        junction_vect /= math.sqrt(junction_vect.dot(junction_vect))
 
        junction_acceleration = limit_value_by_axis(limits.a_max, junction_vect)
        sin_theta_d2 = math.sqrt(0.5*(1.0-junction_cos))
        junction_velocity = (junction_acceleration * limits.junction_deviation * sin_theta_d2)/(1.0-sin_theta_d2)   
        return max(limits.junction_speed,junction_velocity)



def plan_segment(s, v, reverse = False):
    a = s.amax
    p = s.profile if not reverse else s.profile.reverse()
    
    if p.v0 <= v:
        yield s
        return
    
    da = a - p.a
    dv = p.v0 - v
    
    if da <= 0 or p.t * da <= dv:
        if reverse:
            p = FirstOrder.normalize(v = v, a = -1 * a, x = p.x)
        else:
            p = FirstOrder.normalize(v0 = v, a = a, x = s.profile.x)
        yield LineSegment(s.parent, s.start, s.end, s.unit, p, s.amax)
        return

    first_profile = FirstOrder.normalize(v0 = v, a = a, t = dv / da)
    second_profile = FirstOrder.normalize(v0 = first_profile.v, v = p.v, x = p.x - first_profile.x)
    
    if reverse:
        first_profile, second_profile = FirstOrder.reverse(second_profile), FirstOrder.reverse(first_profile)
        
    crossing = s.start + s.unit * first_profile.x
    
    yield LineSegment(s.parent, s.start, crossing, s.unit, first_profile, s.amax)
    yield LineSegment(s.parent, crossing, s.end, s.unit, second_profile, s.amax)


def forward_pass(segments, v0, limits):
    prev = None
    for s in segments:
        if isinstance(s, OtherEvent):
            v0 = min(v0, s.v)
            yield s
            continue
            
        p = s.profile
        # How fast can we actually travel along this segment, and at what acceleration?
        v = limit_vector(s.unit, limits.v_max)
        # How fast must we start out the move?
        if prev is not None:
            jv = compute_junction_velocity(prev,s,limits)
            if jv is not None:
                v0 = min(v0, jv)
        # First, clamp the velocity and acceleration of the segment to lie
        # within the limits
        changed = False
        if p.v0 > v or p.v > v:
            p = FirstOrder.normalize(v0 = min(p.v0,v), v = min(p.v,v), x = p.x)
            changed = True
        if abs(p.a) > s.amax:
            p = FirstOrder.normalize(v0 = p.v0, a = s.amax * p.a / abs(p.a), x = p.x)  
            changed = True
            
        if changed:
            s = LineSegment(s.parent, s.start, s.end, s.unit, p, s.amax)
        
        for sub in plan_segment(s, v0):
            v0 = sub.profile.v
            yield sub
        prev = s

def backward_pass(segments, v = 0):
    n = len(segments)
    out = [None] * len(segments)
    
    for i in range(n):
        i = n - i - 1
        s = segments[i]
        if isinstance(s, OtherEvent):
            out[i] = [s]
            v = min(v, s.v)
        else:
            out[i] = list(plan_segment(segments[i], v, reverse = True))
            v = out[i][0].profile.v0
        
    
    return out



def plan_segments(segs, kl, v0 = 0.0, v1 = 0.0):
    
    prev_chunk = None
    chunk, v_start, v_end = [], v0,v0
  
    for s in forward_pass(segs, v0, kl):
        if isinstance(s, OtherEvent):
            chunk.append(s)
            continue
        v_end = FirstOrder.minimum_speed(v_end, s.amax, s.profile.x)
        if v_end == 0.0:
            if prev_chunk:
                for x in backward_pass(prev_chunk, v_start):
                    yield from x
            prev_chunk = chunk
            chunk = [s]
            v_start = s.profile.v0
            v_end = FirstOrder.minimum_speed(v_start, s.amax, s.profile.x)
        else:
            chunk.append(s)
    
    if prev_chunk:
        for x in backward_pass(prev_chunk, v_start):
            yield from x
            
    if chunk:
        for x in backward_pass(chunk, v1):
            yield from x



class MotionPlanner:
    
    def __init__(self, limits, microsteps, position):

        self.kl = limits
        self.microsteps = microsteps
        self.position = position

    def set_position(self,p, microsteps = None):
        if microsteps is not None:
            p = list(x / self.microsteps[i] for i,x in enumerate(p))
            p = np.array(p)
            
        self.position = p

    def plan_moves(self, moves, v = None):
        if v is None:
            vmax = self.kl.v_max
            v = math.sqrt(len(vmax)) * max(vmax)

        segs = []
        prev = self.position
        for i,m, in enumerate(moves):
            delta = m - prev
            if delta.dot(delta) == 0.0:
                continue
            
            segs.append(LineSegment.from_geo(i, v,v, prev, m, self.kl))
            prev = m
        self.position = prev

        for s in plan_segments(segs, self.kl):
            v_scale = np.linalg.norm(s.unit * self.microsteps) * 1e-6
            profile = s.profile
            yield Segment(s.parent, 0, profile.v0 * v_scale, profile.v * v_scale, tuple(s.end * self.microsteps))


    def plan_segments(self, segments, offset = None, adjust_velocity = False):
        vmax = self.kl.v_max
        v = math.sqrt(len(vmax)) * max(vmax)

        segs = []
        prev = self.position
        for s in segments:
            m = np.array(s.coords) + offset
            delta = m - prev
            if delta.dot(delta) == 0.0:
                continue
            
            v0 = v if s.start_velocity <= 0 else s.start_velocity
            v1 = v if s.end_velocity <= 0 else s.end_velocity
            
            segs.append(LineSegment.from_geo(s.move_id,v0,v1, prev, m, self.kl))
            prev = m

        self.position = prev

        if not segs:
            return

        for s in plan_segments(segs, self.kl):
            v_scale = np.linalg.norm(s.unit * self.microsteps) * 1e-6
            profile = s.profile
            yield Segment(s.parent, 0, profile.v0 * v_scale, profile.v * v_scale, tuple(s.end * self.microsteps))


            
    def goto(self, *args):
        pos = np.array(args)
        return list(self.plan_moves([pos], v = None))
        
