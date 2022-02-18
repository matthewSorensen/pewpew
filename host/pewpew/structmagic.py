""" 

D(on't) R(epeat) Y(ourself) is a primary virtue for coding style, right? Minor
details like "don't abuse type annotations" and "avoid behind-the-scenes metaprogramming"
are always worth sacrificing, right? 

"""

import dataclasses
from typing import Any
import numpy as np
import struct
from typing import List,Tuple, Any, Type, Union
from enum import Enum, auto

@dataclasses.dataclass
class CParam:
    name: Any = None
    expr: Any = None

    @staticmethod
    def valid(obj):
        if obj is None:
            return None
        elif isinstance(obj,int):
            if obj < 0:
                raise ValueError
            return obj
        elif isinstance(obj,CParam):
            if obj.name is None and obj.expr is None:
                return 0
            return obj
        raise ValueError

    @staticmethod
    def add(a,b):
        if a == 0:
            return b
        if b == 0:
            return a
        return CParam(None,('+',a,b))
    
    @staticmethod 
    def mul(a,b):
        if a == 0 or b == 0:
            return 0
        if a == 1:
            return b
        if b == 1:
            return a
        return CParam(None,('*',a,b))
    
    def __add__(self, other):
        return CParam.add(CParam.valid(self),CParam.valid(other))
    
    def __radd__(self, other):
        return CParam.add(CParam.valid(self),CParam.valid(other))

    def __mul__(self, other):
        return CParam.mul(CParam.valid(self),CParam.valid(other))
    
    def __rmul__(self, other):
        return CParam.mul(CParam.valid(self),CParam.valid(other))

    @staticmethod
    def eval(expr, env):
        if isinstance(expr, int):
            return expr
        elif isinstance(expr, CParam):
            if expr.name is not None:
                if expr.name in env:
                    return env[expr.name]
                raise KeyError
            else:
                op,l,r = expr.expr
                if op == '+':
                    return CParam.eval(l,env) + CParam.eval(r,env)
                elif op == '*':
                    return CParam.eval(l,env) * CParam.eval(r,env)             
        raise ValueError

    @staticmethod
    def cexpr(expr):
        if isinstance(expr, int):
            return str(expr), False
        elif dataclasses.is_dataclass(expr):
            if expr.name is not None:
                return expr.name, False
            else:
                op,l,r = expr.expr
                l,lp = CParam.cexpr(l)
                r,rp = CParam.cexpr(r)

                if op == '+':
                    return l + '+' + r, True
                elif op == '*':
                    if lp:
                        l = '(' + l + ')'
                    if rp:
                        r = '(' + r + ')'
                    return l + '*' + r, False
               
        raise ValueError

    @staticmethod
    def expanded(expr):
        """ Expand an expression into a sum of monomials, represented as a dict """
        if isinstance(expr, int):
            return {() : expr}
        elif dataclasses.is_dataclass(expr):
            if expr.name is not None:
                return {((expr.name,1),) : 1}
            else:
                op,l,r = expr.expr
                l = CParam.expanded(l)
                r = CParam.expanded(r)
                if op == '+':
                    for k,v in r.items():
                        if k in l:
                            l[k] += v
                        else:
                            l[k] = v
                    return l
                elif op == '*':
                    ret = {}
                    for k,v in l.items():
                        for l,w in r.items():
                            key = None
                            if k == ():
                                key = l
                            elif l == ():
                                key = k
                            else:
                                # l is a list of ('variable', power) pairs,
                                # as is r - all we need to do is add like terms, and then sort cannonically
                                print(k,l)
                                orders = {}
                                for var,power in k:
                                    if var in orders:
                                        orders[var] += power
                                    else:
                                        orders[var] = power
                                for var,power in l:
                                    if var in orders:
                                        orders[var] += power
                                    else:
                                        orders[var] = power
                                key = tuple(sorted(orders.items(), key = lambda x: x[0]))
                            ret[key] = v * w
                    return ret
        raise ValueError
    
    @staticmethod
    def compare_expanded(s,p):

        ks,kp = set(s.keys()), set(p.keys())
        common = ks & kp

        p_bigger = any(s[k] < p[k] for k in common)
        s_bigger = any(p[k] < s[k] for k in common)

        # If we can't order the shared terms, don't try to order the rest of it
        if p_bigger and s_bigger:
            return None
        # Otherwise, they are exactly equal on a (possible empty) set of common terms
        if not (p_bigger or s_bigger):
            if ks.issubset(kp):
                return False, True
            if kp.issubset(ks):
                return True, False
            return None

        if p_bigger and ks.issubset(kp):
            return False, True
        
        if s_bigger and kp.issubset(ks):
            return True, False
            
        return None
        
    
type_to_struct = {}
type_to_struct[np.uint8] = 'c', 'uint8_t', 1
type_to_struct[np.uint32] = 'L', 'uint32_t', 4
type_to_struct[np.int32] = 'l', 'int32_t', 4
type_to_struct[np.uint64] = 'Q', 'uint64_t', 8
type_to_struct[np.int64] = 'q', 'int64_t', 8
type_to_struct[float] = 'd', 'double', 8

def resolve_type_hint(t, env = None):

    if isinstance(t, tuple):
        ftype,_,en = resolve_type_hint(t[0], None)
        size = t[1] if env is None else CParam.eval(t[1],env)
        return ftype, size, en

    if issubclass(t,Enum):
        stype,ctype,size = type_to_struct[np.uint32]
        if 'cname' in t.__dict__:
            ctype = t.cname()
        
        return (stype,ctype,size), None, t
    
    if t in type_to_struct:
        return type_to_struct[t], None, False

    raise ValueError


def make_c_struct(cls,cname):
    defs = []
    const_size = 0
    var_size = 0
    for f in dataclasses.fields(cls):
        (_,ctype,csize), fsize, _ = resolve_type_hint(f.type)
        
        if fsize is None:
            defs.append(f"    {ctype} {f.name};")
            const_size += csize
        elif isinstance(fsize, int):
            defs.append(f"    {ctype} {f.name}[{fsize}];")
            const_size += csize * fsize
        else:
            defs.append(f"    {ctype} {f.name}[{CParam.cexpr(fsize)[0]}];")
            var_size = var_size + csize * fsize

    defs = "{\n" + '\n'.join(defs) + "\n}"
    return f"typedef struct {cname}{defs} {cname};", CParam.cexpr(const_size + var_size)[0]


@dataclasses.dataclass
class TableEntry:
    
    obj_class : Type
    struct : struct.Struct
    flatten: List[bool]
    fields : List[Union[int,List[int]]]
    enums: List[Any]
    size : int

    @staticmethod
    def make_entry(cls, env):
        variadic = []
        s = ['<']
        i = 0
        fields = []
        enums = []
    
        for f in dataclasses.fields(cls):
            (stype,_,_), fsize,en = resolve_type_hint(f.type, env)
            if fsize is None:
                s.append(stype)
                fields.append(i)
                i += 1
                variadic.append(False)
            else:
                 s.append(f"{fsize}{stype}")
                 fields.append(list(i + j for j in range(fsize)))
                 i += fsize
                 variadic.append(True)
                 
            enums.append(en)
                 
        if not (any(variadic) or any(enums)):
            fields = None

        s = struct.Struct(''.join(s))
        # Do we add padding bytes at the end of the struct?
        if 'pad' in cls.__dict__:
            pad = cls.pad
            if not isinstance(pad, int):
                pad = TableEntry.make_entry(pad, env).size
            
            if s.size > pad:
                print("Negative padding size for", cls)
                raise ValueError
            pad -= s.size
            s = struct.Struct(s.format + f"{pad}x")
            
        
        return TableEntry(cls,s,variadic,fields,enums,s.size)

    def encode(self, obj):
        args = []
        for i,x in enumerate(dataclasses.astuple(obj)):
            e = self.enums[i]
            if self.flatten[i]:
                for j in x:
                    args.append(j.value if e else j)
            else:
                args.append(x.value if e else x)
                
        return self.struct, args

    def decode(self, bytestream):
        raw = self.struct.unpack(bytestream)

        if self.fields is None:
            return self.obj_class(*raw)
        
        packed = []
        for i,j in enumerate(self.fields):
            e = self.enums[i]
            
            if self.flatten[i]:
                packed.append(tuple((e(raw[x]) if e else raw[x]) for x in j))
            else:
                packed.append(e(raw[j]) if e else raw[j])

        return self.obj_class(*packed)


    
