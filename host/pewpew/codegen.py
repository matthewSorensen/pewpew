import structmagic
import definitions as defs
from definitions import MessageType
import dataclasses

def c_format_expanded(expanded):
    terms = []
    for term, coeff in expanded.items():
        if term == ():
            terms.append(str(coeff))
            continue
        subterms = [] if coeff == 1 else [str(coeff)]
        for y,n in term:
            for i in range(n):
                subterms.append(y)
        terms.append('*'.join(subterms))
    return '+'.join(terms)
    
def build_message_buffer_union(sizes,name):
    dominant = []
    for x in sizes.values():
        next_set = []
        include = True
        for y in dominant:
            comp = structmagic.CParam.compare_expanded(y,x)
            if comp is None:
                next_set.append(y)
            elif comp[0]:
                include = False
                next_set.append(y)
            
        if include:
            next_set.append(x)

        dominant = next_set

    fields = []
    for i,x in enumerate(dominant):
        fields.append(f"""char field{i}[{c_format_expanded(x)}];""")

    return f"""typedef union {{{' '.join(fields)}}} {name};"""


def generate_enum(fields, name, field_prefix):
    fields = ',\n'.join(f"""    {field_prefix}{x.name} = {x.value}""" for x in sorted(fields, key = lambda x: x.value))

    return f"typedef enum {name} {{\n{fields}\n}} {name};"



def generate_protocol_constant_header(stream,sizes):

    stream.write(f"""// Auto-generated file containing enum definitions shared with python client. Do not edit directly!
// Regenerate by running host/pewpew/codegen.py from the project home directory.
#ifndef protocol_constants_h 
#define protocol_constants_h

#include <stdint.h>
#include "pin_maps.h"

#define MAX_MESSAGE {max(v.value for v in sizes.keys())}""")
    stream.write("\n\n")

    stream.write(generate_enum(sizes.keys(), "message_type_t", "MESSAGE_") + '\n\n')

    stream.write("""typedef enum homing_phase_t {
    HOMING_APPROACH = 1,
    HOMING_BACKOFF = 2,
    HOMING_DONE = 3
} homing_phase_t;

typedef enum status_flag_t {
    STATUS_IDLE = 1,
    STATUS_BUSY = 2,
    STATUS_HALT = 3,
    STATUS_HOMING = 4,
    STATUS_DEAD = 5,
    STATUS_BUFFER_UNDERFLOW = 6
} status_flag_t;""")

    stream.write("\n\n")
    stream.write(build_message_buffer_union(sizes,"message_buffer_size")+'\n\n')

    stream.write(f"""#define MESSAGE_BUFFER_SIZE sizeof(message_buffer_size)

extern const uint32_t message_sizes[{len(sizes.values())}];
extern uint8_t message_buffer[MESSAGE_BUFFER_SIZE];
#endif""")
    stream.write("\n\n")


def generate_protocol_constant_cpp(stream, sizes):
    size_table = ', '.join(c_format_expanded(x) for _,x in sorted(sizes.items(), key = lambda x: x[0].value))

    stream.write(f"""// Auto-generated file containing enum definitions shared with python client. Do not edit directly!
// Regenerate by running host/pewpew/codegen.py from the project home directory.
#include "protocol_constants.h"
const uint32_t message_sizes[{len(sizes)}] = {{{size_table}}};

uint8_t message_buffer[MESSAGE_BUFFER_SIZE];""" + '\n')


if __name__ == "__main__":

    table = {}
    sizes = {}
    for x in MessageType:
        table[x] = None
        sizes[x] = structmagic.CParam.expanded(0)

    for x in [defs.SpecialEvent, defs.Status, defs.Segment, defs.Immediate, defs.PeripheralStatus,
              defs.SystemDescription, defs.Ask, defs.BufferMessage, defs.HomingMessage, defs.OverrideMessage]:
        if table[x.tag] is None:
            table[x] = x
        else:
            print("Conflicts for tag", x.tag)

        const_term = 0
        free_term = []

        for f in dataclasses.fields(x):
            (_,_,atomic), number, y = structmagic.resolve_type_hint(f.type)

            if number is None:
                const_term += atomic
            elif isinstance(number, int):
                const_term += atomic * number
            else:
                free_term.append(atomic * number)

        size = structmagic.CParam.expanded(const_term)
        n = len(free_term)
        if n:
            if n > 1:
                acc = free_term[0]
                for q in free_term[1:]:
                    acc = acc + q
                free_term = acc
            else:
                free_term = free_term[0]
            if const_term != 0:
                free_term = const_term + free_term

            size = structmagic.CParam.expanded(free_term)

        sizes[x.tag] = size


    with open("protocol_constants.h","w") as f:
        generate_protocol_constant_header(f, sizes)
    
    with open("protocol_constants.cpp","w") as f:
        generate_protocol_constant_cpp(f, sizes)
