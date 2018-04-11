import sys
import struct

def read_float():
    return struct.unpack('f', sys.stdin.buffer.read(4))[0]

fval = read_float()

sys.stderr.write("CHILD STDERR\n")

if fval < 10.0:
    sys.stdout.buffer.write(struct.pack('f', -101.5))
    sys.stdout.flush()
else:
    sys.stdout.buffer.write(struct.pack('f', 205.99))
    sys.stdout.flush()