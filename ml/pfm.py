# This class defines the PFM image format loader and
# saver
# The loaded class uses a numpy array as storage format
# for easy use in PyTorch

import numpy
import struct
import math

# =============================================================================
# Transform callables

class NormalizeTransform:

    def __init__(self, min_val, max_val):
        self.min_val = min_val
        self.max_val = max_val
    
    def __call__(self, x):
        y = x + self.min_val
        new_max = self.max_val + self.min_val
        y = x / new_max
        if y < 0.0:
            return 0.0
        elif y > 1.0:
            return 1.0
        else:
            return y

class LogTransform:

    def __init__(self):
        pass
    
    def __call__(self, x):
        return math.log(x + 1.0)

class SqrtTransform:

    def __init__(self):
        pass
    
    def __call__(self, x):
        if x < 0.0:
            return 0.0
        return math.sqrt(x)

# =============================================================================
# Class definitions

class PfmImage:

    def __init__(self, data):
        self.data = data
    
    def print_shape(self):
        print(self.data.shape)
    
    def print_array(self):
        print(self.data)
    
    def map(self, f):
        f = numpy.vectorize(f)
        self.data = f(self.data)
    
    # Given min and max vals in the original range,
    # Remaps everything into the 0-1 range
    # And clips any values that stay outside
    def normalize(self, min_val, max_val):
        t = NormalizeTransform(min_val, max_val)
        self.map(t)
    
    # Applies a natural logarithm on the value
    # And normalizes according to given max_value
    def normalize_log(self, max_value):
        self.map(LogTransform())
        self.normalize(0.0, max_value)
    
    # 1 - Apply the square root
    # 2 - Normalize according to the max value. Min value is -1
    #     for the pixels that have no intersection
    def normalize_sqrt(self, max_value):
        self.map(SqrtTransform())
        self.normalize(-1.0, max_value)

# =============================================================================
# Utilities

def read_line(f):
    buff = b""
    while True:
        c = f.read(1)
        if not c:
            raise Exception("Unexpected end of file")
        elif c == b'\n':
            return buff.decode("UTF-8")
        else:
            buff += c

def read_float_32(f):
    return struct.unpack('f', f.read(4))[0]

def load_pixel(f, y, x, channels, data):
    for p in range(channels):
        val = read_float_32(f)
        data[y, x, p] = val

def load_row(f, y, width, channels, data):
    # 2 dimensions: width, channels
    for x in range(width):
        load_pixel(f, y, x, channels, data)
    
# =============================================================================
# Load

def load(file_path):

    # Use a large 10KB buffer
    f = open(file_path, "rb", 10000)

    # Read the identifier line
    identifier_line = read_line(f)
    if identifier_line == "PF":
        channels = 3
    elif identifier_line == "Pf":
        channels = 1
    else:
        raise Exception("Unrecognized identifier line {}".format(identifier_line))
    
    # Read the dimensions line
    dimensions_line = read_line(f)
    dimensions_line_split = dimensions_line.split(" ")
    if len(dimensions_line_split) != 2:
        raise Exception("Could not recognize PFM dimensions line in [{}]".format(dimensions_line))
    width = int(dimensions_line_split[0])
    height = int(dimensions_line_split[1])

    # Read scale factor and endianness
    read_line(f)
    # Ignore the value

    # Read pixel values
    # The array has 3 dimensions: Height, Width, Channels
    data = numpy.zeros(shape=(height, width, channels), dtype=numpy.float32)
    for y in range(height):
        load_row(f, y, width, channels, data)
    
    f.close()

    # Create final object
    return PfmImage(data)

# =============================================================================
# Quick test

p = load("z_0_0.pfm")
p.print_shape()
p.print_array()

p.normalize(0.0, 100.0)
p.print_shape()
p.print_array()

p.normalize_log(2.0)
p.print_shape()
p.print_array()

p.normalize_sqrt(2.0)
p.print_shape()
p.print_array()