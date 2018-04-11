# This class defines the PFM image format loader and
# saver
# The loaded class uses a numpy array as storage format
# for easy use in PyTorch

import numpy
import struct
import math

import iispt_transforms

# =============================================================================
# Class definitions

class PfmImage:

    # -------------------------------------------------------------------------
    def __init__(self, data):
        self.data = data
    
    # -------------------------------------------------------------------------
    def print_shape(self):
        print(self.data.shape)
    
    # -------------------------------------------------------------------------
    def print_array(self):
        print(self.data)
    
    # -------------------------------------------------------------------------
    def get_numpy_array(self):
        return self.data
    
    # -------------------------------------------------------------------------
    def map(self, f):
        f = numpy.vectorize(f)
        self.data = f(self.data)
    
    # -------------------------------------------------------------------------
    # Given min and max vals in the original range,
    # Remaps everything into the [-1, +1] range
    # And clips any values that stay outside
    def normalize(self, min_val, max_val):
        t = iispt_transforms.NormalizeTransform(min_val, max_val)
        self.map(t)
    
    # -------------------------------------------------------------------------
    # Applies a natural logarithm on the value
    # And normalizes according to given max_value
    def normalize_log(self, max_value):
        self.map(iispt_transforms.LogTransform())
        self.normalize(0.0, max_value)
    
    # -------------------------------------------------------------------------
    # Applies a natural logarithm followed by a gamma curve
    # to boost the smaller values
    # Normalizes according to the given max_value
    def normalize_log_gamma(self, max_value, gamma):
        self.map(iispt_transforms.IntensitySequence(max_value, gamma))

    # -------------------------------------------------------------------------
    # 1 - Apply the square root
    # 2 - Normalize according to the max value. Min value is -1
    #     for the pixels that have no intersection
    def normalize_sqrt(self, max_value):
        self.map(iispt_transforms.SqrtTransform())
        self.normalize(-1.0, max_value)

    # -------------------------------------------------------------------------
    # 1 - Apply the square root
    # 2 - Normalize according to max value into [0,1]
    # 3 - Apply gamma correction
    def normalize_sqrt_gamma(self, max_value, gamma):
        self.map(iispt_transforms.DistanceSequence(max_value, gamma))
    
    # -------------------------------------------------------------------------
    # Write out to .pfm file
    def save_pfm(self, out_path):
        print("Writing {}".format(out_path))
        out_file = open(out_path, "wb")

        # Write identifier line
        out_file.write(b"PF\n")

        # Write dimensions line
        width, height, channels = self.data.shape
        out_file.write("{} {}\n".format(width, height).encode())

        # Write scale factor and endianness
        out_file.write(b"1\n")

        # Write pixel values
        for y in range(height):
            for x in range(width):
                for c in range(channels):
                    write_float_32(out_file, self.data[y, x, c])

        out_file.close()

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

def write_float_32(f, v):
    data = struct.pack('f', v)
    f.write(data)

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
# Load from flattened numpy array

def load_from_flat_numpy(narray, width=32, height=32, channels=3):
    shape = (height, width, channels)
    narray = narray.reshape(shape)
    return PfmImage(narray)

# =============================================================================
# Quick test

def test_main():
    p = load("/home/gj/git/pbrt-v3-IISPT-dataset/barcelona_pavilion_day/p_160_420.pfm")
    p.print_shape()
    p.print_array()
    p.save_pfm("test.pfm")

# test_main()