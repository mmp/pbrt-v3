import sys
import os
import subprocess
import struct

import torch
from torch.autograd.variable import Variable
import numpy

import config
import iispt_transforms
import iispt_dataset

# -----------------------------------------------------------------------------
# Constants

IISPT_IMAGE_SIZE = 32

# -----------------------------------------------------------------------------
# Init

pydir = os.path.dirname(os.path.abspath(__file__)) # root/ml
rootdir = os.path.dirname(pydir)
os.chdir(rootdir)

# =============================================================================
# Utilities

def print_stderr(s):
    sys.stderr.write(s + "\n")

def read_float():
    return struct.unpack('f', sys.stdin.buffer.read(4))[0]

def read_float_nparray(data, num):
    buff = sys.stdin.buffer.read(num * 4)
    floats = struct.unpack('{}f'.format(num), buff)
    for i in range(num):
        data[i] = floats[i]

def read_float_raster_to_nparray(intensity_data):
    read_float_nparray(intensity_data, intensity_data.shape[0])

# Returns a flattened and processed numpy array
def read_input():
    # Read intensity raster
    # intensity_data is a flattened array
    intensity_data = numpy.zeros(shape=(IISPT_IMAGE_SIZE * IISPT_IMAGE_SIZE * 3), dtype=numpy.float32)
    read_float_raster_to_nparray(intensity_data)

    # Read distance raster
    distance_data = numpy.zeros(shape=(IISPT_IMAGE_SIZE * IISPT_IMAGE_SIZE * 1), dtype=numpy.float32)
    read_float_raster_to_nparray(distance_data)

    # Read normals raster
    normals_data = numpy.zeros(shape=(IISPT_IMAGE_SIZE * IISPT_IMAGE_SIZE * 3), dtype=numpy.float32)
    read_float_raster_to_nparray(normals_data)

    # Read intensity normalization
    intensity_normalization = read_float()

    # Read distance normalization
    distance_normalization = read_float()

    print_stderr("Completed reading values")

    # Transform intensity
    intensity_data = numpy.vectorize(iispt_transforms.IntensitySequence(intensity_normalization, iispt_dataset.GAMMA_VALUE))(intensity_data)

    # Transform normals
    normals_data = numpy.vectorize(iispt_transforms.NormalizeTransform(-1.0, 1.0))(normals_data)

    # Transform distance
    distance_data = numpy.vectorize(iispt_transforms.DistanceSequence(distance_normalization, iispt_dataset.GAMMA_VALUE))(normals_data)

    # Concatenate the arrays
    concatenated = numpy.concatenate([intensity_data, normals_data, distance_data])

    return concatenated

# =============================================================================
# Main

def main():

    # Load model
    net = torch.load(config.model_path)
    print_stderr("Model loaded")

    # Read input from stdin
    input_data = read_input()
    print_stderr("Got the input numpy array {}".format(input_data))

main()