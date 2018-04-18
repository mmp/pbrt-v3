import sys
import os
import subprocess
import struct
import time

import torch
from torch.autograd.variable import Variable
import numpy

import config
import iispt_transforms
import iispt_dataset
import km

# =============================================================================

# @prop
# Will be populated with keys "int_norm" and "dist_norm"

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

def write_float_array(xs):
    data = struct.pack("{}f".format(len(xs)), *xs)
    sys.stdout.buffer.write(data)
    sys.stdout.flush()

def write_char(c):
    sys.stdout.buffer.write(c.encode())
    sys.stdout.flush()

def read_float_nparray(data, num):
    buff = sys.stdin.buffer.read(num * 4)
    floats = struct.unpack('{}f'.format(num), buff)
    for i in range(num):
        data[i] = floats[i]

def read_float_raster_to_nparray(intensity_data):
    read_float_nparray(intensity_data, intensity_data.shape[0])

# Returns a flattened and processed numpy array
# <prop> is of type @prop
def read_input(prop):
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

    # Populate prop
    prop.int_norm = intensity_normalization
    prop.dist_norm = distance_normalization

    # Transform intensity
    intensity_data = numpy.vectorize(iispt_transforms.IntensitySequence(intensity_normalization, iispt_dataset.GAMMA_VALUE))(intensity_data)

    # Transform normals
    normals_data = numpy.vectorize(iispt_transforms.NormalizeTransform(-1.0, 1.0))(normals_data)

    # Transform distance
    distance_data = numpy.vectorize(iispt_transforms.DistanceSequence(distance_normalization, iispt_dataset.GAMMA_VALUE))(distance_data)

    # Concatenate the arrays
    concatenated = numpy.concatenate([intensity_data, normals_data, distance_data])

    return concatenated

# =============================================================================
# Inverse transforms
# <prop> is of type @prop
def inverse_transform(nparray, prop):

    # Inverse transform
    nparray = numpy.vectorize(iispt_transforms.IntensityInvSequence(prop.int_norm, iispt_dataset.GAMMA_VALUE))(nparray)

    return nparray

# =============================================================================
def output_to_stdout(nparray):
    length = nparray.shape[0]
    ptarray = []
    for i in range(length):
        ptarray.append(nparray[i])
    write_float_array(ptarray)
    write_char("x")
    write_char("\n")

# =============================================================================
# Processing function
def process_one(net):

    # Read input from stdin
    prop = km.KM()
    input_data = read_input(prop)
    torch_data = torch.from_numpy(input_data).float()
    input_variable = Variable(torch_data)

    # Run the network
    output_variable = net(input_variable)

    # Do inverse transform
    output_numpy = output_variable.data.numpy()
    output_transformed = inverse_transform(output_variable.data.numpy(), prop)

    # Output to STDOUT
    output_to_stdout(output_transformed)

# =============================================================================
# Main

def main():

    # Load model
    net = torch.load(config.model_path)
    print_stderr("Model loaded")

    while True:
        process_one(net)

def main_benchmark():

    # Load model
    net = torch.load(config.model_path)
    print_stderr("Model loaded")

    torch_data = torch.randn(7168).float()
    input_variable = Variable(torch_data)

    ITERATIONS = 100

    start = time.time()

    for i in range(ITERATIONS):
        output_variable = net(input_variable)

    end = time.time()

    elapsed_millis = (end - start) * 1000.0
    elapsed_per_iter = elapsed_millis / ITERATIONS
    print_stderr("Time per iteration: {}".format(elapsed_per_iter))

def main_benchmark_with_numpy():

    # Load model
    net = torch.load(config.model_path)
    print_stderr("Model loaded")

    ITERATIONS = 100

    start = time.time()

    for i in range(ITERATIONS):
        intensity_data = numpy.random.randn(IISPT_IMAGE_SIZE * IISPT_IMAGE_SIZE * 3)
        distance_data = numpy.random.randn(IISPT_IMAGE_SIZE * IISPT_IMAGE_SIZE * 1)
        normals_data = numpy.random.randn(IISPT_IMAGE_SIZE * IISPT_IMAGE_SIZE * 3)
        input_data = numpy.concatenate([intensity_data, normals_data, distance_data])
        torch_data = torch.from_numpy(input_data).float()
        input_variable = Variable(torch_data)

        output_variable = net(input_variable)
        output_numpy = output_variable.data.numpy()

    end = time.time()

    elapsed_millis = (end - start) * 1000.0
    elapsed_per_iter = elapsed_millis / ITERATIONS
    print_stderr("Time per iteration: {}".format(elapsed_per_iter))

def main_benchmark_with_transforms():

    # Load model
    net = torch.load(config.model_path)
    print_stderr("Model loaded")

    ITERATIONS = 100

    start = time.time()

    for i in range(ITERATIONS):
        intensity_data = numpy.random.randn(IISPT_IMAGE_SIZE * IISPT_IMAGE_SIZE * 3)
        distance_data = numpy.random.randn(IISPT_IMAGE_SIZE * IISPT_IMAGE_SIZE * 1)
        normals_data = numpy.random.randn(IISPT_IMAGE_SIZE * IISPT_IMAGE_SIZE * 3)
        
        intensity_normalization = 12.0
        distance_normalization = 1.0

        intensity_data = numpy.vectorize(iispt_transforms.IntensitySequence(intensity_normalization, iispt_dataset.GAMMA_VALUE))(intensity_data)

        normals_data = numpy.vectorize(iispt_transforms.NormalizeTransform(-1.0, 1.0))(normals_data)

        distance_data = numpy.vectorize(iispt_transforms.DistanceSequence(distance_normalization, iispt_dataset.GAMMA_VALUE))(distance_data)
        
        input_data = numpy.concatenate([intensity_data, normals_data, distance_data])

        torch_data = torch.from_numpy(input_data).float()
        input_variable = Variable(torch_data)

        output_variable = net(input_variable)
        output_numpy = output_variable.data.numpy()

    end = time.time()

    elapsed_millis = (end - start) * 1000.0
    elapsed_per_iter = elapsed_millis / ITERATIONS
    print_stderr("Time per iteration: {}".format(elapsed_per_iter))


#main_benchmark_with_transforms()
main()