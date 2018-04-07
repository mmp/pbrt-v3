import sys
import os
import subprocess

import torch
from torch.autograd.variable import Variable
import numpy

import config
import iispt_dataset
import pfm

pydir = os.path.dirname(os.path.abspath(__file__)) # root/ml
rootdir = os.path.dirname(pydir)
os.chdir(rootdir)

def print_force(s):
    print(s)
    sys.stdout.flush()

def convert_image(pfm_path, output_path, png_path):
    subprocess.call(["pfm", pfm_path, "--out=" + output_path])
    subprocess.call(["convert", output_path, png_path])

def main():

    # Load dataset
    trainset, testset = iispt_dataset.load_dataset("/home/gj/git/pbrt-v3-IISPT-dataset", 0.0)

    # Load model
    net = torch.load(config.model_path)
    print_force("#LOADCOMPLETE")


    # Loop for console info
    for line in sys.stdin:
        if line.endswith("\n"):
            line = line[:-1]

        idx = int(line)
        print_force("Requesting index {}".format(idx))

        datum = trainset.get_datum(idx)
        if datum is None:
            print_force("Out of range!")
            continue
        item = trainset.__getitem__(idx)
        item_input = item["t"]
        item_expected = item["p"]

        # Run the network on the data
        input_variable = Variable(item_input)
        result = net(input_variable)
        print_force("Result.data is {}".format(result.data))
        print_force("Expected is {}".format(item_expected))

        # TODO inverse log transform

        # Save the created result
        result_image = pfm.load_from_flat_numpy(result.data.numpy())
        result_image.save_pfm("created.pfm")

        # Save the expected result
        expected_image = pfm.load_from_flat_numpy(item_expected.numpy())
        expected_image.save_pfm("expected.pfm")

        # Convert images to PNG
        convert_image("created.pfm", "created.ppm", "created.png")
        convert_image("expected.pfm", "expected.ppm", "expected.png")

        print_force("#EVALUATECOMPLETE")


main()