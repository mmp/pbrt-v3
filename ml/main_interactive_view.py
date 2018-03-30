import sys

import torch
from torch.autograd.variable import Variable
import numpy

import config
import iispt_dataset
import pfm

def main():

    # Load dataset
    trainset, testset = iispt_dataset.load_dataset("/home/gj/git/pbrt-v3-IISPT-dataset", 0.0)

    # Load model
    net = torch.load(config.model_path)
    print("Model loaded")


    # Loop for console info
    for line in sys.stdin:
        if line.endswith("\n"):
            line = line[:-1]

        idx = int(line)
        print("Requesting index {}".format(idx))

        datum = trainset.get_datum(idx)
        if datum is None:
            print("Out of range!")
            continue
        item = trainset.__getitem__(idx)
        item_input = item["t"]
        item_expected = item["p"]

        # Run the network on the data
        input_variable = Variable(item_input)
        result = net(input_variable)
        print("Result.data is {}".format(result.data))
        print("Expected is {}".format(item_expected))

        # TODO inverse log transform

        # Save the created result
        result_image = pfm.load_from_flat_numpy(result.data.numpy())
        result_image.save_pfm("created.pfm")

        # Save the expected result
        expected_image = pfm.load_from_flat_numpy(item_expected.numpy())
        expected_image.save_pfm("expected.pfm")


main()