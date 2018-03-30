import torch

import config

def main():

    # Load model
    the_model = torch.load(config.model_path)
    print("Model loaded")

    # Display image

main()