import torch
from torch import nn, optim
from torch.autograd.variable import Variable
from torchvision import transforms, datasets
import torch.optim as optim

import iispt_dataset
import iispt_net

def main():

    trainset, testset = iispt_dataset.load_dataset("/home/gj/git/pbrt-v3-IISPT-dataset", 0.1)

    trainloader = torch.utils.data.DataLoader(trainset, batch_size=20, shuffle=True, num_workers=2)
    testloader = torch.utils.data.DataLoader(testset, batch_size=20, shuffle=False, num_workers=2)

    net = iispt_net.IISPTNet()

    criterion = nn.MSELoss()
    optimizer = optim.SGD(net.parameters(), lr=0.001, momentum=0.9)

    for epoch in range(2):

        running_loss = 0.0

        for i, data in enumerate(trainloader, 0):

            # Get the inputs
            input_x = data["t"]
            expected_x = data["p"]

            # Wrap in variables
            input_v = Variable(input_x)
            expected_v = Variable(expected_x)

            # Zero the parameter gradients
            optimizer.zero_grad()

            # Forward, backward, optimize
            outputs = net(input_v)
            loss = criterion(outputs, expected_v)
            loss.backward()
            optimizer.step()

            # Print statistics
            running_loss += loss.data[0]
            if i % 2 == 0:
                print("Epoch [{}] i [{}] Running loss [{}]".format(epoch, i, running_loss))
                running_loss = 0.0

    print("Finished training")

main()