import torch
from torch import nn, optim
from torch.autograd.variable import Variable
from torchvision import transforms, datasets
import torch.optim as optim

import iispt_dataset
import iispt_net
import config

NO_EPOCHS = 1
BATCH_SIZE = 10
NO_WORKERS = 4

def main():

    trainset, testset = iispt_dataset.load_dataset("/home/gj/git/pbrt-v3-IISPT-dataset", 0.1)

    trainloader = torch.utils.data.DataLoader(trainset, batch_size=BATCH_SIZE, shuffle=True, num_workers=NO_WORKERS)
    testloader = torch.utils.data.DataLoader(testset, batch_size=BATCH_SIZE, shuffle=False, num_workers=NO_WORKERS)

    net = iispt_net.IISPTNet().cuda()

    criterion = nn.MSELoss()
    optimizer = optim.SGD(net.parameters(), lr=0.001, momentum=0.9)

    for epoch in range(NO_EPOCHS):

        # each i is a batch
        for i, data in enumerate(trainloader, 0):

            # Get the inputs
            input_x = data["t"]
            expected_x = data["p"]

            # Wrap in variables
            input_v = Variable(input_x.cuda())
            expected_v = Variable(expected_x.cuda())

            # Zero the parameter gradients
            optimizer.zero_grad()

            # Forward, backward, optimize
            outputs = net(input_v)
            loss = criterion(outputs, expected_v)
            loss.backward()
            optimizer.step()

            # Print statistics
            running_loss = loss.data[0]
            print("Epoch [{}] example [{}] Running loss [{}]".format(epoch, i * BATCH_SIZE, running_loss))

    print("Finished training")

    net = net.cpu()
    torch.save(net, config.model_path)
    print("Model saved: {}".format(config.model_path))

main()