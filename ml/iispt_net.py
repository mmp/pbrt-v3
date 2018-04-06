import torch
from torch import nn, optim
from torch.autograd.variable import Variable
from torchvision import transforms, datasets

class IISPTNet(torch.nn.Module):

    def __init__(self):
        super(IISPTNet, self).__init__()

        self.hidden0 = nn.Sequential(
            nn.Linear(7168, 5000),
            nn.Sigmoid()
        )

        self.hidden1 = nn.Sequential(
            nn.Linear(5000, 4000),
            nn.LeakyReLU(0.2)
        )

        self.hidden2 = nn.Sequential(
            nn.Linear(4000, 3000),
            nn.LeakyReLU(0.2)
        )

        self.hidden3 = nn.Sequential(
            nn.Linear(3000, 1500),
            nn.LeakyReLU(0.2),
            nn.Dropout(0.1)
        )

        self.out = nn.Sequential(
            nn.Linear(1500, 3072),
            nn.Tanh()
        )
    
    def forward(self, x):
        x = self.hidden0(x)
        x = self.hidden1(x)
        x = self.hidden2(x)
        x = self.hidden3(x)
        x = self.out(x)
        return x