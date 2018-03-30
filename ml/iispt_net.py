import torch
from torch import nn, optim
from torch.autograd.variable import Variable
from torchvision import transforms, datasets

class IISPTNet(torch.nn.Module):

    def __init__(self):
        super(IISPTNet, self).__init__()

        self.hidden0 = nn.Sequential(
            nn.Linear(7168, 3000),
            nn.Sigmoid(),
            nn.Dropout(0.1)
        )

        self.hidden1 = nn.Sequential(
            nn.Linear(3000, 2800),
            nn.Tanh(),
            nn.Dropout(0.1)
        )

        self.hidden2 = nn.Sequential(
            nn.Linear(2800, 2500),
            nn.Tanh(),
            nn.Dropout(0.1)
        )

        self.hidden3 = nn.Sequential(
            nn.Linear(2500, 2400),
            nn.Tanh(),
            nn.Dropout(0.1)
        )

        self.out = nn.Sequential(
            nn.Linear(2400, 3072),
            nn.Tanh()
        )
    
    def forward(self, x):
        x = self.hidden0(x)
        x = self.hidden1(x)
        x = self.hidden2(x)
        x = self.hidden3(x)
        x = self.out(x)
        return x