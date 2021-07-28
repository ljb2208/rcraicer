import torch
import torch.nn as nn


class DynModel(nn.Module):
    def __init__(self):
        super().__init__()

        in_features = 9
        layer1_size = 32 # orig 32
        layer2_size = 32 # orig 32
        layer3_size = 32 # orig 32
        out_features = 3


        self.bn0 = nn.BatchNorm1d(in_features, eps=1e-5, momentum=0.1, affine=True, track_running_stats=True)
        self.bn1 = nn.BatchNorm1d(in_features, eps=1e-5, momentum=0.1, affine=True, track_running_stats=True)
        self.lin1 = nn.Linear(in_features=in_features, out_features=layer1_size, bias=False)
        # self.rel1 = nn.ReLU(inplace=True)
        # self.rel1 = nn.Tanh()
        self.rel1 = nn.LeakyReLU()

        self.bn2 = nn.BatchNorm1d(layer1_size, eps=1e-5, momentum=0.1, affine=True, track_running_stats=True)
        self.lin2 = nn.Linear(in_features=layer1_size, out_features=layer2_size, bias=False)
        # self.rel2 = nn.ReLU(inplace=True)
        # self.rel2 = nn.Tanh()
        self.rel2 = nn.LeakyReLU()

        # self.bn3 = nn.BatchNorm1d(layer2_size, eps=1e-5, momentum=0.1, affine=True, track_running_stats=True)
        # self.lin3 = nn.Linear(in_features=layer2_size, out_features=layer3_size, bias=False)
        # # self.rel2 = nn.ReLU(inplace=True)
        # self.rel3 = nn.Tanh()

        # self.lin_last = nn.Linear(in_features=layer2_size, out_features=out_features, bias=True)
        self.lin_last = nn.Linear(in_features=layer2_size, out_features=out_features, bias=True)
        
        nn.init.kaiming_uniform_(self.lin1.weight.data)
        nn.init.kaiming_uniform_(self.lin2.weight.data)
        # nn.init.kaiming_normal_(self.lin3.weight.data)
        nn.init.kaiming_uniform_(self.lin_last.weight.data)

    def forward(self, x):
        x = self.bn0(x)
        x = self.lin1(self.bn1(x))
        x = self.rel1(x)
        x = self.lin2(self.bn2(x))
        x = self.rel2(x)
        # x = self.lin3(self.bn3(x))
        # x = self.rel3(x)
        x = self.lin_last(x)

        return x
        




