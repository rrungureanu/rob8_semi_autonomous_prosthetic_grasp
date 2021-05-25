import torch
import torch.nn as nn
from torch.nn.functional import normalize
from torchvision import models


class EmbeddingResNet(nn.Module):
    def __init__(self, embedding_size):
        super(EmbeddingResNet, self).__init__()
        model = models.resnet18(pretrained=True)
        self.cnn = torch.nn.Sequential(*(list(model.children())[:-2]))

        self.fc1 = nn.Sequential(
            nn.Flatten(),
            nn.Linear(512 * 7 * 7, embedding_size)
        )

    def forward(self, input_tensor):
        features = self.cnn(input_tensor)
        embeddings = normalize(self.fc1(features), p=2.0)
        return embeddings