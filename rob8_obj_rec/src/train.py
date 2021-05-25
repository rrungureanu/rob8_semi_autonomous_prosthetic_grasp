import cv2 as cv
from torchvision import transforms
from PIL import Image as pilImage
import torch
import models.res_net_siamese_triplet as models
import pandas as pd
import os


def process_image(img):
    transform = transforms.Compose([transforms.Resize((224, 224)),
                                    transforms.ToTensor(),
                                    transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                                         std=[0.229, 0.224, 0.225])])
    img = pilImage.fromarray(img)
    img = transform(img)
    return torch.unsqueeze(img, dim=0)


if __name__ == "__main__":
    embedding_size = 128
    rl = 0.0001
    device = 'cuda'
    model_path = f"models/resnet18_household_triplet_{rl}_100_100_3_{embedding_size}.pt"
    model = models.EmbeddingResNet(embedding_size).to(device)
    model.load_state_dict(torch.load(model_path)['model_state_dict'])
    print("model loaded")

    img_dir = "images"
    output_dir = "database"

    embedding_list = []
    class_list = []

    for img_name in os.listdir(img_dir):
        img = cv.imread(os.path.join(img_dir, img_name))
        img = process_image(img).cuda()
        print(img.shape)
        with torch.no_grad():
            embedding = model(img).cpu().numpy()[0]
        print(embedding)
        embedding_list.append(embedding)
        class_list.append(img_name)
    # pd
    df = pd.DataFrame(embedding_list, columns=[i for i in range(embedding.shape[0])])
    df['class'] = class_list
    print(df.head())
    df.to_csv(os.path.join(output_dir, "database.csv"))
