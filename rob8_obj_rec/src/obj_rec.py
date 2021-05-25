#!/usr/bin/env python3
import numpy as np
import pandas as pd
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from PIL import Image as pilImage
from torchvision import transforms
import torch
import models.res_net_siamese_triplet as models
from std_msgs.msg import String


class ObjectRecognition:
    def __init__(self):
        # Load model for inference.
        # Set embedding size and learning rate accordingly to the model used.
        self.embedding_size = 128
        self.rl = 0.0001
        self.device='cuda'
        self.model_path = f"models/resnet18_household_triplet_{self.rl}_100_100_3_{self.embedding_size}.pt"
        self.model = models.EmbeddingResNet(self.embedding_size).to(self.device)
        self.model.load_state_dict(torch.load(self.model_path)['model_state_dict'])
        print("model loaded")
        self.bridge = CvBridge()
        self.transform = transforms.Compose([transforms.Resize((224, 224)),
                                                               transforms.ToTensor(),
                                                               transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                                                                    std=[0.229, 0.224, 0.225])])

        # Load database
        self.db_df = pd.read_csv("database/database.csv", index_col=0)

        # Initialize ros node.
        rospy.init_node("object_recognition_node", anonymous=True)
        # rospy.Subscriber("/image_converter/output_video", Image, self.callback)
        rospy.Subscriber("/ori_img", Image, self.callback)
        # rospy.Subscriber("/camera/color/image_raw", Image, self.callback)
        # publisher for topic 'obj_rec_class_name'
        self.pub = rospy.Publisher("obj_rec_class_name", String, queue_size=1)

        # treshold for recognition:
        self.threshold = 1.25

    def listener(self):
        rospy.spin()

    def callback(self, image_msg):
        rospy.loginfo("image received")
        img = np.frombuffer(image_msg.data, dtype=np.uint8).reshape(image_msg.height, image_msg.width, -1)
        img = self.process_image(img).cuda()
        embedding = self.get_embedding(img)
        class_name = self.search_for_object(embedding)
        self.pub.publish(class_name)

    def process_image(self, img):
        img = pilImage.fromarray(img)
        img = self.transform(img)
        return torch.unsqueeze(img,dim=0)

    def get_embedding(self, img):
        with torch.no_grad():
            embedding = self.model(img).cpu().numpy()[0]
        return embedding

    def search_for_object(self, embedding):
        db_embeddings = self.db_df.iloc[:,:128].to_numpy()
        classes = self.db_df['class'].to_numpy()
        distances = np.linalg.norm(db_embeddings - embedding, axis=1)
        for i, distance in enumerate(distances):
            print("distance from ", classes[i], ": ", distance)
        arg_min = np.argmin(distances)
        if distances[arg_min] < self.threshold:
            print("Class recognized: ", classes[arg_min])
            return classes[arg_min]
        return "obj_not_found"


if __name__ == '__main__':
    try:
        obj_rec = ObjectRecognition()
        obj_rec.listener()
    except rospy.ROSInterruptException:
        pass
