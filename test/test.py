import cv2
import torch
from PIL import Image
import numpy as np
from torchvision import models
from torchvision import transforms

img_path = 'test.png'

tusimple = torch.load('tusimple_r18.pth')
model = models.resnet18(progress=True)
model.load_state_dict(tusimple, strict=False)

ori_img = cv2.imread(img_path)
img = ori_img[320:, :, :].astype(np.float32)
data = {'img': img, 'lanes': []}
data['img'] = data['img'].unsqueeze(0)
data.update({'img_path': img_path, 'ori_img': ori_img})

if torch.cuda.is_available():
    input_batch = input_batch.to('cuda')
    model.to('cuda')

with torch.no_grad():
    output = model(input_batch)

print(output.shape)
