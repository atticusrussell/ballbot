import time
import cv2
import torch
import numpy as np
from numpy import random
from models.experimental import attempt_load
from utils.general import (non_max_suppression, scale_coords, xyxy2xywh, plot_one_box)
from utils.torch_utils import select_device, time_synchronized

# Initialize
device = select_device()
# Load model
model = attempt_load('model.pt', map_location=device)  # load FP32 model
# Get names and colors
names = model.module.names if hasattr(model, 'module') else model.names
colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(names))]
cap = cv2.VideoCapture(0)
while cap.isOpened():
    _, frame = cap.read()
    img = frame.copy()
    img = np.transpose(img, (2, 0, 1))
    img = torch.from_numpy(img).to(device)
    img = img.float()  # uint8 to fp16/32
    img /= 255.0  # 0 - 255 to 0.0 - 1.0
    if img.ndimension() == 3:
        img = img.unsqueeze(0)
    # Inference
    pred = model(img)[0]
    prev_time = time.time()
    # Apply NMS
    pred = non_max_suppression(pred, 0.4, 0.5)
    gn = torch.tensor(frame.shape)[[1, 0, 1, 0]]
    if pred != [None]:
        # Process detections
        for i, det in enumerate(pred):  # detections per image
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], frame.shape).round()
            # Write results
            for *xyxy, conf, cls in reversed(det):
                xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                cv2.circle(frame, (np.int(xywh[0] * 640), np.int(xywh[1] * 480)), 5, (0, 0, 255), -1)
                label = '%s %.2f' % (names[int(cls)], conf)
                plot_one_box(xyxy, frame, label=label, color=colors[int(cls)], line_thickness=3)
    curr_time = time.time()
    exec_time = curr_time - prev_time
    info = "time: %.2f ms" % (1000 * exec_time)
    cv2.putText(frame, text=info, org=(50, 70), fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=1, color=(255, 0, 0), thickness=2)
    cv2.imshow("frame", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'): break
cap.release()
cv2.destroyAllWindows()
