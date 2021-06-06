import cv2
import numpy as np
import matplotlib.pylab as plt
import math
import os

img = cv2.imread("../../../duckieass21cm.png")

# RGB filter
(B, G, R) = cv2.split(img)

R[R < 250] = 0

R[R >= 250] = 255

R[G < 250] = 0

R[B < 250] = 0


# HLS filter
hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)

(h, l, s) = cv2.split(hls)

l[l < int(255 * 0.99)] = 0
l[l >= int(255 * 0.99)] = 255

R[l < 255] = 0

# gaussian blur
kernel = np.ones((7,7), np.uint8)

d_im = cv2.dilate(R, kernel, iterations=1)

e_im = cv2.erode(d_im, kernel, iterations=1)



contours = cv2.findContours(e_im, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
contours = contours[0] if len(contours) == 2 else contours[1]

print(img.shape)

centroids = []
for c in contours:
    if len(c) < 10:
        continue
    
    x_sum = 0
    y_sum = 0
    for p in c:
        x_sum += p[0][0]
        y_sum += p[0][1]
    
    x_avg = x_sum / len(c)
    y_avg = y_sum / len(c)
    centroids.append([x_avg, y_avg])
    print(x_avg)
    print(y_avg)
    
    cv2.circle(img, (int(x_avg), int(y_avg)), 10, (0,0,255), -1)

print(math.sqrt((centroids[1][0] - centroids[0][0])**2 + (centroids[1][1] - centroids[0][1])**2) / img.shape[1])

cv2.imshow('rgb', img)



cv2.waitKey(0); cv2.destroyAllWindows(); cv2.waitKey(1)