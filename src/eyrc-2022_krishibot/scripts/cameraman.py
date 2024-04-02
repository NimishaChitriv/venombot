import cv2 
from matplotlib import pyplot as plt
import numpy as np

image1=cv2.imread('/home/vedh/catkin_ws/src/eyrc-2022_krishibot/scripts/image8.jpg',0)
# cv2.imshow("Image", image1)
# cv2.waitKey(0)
histr = cv2.calcHist([image1],[0],None,[256],[0,256])

# src = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
eq = cv2.equalizeHist(image1)
histeq = cv2.calcHist([eq],[0],None,[256],[0,256])
res = np.hstack((image1, eq))

kernelSizes = [(3, 3)]

for (kX, kY) in kernelSizes:
	blurred = cv2.blur(image1, (kX, kY))
cv2.imshow('image2',blurred)

histo=plt.plot(histr)
plt.show(histo)
histoeq=plt.plot(histeq)
plt.show(histoeq)
cv2.imshow('image', res)
cv2.waitKey(0)
cv2.destroyAllWindows()