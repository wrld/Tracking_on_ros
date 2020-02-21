# --** coding="UTF-8" **--
from skimage.measure import compare_ssim
from scipy.misc import imread
import numpy as np
 
img1 = imread('/home/gjx/ROS/SRTP-drone/pic_010.jpg')
img2 = imread('/home/gjx/ROS/SRTP-drone/pic_012.jpg')
img3 = imread('/home/gjx/ROS/SRTP-drone/pic_011.jpg')
 
img2 = np.resize(img2, (img1.shape[0], img1.shape[1], img1.shape[2]))
img3 = np.resize(img3, (img1.shape[0], img1.shape[1], img1.shape[2]))

print(img2.shape)
print(img1.shape)
ssim = compare_ssim(img1, img2, multichannel=True)
ssimdiff = compare_ssim(img1, img3, multichannel=True)
 
print(ssim)
print(ssimdiff)
