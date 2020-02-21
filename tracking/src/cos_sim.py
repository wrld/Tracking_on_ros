from PIL import Image
from numpy import average, linalg, dot
from scipy.misc import imread
 
def get_thumbnail(image, size=(120, 75), greyscale=False):
    image = image.resize(size, Image.ANTIALIAS)
    if greyscale:
        image = image.convert('L')
    return image
 
 
def image_similarity_vectors_via_numpy(image1, image2):
 
    image1 = get_thumbnail(image1)
    image2 = get_thumbnail(image2)
    images = [image1, image2]
    vectors = []
    norms = []
    for image in images:
        vector = []
        for pixel_tuple in image.getdata():
            vector.append(average(pixel_tuple))
        vectors.append(vector)
        norms.append(linalg.norm(vector, 2))
    a, b = vectors
    a_norm, b_norm = norms
    res = dot(a / a_norm, b / b_norm)
    return res
 
 
image1 = imread('/home/gjx/ROS/SRTP-drone/pic_010.jpg')
image2 = imread('/home/gjx/ROS/SRTP-drone/pic_012.jpg')
image3 = imread('/home/gjx/ROS/SRTP-drone/pic_011.jpg')
cosin = image_similarity_vectors_via_numpy(image1, image2)
 
print(cosin)