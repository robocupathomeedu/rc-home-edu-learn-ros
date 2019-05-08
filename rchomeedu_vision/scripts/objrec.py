import cv2
import mobilenet
import take_photo2
import webimages
import argparse

parser = argparse.ArgumentParser(description='Object Recognition Example')
parser.add_argument('--web', help='get image from web', action='store_true')
args = parser.parse_args()

if args.web:
    img = webimages.take_image()
else:
    img = take_photo2.take_image()

cv2.imshow('image',img)
cv2.waitKey(1000)

mnet = mobilenet.MNetObjRec()
r = mnet.evalCVImage(img)
print(r)

