import os
import cv2
import argparse
import take_photo2
import webimages

# Haar detector
def findCascadeModel():
    trylist = ['/usr/share/opencv/', '/opt/ros/kinetic/share/OpenCV-3.3.1-dev/' ]
    for t in trylist:
        f = t + 'haarcascades/haarcascade_frontalface_default.xml'
        if os.path.isfile(f):
            return cv2.CascadeClassifier(f)
    return None

faceCascade = None

def faceDetection(img, showimg=False):
    global faceCascade

    if showimg:
        cv2.imshow('image', img)
        cv2.waitKey(1000)

    if faceCascade is None:
        faceCascade = findCascadeModel()
        if faceCascade is None:
            print("ERROR Cannot find Haar cascade model")
            return -1
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Detect faces in the image
    faces = faceCascade.detectMultiScale(gray,
        scaleFactor=1.1,
        minNeighbors=5,
        minSize=(30, 30)
    )

    if showimg and len(faces)>0:
        # Draw a rectangle around the faces
        for (x, y, w, h) in faces:
            cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
        cv2.imshow("image", img)
        cv2.waitKey(3000)

    return faces



if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Object Recognition Example')
    parser.add_argument('--web', help='get image from web', action='store_true')
    parser.add_argument('--show', help='show image', action='store_true')
    args = parser.parse_args()

    if args.web:
        img = webimages.take_image('fake faces')
    else:
        img = take_photo2.take_image()

    f = faceDetection(img, showimg=args.show)

    print("Found %d faces" %(len(f)))

