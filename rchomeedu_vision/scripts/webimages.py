import os,sys
import random
import cv2

try:
    from google_images_download import google_images_download
except:
    print('pip install --user google_images_download')

list_objects = ['banana fruit', 'orange fruit', 'pineapple fruit', 'water bottle', 'coffee mug', 'plastic bag', 'volleyball ball', 'fake faces']

downloaddir = os.getenv('HOME')+"/Downloads/googleimagesdownload"

if not os.path.isdir(downloaddir):
    os.makedirs(downloaddir, 0755)

def download_images(obj,n=30):
    #class instantiation
    response = google_images_download.googleimagesdownload()   
    #creating list of arguments
    arguments = {"keywords":obj, "limit":n, 
        "print_urls":True, "output_directory":downloaddir}
    #passing the arguments to the function and download in output directory
    response.download(arguments)


def get_from_cache(obj):

    opath = downloaddir+'/'+obj
    if os.path.isdir(opath):
        #print(opath)
        l = os.listdir(opath)
    else:
        l = []

    if len(l)<30:
        download_images(obj,30)

    l = os.listdir(opath)

    return opath+"/"+random.choice(l)


def take_image(objcat=None, showimg=False):

    if objcat is None:
        objcat = random.choice(list_objects)

    f = get_from_cache(objcat)

    print(f)

    img = cv2.imread(f)

    if (showimg):
        cv2.namedWindow('image', cv2.WINDOW_NORMAL)
        cv2.imshow('image',img)
        cv2.waitKey(3000)
        cv2.destroyAllWindows()

    return img


if __name__ == '__main__':

    for obj in list_objects:
        get_from_cache(obj)


