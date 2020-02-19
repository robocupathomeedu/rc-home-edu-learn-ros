# Requires Keras and Tensorflow
# python2 -m pip install --user scipy==1.2.1 numpy==1.16.6 tensorflow==1.13.1 keras==2.2.4

# For raspberry PI
# swapon
# wget https://github.com/lhelontra/tensorflow-on-arm/releases/download/v1.13.1/tensorflow-1.13.1-cp27-none-linux_armv7l.whl
# sudo -H pip2 install tensorflow-1.13.1-cp27-none-linux_armv7l.whl 


import os, sys, time, socket
import threading

import numpy as np
import keras
import json
import argparse
import cv2

from keras import utils
from keras.preprocessing import image
from keras.applications import imagenet_utils, mobilenet

categories = [
    ['banana', 'slug'], ['orange', 'ping-pong_ball'], 
    ['pineapple'], ['cup', 'coffee_mug', 'coffeepot'], ['water_bottle', 'wine_bottle'],
    ['plastic_bag'],
    ['volleyball','tennis_ball','soccer_ball',
     'rugby_ball','basketball','football_helmet'],
    ['teddy', 'toy_poodle']
 ]


class MNetObjRec:

    def __init__(self):
        print('Loading mobilenet model...')
        self.mnet = mobilenet.MobileNet()   # define the mobilenet model (not thread safe!!!)
        self.flat_categories = [y for x in categories for y in x]
        self.imagenet_idx = {}
        self.getimagenetclasses()
        print('Done')

    def getimagenetclasses(self):
        print('Get imagenet classes...')
        CLASS_INDEX_PATH = ('https://storage.googleapis.com/download.tensorflow.org/data/imagenet_class_index.json')
        
        fpath = keras.utils.get_file(
            'imagenet_class_index.json',
            CLASS_INDEX_PATH,
            cache_subdir='models',
            file_hash='c2c37ea517e94d9795004a39431a14cb')
        with open(fpath) as f:
            CLASS_INDEX = json.load(f)

        #print(CLASS_INDEX)
        
        for k in CLASS_INDEX:
            c = CLASS_INDEX[k][1]
            if c in self.flat_categories:
                self.imagenet_idx[c] = int(k)
                #print('%s: %d' %(c,int(k)))

               
    # process an image to be mobilenet friendly 224x224x3
    def process_image(self,img_path):
      try:
          img = image.load_img(img_path, target_size=(224, 224))
          img_array = image.img_to_array(img)
          img_array = np.expand_dims(img_array, axis=0)
          pImg = mobilenet.preprocess_input(img_array)
          return pImg
      except:
          print("Error in opening image file %s" %img_path)
          return None


    def evalCVImage(self, img):
        imfile = '/tmp/lastimage.png'
        cv2.imwrite(imfile, img)
        return self.evalImageFile(imfile)


    def evalImageFile(self, imfile):
        # pre-process the test image
        pImg = self.process_image(imfile)
        return self.evalImage(pImg)


    def evalImage(self, pImg):

        if pImg is None:  # not an image
            return None  

        # make predictions on test image using mobilenet
        prediction = self.mnet.predict(pImg)
        # obtain the top-5 predictions
        results = imagenet_utils.decode_predictions(prediction)

        sr = '=== Image Classification ==='

        sr += '\n  ImageNet labels: '
        for rs in results[0]:
            sr += '%s: %.2f, ' %(rs[1],rs[2]*100)

        sr += '\n  Knonw labels: '
        
        pbest = 0
        for c in categories:
            p = 0
            for k in c:
                i = self.imagenet_idx[k]
                p += prediction[0][i]
            sr += ' %s: %.2f ' %(c[0],p*100)
            if p>pbest:
                pbest = p
                cbest = c[0]
        sr += '\n  >>> %s %.2f <<<\n' %(cbest,pbest*100)
        print(sr)
        return cbest,pbest*100



class MobileNetServer(threading.Thread):

    def __init__(self, port):
        threading.Thread.__init__(self)

        # Create a TCP/IP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.settimeout(3) # timeout when listening (exit with CTRL+C)
        # Bind the socket to the port
        server_address = ('', port)
        self.sock.bind(server_address)
        self.sock.listen(1)
        print("MobileNet Server running on port ", port, " ...")
        
        self.dorun = True # server running
        self.connection = None  # connection object


    def stop(self):
        self.dorun = False

    def connect(self):
        connected = False
        while (self.dorun and not connected):
            try:
                # print 'Waiting for a connection ...'
                # Wait for a connection
                self.connection, client_address = self.sock.accept()
                self.connection.settimeout(3)
                connected = True
                print('MobileNet Server: Connection from ', client_address)
            except:
                pass #print("Listen again ...")   


    def run(self):
        self.mnet = MNetObjRec()
        while (self.dorun):
            self.connect()  # wait for connection
            try:
                # Receive data
                while (self.dorun):
                    try:
                        data = self.connection.recv(2048)
                        data = data.strip()
                    except socket.timeout:
                        data = "***"
                    except:
                        data = None
                    
                    if (data!=None and data !="" and data!="***"):
                        self.received = data
                        if data=='REQ':
                            self.connection.send('ACK\n\r')
                        else:
                            v = data.split(' ')
                            if v[0]=='EVAL' and len(v)>1:
                                print('eval image [%s]' %v[1])
                                res = self.mnet.evalImage(v[1])
                                self.connection.send('%s\n\r' %res)
                            else:
                                print('MobileNet received: %s' %data)
                    elif (data == None or data==""):
                        break
            finally:
                print('MobileNet Server Connection closed.')
                # Clean up the connection
                if (self.connection != None):
                    self.connection.close()
                    self.connection = None





# wait for Keyboard interrupt
def dospin():
    run = True
    while (run):
        try:
            time.sleep(120)
        except KeyboardInterrupt:
            print("Exit")
            run = False




# main function
if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='MobileNet-ImageNet')
    parser.add_argument('-image', type=str, default=None, help='image file')
    parser.add_argument('--server', help='start server', action='store_true')

    args = parser.parse_args()

    # mobilenet server port    
    mnetport=9300

    if args.image is not None:
        print('Predict image %s' %args.image)
        mnet = MNetObjRec()
        r = mnet.evalImageFile(args.image)
        print(r)

    if args.server:
        mnetserver = MobileNetServer(mnetport)
        mnetserver.start()
        dospin() 
        mnetserver.stop()

    sys.exit(0)

