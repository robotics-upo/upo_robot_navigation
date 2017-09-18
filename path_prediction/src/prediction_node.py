#!/usr/bin/python2.7

# ROS
import rospy
import rospkg

import numpy as np
#import pylab as plt
import matplotlib.pyplot as plt

#keras
from keras.models import load_model, Sequential, Model
from keras.preprocessing.image import load_img, img_to_array, array_to_img
from keras import backend as K
from keras.utils import plot_model

#openCV
import cv2

#service 
from path_prediction.srv import PathPrediction, PathPredictionResponse
 

#for directories and files manipulation
from os import listdir
from os.path import isfile, join








class Prediction(object):


  #--------------------------------------------------------------------------
  def __init__(self, norm, six):

    rospack = rospkg.RosPack()
    model_route = rospack.get_path('path_prediction') + '/model/my_model.h5'
    
    self.save_route = rospack.get_path('path_prediction') + '/saved_images/'
    self.save_count = 1
    self.save_img = False   

    self.model_ = load_model(model_route)
    print "Loaded_model:"
    print self.model_.summary()
    #plot_model(model, to_file=(test_dir+'model.png'), show_shapes=True)
    self.norm_img_data_ = norm
    self.use_six_ = six




  #--------------------------------------------------------------------------
  def preprocess_image_array(self, img_array, rows, cols, norm=False, six=False):

    #print('input:')
    #print(img_array)
    # Transform 1D array to greyscale image numpy array (1, rows, cols)
    inputimg = np.asarray(img_array, np.int16) # dtype=K.floatx()
    #print('input numpy array shape:')
    #print(inputimg)
    #print(inputimg.shape)
    img = []
    for i in range(rows):
      if i==0:
        #print('\ni: %i' % i)
        #print('Subarray:')
        #print(inputimg[(i*cols):(i*cols+(cols))])
        img = np.array(inputimg[(i*cols):(i*cols+(cols))], dtype=K.floatx())
        img = np.expand_dims(img, axis=0)
        #print(img)
        #print(img.shape)
      else:
        #print('\ni: %i' % i)
        #print('Subarray:')
        #print(inputimg[(i*cols):(i*cols+(cols))])
        aux = np.array(inputimg[(i*cols):(i*cols+(cols))], dtype=K.floatx())
        aux = np.expand_dims(aux, axis=0)
        img = np.append(img, aux, axis=0)
        #print(img)

    #print('prepocess. numpyarray shape:')
    #print(img.shape)
  
    #img = np.asarray(img_array, dtype=np.float32)
    img = img.reshape((1, img.shape[0], img.shape[1]))
    #print('img.dtype: %s' % img.dtype)

    if(norm==True):
      if(six==True):
        img[img <=15] = 0     #free: 0
        img[img >=235] = 1    #path: 255
        img[img >=184] = 0.8  #obstacles: 204
        img[img >=133] = 0.4  #people front: 153
        img[img >=82] = 0.2   #people back: 102
        img[img >=43] = 0.6   # goal: 63
      else:
        img[img <=15] = 0     #free: 0
        img[img >=235] = 1    #path: 255
        img[img >=184] = 0.25  #obstacles: 204
        img[img >=82] = 0.5  #people front: 153, back: 102
        img[img >=43] = 0.75   # goal: 63
    else:
      if(six==True):
        img[img <=15] = 0    #free: 0
        img[img >=235] = 3   #path: 255
        img[img >=184] = 2   #obstacles: 204
        img[img >=133] = -2  #people front: 153
        img[img >=82] = -1   #people back: 102
        img[img >=43] = 1    # goal: 63
      else:
        img[img <=15] = 0    #free: 0
        img[img >=235] = 2   #path: 255
        img[img >=184] = -1   #obstacles: 204
        img[img >=82] = -2  #people front: 153, back: 102
        img[img >=43] = 1    # goal: 63


    img = np.expand_dims(img, axis=0) 
    #print('Preprocess. Final Image shape:')
    #print(img.shape)
    return img



  #--------------------------------------------------------------------------
  def deprocess_image_pred(self, img):
    x = []
    # Util function to convert a tensor into a valid image.
    if K.image_data_format() == 'channels_first':
      #x = img.reshape((1, img.shape[2], img.shape[3]))
      #x = x.transpose((1, 2, 0))
      x = img.reshape(img.shape[2], img.shape[3])
    else:
      #x = img.reshape((img.shape[1], img.shape[2], 1))
      x = img.reshape(img.shape[1], img.shape[2])
  
    #print('Deprocess. Image shape:')
    #print(x.shape)

    #Transform numpy array to 1D array
    x = x.flatten()

    return x 


  #--------------------------------------------------------------------------
  def deprocess_image_grey(self, x):

    img = x
    # Util function to convert a tensor into a valid image.
    if K.image_data_format() == 'channels_first':
      img = img.reshape((1, img.shape[2], img.shape[3]))
      img = img.transpose((1, 2, 0))
    else:
      img = img.reshape((img.shape[1], img.shape[2], 1))
      
    #x /= 2.
    #x += 0.5
    img *= 255.
    img = np.clip(img, 0, 255).astype('uint8')

    return img




  #--------------------------------------------------------------------------
  def predict_path(self, req):

    img = self.preprocess_image_array(req.input, req.input_rows, req.input_cols, self.norm_img_data_, self.use_six_)
    output = self.model_.predict(img)
    pred = self.deprocess_image_pred(output)
    if(self.save_img==True):
      ig = self.deprocess_image_grey(output)
      name = 'pred_{0}'.format(self.save_count)		
      plt.imsave((self.save_route + name + ".jpeg"), ig[:,:,0]) 
      self.save_count = self.save_count + 1

    return PathPredictionResponse(pred, 200, 200)
   




#--------------------------------------------------------------------------
if __name__ == '__main__':


  rospy.init_node('prediction_node')
  print('--Starting prediction service--')

  normalize_values = rospy.get_param('~normalize_image_values', True)
  use_six_categories = rospy.get_param('~use_six_categories', True)

  print('Normalize values: %i ' % normalize_values)
  print('Use six categories: %i ' % use_six_categories)

  pred_node = Prediction(normalize_values, use_six_categories)

  # We call for prediction once for loading 
  vec = np.ones(200*200, dtype=np.int16)
  img = pred_node.preprocess_image_array(vec, 200, 200, False, False)
  output = pred_node.model_.predict(img)

  # Declare service
  service = rospy.Service('path_prediction', PathPrediction, pred_node.predict_path)

  rospy.spin()




     
