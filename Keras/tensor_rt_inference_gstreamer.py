#!/usr/bin/env python
# coding: utf-8

# In[1]:


output_names = ['dense_2/Sigmoid']
input_names = ['conv2d_1_input']

import tensorflow as tf


def get_frozen_graph(graph_file):
    """Read Fr
    ozen Graph file from disk."""
    with tf.gfile.FastGFile(graph_file, "rb") as f:
        graph_def = tf.GraphDef()
        graph_def.ParseFromString(f.read())
    return graph_def


trt_graph = get_frozen_graph('./model/trt_graph.pb')

# Create session and load graph
tf_config = tf.ConfigProto()
tf_config.gpu_options.allow_growth = True
tf_sess = tf.Session(config=tf_config)
tf.import_graph_def(trt_graph, name='')


# In[2]:


# Get graph input size
# for node in trt_graph.node:
#     if 'input_' in node.name:
#         print('im here')
#         size = node.attr['shape'].shape
#         image_size = [size.dim[i].size for i in range(1, 4)]
#         break
# print("image_size: {}".format(image_size))

import time
start = time.time()
# input and output tensor names.
input_tensor_name = input_names[0] + ":0"
output_tensor_name = output_names[0] + ":0"

print("input_tensor_name: {}\noutput_tensor_name: {}".format(
    input_tensor_name, output_tensor_name))

output_tensor = tf_sess.graph.get_tensor_by_name(output_tensor_name)

end = time.time()

print('Total time : ', end - start)


# In[6]:


from tensorflow.keras.preprocessing import image
import cv2
import numpy as np
start = time.time()
img_path = 'full/554.jpg'
img = image.load_img(img_path, target_size=(240,240), color_mode = 'rgb', interpolation = 'nearest')
x = image.img_to_array(img)
x = np.expand_dims(x, axis=0)
end = time.time()
print(end - start)

start = time.time()
img = cv2.imread(img_path)
img = cv2.resize(img, (240,240), interpolation = cv2.INTER_AREA)
x = np.expand_dims(img, axis=0)
end = time.time()
print(end - start)


# In[7]:


feed_dict = {
    input_tensor_name: x
}
preds = tf_sess.run(output_tensor, feed_dict)

# decode the results into a list of tuples (class, description, probability)
# (one such list for each sample in the batch)
print('Predicted:', preds[0])


# In[10]:


import os
times = []
a = os.listdir('full/')
for i in a[:60]:
    start_time = time.time()
    img = cv2.imread('full/'+ i)
    img = cv2.resize(img, (240,240), interpolation = cv2.INTER_AREA)
    x = np.expand_dims(img, axis=0)
    feed_dict = {input_tensor_name:x}
    one_prediction = tf_sess.run(output_tensor, feed_dict)
    delta = (time.time() - start_time)
    times.append(delta)
mean_delta = np.array(times).mean()
fps = 1 / mean_delta
print('average(sec):{:.2f},fps:{:.2f}'.format(mean_delta, fps))


# In[12]:


import numpy as np 
import cv2
import time

import jetson.utils

height = 360
width = 640
input_size = 224
camera = '/dev/video0'

# create camera device
camera = jetson.utils.gstCamera(width, height, camera)

# create display windowdisplay = jetson.utils.glDisplay()
display = jetson.utils.glDisplay()

# open the camera for streaming
camera.Open()

# capture frames until user exits
while display.IsOpen():
    t1 = time.time()
    k = camera.CaptureRGBA(zeroCopy=1)
    image, width, height = k
    #img_arr = simple_cpu_image_hash(k)
    #outputs = rep.run(np.random.randn(10, 3, 224, 224).astype(np.float32))
    #img_arr =  simple_cpu_image_hash(k, size = 224)
    img_arr =  jetson.utils.cudaToNumpy(image, width, height, 4)
    rgbImage = cv2.cvtColor(img_arr, cv2.COLOR_RGBA2BGR)
     
    img = cv2.resize(rgbImage, (240,240), interpolation = cv2.INTER_AREA)
    #img = cv2.fastNlMeansDenoisingColored(img, None, 10, 10, 7,21)
    #img = cv2.fastNlMeansDenoisingColored(img,None,10,10,7,21)  
    img = img/255
    x = np.expand_dims(img, axis=0)
    
    feed_dict = {input_tensor_name:x}
    one_prediction = tf_sess.run(output_tensor, feed_dict)
   	        
    print(one_prediction)
    
      
    print("FPS: ", 1/(time.time()-t1))
    
    display.RenderOnce(image, width, height)
    display.SetTitle("{:s} | {:d}x{:d} | {:.0f} FPS".format("Camera Viewer", width, height, display.GetFPS()))


# In[ ]:




