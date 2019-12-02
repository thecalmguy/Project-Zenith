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


# In[ ]:


from tensorflow.keras.preprocessing import image
import cv2
import numpy as np
start = time.time()
img_path = './boxes/100.jpg'
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


# In[ ]:


feed_dict = {
    input_tensor_name: x
}
preds = tf_sess.run(output_tensor, feed_dict)

# decode the results into a list of tuples (class, description, probability)
# (one such list for each sample in the batch)
print('Predicted:', preds[0])


# In[ ]:


import os
times = []
a = os.listdir('boxes/')
for i in a[:60]:
    start_time = time.time()
    img = cv2.imread('boxes/'+ i)
    img = cv2.resize(img, (240,240), interpolation = cv2.INTER_AREA)
    x = np.expand_dims(img, axis=0)
    feed_dict = {input_tensor_name:x}
    one_prediction = tf_sess.run(output_tensor, feed_dict)
    delta = (time.time() - start_time)
    times.append(delta)
mean_delta = np.array(times).mean()
fps = 1 / mean_delta
print('average(sec):{:.2f},fps:{:.2f}'.format(mean_delta, fps))


# In[4]:


import numpy as np 
import cv2
import time

cap = cv2.VideoCapture(0)
cap.set(3,320)
cap.set(4,240)
 
  
font  = cv2.FONT_HERSHEY_SIMPLEX
bottomLeftCornerOfText = (10,10)
fontScale              = 1
fontColor              = (255,255,255)
lineType               = 2
while(1):

  # Capture frame-by-frame
    ret, frame = cap.read()
    start = time.time()
    if ret == True:

        # Display the resulting frame
        img = cv2.resize(frame, (240,240), interpolation = cv2.INTER_AREA)
        img = cv2.fastNlMeansDenoisingColored(img, None, 10, 10, 7,21)
        x = np.expand_dims(img, axis=0)
        feed_dict = {input_tensor_name:x}
        one_prediction = tf_sess.run(output_tensor, feed_dict)
#         cv2.putText(img,one_prediction[0], 
#             bottomLeftCornerOfText, 
#             font, 
#             fontScale,
#             fontColor)
        # Press Q on keyboard to  exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
          break
        
        cv2.imshow('Frame',img)
        

    
    # Break the loop
    else: 
        print('ret was false')
        break
    end = time.time()
    print('fps:', 1/(end - start)) 
    for i in range(20):
        cap.read()
# When everything done, release the video capture object
cap.release()

# Closes all the frames
cv2.destroyAllWindows()


# In[ ]:




