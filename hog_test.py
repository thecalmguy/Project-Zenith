import matplotlib.pyplot as plt
from skimage.feature import hog
from skimage import data, exposure
import cv2
import os
from sklearn.linear_model import LogisticRegression
import numpy as np
import time

def main():
    path = os.listdir("cropped1")
    data0 = []
    data1 = []
    data = []
    labels = []
    total = 0
    # f_prime = np.array([16,44,37,62,22,28,66,2,40,14,11,54,42,30,36,51,67,27,33,32])
    # f_prime = np.array([16,44,37,28,66,40,22,2,33,11,32,48,35,14,39,56,62,53,30,27,54,51,68,71,31])
    f_prime = np.array([16,	44,	37,	28,	36,	66,	2,	22,	32,	48,	56,	14,	62,	42,	54,	30,	27,	31,	72,	24,	41,	25,	51,	55,	38]) - np.ones((1,25), int)
    f_prime =f_prime[0]
    for i in path:
        #Load image
        img = cv2.imread("cropped1/"+i)
        # print(i)
        #Resize image
        # res_img = cv2.resize(img, (48,48), interpolation=cv2.INTER_AREA)
        #Get HOG features
        a = time.time()
        fd = hog(img, orientations=8, pixels_per_cell=(16,16), cells_per_block=(1,1),
                 multichannel=True, block_norm="L2-Hys", feature_vector=True)
        # print(time.time() - a)
        total+=time.time() - a
        data.append(fd[f_prime])
        #Negative and positive classes
        if i == "2.jpg": print(fd)
        try:
            name_to_int = int(i.split(".")[0])
            # print(name)
            if name_to_int < 1000:
                labels.append(1)
        except:
            labels.append(0)
        # print(type(fd))

    model = LogisticRegression().fit(data,np.transpose(labels))
    pred = model.predict(data)
    print("Training Accuracy: " + str(model.score(data, np.transpose(labels))))
    tp = 0
    tn = 0
    fp = 0
    fn = 0
    for i in range(len(pred)):
        if pred[i] == labels[i]:
            if pred[i] == 1:
                tn += 1
            elif pred[i] == 0:
                tp += 1
        else:
            if pred[i] == 0 and labels[i] == 1:
                fp += 1
            elif pred[i] == 1 and labels[i] == 0:
                fn += 1
    recall = tp/(tp+fn)
    precision = tp/(tp+fp)
    f1 = 2* precision*recall/(precision+recall)
    print("Recall: " + str(recall))
    print("Precision: " + str(precision))
    print("F1 score: " + str(f1))
    print("TP: "+str(tp))
    print("FP: "+str(fp))
    print(data[43].shape)
    num = 50
    a = time.time()
    b = model.predict(np.expand_dims(data[num], axis=0))
    print("PRED TIME: " + str(time.time() - a))
    print("TIME: " + str(total/len(path)))
    print("Actual: {0}, pred: {1}".format(labels[num], b))

if __name__ == "__main__":
    main()
