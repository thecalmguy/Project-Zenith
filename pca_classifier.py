import numpy as np
import cv2
import os
from sklearn.decomposition import PCA
import matplotlib.pyplot as plt

def main():
    path = os.listdir("cropped1")
    data = []
    labels = []
    data0 = []
    data1 = []
    for i in path:
        #Load image
        img = cv2.imread("cropped1/" + i)
        print(i)
        #Resize
        # res_img = cv2.resize(img, (48,48), interpolation=cv2.INTER_AREA)
        #Change color space
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        #Flatten
        flatten_img = np.reshape(hsv, 48*48*3)
        #Store data
        data.append(flatten_img)
        try:
            name_to_int = int(i.split(".")[0])
            # print(name)
            if name_to_int < 1000:
                labels.append(1)
                data1.append(flatten_img)
        except:
            labels.append(0)
            data0.append(flatten_img)
    #Fit PCA
    # print(data)
    pca = PCA(n_components=2)
    prin_components_0 = pca.fit_transform(data0)
    prin_components_1 = pca.fit_transform(data1)
    #plot
    # print(prin_components[1])
    fig = plt.figure(figsize = (8,8))
    ax = fig.add_subplot(1,1,1)
    ax.set_xlabel('Principal Component 1', fontsize = 15)
    ax.set_ylabel('Principal Component 2', fontsize = 15)
    ax.set_title('2 component PCA', fontsize = 20)
    length0 = len(prin_components_0)
    length1 = len(prin_components_1)
    for i in range(max([length0, length1])):
        if i < length0:
            ax.scatter(prin_components_0[i][0], prin_components_0[i][1], c = "r")
        if i < length1:
            ax.scatter(prin_components_1[i][0], prin_components_1[i][1], c = "b")
        # plt.hold()
    # plt.plot(prin_components)
    ax.grid()
    plt.show()

if __name__ == "__main__":
    main()
