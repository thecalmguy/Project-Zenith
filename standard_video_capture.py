import cv2 
import numpy as np 

def show(img):
	cv2.imshow('FRAME', img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()
print('started code')

#read from camera
cap = cv2.VideoCapture('convert.mp4')
 
# Check if camera opened successfully
if (cap.isOpened()== False): 
  print("Error opening video stream or file")

counter = 1
 
# Read until video is completed or the camera malfunctions!
while(cap.isOpened()):
  # Capture frame-by-frame
  ret, frame = cap.read()
  if ret == True:
 
    # Display the resulting frame
    cv2.imshow('Frame',frame)
 
    # Press Q on keyboard to  exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
      break
    
    stri = str(counter) + '.jpg'
	
    cv2.imwrite(stri,frame)
    counter+=1
  # Break the loop
  else: 
    break
 
# When everything done, release the video capture object
cap.release()
 
# Closes all the frames
cv2.destroyAllWindows()
