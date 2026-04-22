import cv2
import numpy as np


def img_preprocess(img): #pre-process our data to be used inside our model
    img = img[60:135,: ] #crops out the parts of the image that isnt in the range of 60:135, hence keeping only the road 
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) #converts the image from RGB to HSV colour space, which is more suitable for colour detection

    # Yellow mask
    lower_yellow = np.array([20, 100, 100]) #lower bound for yellow in YUV
    upper_yellow  = np.array([40, 255, 255]) #creates a mask for the yellow lines on the road
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    # Blue mask
    lower_blue = np.array([100, 100, 100])
    upper_blue = np.array([140, 255, 255])
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

    #Combine masks
    mask = cv2.bitwise_or(yellow_mask, blue_mask) #combines the two masks to create a single mask that highlights both yellow and blue lines

    # Apply mask to original image (show only yellow and blue lines)
    img = cv2.bitwise_and(img, img, mask=mask)

    #img = yellow_mask + blue_mask
    img = cv2.GaussianBlur(img, (3,3), 0)# smoothens the iamge and reduces noise. It works by using convultion
    img = cv2.resize(img, (200, 66)) #reduces computational costs and is also used as the input size in teh nivida model
    img = img/255.0 #normalises the image

    if len(img.shape) == 2:
        img = np.stack((img,)*3, axis = -1)
    return img

#jsut for testing
if __name__=="__main__":
    
    cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
    while True:
        ret, frame = cap.read()
        if not ret:
            print("failed")
            break
        processed_img = img_preprocess(frame)
        cv2.imshow("Prosses image", processed_img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.release()
    cv2.destroyAllWindows()
