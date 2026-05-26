import cv2
import numpy as np
#

def img_preprocess(img,blackWhite=False): #pre-process our data to be used inside our model
    img = img[60:,: ] #crops out the parts of the image that isnt in the range of 60:135, hence keeping only the road

    img = cv2.GaussianBlur(img, (5,5), 0)  
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) #converts the image from RGB to HSV colour space, which is more suitable for colour detection

    # # Yellow mask
    # lower_yellow = np.array([20, 100, 100]) #lower bound for yellow in YUV
    # upper_yellow  = np.array([40, 255, 255]) #creates a mask for the yellow lines on the road
    # yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    # # Blue mask
    # lower_blue = np.array([100, 100, 100])
    # upper_blue = np.array([140, 255, 255])
    # blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # Yellow mask - allow lower saturation for reflections/distance
    lower_yellow = np.array([18, 50, 50])   # Much lower S and V
    upper_yellow  = np.array([35, 255, 255])
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # Blue mask - same approach
    lower_blue = np.array([95, 50, 50])
    upper_blue = np.array([145, 255, 255])
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

    #Combine masks
    mask = cv2.bitwise_or(yellow_mask, blue_mask) #combines the two masks to create a single mask that highlights both yellow and blue lines

    # noise reduction reommended by copilot
    noise_kernel = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, noise_kernel)

    # Morphological closing to close the gaps between the lines
    kernel = np.ones((12,12), np.uint8)#use bigger kernel to connect bigger gaps
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    if blackWhite:
        img = np.stack([mask, mask, mask], axis = -1)


    else:
        # Apply mask to original image (show only yellow and blue lines)
        img = cv2.bitwise_and(img, img, mask=mask)
    

    #img = cv2.GaussianBlur(img, (3,3), 0)# smoothens the iamge and reduces noise. It works by using convultion
    img = cv2.resize(img, (200, 66)) #reduces computational costs and is also used as the input size in teh nivida model
    img = img/255.0 #normalises the image

    if len(img.shape) == 2:
        img = np.stack((img,)*3, axis = -1)
    return img

#just for testing
if __name__=="__main__":
    
    cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
    while True:
        ret, frame = cap.read()
        if not ret:
            print("failed")
            break
        processed_img = img_preprocess(frame,blackWhite=True)
        cv2.imshow("Prosses image", processed_img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()
