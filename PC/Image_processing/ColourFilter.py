import cv2
import numpy as np
#=== Finding lanes stuff ===
# This=stuff was form some work done in 2025 in the lead up to behavioriual cloning

def make_coordinates(image, line_parameters): # function to convert slope and intercept into coordinates for drawing lines
    slope, intercept = line_parameters
    y1 = image.shape[0] # get the height of the image
    y2 = int(y1*(3/5)) # calculate the y-coordinate for the bottom of the line, here we take 3/5 of the height of the image
    x1 = int((y1-intercept)/slope) # calculate the x-coordinate for the start of the line using the slope and intercept
    x2 = int((y2-intercept)/slope) # calculate the x-coordinate for the end of the line using the slope and intercept
    return np.array ([x1, y1, x2, y2]) # return the coordinates as a numpy array

def average_slope_intercept(image, lines):
  left_fit = []
  right_fit =[]
  for line in lines:
     x1, y1, x2, y2 = line.reshape(4) # reshape the lines to get the start and end points of each line
     parameters = np.polyfit((x1, x2), (y1, y2), 1) # fit a line to the points using least squares polynomial fit this returns the slope and intercept of the line
     slope = parameters[0] # slope of the line, grabs the first element of the parameters array
     intercept = parameters[1] # intercept of the line, grabs the second element of the parameters array
     if slope < 0:
        left_fit.append((slope, intercept))
     else:
        right_fit.append((slope, intercept))
  left_fit_average = np.average(left_fit, axis = 0) # average the slope and intercept of the left lines
  right_fit_average = np.average(right_fit, axis = 0) # average the slope and intercept of the right lines
  left_line = make_coordinates(image, left_fit_average) # convert the slope and intercept of the left line to coordinates
  right_line = make_coordinates(image, right_fit_average) # convert the slope and intercept of the right line to coordinates
  return np.array([left_line, right_line])

def canny(image): # function to apply Canny edge detection
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) # convert the image to grayscale
    blur = cv2.GaussianBlur(gray, (5, 5), 0) # apply Gaussian blur to reduce noise and improve edge detection
    canny = cv2.Canny(blur, 50, 150) # apply Canny edge detection to find edges in the image
    return canny

def display_lines(image, lines): # function to draw lines on the image
    line_image = np.zeros_like(image) # create a blank image with the same shape as the input image to draw lines on
    if lines is not None: # check if lines are detected
        for x1, y1, x2, y2 in lines: # iterate through each line
            cv2.line(line_image, (x1, y1), (x2, y2), (255, 0, 0), 10) # 2nd parameter is the start point of the line, 3rd parameter is the end point of the line
    return line_image

def region_of_interest(image): # function to define a region of interest in the image
    height = image.shape[0]
    polygons = np.array([
        [(200,height), (1100, height), (550, 250)]]) #1st horizontal vert, 2nd horizontal vert, top vert. here we deinfe thr traingle to be an array of oonly one polygon 
    mask = np.zeros_like(image) # create a mask with the same shape as the image
    cv2.fillPoly(mask, polygons, 255) # fill the triangle area in the mask with white
    masked_image = cv2.bitwise_and( image, mask) # does a bnitwise AND operation between the image and the mask, keeping only the area inside the triangle
    return masked_image


# === img processing ===
def img_preprocess(image,blackWhite=False, lanes = False): #pre-process our data to be used inside our model
    image = image[60:,: ] #crops out the parts of the image that isnt in the range of 60:135, hence keeping only the road 
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) #converts the image from RGB to HSV colour space, which is more suitable for colour detection

    # === Yellow mask - allow lower saturation for reflections/distance ===
    lower_yellow = np.array([18, 80, 80])   # track 18, 50, 50
    upper_yellow  = np.array([35, 255, 255])
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # Blue mask - same approach
    lower_blue = np.array([95, 80, 80]) #track 95, 50, 50
    upper_blue = np.array([145, 255, 255])
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # ===Combine masks ===
    mask = cv2.bitwise_or(yellow_mask, blue_mask) #combines the two masks to create a single mask that highlights both yellow and blue lines

    # Morphological closing to close the gaps between the lines
    kernel = np.ones((10,10), np.uint8)#use bigger kernel to connect bigger gaps
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # === the option of white lines on black screen or raw yellow & blue on black ===
    if blackWhite:
        processed_img = np.stack([mask, mask, mask], axis = -1)

    else:
        # Apply mask to original image (show only yellow and blue lines)
        processed_img = cv2.bitwise_and(image, image, mask=mask)
    
    if lanes:
        # Note: the mask is smaller than the original image because we cropped `image_copy = image[60:,:]`
        cropped_for_lines = region_of_interest(mask)
        # Find lines on the single channel mask
        lines = cv2.HoughLinesP(cropped_for_lines, 2, np.pi/180, 100, np.array([]), minLineLength=40, maxLineGap=5)
        
        if lines is not None:
            # We average them based on the shape of our current cropped image
            averaged_lines = average_slope_intercept(processed_img, lines)
            line_img = display_lines(processed_img, averaged_lines)
            processed_img = cv2.addWeighted(processed_img, 0.8, line_img, 1, 1)


    processed_img = cv2.GaussianBlur(processed_img, (3,3), 0)# smoothens the iamge and reduces noise. It works by using convultion
    processed_img = cv2.resize(processed_img, (200, 66)) #reduces computational costs and is also used as the input size in teh nivida model
    processed_img = processed_img/255.0 #normalises the image

    if len(processed_img.shape) == 2:
        processed_img = np.stack((processed_img,)*3, axis = -1)
    return processed_img



#=== just for testing ===
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
