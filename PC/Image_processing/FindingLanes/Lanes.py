import cv2
import numpy as np
#import matplotlib.pyplot as plt

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
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY) # convert the image to grayscale
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

#image = cv2.imread('test_image.jpg') # reads the image and returns its a multidimensional numpy array
#lane_image =np.copy(image)
#canny_image = canny(lane_image)
#cropped_image = region_of_interest(canny_image)
#lines = cv2.HoughLinesP(cropped_image, 2, np.pi/180, 100, np.array([]), minLineLength=40, maxLineGap=5) #2nd parameter is the resolution of the accumulator in pixels, 3rd parameter is the angle resolution in radians, 4th parameter is the threshold for the number of votes to consider a line
#averaged_lines = average_slope_intercept(lane_image, lines) # function to average the slope and intercept of the detected lines
#line_image = display_lines(lane_image, averaged_lines) # function to draw the averaged lines on a blank image
#combined_image = cv2.addWeighted(lane_image, 0.8, line_image, 1, 1) # combine the original image with the line image
#cv2.imshow("result", combined_image) # display the image with the detected lines
#cv2.waitKey(0)

cap = cv2.VideoCapture("test2.mp4") # open the video file
while(cap.isOpened()): # check if the video file is opened
    _, frame = cap.read()
    canny_image = canny(frame)
    cropped_image = region_of_interest(canny_image)
    lines = cv2.HoughLinesP(cropped_image, 2, np.pi/180, 100, np.array([]), minLineLength=40, maxLineGap=5) #2nd parameter is the resolution of the accumulator in pixels, 3rd parameter is the angle resolution in radians, 4th parameter is the threshold for the number of votes to consider a line
    averaged_lines = average_slope_intercept(frame, lines) # function to average the slope and intercept of the detected lines
    line_image = display_lines(frame, averaged_lines) # function to draw the averaged lines on a blank image
    combined_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1) # combine the original image with the line image
    cv2.imshow("result", combined_image) # display the image with the detected lines
    if cv2.waitKey(1) == ord('q'): # wait for 1 millisecond before displaying the next frame
        break
cap.release() # release the video capture object
cv2.destroyAllWindows() # close all OpenCV windows