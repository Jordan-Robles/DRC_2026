import cv2

#This is soley to test if the camera is working with device
for i in range(5):
    cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
    ret, frame = cap.read()
    print(f"Camera {i}: {'Success' if ret else 'Failed'}")
    cap.release()