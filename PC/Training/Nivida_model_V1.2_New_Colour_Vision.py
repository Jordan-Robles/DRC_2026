import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import keras
from keras.models import Sequential
from keras.optimizers import Adam
from keras.layers import Convolution2D, MaxPooling2D, Dropout, Flatten, Dense
from sklearn.utils import shuffle
from sklearn.model_selection import train_test_split # used ofr training and validation split
from imgaug import augmenters as iaa #Image Aug
import cv2
import pandas as pd
import ntpath #used to delete the path 
import random


#===read the data with pandas===
datadir = r'C:\Users\jorda\DRC\Testing_Data\Test45'
csv_path = os.path.join(datadir, 'labels.csv')

data = pd.read_csv(csv_path)

data = data[['image', 'mapped_value']]
data.rename(columns={'image': 'centre', 'mapped_value': 'steering'}, inplace=True)

#===Normilse (0 to 100) to (-1 to 1)===
data['steering'] = data['steering'].astype(float)
data['steering'] = (data['steering'] - 50) / 50

pd.set_option('display.max_colwidth', None) #this makes sure thet the columns are full represented instead of C:\user\...



def path_leaf(path):
    head, tail = ntpath.split(path)
    return tail
data['centre'] = data['centre'].apply(path_leaf) #Deals with removing the print out of the path

data['centre'] = data['centre'].apply(path_leaf)

#==Remove this as we dont have three cameras running at the moment

#data['left'] = data['left'].apply(path_leaf)
#data['right'] = data['right'].apply(path_leaf)
print(data.head()) #prints out the first 5 lines of the csv



####This code is used to ensure our data is not bias on left or right turns, hence we centre the data####

#for every set of iamges, we are going to plot the steering angles on a histogram to visualise the distribution so we know which one is more frequent
num_bins = 25 #make sure it is odd number as we want a centred distribution
samples_per_bin = 400 # 400, ensures a max value per bin of 200 to ensure the model isnt bias towards driving straight
hist, bins = np.histogram(data['steering'], num_bins) #divides the steering data into 25 equally spaced bins
center = (bins[:-1]+ bins[1:]) *0.5 #instead of having -0.04 and 0.04 as the middle, we centre it at 0 by adding each element.
plt.figure()
plt.bar(center,hist,width = 0.05)
plt.plot((np.min(data['steering']), np.max(data['steering'])), (samples_per_bin, samples_per_bin)) #plots a line spans over the min to max at 200 to show a uniformly distributed data
print(bins)

#for each bin, we collect indices of all smaples whoose sterring angles falls within that bin
print('total data:', len(data))
remove_list = []#removes samples
for j in range(num_bins): #iterates through bins
    list_ =[]
    for i in range(len(data['steering'])): #collects indices of all samples whoose steering anlge falls within current bin.
        if data['steering'][i] >= bins[j] and data['steering'][i] <= bins[j+1]:
            list_.append(i)
    list_ = shuffle(list_) #shuffles these indices to randomize which samples are kept/removed.
    list_ = list_[samples_per_bin:] #keeps indices above 200 for removal
    remove_list.extend(list_) #removes the part of the list
print('removed', len(remove_list))
data.drop(data.index[remove_list], inplace=True ) #removes the data above 200 from the data frame
print('remaining:', len(data))

#we cna take a seperate recoding of the car constanly steering back from the sides
#we have to only record when we are steering back towards the middle after going off track

hist, _ =np.histogram(data['steering'], (num_bins))
plt.figure()
plt.bar(center,hist,width = 0.05)
plt.plot((np.min(data['steering']), np.max(data['steering'])), (samples_per_bin, samples_per_bin)) #plots a line spans over the min to max at 200 to show a uniformly distributed data



#=== here we sort the data from teh csv to the corresponding turns and split the data into training and validation sets

#Taking each of the centre images and coresponding steering angle and creates 2 arrays, one for images and one for both image and assigned angle
print(data.iloc[1])
def load_img_steering(datadir, df):
    image_path = []
    steering = []
    for i in range(len(df)):
        indexed_data = df.iloc[i]
        center = indexed_data['centre']
        angle = indexed_data['steering']
        full_path = os.path.join(datadir, center.strip())

        if os.path.exists(full_path):
            image_path.append(full_path)
            steering.append(float(angle))
        else:
            print(f"[Missing] Skipping: {full_path}")
    
    return np.asarray(image_path), np.asarray(steering)

image_paths, steerings = load_img_steering(r'C:\Users\jorda\DRC\Testing_Data\Test45', data) # two arrays, 1 for images and other for array conataining each images corresponding steering angle

X_train, X_valid, Y_train, Y_valid = train_test_split(image_paths, steerings, test_size = 0.2, random_state=6) #splits the data into training and validation 
print('Training Samples: {}\nValid Samples: {}'. format(len(X_train), len(X_valid)))

plt.figure() # we plot both the training and validation set to see as to how uniform each split is to ensure that none of them are bias
fig, axis = plt.subplots(1,2,figsize=(12,4))
axis[0].hist(Y_train, bins=num_bins, width=0.05, color = 'blue')
axis[0].set_title('Training set')
axis[1].hist(Y_valid, bins=num_bins, width=0.05, color = 'red')
axis[1].set_title('Validation set')



#===Img Augmentation===

def zoom(image): #Zoom
    zoom = iaa.Affine(scale=(1, 1.3)) # affine refers to a category of image augmentaion that perserves straight lines and planes with the object, 1st argument is current zoom, 2n is how much we are zomming in (30%)
    image = zoom.augment_image(image) #apply zoom to image
    return image

def pan(image): # Translates the image
   pan = iaa.Affine(translate_percent= {"x" : (-0.1, 0.1), "y" : (-0.1, 0.1)})
   image = pan.augment_image(image)
   return image

def image_random_brightness(image):
    brightness = iaa.Multiply((0.2, 1.2))# multiplys all pixel intensities inside image by the defiend range of teh arguments
    image = brightness.augment_image(image)
    return image

def image_random_flip(image, steering_angle):
    image = cv2.flip(image, 1) #flips the image horizontally
    steering_angle = -steering_angle #steering anlge has to match the flipped image
    return image, steering_angle

def random_augment(image, steering_angle):
    image = mpimg.imread(image)
    if np.random.rand() < 0.5: # image is selected 50% of the time 
        image = pan(image)
    if np.random.rand() < 0.5: 
        image = zoom(image)        
    if np.random.rand() < 0.5: 
        image = image_random_brightness(image)
    if np.random.rand() < 0.5: 
        image, steering_angle = image_random_flip(image, steering_angle)
    return image, steering_angle




#===Here we preporcess the images to the nividas specified YUV format

#will need to replace the funciton with code that reads the csv file gnereated from the gui

def img_preprocess(img): #pre-process our data to be used inside our model
    img = img[60:135,: ] #crops out the parts of the image that isnt in the range of 60:135, hence keeping only the road 
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV) #converts the image from RGB to HSV colour space, which is more suitable for colour detection

    # Yellow mask
    lower_yellow = np.array([15, 80, 80]) #lower bound for yellow in YUV
    upper_yellow  = np.array([255, 255, 0]) #creates a mask for the yellow lines on the road
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    # Blue mask
    lower_blue = np.array([90, 80, 80])
    upper_blue = np.array([130, 255, 255])
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

    #Combine masks
    mask = cv2.bitwise_or(yellow_mask, blue_mask) #combines the two masks to create a single mask that highlights both yellow and blue lines

    # Apply mask to original image (show only yellow and blue lines)
    img = cv2.bitwise_and(img, img, mask=mask)

    img = yellow_mask + blue_mask
    img = cv2.GaussianBlur(img, (3,3), 0)# smoothens the iamge and reduces noise. It works by using convultion
    img = cv2.resize(img, (200, 66)) #reduces computational costs and is also used as the input size in teh nivida model
    img = img/255 #normalises the image

    if len(img.shape) == 2:
        img = np.stack((img,)*3, axis = -1)
    return img

#image = image_paths[r'C:\Users\jorda\DRC\Testing_Data\Test5\centre_20250703_154513_666341.jpg'] #selects the image x to he used
image = r'C:\Users\jorda\DRC\Testing_Data\Test5\centre_20250703_154513_666341.jpg'
original_image = mpimg.imread(image)
preprocessed_image = img_preprocess(original_image)

fig, axis = plt.subplots(1,2, figsize = (15,10)) #creates the a figure with both original image and pre-processed next to each other
fig.tight_layout() #ensures the image is formated and that the axis dont overlap
axis[0].imshow(original_image) #first image
axis[0].set_title('original_image')
axis[1].imshow(preprocessed_image) #second iamge
axis[1].set_title('preprocessed_image')

#===Batch_generator===

def batch_generator(image_paths, steering_angle, batch_size, istraining): #last argument is to ensure we dont feed any validation data into our generator, as we dont want to validated our model on augmented images
    while True:
        batch_img = []
        batch_steering = []

        for i in range(batch_size):
            random_index = random.randint(0, len(image_paths)-1)
            if istraining:
                im, steering = random_augment(image_paths[random_index], steering_angle[random_index])
            else:
                im = mpimg.imread(image_paths[random_index])# reads an iamge file from the path specified by img and loads it into a NumPy array (pixel data)
                steering = steering_angle[random_index]

            im = img_preprocess(im)
            batch_img.append(im)
            batch_steering.append(steering)
        yield (np.array(batch_img), np.asarray(batch_steering))

#===Fit Generator===
#requests data from the batch_generator
x_train_gen, y_train_gen = next(batch_generator(X_train, Y_train, 1, 1)) #calls the batch generator to give us x images denoted by the third argument
x_valid_gen, y_valid_gen = next(batch_generator(X_valid, Y_valid, 1, 0))

plt.figure()
fig, axes = plt.subplots(1, 2, figsize = (15, 10))
fig.tight_layout()

axis[0].imshow(x_train_gen[0])
axis[0].set_title('Training Image')

axis[1].imshow(x_valid_gen[0]) #second iamge
axis[1].set_title('Validation Image')
plt.show()



#===Nvidia_model===

def nvidia_model(): #uses 200x66 images
    #Note: look out for dead neurons, if a node gets the input of a negative number, it will return a zero
    model = Sequential()
    model.add(Convolution2D(24, (5, 5), strides=(2,2), input_shape=(66,200,3), activation = 'elu' ))#1st argument is the number of filters and the 2n&3rd are the dimensions of the kernal
    #subsample(2,2), it will move 2 pixels across and 2 pixels vertical
    model.add(Convolution2D(36, (5, 5), strides =(2,2), activation = 'elu'))
    
    #model.add(Dropout(0.5)) #extra dropout layers added to solve the problem of the data overfitting 

    model.add(Convolution2D(48, (5, 5), strides =(2,2), activation = 'elu'))

    #model.add(Dropout(0.5)) #extra dropout layers added to solve the problem of the data overfitting 

    model.add(Convolution2D(64, (3, 3), activation = 'elu')) # we take away the subsampling as the image has been propocessed enough so we stick to a stride lenght of 1
    model.add(Convolution2D(64, (3, 3), activation = 'elu'))
    #model.add(Dropout(0.4)) #Turns in the random inputs they recieve into 0 (50% in this case). Helps with overfitting data

    model.add(Flatten()) #flattens the data to a 1D array to pass throguht the fully connected layer
    model.add(Dense(100, activation ='elu'))
    #model.add(Dropout(0.4))
    model.add(Dense(50, activation ='elu'))
    model.add(Dense(10, activation ='elu'))
    model.add(Dense(1))

    optimizer = Adam(learning_rate = 1e-3) #lower learning rate to reduce jagged (noise) training and validation loss curve
    model.compile(loss = 'mse', optimizer = optimizer)
    return model

model = nvidia_model()
print(model.summary())

history = model.fit(batch_generator(X_train, Y_train, 100, 1), 
                              steps_per_epoch =300, 
                              epochs = 10, 
                              validation_data = batch_generator(X_valid, Y_valid, 100, 0), 
                              validation_steps=200, 
                              verbose = 1, 
                              shuffle = 1)


plt.figure()
plt.plot(history.history['loss'])
plt.plot(history.history['val_loss'])
plt.legend(['training', 'validation'])
plt.title('Loss')
plt.xlabel('Epoch')

plt.show()

model.save('model_DRC_V2026.h5')