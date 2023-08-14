import tensorflow as tf
import os
import cv2
import matplotlib.pyplot as plt
import numpy as np
import sklearn.model_selection as sk
import pandas as pd

def plot_img(data):
    plt.figure()
    plt.imshow(data, cmap='gray', vmin=0, vmax=255)
    # plt.axis('off')
    plt.show()
    
# # load data
# image_dataset_test = []
# labels_test = []
# datafolder = "D:/FTP/"
# allFileList = [datafolder+i for i in os.listdir(datafolder)]
# for filefolder in allFileList:
#     allFile = os.listdir(filefolder)
#     for file in allFile:
#         filename = filefolder+"/"+file
#         gray = cv2.imread(filename)
#         gray = gray[360:660, 580:880]
#         # plot_img(gray)
#         # image = cv2.resize(gray, (96, 96))
#         # plot_img(image)
#         # image = np.reshape(image, (96, 96, 1))
#         image = np.reshape(gray, (300, 300, 3))
#         image_dataset_test.append(image)
#         labels_test.append(int(file[:2]))
#         print(int(file[:2]))
## adjust label to LPG label
# labels[labels["hour"] == 0] = '50'
# labels[labels["hour"] == 1] = '65'
# labels[labels["hour"] == 2] = '75'
# labels[labels["hour"] == 3] = '85'
# labels[labels["hour"] == 4] = '92'
# labels[labels["hour"] == 5] = '95'
# labels[labels["hour"] == 6] = '0'
# labels[labels["hour"] == 7] = '5'
# labels[labels["hour"] == 8] = '8'
# labels[labels["hour"] == 9] = '15'
# labels[labels["hour"] == 10] = '25'
# labels[labels["hour"] == 11] = '35'

## load clock's data
image_dataset = []
datafolder = "D:/ESP32cam-git/archive/analog_clocks/images"
allFileList = [datafolder]
for filefolder in allFileList:
    allFile = os.listdir(filefolder)
    for file in allFile:
        filename = filefolder+"/"+file
        image = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)
        image = cv2.resize(image, (96, 96))
        image = np.reshape(image, (96, 96, 1))
        image_dataset.append(image)
# 將訓練數據轉換為NumPy數組
image_dataset = np.array(image_dataset)
labels = pd.read_csv("D:/ESP32cam-git/archive/analog_clocks/label.csv")
labels = np.array(labels["hour"])
X_train, X_test, y_train, y_test = sk.train_test_split(image_dataset,labels,test_size=0.33, random_state = 42)
# Create CNN Regression Model
model = tf.keras.Sequential()
# model.add(tf.keras.layers.Conv2D(128, kernel_size=(2,2), input_shape=X_train.shape[1:], activation="relu"))
# model.add(tf.keras.layers.MaxPool2D(pool_size=(2,2)))
model.add(tf.keras.layers.Conv2D(64, kernel_size=(2,2), input_shape=X_train.shape[1:], activation="relu"))
model.add(tf.keras.layers.MaxPool2D(pool_size=(2,2)))
model.add(tf.keras.layers.Conv2D(32, kernel_size=(2,2), activation="relu"))
# model.add(tf.keras.layers.GlobalAveragePooling2D())
model.add(tf.keras.layers.MaxPool2D())
model.add(tf.keras.layers.Flatten())
model.add(tf.keras.layers.Dense(12, activation="softmax"))
model.summary()
Nadam = tf.keras.optimizers.Nadam(learning_rate=0.002)
model.compile(loss="sparse_categorical_crossentropy", optimizer='adam', metrics=['accuracy'])
model.fit(x=X_train, y=y_train, 
    validation_data=(X_test, y_test),
    epochs=1, batch_size=128)
model.save("clock.h5")
# model.predict(image_dataset_test[0][np.newaxis,:])
# TensorFlow Lite Converter ( .h5 to .tflite )
# https://www.wpgdadatong.com/blog/detail/70552
converter = tf.compat.v1.lite.TFLiteConverter.from_keras_model_file( 'clock.h5' )
converter.optimizations = [tf.lite.Optimize.DEFAULT]
tflite_model = converter.convert()
with tf.io.gfile.GFile( "clock.tflite" , 'wb') as f:
   f.write(tflite_model)
print("tranfer done!!")
