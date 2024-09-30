import tensorflow 
from tensorflow import keras
from tensorflow.keras.applications import VGG16, VGG19
from tensorflow.keras.applications.vgg16 import preprocess_input
from tensorflow.keras.preprocessing.image import img_to_array
from tensorflow.keras.models import Model
import numpy as np
import cv2
import os

hideBottom = False
modelType = "vgg16"  # vgg16, vgg19
layer = "block3_pool"
square = False

print("Loading VGG")
basemodel = None
if modelType == "vgg16":
    basemodel = VGG16(weights="imagenet")
elif modelType == "vgg19":
    basemodel = VGG19(weights="imagenet")

# Create model to extract features from the specified layer
model = Model(inputs=basemodel.input, outputs=basemodel.get_layer(layer).output)
print("Finished loading model")

# Network comparison function
def getDiff(a, b):
    flatA = a.flat
    flatB = b.flat
    diff = np.subtract(flatA, flatB)
    diff = np.abs(diff)
    if square:
        diff = np.square(diff)
    diff = np.sum(diff)
    return diff

def eraseBottom(img):
    img[240:, :] = [0, 0, 0]

def align(baseimg, img):
    print("Begin nn align")
    
    # Process base image
    baseimgcrop = baseimg[:, 136:616]
    baseimgcrop = cv2.resize(baseimgcrop, (224, 224))
    baseimgcrop = img_to_array(baseimgcrop)
    baseimgcrop = np.expand_dims(baseimgcrop, axis=0)
    baseimgcrop = preprocess_input(baseimgcrop)
    baseimgdescriptor = model.predict(baseimgcrop)
    
    # Process img
    imgcrop = img[:, 136:616]
    imgcrop = cv2.resize(imgcrop, (224, 224))
    imgcrop = img_to_array(imgcrop)
    imgcrop = np.expand_dims(imgcrop, axis=0)
    imgcrop = preprocess_input(imgcrop)
    imgdescriptor = model.predict(imgcrop)

    bestOffset = -1
    bestOffsetValue = float('inf')
    offsetResults = []
    offsetValues = []

    for offset in range(0, 272):
        offset -= 136
        diff = getDiff(imgdescriptor, baseimgdescriptor)

        offsetValues.append(offset)
        offsetResults.append(diff)

        if diff < bestOffsetValue:
            bestOffsetValue = diff
            bestOffset = offset

    if bestOffset < -30 or bestOffset > 30:
        bestOffset = 0

    print("Best:", bestOffset, bestOffsetValue)

    return bestOffset, bestOffsetValue, offsetResults

def align_t(baseimg, img):
    if hideBottom:
        eraseBottom(baseimg)
    print("Begin nn align")
    
    baseimgcrop = baseimg[:, 136:616]
    baseimgcrop = cv2.resize(baseimgcrop, (224, 224))
    baseimgcrop = img_to_array(baseimgcrop)
    baseimgcrop = np.expand_dims(baseimgcrop, axis=0)
    baseimgcrop = preprocess_input(baseimgcrop)
    baseimgdescriptor = model.predict(baseimgcrop)

    if hideBottom:
        eraseBottom(img)

    bestOffset = -1
    bestOffsetValue = float('inf')
    offsetResults = []
    offsetValues = []

    for offset in range(100, 160):
        if offset % 5 != 0:
            continue
        
        imgcrop = img[:, offset:offset+480]
        imgcrop = cv2.resize(imgcrop, (224, 224))
        imgcrop = img_to_array(imgcrop)
        imgcrop = np.expand_dims(imgcrop, axis=0)
        imgcrop = preprocess_input(imgcrop)
        descriptor = model.predict(imgcrop)

        offset -= 136
        diff = getDiff(descriptor, baseimgdescriptor)

        offsetValues.append(offset)
        offsetResults.append(diff)

        if diff < bestOffsetValue:
            bestOffsetValue = diff
            bestOffset = offset

    print("Best:", bestOffset, bestOffsetValue)

    return bestOffset, bestOffsetValue, offsetResults
