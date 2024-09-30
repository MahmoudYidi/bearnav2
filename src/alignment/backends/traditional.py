#!/usr/bin/env python3

import cv2
import logging

# Configure logging
logging.basicConfig(level=logging.WARNING)
logger = logging.getLogger(__name__)

bf = cv2.BFMatcher()
featureTypes = {}

# SIFT
try:
    featureTypes["SIFT"] = cv2.SIFT_create()
except:
    logger.warning("SIFT feature detector not available.")
try:
    featureTypes["SIFT"] = cv2.xfeatures2d.SIFT_create()
except:
    logger.warning("SIFT feature detector not available in xfeatures2d.")

# SURF
try:
    featureTypes["SURF"] = cv2.SURF_create()
except:
    logger.warning("SURF feature detector not available.")
try:
    featureTypes["SURF"] = cv2.xfeatures2d.SURF_create()
except:
    logger.warning("SURF feature detector not available in xfeatures2d.")

# KAZE
try:
    featureTypes["KAZE"] = cv2.KAZE_create()
except:
    logger.warning("KAZE feature detector not available.")

# AKAZE
try:
    featureTypes["AKAZE"] = cv2.AKAZE_create()
except:
    logger.warning("AKAZE feature detector not available.")

# BRISK
try:
    featureTypes["BRISK"] = cv2.BRISK_create()
except:
    logger.warning("BRISK feature detector not available.")

# ORB
try:
    featureTypes["ORB"] = cv2.ORB_create()
except:
    logger.warning("ORB feature detector not available.")

def detect(img, featureType):
    if featureType not in featureTypes:
        logger.warning("Feature type unknown or unavailable on this machine/installation. Not correcting heading!")
        return None, None

    kps, des = featureTypes[featureType].detectAndCompute(img, None)
    return kps, des

def match(kpsA, desA, kpsB, desB):
    matches = bf.match(desA, desB)
    
    displacements = []
    for match in matches:
        xA = kpsA[match.queryIdx].pt[0]
        xB = kpsB[match.trainIdx].pt[0]
        dist = xB - xA
        displacements.append(dist)

    return displacements
