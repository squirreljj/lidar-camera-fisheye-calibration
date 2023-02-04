
import os
import cv2
import numpy
import numpy as np
rotation_vector=numpy.array([[0.104],[-0.060],[-3.242]])
rotM = cv2.Rodrigues(rotation_vector)[0]
print(rotM)