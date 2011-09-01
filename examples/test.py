import pyublas
import Image
import numpy as np
import sys
import matplotlib.pyplot as plt

# to find the build library
sys.path.append("../lib")

from turbopixel_python import turbopixels

# load image
image = np.array(Image.open("example.jpg"))

# apply turbopixel algorithm
result = turbopixels(image, 1000)

# shuffle superpixel labels for better visualization
indices = np.arange(result.max()+1)
np.random.shuffle(indices)

# show original and superpixels
plt.matshow(indices[asdf])
plt.figure()
plt.imshow(image)
plt.show()
