import numpy as np
import sys
import matplotlib.pyplot as plt
import os
import json

if len(sys.argv) < 2:
    print("Usage: python print_npy.py <path_to_npy>")
    sys.exit(1)

file_path = sys.argv[1]
data = np.load(file_path)
print(data)
print("Shape:", data.shape)
print("Dtype:", data.dtype)