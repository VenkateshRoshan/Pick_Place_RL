import numpy as np

def euclidean_distance(a, b):
    """
    Calculate the Euclidean distance between two points.
    """
    return np.linalg.norm(a - b)