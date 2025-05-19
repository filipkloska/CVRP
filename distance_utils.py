import math

def euclidean_distance(a, b):
    return int(math.hypot(a[0] - b[0], a[1] - b[1]))

def compute_distance_matrix(coords):
    nodes = list(coords.keys())
    return [
        [euclidean_distance(coords[i], coords[j]) for j in nodes]
        for i in nodes
    ]
