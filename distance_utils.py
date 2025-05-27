import math

def euclidean_distance(a, b, dtype=int):
    return dtype(round(math.hypot(a[0] - b[0], a[1] - b[1]),3))

def compute_distance_matrix(coords, dtype=int):
    nodes = list(coords.keys())
    return [
        [euclidean_distance(coords[i], coords[j], dtype) for j in nodes]
        for i in nodes
    ]
