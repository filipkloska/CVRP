import math
SCALE = 1000

def euclidean_distance(a, b):
    return int(math.hypot(a[0] - b[0], a[1] - b[1]))

def compute_distance_matrix(coords):
    nodes = list(coords.keys())
    return [
        [
            euclidean_distance(
                (coords[i][0] * SCALE, coords[i][1] * SCALE),
                (coords[j][0] * SCALE, coords[j][1] * SCALE)
            )
            for j in nodes
        ]
        for i in nodes
    ]

