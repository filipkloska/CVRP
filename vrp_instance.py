import tsplib95
import urllib.request
import os
import numpy as np
from distance_utils import euclidean_distance
from distance_utils import compute_distance_matrix

class VRPInstance:
    def __init__(self, path_or_url):
        if path_or_url.startswith("http"):
            self.filename = path_or_url.split("/")[-1]
            urllib.request.urlretrieve(path_or_url, self.filename)
        else:
            self.filename = path_or_url

        if not os.path.exists(self.filename):
            raise FileNotFoundError(f"Plik '{self.filename}' nie istnieje.")

        self.problem = tsplib95.load(self.filename)
        self.nodes = list(self.problem.get_nodes())
        self.coords = {i: self.problem.node_coords[i] for i in self.nodes}
        self.demands = [self.problem.demands[i] for i in self.nodes]
        self.depot = list(self.problem.depots)[0] - 1
        self.capacity = self.problem.capacity
        self.num_vehicles = self._get_vehicle_count_from_filename()
        self.distance_matrix = compute_distance_matrix(self.coords)

    def _get_vehicle_count_from_filename(self):
        if "-k" in self.filename:
            return int(self.filename.split("-k")[1].split(".")[0])
        return 1
    
    def calculate_radius(self):
        num_nodes = len(self.distance_matrix)
        average_distances = []

        for i in range(num_nodes):
            distances = []
            for j in range(num_nodes):
                if i != j:
                    distances.append(self.distance_matrix[i][j])
            average_distances.append(np.mean(distances))

        return np.mean(average_distances)

    def print_distance_matrix(self):
        for i in range(len(self.distance_matrix)):
            for j in range(len(self.distance_matrix[i])):
                print(f"Distance from {i} to {j}: {self.distance_matrix[i][j]}")
    
    def find_max_distance(self):
        max_dist = 0

        for i in range(len(self.distance_matrix)):
            for j in range(len(self.distance_matrix[i])):
                if self.distance_matrix[i][j] > max_dist:
                    max_dist = self.distance_matrix[i][j]
        return max_dist
    