import random
import numpy as np
import copy
import math
from distance_utils import compute_distance_matrix
from collections import Counter
import rmea_utils as rmea

class VRPSolverRMEA:
    def __init__(self, instance):
        self.instance = instance
        self.distance_matrix = compute_distance_matrix(instance.coords,dtype=float)
        self.best_solution = None
        self.best_cost = float('inf')
        self.routes = []
        self.neighbour_radius = instance.calculate_radius()
        self.population_size = 100
        self.max_generations = 30000
        self.no_improve_limit = 500
        self.relevance_matrix = None
        #matrix containing the times where node i j were adjacent to each other 
        self.adjancency_count_matrix = np.zeros((len(self.distance_matrix),len(self.distance_matrix)))
        #matrix contating the first time where node i j became adjacent to each other
        self.adjancency_time_matrix = np.ones((len(self.distance_matrix),len(self.distance_matrix)))
        self.current_generation = 0
        self.previous_alpha = 0
        

    def initialize_population(self):
        population = []
        for _ in range(self.population_size):
            solution = self.random_solution()
            population.append(solution)
        return population

    def random_solution(self):
        customers = list(range(1, len(self.instance.demands)))
        random.shuffle(customers)

        routes = []
        vehicle_capacity = self.instance.capacity
        current_route = [0]
        current_load = 0

        for customer in customers:
            demand = self.instance.demands[customer]
            if current_load + demand <= vehicle_capacity:
                current_route.append(customer)
                current_load += demand
            else:
                current_route.append(0)
                routes.append(current_route)
                current_route = [0, customer]
                current_load = demand
        current_route.append(0)
        routes.append(current_route)
        return routes

    def evaluate(self, solution):
        total_distance = 0
        for route in solution:
            for i in range(len(route) - 1):
                total_distance += self.distance_matrix[route[i]][route[i + 1]]
        return total_distance

    def build_relevance_matrix(self):
        
        #not sure if it should be 29 or 30 - does this make a difference (i think it should)
        N = len(self.distance_matrix)
        dist_max = self.instance.find_max_distance()
        radius = self.neighbour_radius
        coords = self.instance.coords
        
        matrix = np.zeros((N, N),dtype=float)
        for i in range(1,N):
            for j in range(1,N):
                if i == j:
                    matrix[i][j] = matrix[j][i] = 0
                else:
                    x_i, y_i= coords[i+1]  
                    x_j, y_j = coords[j+1]
                    center_x = (x_i + x_j) / 2
                    center_y = (y_i + y_j) / 2
                    dist_ij = self.distance_matrix[i][j]
                    print(f"Dist ij: {dist_ij}")
                    relevance = 0
                    M = 0
                    for k in range(1,N):
                        if k == i or k == j:
                            continue

                        x_k, y_k = coords[k+1]
                        dist_to_center = math.hypot(x_k - center_x, y_k - center_y)
                        if dist_to_center <= radius:
                            M += 1
                    
                    alpha = self.previous_alpha + (self.current_generation/self.max_generations)
                    self.previous_alpha = alpha

                    relevance = ((1 - alpha) * (N - M)) + ((1 - alpha) * (dist_max - dist_ij)) + (alpha * (self.adjancency_count_matrix[i][j] + 1/self.adjancency_time_matrix[i][j]))
                    matrix[i][j] = matrix[j][i] = round(relevance,3)
                print(f"Relevance for node {i} and {j}: {matrix[i][j]}")        
        print(f"coords size: {len(coords)}")

        return matrix
    

    def crossover(self, parent1, parent2):

        customers = np.array(range(1, len(self.instance.demands)))
        p1 = rmea.solution_to_array(parent1)
        p2 = rmea.solution_to_array(parent2)


        #crossover point where we start dividing
        crossover_point_p1 = 0
        crossover_idx = 0
        
        while(crossover_point_p1 == 0):
            crossover_idx = np.random.randint(2,len(p1)-1)
            crossover_point_p1 = p1[crossover_idx]
        p1_split1, p1_split2 = rmea.split_solution(p1,crossover_idx)

        #highest relevancy point of crossover_point_p1
        rlv_p1 = np.where(self.relevance_matrix[crossover_point_p1] == np.max(self.relevance_matrix[crossover_point_p1]))[0][0]


        p2_split1, p2_split2 = rmea.split_solution(p2, np.where(p2 == rlv_p1)[0][0])
        
        m1 = np.concatenate((p1_split1,p2_split2))
        m2 = np.concatenate((p2_split1,p1_split2))
        m1, m_n1 = rmea.remove_duplicates(m1, self.relevance_matrix, customers)
        m2, m_n2 = rmea.remove_duplicates(m1, self.relevance_matrix, customers)
        m1, m_n1 = rmea.adjust_bad_routes(m1, m_n1, self.instance)
        m2, m_n2 = rmea.adjust_bad_routes(m2, m_n2, self.instance)
        c1 = rmea.add_missing_nodes(m1, m_n1, self.instance, self.relevance_matrix, customers)
        c2 = rmea.add_missing_nodes(m2, m_n2, self.instance, self.relevance_matrix, customers)
        return c1,c2

    def mutate(self, solution):
        new_solution = copy.deepcopy(solution)
        if len(new_solution) < 2:
            return new_solution
        r1, r2 = random.sample(range(len(new_solution)), 2)
        if len(new_solution[r1]) > 2 and len(new_solution[r2]) > 2:
            i1 = random.randint(1, len(new_solution[r1]) - 2)
            i2 = random.randint(1, len(new_solution[r2]) - 2)
            new_solution[r1][i1], new_solution[r2][i2] = new_solution[r2][i2], new_solution[r1][i1]
        return new_solution

    def local_search(self, solution):
        for route in solution:
            if len(route) > 3:
                i, j = random.sample(range(1, len(route) - 1), 2)
                route[i], route[j] = route[j], route[i]
        return solution

    def get_routes(self):
        return self.routes

    def solve(self):
        population = self.initialize_population()
        self.relevance_matrix = self.build_relevance_matrix()
        
        no_improve = 0
        for generation in range(self.max_generations):
            c1, c2 = self.crossover(population[0],population[1])


        #TODO: choose crossover strategy, mutation strategy, REMEMBER TO UPDATE THE TIME AND ADJENCENCY MATRICES
    def print_solution(self):
        if not self.routes:
            print("Nie znaleziono rozwiązania.")
            return

        total_distance = 0
        for i, route in enumerate(self.routes):
            distance = 0
            plan_output = f"Trasa pojazdu {i}:"
            for j in range(len(route) - 1):
                distance += self.instance.distance_matrix[route[j]][route[j + 1]]
                plan_output += f" {route[j]} ->"
            plan_output += f" {route[-1]}"
            plan_output += f"\n Długość trasy: {distance}"
            print(plan_output)
            total_distance += distance
        print(f"Łączna długość tras wszystkich pojazdów: {total_distance}")