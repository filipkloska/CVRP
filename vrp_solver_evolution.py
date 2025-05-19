import random
import numpy as np
import copy
import math

class VRPSolverRMEA:
    def __init__(self, instance):
        self.instance = instance
        self.best_solution = None
        self.best_cost = float('inf')
        self.routes = []
        self.neighbour_radius = instance.calculate_radius()
        self.population_size = 100
        self.max_generations = 30000
        self.no_improve_limit = 500
        self.relevance_matrix = None
        #matrix containing the times where node i j were adjacent to each other 
        self.adjancency_count_matrix = np.zeros((len(self.instance.distance_matrix),len(self.instance.distance_matrix)))
        #matrix contating the first time where node i j became adjacent to each other
        self.adjancency_time_matrix = np.ones((len(self.instance.distance_matrix),len(self.instance.distance_matrix)))
        self.current_generation = 0
        self.previous_alpha = 0
        print(len(self.instance.coords))

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
                total_distance += self.instance.distance_matrix[route[i]][route[i + 1]]
        return total_distance

    def build_relevance_matrix(self, population):
        N = len(self.instance.distance_matrix)
        dist_max = self.instance.find_max_distance()
        radius = self.neighbour_radius
        coords = self.instance.coords
        matrix = np.zeros((N, N))
        for i in range(N):
            for j in range(N):
                x_i, y_i= coords[i+1]  
                x_j, y_j = coords[j+1]
                center_x = (x_i + x_j) / 2
                center_y = (y_i + y_j) / 2
                dist_ij = self.instance.distance_matrix[i][j]
                print(f"Dist ij: {dist_ij}")
                relevance = 0
                M = 0
                for k in range(N):
                    if k == i or k == j:
                        continue

                    x_k, y_k = coords[k+1]
                    dist_to_center = math.hypot(x_k - center_x, y_k - center_y)
                    if dist_to_center <= radius:
                        M += 1
                
                alpha = self.previous_alpha + (self.current_generation/self.max_generations)
                self.previous_alpha = alpha

                relevance = ((1 - alpha) * (N - M)) + ((1 - alpha) * (dist_max - dist_ij)) + (alpha * (self.adjancency_count_matrix[i][j] + 1/self.adjancency_time_matrix[i][j]))

                matrix[i][j] = matrix[j][i] = relevance
                print(f"Relevance for node {i} and {j}: {relevance}")        
        
        return matrix

    def crossover(self, parent1, parent2):
        child = []
        customers = set(range(1, len(self.instance.demands)))
        used = set()
        vehicle_capacity = self.instance.capacity

        def get_next_customer(last, available):
            if not available:
                return None
            probs = [(c, self.relevance_matrix[last][c]) for c in available]
            total = sum(p for _, p in probs)
            if total == 0:
                return random.choice(list(available))
            r = random.uniform(0, total)
            s = 0
            for c, p in probs:
                s += p
                if s >= r:
                    return c
            return random.choice(list(available))

        while customers:
            route = [0]
            load = 0
            current = 0
            while True:
                next_customer = get_next_customer(current, customers - used)
                if next_customer is None or load + self.instance.demands[next_customer] > vehicle_capacity:
                    break
                route.append(next_customer)
                load += self.instance.demands[next_customer]
                used.add(next_customer)
                current = next_customer
            route.append(0)
            child.append(route)
            customers -= used
        return child

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
        no_improve = 0

        for generation in range(self.max_generations):
            self.build_relevance_matrix(population[0])
            population.sort(key=self.evaluate)
            current_best = self.evaluate(population[0])
            if current_best < self.best_cost:
                self.best_cost = current_best
                self.best_solution = copy.deepcopy(population[0])
                no_improve = 0
            else:
                no_improve += 1

            if generation % 10 == 0 or generation == self.max_generations - 1:
                print(f"Epoka {generation}: najlepszy koszt = {self.best_cost:.2f}")
            if no_improve >= self.no_improve_limit:
                break

            self.relevance_matrix = self.build_relevance_matrix(population[:10])
            new_population = population[:10]

            while len(new_population) < self.population_size:
                p1, p2 = random.sample(population[:15], 2)
                child = self.crossover(p1, p2)
                child = self.mutate(child)
                child = self.local_search(child)
                new_population.append(child)

            population = new_population
            self.current_generation += 1

        self.routes = self.best_solution

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