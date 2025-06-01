import random
import numpy as np
import copy
import math
from distance_utils import compute_distance_matrix
from distance_utils import SCALE
from collections import Counter
import rmea_utils as rmea

class VRPSolverRMEA:
    def __init__(self, instance):
        self.instance = instance
        self.distance_matrix = compute_distance_matrix(instance.coords)
        self.best_solution = None
        self.best_cost = float('inf')
        self.routes = []
        self.neighbour_radius = instance.calculate_radius()
        self.mutation_prob = 0.2
        self.population_size = 100
        self.max_generations = 30000
        self.no_improve_soft_limit = 100
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

        solution = [0]
        vehicle_capacity = self.instance.capacity
        current_load = 0
        vehicle_count = 1

        for customer in customers:
            demand = self.instance.demands[customer]
            if current_load + demand <= vehicle_capacity:
                solution.append(customer)
                current_load += demand
            else:
                solution.append(0)
                vehicle_count += 1
                current_load = demand
                solution.append(customer)

        route_count = solution.count(0) - 1  # -1 bo pierwsze 0 to początek
        while route_count < self.instance.num_vehicles:
            solution.append(0)
            route_count += 1

        solution.append(0)  # końcowe 0
        return np.array(solution)
        
    def evaluate(self, solution):
        total_distance = 0
        for customer in range(len(solution) - 1):
            total_distance += self.distance_matrix[solution[customer]][solution[customer+1]]
        return total_distance

    def update_relevance_matrix(self,population):
        for solution in population:
            for i in range(len(solution) - 1):
                self.adjancency_count_matrix[solution[i]][solution[i+1]] += 1
                self.adjancency_count_matrix[solution[i+1]][solution[i]] += 1

                if self.adjancency_time_matrix[solution[i]][solution[i+1]] == 1 and self.current_generation != 0:
                    self.adjancency_time_matrix[solution[i]][solution[i+1]] = self.current_generation
                    self.adjancency_time_matrix[solution[i+1]][solution[i]] = self.current_generation

    def compute_scaled_fitness(self, population):
        fitness_values = np.array([self.evaluate(individual) for individual in population])
        max_fitness = fitness_values.max()
        scaled_fitness = max_fitness - fitness_values + 1e-6 
        return scaled_fitness


    def roulette_selection(self, population, scaled_fitness):
        probabilities = scaled_fitness / scaled_fitness.sum()
        selected_indices = np.random.choice(len(population), size=2, p=probabilities)
        return [population[i] for i in selected_indices]

    def build_relevance_matrix(self):
        
        #not sure if it should be 29 or 30 - does this make a difference (i think it should)
        N = len(self.distance_matrix)
        dist_max = self.instance.find_max_distance() / SCALE
        radius = self.neighbour_radius
        coords = self.instance.coords
        
        matrix = np.zeros((N, N),dtype=float)
        for i in range(1,N):
            x_i, y_i= coords[i+1] 
            for j in range(1,N):
                if i == j:
                    continue
                else:
                    x_j, y_j = coords[j+1]
                    center_x = (x_i + x_j) / 2
                    center_y = (y_i + y_j) / 2
                    dist_ij = self.distance_matrix[i][j] /SCALE
                    # print(f"Dist ij: {dist_ij}")
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
                    # print(f"1 - alpha: {1 - alpha}")
                    # print(f"N - M: {N - M}")
                    # print(f"dist_max - dist_ij: {dist_max - dist_ij}")
                    # print(f"self.adjancency_count_matrix[i][j] {self.adjancency_count_matrix[i][j]}")
                    # print(f"1/self.adjancency_time_matrix[i][j] {1/self.adjancency_time_matrix[i][j]}")
                    matrix[i][j] = matrix[j][i] = round(relevance,3)
                # print(f"Relevance for node {i} and {j}: {matrix[i][j]}")        
        # print(f"coords size: {len(coords)}")

        return matrix
    

    def crossover(self, parent1, parent2):

        customers = np.array(range(1, len(self.instance.demands)))
        #p1 = rmea.solution_to_array(parent1)
        #p2 = rmea.solution_to_array(parent2)
        p1 = parent1
        p2 = parent2

        # print(f"P1: {p1}")
        # print(f"P2: {p2}")
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

        #print(f"Demand for m1: {rmea.calculate_solution_demands(m1,self.instance)}")
        #print(f"Demand for m2: {rmea.calculate_solution_demands(m2,self.instance)}")

        m1, m_n1 = rmea.remove_duplicates(m1, self.relevance_matrix, customers)
        m2, m_n2 = rmea.remove_duplicates(m2, self.relevance_matrix, customers)
        m1, m_n1 = rmea.adjust_bad_routes(m1, m_n1, self.instance)
        m2, m_n2 = rmea.adjust_bad_routes(m2, m_n2, self.instance)
        m1, m_n1 = rmea.truncate_solution(m1, m_n1, self.instance)
        m2, m_n2 = rmea.truncate_solution(m2, m_n2, self.instance)
        c1 = rmea.add_missing_nodes(m1, m_n1, self.instance, self.relevance_matrix, customers)
        c2 = rmea.add_missing_nodes(m2, m_n2, self.instance, self.relevance_matrix, customers)


        # print(c1)
        # print(c2)
        #print(f"Demand for c1: {rmea.calculate_solution_demands(c1,self.instance)}")
        #print(f"Demand for c2: {rmea.calculate_solution_demands(c2,self.instance)}")
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

    def local_search(self, population):
        improved_population = []

        for solution in population:
            best_local = solution
            best_cost = self.evaluate(solution)
            routes = rmea.split_solution_to_routes(solution)
            #select random route
            random_route = random.choice([r for r in routes if len(r) > 2])
            #select a point from the route
            if len(random_route) < 2:
                continue
            random_node = random_route[random.randint(1, len(random_route) - 1)]
            other_clients = [n for n in random_route if n != random_node]
            most_relevant_node = max(other_clients, key=lambda n: self.relevance_matrix[random_node][n])
            idx1 = random_route.index(random_node)
            idx2 = random_route.index(most_relevant_node)

            start = min(idx1, idx2)
            end = max(idx1, idx2)

            nodes_between = random_route[start + 1:end]

            reference_node = random_route[start]
            nodes_between_sorted = sorted(
                nodes_between,
                key=lambda n: self.relevance_matrix[reference_node][n],
                reverse=True
            )
            new_route = random_route[:start + 1]
            new_route.extend(nodes_between_sorted)
            new_route.extend(random_route[end:])

            new_solution = [0]
            for r in routes:
                if r == random_route:
                    new_solution.extend(new_route)
                    new_solution.append(0)
                else:
                    new_solution.extend(r)
                    new_solution.append(0)
            new_solution = np.array(new_solution)
            #print(f"new: {new_solution}")
            if self.evaluate(new_solution) < best_cost:
                best_local = new_solution
            improved_population.append(best_local)
            
        return improved_population
    
    def diversity_strategy(self, population, no_improve):
        #number of customer pairs involved in the diversity strategy
        N = list(range(1, len(self.instance.demands)))
        D = int(len(N) * no_improve / self.max_generations)
        D = min(D, len(N))

        new_population = []
        for solution in population:
            temp = random.sample(N, D)
            new_solution = solution.copy()
            for i in temp:
                other_customers = [c for c in N if c != i]
                ord = sorted(other_customers, key=lambda l: self.relevance_matrix[i][l], reverse=True)
                j = random.choice(ord[-int(len(N)/10):])
                i_idx = np.where(new_solution == i)[0][0]
                j_idx = np.where(new_solution == j)[0][0]
                new_solution[i_idx], new_solution[j_idx] = new_solution[j_idx], new_solution[i_idx]
            new_demands = rmea.calculate_solution_demands(new_solution,self.instance)

            solution_is_wrong = False
            solution_is_wrong = (
                any(demand > self.instance.capacity for demand in new_demands)
                or len(rmea.split_solution_to_routes(new_solution)) > self.instance.num_vehicles
            )
            if solution_is_wrong:
                new_population.append(solution)
                continue

            if self.evaluate(new_solution) < self.evaluate(solution):
                new_population.append(new_solution)
            else:
                new_population.append(solution) 

        return new_population


    def solve(self):
        population = self.initialize_population()
        no_improve = 0

        for generation in range(self.max_generations):
            self.current_generation = generation
            self.relevance_matrix = self.build_relevance_matrix()
            fitness = self.compute_scaled_fitness(population)

            
            valid_population = [ind for ind in population if len(rmea.split_solution_to_routes(ind)) == self.instance.num_vehicles]
            if not valid_population:
                print("Brak poprawnych rozwiązań w populacji.")
                break
            # Elitism
            elite = min(valid_population, key=self.evaluate)
            new_population = [elite]

            while len(new_population) < self.population_size:
                parent1, parent2 = self.roulette_selection(population, fitness)

                # Skip parents with too many routes
                if len(rmea.split_solution_to_routes(parent1)) > self.instance.num_vehicles or \
                len(rmea.split_solution_to_routes(parent2)) > self.instance.num_vehicles:
                    continue

                offspring1, offspring2 = self.crossover(parent1, parent2)

                new_population.extend([offspring1, offspring2])

            if no_improve > self.no_improve_soft_limit:
                population = self.diversity_strategy(population,no_improve)
            
            population = new_population[:self.population_size]
            #population = self.local_search(population)
            best_candidate = min(population, key=self.evaluate)
            best_cost = self.evaluate(best_candidate)
            best_candidate_routes = rmea.split_solution_to_routes(best_candidate)

            # Don't update best if candidate exceeds vehicle limit
            if best_cost < self.best_cost and len(best_candidate_routes) <= self.instance.num_vehicles:
                self.best_cost = best_cost
                self.best_solution = best_candidate
                print(best_candidate)
                print(f"Koszty: {rmea.calculate_solution_demands(best_candidate, self.instance)}")
                no_improve = 0
                print(f"Generacja {generation}: nowe najlepsze rozwiązanie {best_cost / SCALE}")
            else:
                no_improve += 1
            
            population = self.local_search(population)
            self.update_relevance_matrix(population)

            if no_improve >= self.no_improve_limit:
                print("Brak poprawy – zakończenie.")
                break

        self.routes = rmea.split_solution_to_plottable(self.best_solution)
        print(f"Najlepsze rozwiązanie {self.best_solution}")
        print(f"Koszty: {rmea.calculate_solution_demands(self.best_solution, self.instance)}")


    def get_routes(self):
        return self.routes

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
            plan_output += f"\n Długość trasy: {distance / SCALE}"
            print(plan_output)
            total_distance += distance
        print(f"Łączna długość tras wszystkich pojazdów: {total_distance / SCALE}")