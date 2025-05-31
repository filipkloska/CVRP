import numpy as np
from collections import Counter

def remove_consecutive_zeros(array):
    return np.array([val for i, val in enumerate(array) if i == 0 or not (val == 0 and array[i - 1] == 0)])

def solution_to_array(solution):
    array = np.concatenate([np.array(route) for route in solution])
    cleaned_array = remove_consecutive_zeros(array)
    #print(cleaned_array)
    return cleaned_array
    
def split_solution(solution_np,crossover_idx):
    return solution_np[:crossover_idx+1], solution_np[crossover_idx:]
    
def calculate_solution_demands(solution,instance):
    routes = split_solution_to_routes(solution)
    demands = np.array([sum(instance.demands[node] for node in route) for route in routes])
    return demands
    
def find_route_and_node_indices(solution, node):
    route_index = -1
    # print(f"Finding {solution}")
    for idx,solution_node in enumerate(solution):
        if solution_node == 0:
            route_index += 1
        elif solution_node == node:
            # print(f"Index {route_index}")
            return route_index,idx
        #dont ask, im slowly losing it
    # print(f"Index {route_index}")
    return route_index-1, 0
    
def split_solution_to_routes(solution):
    routes = []
    current_route = []

    for point in solution:
        if point == 0:
            if current_route:
                routes.append(current_route)
                current_route = []
        else:
            current_route.append(point)
    return routes

def split_solution_to_plottable(solution):
    routes = []
    current_route = []

    for point in solution:
        if point == 0:
            if current_route:
                routes.append([0] + current_route + [0])
                current_route = []
        else:
            current_route.append(point)

    if current_route:
        routes.append([0] + current_route + [0])

    return routes

def remove_duplicates(solution,relevance_matrix,customers):
    dupes_solution = [item for item, count in Counter(solution).items() if count > 1]
    for duplicate in dupes_solution:
        if duplicate == 0:
            continue
        dupe_indices = [i for i, val in enumerate(solution) if val == duplicate]
        #print(f"Duplicate {duplicate} found on indices: {dupe_indices}")
        highest_relevance_with_idx = (-1,-1)
        for idx in sorted(dupe_indices, reverse=True):
            relevance_next_point = relevance_matrix[duplicate][solution[idx+1]]
            #print(f"Relevance for point: {solution[idx+1]} is {relevance_next_point}")
            if highest_relevance_with_idx[0] < relevance_next_point:
                highest_relevance_with_idx = (relevance_next_point,idx)
        #print(f"The point with highest relevance ({highest_relevance_with_idx[0]}) is point: {solution[highest_relevance_with_idx[1]+1]}")
        #print(f"Highest rel index: {highest_relevance_with_idx[1]}")
        indices_to_delete = [i for i in dupe_indices if i != highest_relevance_with_idx[1]]
        #print(f"Indeces to delete: {indices_to_delete}")
        solution = np.array([node for idx,node in enumerate(solution) if not idx in indices_to_delete])
        #print("New m1")
        #print(solution)
    solution = remove_consecutive_zeros(solution)
    #print("Final M1:")
    #print(solution)
    missing_nodes = np.setdiff1d(customers,solution)
    #print(f"RD: {missing_nodes}")
    return solution, missing_nodes

def truncate_solution(solution, missing_nodes, instance):
    
    routes = split_solution_to_plottable(solution)
    routes_demand = calculate_solution_demands(solution, instance)
    sorted_indices = np.argsort(routes_demand)
    
    routes_copy = routes.copy()

    excess = len(routes_copy) - instance.num_vehicles

    for i in range(excess):
        idx_to_remove = sorted_indices[i]
        route_to_remove = routes[idx_to_remove]
        for customer in route_to_remove:
            if customer != 0:
                missing_nodes = np.append(missing_nodes, customer)

        routes_copy[idx_to_remove] = None 

    routes_copy = [r for r in routes_copy if r is not None]

    #print(f"TS: {missing_nodes}")
    return solution_to_array(routes_copy), missing_nodes


def adjust_bad_routes(solution,missing_nodes,instance):
    #print("Looking for bad routes")
    solution_demands = calculate_solution_demands(solution,instance)
    bad_route_indices = []
    for idx,demand in enumerate(solution_demands):
        if demand > instance.capacity:
            #print(f"FOUND A BAD ROUTE: {demand}")
            bad_route_indices.append(idx)
    routes = split_solution_to_routes(solution)
    
    removed_node = -1
    for idx in bad_route_indices:
        #print(F"Bad route: {np.array(routes[idx])}")
        sorted_nodes = np.array(sorted(routes[idx], key=lambda x: instance.demands[x]))
        overcap = solution_demands[idx] - instance.capacity
        for node in sorted_nodes:
            if overcap <= 0:
                break
            node_idx = np.where(solution == node)[0][0]
            solution = np.delete(solution, node_idx)
            missing_nodes = np.append(missing_nodes, node)
            overcap -= instance.demands[node]
    if removed_node != -1:
        missing_nodes = np.append(missing_nodes,removed_node)
        #print(f"AFTER REDUCTION: {solution}")
        #print(f"After reduction missing: {missing_nodes}")
        #print(f"AFTER RED: {calculate_solution_demands(solution, instance)}")
    #print(f"AB: {missing_nodes}")
    return solution, missing_nodes



def add_missing_nodes(solution, missing_nodes, instance, relevance_matrix, customers):
    for idx,node in enumerate(missing_nodes):
        #look for the highest relevant node to the ones missing
        
        #print(f"Looking for data for node: {node}")
        relevant_nodes_indices = np.argsort(relevance_matrix[node])[::-1]
        relevant_nodes_indices = relevant_nodes_indices[:-2]
        #print(f"Relnodes: {relevant_nodes_indices}")
        route_idx = -1
        rel_node_idx = -1
        isInserted = False
        solution_demands = calculate_solution_demands(solution, instance)
        #print(f"demands: {solution_demands}")
        for rel_node in relevant_nodes_indices:
            #check if existing in solution
            if rel_node in solution:
                #look for route where the rel_node is
                #print(f"Checking {rel_node}")
                route_idx, rel_node_idx = find_route_and_node_indices(solution, rel_node)
                #print(f"Route: {route_idx}, RelNodeIndex: {rel_node_idx}")
                #if you can insert then break if not then look for something else
                if instance.demands[node] + solution_demands[route_idx] <= instance.capacity:
                    solution = np.insert(solution, rel_node_idx+1, node)
                    isInserted = True
                    break
        if not isInserted:
            solution = np.append(solution,(node,0))
            
        #print("NEW M1")
        #print(solution)
        missing_nodes = np.setdiff1d(customers,solution)
        #print("Last diff")
        #print(missing_nodes)
    #print("Final demand")
    #print(calculate_solution_demands(solution,instance))
    return solution
    