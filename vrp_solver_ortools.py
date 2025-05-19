from ortools.constraint_solver import pywrapcp, routing_enums_pb2

class VRPSolverORTools:
    def __init__(self, instance):
        self.instance = instance
        self.data = {
            "distance_matrix": instance.distance_matrix,
            "demands": instance.demands,
            "vehicle_capacities": [instance.capacity] * instance.num_vehicles,
            "num_vehicles": instance.num_vehicles,
            "depot": instance.depot
        }
        self.manager = pywrapcp.RoutingIndexManager(
            len(self.data["distance_matrix"]),
            self.data["num_vehicles"],
            self.data["depot"]
        )
        self.routing = pywrapcp.RoutingModel(self.manager)
    def get_routes(self):
        routes = []
        for vehicle_id in range(self.data["num_vehicles"]):
            index = self.routing.Start(vehicle_id)
            route = []
            while not self.routing.IsEnd(index):
                route.append(self.manager.IndexToNode(index))
                index = self.solution.Value(self.routing.NextVar(index))
            route.append(self.manager.IndexToNode(index))  # dodaj końcowy punkt
            routes.append(route)
        return routes

    def solve(self):
        def distance_callback(from_index, to_index):
            from_node = self.manager.IndexToNode(from_index)
            to_node = self.manager.IndexToNode(to_index)
            return self.data["distance_matrix"][from_node][to_node]

        def demand_callback(from_index):
            from_node = self.manager.IndexToNode(from_index)
            return self.data["demands"][from_node]

        transit_callback_index = self.routing.RegisterTransitCallback(distance_callback)
        self.routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

        demand_callback_index = self.routing.RegisterUnaryTransitCallback(demand_callback)
        self.routing.AddDimensionWithVehicleCapacity(
            demand_callback_index, 0, self.data["vehicle_capacities"], True, "Capacity"
        )

        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        )

        self.solution = self.routing.SolveWithParameters(search_parameters)

    def print_solution(self):
        if not self.solution:
            print("Nie znaleziono rozwiązania.")
            return

        total_distance = 0
        for vehicle_id in range(self.data["num_vehicles"]):
            index = self.routing.Start(vehicle_id)
            plan_output = f"Trasa pojazdu {vehicle_id}:"
            route_distance = 0
            while not self.routing.IsEnd(index):
                node_index = self.manager.IndexToNode(index)
                plan_output += f" {node_index} ->"
                previous_index = index
                index = self.solution.Value(self.routing.NextVar(index))
                route_distance += self.routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
            plan_output += f" {self.manager.IndexToNode(index)}"
            plan_output += f"\n Długość trasy: {route_distance}"
            print(plan_output)
            total_distance += route_distance

        print(f"Łączna długość tras wszystkich pojazdów: {total_distance}")
