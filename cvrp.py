import tsplib95
import urllib.request
import math
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
import matplotlib.pyplot as plt
import re


class Instance:
    """
    Klasa odpowiedzialna za pobieranie pliku instancji problemu VRP z internetu.
    """

    def __init__(self, url, fileName):
        self.url = url
        self.fileName = fileName

    def download(self):
        """
        Pobiera plik z podanego adresu URL i zapisuje go lokalnie.
        """
        urllib.request.urlretrieve(self.url, self.fileName)


class ProblemLoader:
    """
    Klasa odpowiedzialna za wczytanie danych z pliku .vrp i ich przygotowanie.
    """

    def __init__(self, fileName):
        self.problem = tsplib95.load(fileName)

    def getData(self):
        """
        Zwraca dane wejściowe:
            - listę węzłów
            - współrzędne każdego węzła
            - zapotrzebowanie dla każdego węzła
            - indeks magazynu (depot)
            - pojemność pojazdów
            - liczbę pojazdów (wyciągnięta z nazwy pliku, np. k5 → 5)
        """
        nodes = list(self.problem.get_nodes())
        coords = {i: self.problem.node_coords[i] for i in nodes}
        demands = [self.problem.demands[i] for i in nodes]
        depot = list(self.problem.depots)[0]
        capacity = self.problem.capacity
        vehiclesNumber = int(re.search(r'k(\d+)', self.problem.name).group(1))

        return nodes, coords, demands, depot, capacity, vehiclesNumber

    def getDistanceMatrix(self, nodes, coords):
        """
        Oblicza macierz odległości Euklidesowych między wszystkimi parami węzłów.
        """
        def euclideanDistance(a, b):
            return int(math.hypot(a[0] - b[0], a[1] - b[1]))

        distanceMatrix = []
        for i in nodes:
            row = []
            for j in nodes:
                row.append(euclideanDistance(coords[i], coords[j]))
            distanceMatrix.append(row)

        return distanceMatrix


class DataModel:
    """
    Klasa przechowująca dane wejściowe dla solvera VRP.
    """

    def __init__(self, distanceMatrix, demands, vehicleCapacities, VehiclesNumber, depot):
        self.distanceMatrix = distanceMatrix
        self.demands = demands
        self.vehicleCapacities = vehicleCapacities
        self.VehiclesNumber = VehiclesNumber
        self.depot = depot


class Solver:
    """
    Klasa odpowiedzialna za rozwiązanie problemu VRP oraz prezentację wyników.
    """

    def __init__(self, data):
        self.manager = pywrapcp.RoutingIndexManager(
            len(data.distanceMatrix),
            data.VehiclesNumber,
            data.depot
        )
        self.routing = pywrapcp.RoutingModel(self.manager)

        # Funkcja zwracająca odległość między węzłami
        def distanceCallback(fromIndex, toIndex):
            fromNode = self.manager.IndexToNode(fromIndex)
            toNode = self.manager.IndexToNode(toIndex)
            return data.distanceMatrix[fromNode][toNode]

        transitCallbackIndex = self.routing.RegisterTransitCallback(distanceCallback)
        self.routing.SetArcCostEvaluatorOfAllVehicles(transitCallbackIndex)

        # Funkcja zapotrzebowania ładunku dla danego węzła
        def demandCallback(fromIndex):
            fromNode = self.manager.IndexToNode(fromIndex)
            return data.demands[fromNode]

        demandCallbackIndex = self.routing.RegisterUnaryTransitCallback(demandCallback)
        self.routing.AddDimensionWithVehicleCapacity(
            demandCallbackIndex,
            0,  # brak nadmiaru
            data.vehicleCapacities,  # pojemności pojazdów
            True,  # każdy pojazd zaczyna i kończy trasę w tym samym miejscu
            "Capacity"
        )

    def solve(self):
        """
        Rozwiązuje problem przy użyciu algorytmu PATH_CHEAPEST_ARC.
        """
        searchParameters = pywrapcp.DefaultRoutingSearchParameters()
        searchParameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        )

        solution = self.routing.SolveWithParameters(searchParameters)
        return solution

    def printSolution(self, data, solution):
        """
        Wypisuje szczegóły tras każdego pojazdu i całkowitą odległość.
        """
        totalDistance = 0

        for vehicleId in range(data.VehiclesNumber):
            index = self.routing.Start(vehicleId)
            planOutput = f"Route for vehicle {vehicleId}:\n"
            routeDistance = 0
            while not self.routing.IsEnd(index):
                nodeIndex = self.manager.IndexToNode(index)
                planOutput += f"{nodeIndex} -> "
                previousIndex = index
                index = solution.Value(self.routing.NextVar(index))
                routeDistance += self.routing.GetArcCostForVehicle(previousIndex, index, vehicleId)
            planOutput += f"{self.manager.IndexToNode(index)}\n"
            planOutput += f"Distance of the route: {routeDistance} km\n"
            print(planOutput)
            totalDistance += routeDistance
        print(f"Total distance of all routes: {totalDistance} km\n")

    def plotRoutes(self, data, solution, coords):
        """
        Rysuje trasy wszystkich pojazdów na wykresie 2D.
        """
        x = [coords[i + 1][0] for i in range(len(coords))]
        y = [coords[i + 1][1] for i in range(len(coords))]

        plt.figure(figsize=(10, 8))
        plt.scatter(x, y, c="black", zorder=5)
        plt.scatter(x[data.depot], y[data.depot], c='red', s=100, label="Magazyn", zorder=6)

        for i in range(len(x)):
            demand = data.demands[i]
            label = f"{i} (waga: {demand})"
            plt.text(x[i] + 2, y[i] + 2, label, fontsize=9)

        vehicleColors = ['blue', 'green', 'orange', 'purple', 'brown']

        for vehicleId in range(data.VehiclesNumber):
            index = self.routing.Start(vehicleId)
            route = []
            while not self.routing.IsEnd(index):
                nodeIndex = self.manager.IndexToNode(index)
                route.append(nodeIndex)
                index = solution.Value(self.routing.NextVar(index))
            route.append(self.manager.IndexToNode(index))  # dodaj punkt końcowy

            color = vehicleColors[vehicleId % len(vehicleColors)]

            for i in range(len(route) - 1):
                start = route[i]
                end = route[i + 1]
                plt.arrow(
                    x[start], y[start],
                    x[end] - x[start], y[end] - y[start],
                    color=color,
                    length_includes_head=True,
                    head_width=5,
                    alpha=0.7
                )

            # Dodaj pusty wpis do legendy (dla koloru)
            plt.plot([], [], color=color, label=f"Pojazd {vehicleId}")

        plt.title("Trasy pojazdów – wizualizacja CVRP")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.grid(True)
        plt.legend()
        plt.show()


def main():
    """
    Główna funkcja programu. Pobiera dane, wczytuje problem, rozwiązuje go i prezentuje wynik.
    """
    url = "http://vrp.atd-lab.inf.puc-rio.br/media/com_vrp/instances/A/A-n32-k5.vrp"
    fileName = "A-n32-k5.vrp"

    # Pobranie pliku z internetu
    instance = Instance(url, fileName)
    instance.download()

    # Wczytanie danych z pliku
    loader = ProblemLoader(fileName)
    nodes, coords, demands, depot, capacity, vehicleNumber = loader.getData()
    distanceMatrix = loader.getDistanceMatrix(nodes, coords)

    # Przygotowanie danych wejściowych
    data = DataModel(distanceMatrix, demands, [capacity] * vehicleNumber, vehicleNumber, depot - 1)

    # Rozwiązanie problemu
    solver = Solver(data)
    solution = solver.solve()

    # Prezentacja rozwiązania
    if solution:
        solver.printSolution(data, solution)
        solver.plotRoutes(data, solution, coords)
    else:
        print("Nie znaleziono rozwiązania.")


if __name__ == "__main__":
    main()
