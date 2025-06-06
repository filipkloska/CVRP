# import matplotlib.pyplot as plt

# class VRPPlotter:
#     def __init__(self, instance, solver):
#         self.instance = instance
#         self.solver = solver
#         self.routes = solver.get_routes()

#         self.coords = self.instance.coords
#         self.demands = self.instance.demands
#         self.depot = self.instance.depot

#         self.x = [self.coords[i+1][0] for i in range(len(self.coords))]
#         self.y = [self.coords[i+1][1] for i in range(len(self.coords))]

#     def draw(self):
#         if not self.routes:
#             print("Brak tras do wizualizacji.")
#             return

#         plt.figure(figsize=(10, 8))
#         plt.scatter(self.x, self.y, c='black', zorder=5)
#         plt.scatter(self.x[self.depot], self.y[self.depot], c='red', s=100, label="Magazyn", zorder=6)

#         for i in range(len(self.x)):
#             demand = self.demands[i]
#             label = f"{i} (waga: {demand})"
#             plt.text(self.x[i] + 2, self.y[i] + 2, label, fontsize=9)

#         vehicle_colors = ['blue', 'green', 'orange', 'purple', 'brown']

#         for vehicle_id, route in enumerate(self.routes):
#             color = vehicle_colors[vehicle_id % len(vehicle_colors)]
#             for i in range(len(route) - 1):
#                 start = route[i]
#                 end = route[i + 1]
#                 plt.arrow(
#                     self.x[start], self.y[start],
#                     self.x[end] - self.x[start], self.y[end] - self.y[start],
#                     color=color,
#                     length_includes_head=True,
#                     head_width=2,
#                     alpha=0.7
#                 )
#             plt.plot([], [], color=color, label=f"Pojazd {vehicle_id}")

#         plt.title("Trasy pojazdów – wizualizacja CVRP")
#         plt.xlabel("X")
#         plt.ylabel("Y")
#         plt.grid(True)
#         plt.legend()
#         plt.show()

import matplotlib.pyplot as plt

class VRPPlotter:
    def __init__(self, instance, solver):
        self.instance = instance
        self.solver = solver
        self.routes = solver.get_routes()

        self.coords = self.instance.coords
        self.demands = self.instance.demands
        self.depot = self.instance.depot

        self.x = [self.coords[i+1][0] for i in range(len(self.coords))]
        self.y = [self.coords[i+1][1] for i in range(len(self.coords))]

    def draw(self):
        if not self.routes:
            print("Brak tras do wizualizacji.")
            return

        plt.figure(figsize=(12, 8))

        # Rysowanie punktów z numerami
        for i in range(len(self.x)):
            plt.scatter(self.x[i], self.y[i], c='black', zorder=5)
            plt.text(self.x[i] + 1, self.y[i] + 1, str(i), fontsize=9, color='black', zorder=6)

        # Rysowanie magazynu
        plt.scatter(self.x[self.depot], self.y[self.depot], c='red', s=100, zorder=6, label="Magazyn")

        # Kolory pojazdów
        vehicle_colors = ['blue', 'green', 'orange', 'purple', 'brown']

        # Rysowanie tras pojazdów
        for vehicle_id, route in enumerate(self.routes):
            color = vehicle_colors[vehicle_id % len(vehicle_colors)]
            for i in range(len(route) - 1):
                start = route[i]
                end = route[i + 1]
                plt.arrow(
                    self.x[start], self.y[start],
                    self.x[end] - self.x[start], self.y[end] - self.y[start],
                    color=color,
                    length_includes_head=True,
                    head_width=1,
                    alpha=0.7
                )
            plt.plot([], [], color=color, label=f"Pojazd {vehicle_id}")

        # Dodanie punktów jako osobne pozycje w legendzie
        for i in range(len(self.x)):
            label = f"Punkt {i} (waga: {self.demands[i]})"
            plt.plot([], [], 'o', color='black', label=label)

        plt.title("Trasy pojazdów – wizualizacja CVRP")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.grid(True)

        # Przeniesienie legendy na bok
        plt.legend(loc='center left', bbox_to_anchor=(1.02, 0.5), fontsize=8)
        plt.tight_layout()
        plt.show()
