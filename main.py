from vrp_instance import VRPInstance
from vrp_solver_ortools import VRPSolverORTools
from vrp_plotter import VRPPlotter
from vrp_solver_evolution import VRPSolverRMEA

url = "http://vrp.atd-lab.inf.puc-rio.br/media/com_vrp/instances/E/E-n30-k3.vrp"

instance = VRPInstance("E-n30-k3.vrp")
instance.print_distance_matrix()

solver = VRPSolverORTools(instance)

solver.solve()
solver.print_solution()

plotter = VRPPlotter(instance, solver)
plotter.draw()

solver = VRPSolverRMEA(instance)

print(f"RADIUSSSS: {solver.neighbour_radius}")

solver.solve()
solver.print_solution()

# plotter = VRPPlotter(instance, solver)
# plotter.draw()