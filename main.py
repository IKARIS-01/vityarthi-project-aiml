import time
import environment
import algorithms
import utils

# test settings
MAP_FILE_TO_RUN = "maps/dynamic.txt"
ALGORITHM_TO_RUN = "astar"

def run_simulation():
	print(f"map: {MAP_FILE_TO_RUN}")
	grid, dynamic_obstacles, start_node, goal_node = environment.load_map_data(MAP_FILE_TO_RUN)

	if not start_node or not goal_node:
		print("there is some error in the map file")
		return

	print(f"start: {start_node}")
	print(f"goal: {goal_node}")
	print(f"algo: {ALGORITHM_TO_RUN.upper()}")
	print("*" * 20)
	start_time = time.time()
	
	final_path = None
	final_cost = 0
	nodes_expanded_count = 0

	if ALGORITHM_TO_RUN == "bfs":
		final_path, nodes_expanded_count = algorithms.breadth_first_search(grid, start_node, goal_node)
		if final_path:
			final_cost = len(final_path) - 1
	elif ALGORITHM_TO_RUN == "ucs":
		final_path, final_cost, nodes_expanded_count = algorithms.uniform_cost_search(grid, start_node, goal_node)
	elif ALGORITHM_TO_RUN == "astar":
		final_path, final_cost, nodes_expanded_count = algorithms.a_star_search(grid, start_node, goal_node, dynamic_obstacles)
	elif ALGORITHM_TO_RUN == "dynamic":
		if not dynamic_obstacles:
			print("map has no dynamic obstacles, still proceeding")
		final_path, final_cost, nodes_expanded_count = algorithms.run_dynamic_simulation(grid, start_node, goal_node, dynamic_obstacles)

	end_time = time.time()
	execution_time = end_time - start_time

	print("results:")
	if final_path:
		print(f"path: {' -> '.join(map(str, final_path))}")
		print(f"cost: {final_cost}")
		print(f"time taken: {execution_time:.6f} seconds")
		print("\npath on grid:")
		utils.print_grid_with_path(grid, final_path, start_node, goal_node)
	else:
		print("no path found")
		print(f"nodes expanded: {nodes_expanded_count}")
		print(f"time taken: {execution_time:.6f} seconds")
	print("=" * 50)

run_simulation()
