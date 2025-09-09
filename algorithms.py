import utils
import environment

def breadth_first_search(grid, start_node, goal_node):
	# queue of paths
	queue = [(start_node, [start_node])]
	visited_nodes = {start_node}
	nodes_expanded_count = 0

	while len(queue) > 0:
		nodes_expanded_count = nodes_expanded_count + 1
		current_position, path = queue.pop(0)

		if current_position == goal_node:
			return path, nodes_expanded_count

		for neighbor in environment.get_neighbors(current_position):
			if environment.is_valid_move(grid, neighbor) and neighbor not in visited_nodes:
				visited_nodes.add(neighbor)
				new_path = list(path)
				new_path.append(neighbor)
				queue.append((neighbor, new_path))

	return None, nodes_expanded_count

def uniform_cost_search(grid, start_node, goal_node):
	# priority queue for list of nodes
	frontier = [(0, start_node, [start_node])]
	visited_nodes = set()
	nodes_expanded_count = 0

	while len(frontier) > 0:
		# find smallest cost by sorting
		frontier.sort()
		current_cost, current_position, path = frontier.pop(0)
		nodes_expanded_count = nodes_expanded_count + 1

		if current_position in visited_nodes:
			continue
		
		visited_nodes.add(current_position)

		if current_position == goal_node:
			return path, current_cost, nodes_expanded_count

		for neighbor in environment.get_neighbors(current_position):
			if environment.is_valid_move(grid, neighbor) and neighbor not in visited_nodes:
				move_cost = environment.get_terrain_cost(grid, neighbor)
				new_cost = current_cost + move_cost
				new_path = list(path)
				new_path.append(neighbor)
				frontier.append((new_cost, neighbor, new_path))
	
	return None, 0, nodes_expanded_count

def a_star_search(grid, start_node, goal_node, dynamic_obstacles, time_offset=0):
	cost_from_start = 0
	heuristic_cost = utils.manhattan_distance(start_node, goal_node)
	total_estimated_cost = cost_from_start + heuristic_cost
	
	frontier = [(total_estimated_cost, cost_from_start, start_node, [start_node])]
	visited_nodes = set()
	nodes_expanded_count = 0

	while len(frontier) > 0:
		# sort to find smallest total estimated cost
		frontier.sort()
		_, current_cost, current_position, path = frontier.pop(0)
		nodes_expanded_count = nodes_expanded_count + 1

		if current_position in visited_nodes:
			continue
		
		visited_nodes.add(current_position)

		if current_position == goal_node:
			return path, current_cost, nodes_expanded_count
		
		current_time = len(path) - 1 + time_offset
		for neighbor in environment.get_neighbors(current_position):
			is_safe = environment.is_safe_at_time(neighbor, current_time + 1, dynamic_obstacles)
			
			if environment.is_valid_move(grid, neighbor) and neighbor not in visited_nodes and is_safe:
				move_cost = environment.get_terrain_cost(grid, neighbor)
				new_cost_from_start = current_cost + move_cost
				new_heuristic_cost = utils.manhattan_distance(neighbor, goal_node)
				new_total_estimated_cost = new_cost_from_start + new_heuristic_cost
				
				new_path = list(path)
				new_path.append(neighbor)
				frontier.append((new_total_estimated_cost, new_cost_from_start, neighbor, new_path))

	return None, 0, nodes_expanded_count

def run_dynamic_simulation(grid, start_node, goal_node, dynamic_obstacles):
	agent_position = start_node
	full_path_taken = [start_node]
	total_accumulated_cost = 0
	total_nodes_expanded = 0
	time_step = 0
	
	print("\nStarting dynamic simulation...")

	while agent_position != goal_node:
		path_segment, cost_segment, nodes_expanded = a_star_search(grid, agent_position, goal_node, dynamic_obstacles, time_offset=time_step)
		total_nodes_expanded = total_nodes_expanded + nodes_expanded
		
		if not path_segment:
			print(f"time {time_step:02d} | no path found from {agent_position}, agent is stuck.")
			return None, total_accumulated_cost, total_nodes_expanded

		next_step_in_plan = path_segment[1]
		
		if not environment.is_safe_at_time(next_step_in_plan, time_step + 1, dynamic_obstacles):
			print(f"time {time_step:02d} | obstacle detected at {next_step_in_plan}, agent waits at {agent_position}.")
			time_step = time_step + 1
			continue
		
		time_step = time_step + 1
		move_cost = environment.get_terrain_cost(grid, next_step_in_plan)
		total_accumulated_cost = total_accumulated_cost + move_cost
		agent_position = next_step_in_plan
		full_path_taken.append(agent_position)
		
		print(f"time {time_step:02d} | agent moved to {agent_position}. total Cost: {total_accumulated_cost}")

	print("\ngoal reached!")
	return full_path_taken, total_accumulated_cost, total_nodes_expanded
