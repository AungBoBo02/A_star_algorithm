import numpy as np
import csv

def Astar(nodes, edges):
	# Import data from csv files
	nodes = np.loadtxt(nodes, delimiter=',')
	nodes = np.delete(nodes, [1,2], 1)
	nodes = nodes.tolist()

	edges = np.loadtxt(edges, delimiter=',')
	edges = edges.tolist()

	# Initialize required lists
	open_list = [(0, nodes[0])]
	close_list = []

	past_cost = [float('inf')] * len(nodes)
	past_cost[0] = 0
	total_past_cost = past_cost.copy()
	parent = [0] * len(nodes)

	goal = nodes[-1]

	# Loop until find the goal
	while len(open_list) > 0:

		# Set current_node
		current_node = open_list.pop(0)[1]
		close_list.append(current_node)

		# Found the goal
		if current_node[0] == goal[0]:
			path = find_path(parent)

			# Write path.csv file
			with open('path.csv', 'w') as myfile:
				wr = csv.writer(myfile)
				wr.writerow(path)

			return path


		# Get all neighbor edges
		current_edges = get_edges(current_node[0], edges)

		# Remove edges that already in close_list
		current_edges = remove_edges(current_edges, close_list, current_node[0])


		for edge in current_edges:
			next_node = nodes[int(edge[0])-1] if edge[0] != current_node[0] else nodes[int(edge[1])-1]

			# Sum current edge cost and past_cost
			tentative_past_cost = edge[-1] + past_cost[int(current_node[0])-1]

			# Sum all past_cost and heuristic cost-to-go to get toal cost
			total_cost = tentative_past_cost + next_node[1]


			# Check if total cost is less than total_past_cost
			if total_cost < total_past_cost[int(next_node[0])-1]:

				# Past_cost is updated
				past_cost[int(next_node[0])-1] = tentative_past_cost

				# Parent node is updated to current_node
				parent[int(next_node[0])-1] = current_node[0]

				# Total_past_cost is updated
				total_past_cost[int(next_node[0])-1] = past_cost[int(next_node[0])-1] + next_node[1]

				# Remove same node with larger value in open_list
				id_no = [lis[1][0] for lis in open_list]
				for id_no_idex, id_no_value in enumerate(id_no):
					if next_node[0] == id_no_value:
						open_list.pop(id_no_idex)

				# Tuple is added to open_list
				open_list.append((total_cost, next_node))
				# Sort the open_list
				open_list.sort()
				
	# If there is no way to reach to goal, False is returned
	return False


def get_edges(node_id, edges):
	# This function returns all neighbor edges
	return [edge for edge in edges if (edge[0] == node_id or edge[1] == node_id)]


def remove_edges(current_edges, close_list, node_id):
	#This function removes edges that already in close_list
	new_current_edges = current_edges.copy()

	for elem in current_edges:
		for node in close_list[:-1]:
			if node[0] == elem[0] or node[0] == elem[1]:
				new_current_edges.remove(elem)

	return new_current_edges


def find_path(parent):
	# This function returns path using the parent list
	path = [int(parent[-1])]

	while path[-1] != 1:
		path.append(int(parent[int(path[-1])-1]))

	path.reverse()
	path.append(len(parent))

	return path


print(Astar('nodes.csv', 'edges.csv'))