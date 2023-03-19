import heapq


# function to write the processed data into the output file
def write_to_output(reached, grid_exit, algorithm):
    node_with_cost = []
    path_length = 0
    cost = 0
    node = grid_exit
    if reached == 'FAIL':
        file_output = open('output.txt', 'w')
        file_output.write('FAIL')
        file_output.close()
    else:
        if algorithm == 'BFS':
            while reached[node] is not None:
                node_with_cost.insert(0, (node, 1))
                path_length += 1
                cost += 1
                node = reached[node]
            if reached[node] is None:
                node_with_cost.insert(0, (node, 0))
                path_length += 1
        else:
            while reached[node][1] is not None:
                node_with_cost.insert(0, (node, action_code_with_cost[reached[node][2]][1]))
                path_length += 1
                cost += action_code_with_cost[reached[node][2]][1]
                node = reached[node][1]
            if reached[node][1] is None:
                node_with_cost.insert(0, (node, 0))
                path_length += 1
        file_output = open('output.txt', 'w')
        file_output.write(str(cost) + '\n' + str(path_length) + '\n')
        for i in range(len(node_with_cost)):
            statement = ' '.join(map(str, node_with_cost[i][0])) + " " + str(node_with_cost[i][1])
            if i == (len(node_with_cost)-1):
                file_output.write(statement)
            else:
                file_output.write(statement + '\n')
        file_output.close()


""" ALGORITHM - breadth first search algorithm
    DATA STRUCTURE - queue
"""
def bfs(grid_entrance, grid_exit, bounds):
    reached = {}
    # create queue and add (start point, parent node)
    frontier = [(grid_entrance, None)]
    while frontier:
        # pop node from front
        current_node, parent = frontier.pop(0)
        # if current_node is equal to exit node of the grid, then update reached and stop search
        if current_node == grid_exit:
            reached[current_node] = parent
            return reached
        # if current node not in reached, update reached -> get the next_nodes based on actions -> add them to queue (if not reached)
        if current_node not in reached:
            reached[current_node] = parent
            for code in location_actions[current_node]:
                next_node = tuple(map(lambda i, j: i + j, current_node, action_code_with_cost[code][0]))
                if (next_node not in reached) and (next_node < bounds):
                    frontier.append([next_node, current_node])
    return 'FAIL'


""" ALGORITHM - uniform cost search algorithm
    DATA STRUCTURE - priority queue/min heap
"""
def ucs(grid_entrance, grid_exit, bounds):
    reached = {}
    frontier = []
    # create priority queue and add (cost, start node, parent node, current action) to heap
    heapq.heappush(frontier, (0, grid_entrance, None, None))
    while frontier:
        # pop node values in front of the priority queue
        cost, current_node, parent_node, current_action = heapq.heappop(frontier)
        # if current node is equal to grid exit, update current node values in reached and stop search
        if current_node == grid_exit:
            reached[current_node] = (cost, parent_node, current_action)
            return reached
        # if current node not in reached , update current node values in reached -> get the next node values based on the associated action and cumulative cost
        if current_node not in reached:
            reached[current_node] = (cost, parent_node, current_action)
            for code in location_actions[current_node]:
                next_node = tuple(map(lambda i, j: i + j, current_node, action_code_with_cost[code][0]))
                cumulative_cost = cost + action_code_with_cost[code][1]
                if (next_node not in reached) or (reached[next_node][0] > cumulative_cost):
                    if next_node < bounds:
                        neighbor_node = (cumulative_cost, next_node, current_node, code)
                        heapq.heappush(frontier, neighbor_node)

    return 'FAIL'


def heuristic_manhattan(node_a, node_b):
    total = abs(node_a[0]-node_b[0]) + abs(node_a[1]-node_b[1]) + abs(node_a[2]-node_b[2])
    return total


def heuristic_diagonal_distance(node_a, node_b):
    dx = abs(node_a[0]-node_b[0])
    dy = abs(node_a[1]-node_b[1])
    dz = abs(node_a[2]-node_b[2])
    return 10*(dx+dy+dz) + (14 - 2 * 10) * min(dx, dy, dz)


""" ALGORITHM - a star algorithm
    DATA STRUCTURE - priority queue/min heap
    HEURISTIC - Diagonal Distance formula to find the estimated cost in 3D square grid
"""
def a_star(grid_entrance, grid_exit, bounds):
    reached = {}
    # create priority queue and add (cost, start node, parent node, current action, heuristic cost) to heap
    priority_queue = []
    heapq.heappush(priority_queue, (0, grid_entrance, None, None, 0))
    while priority_queue:
        # pop node values in front of the priority queue
        cost, current_node, parent_node, current_action, parent_heuristic_cost = heapq.heappop(priority_queue)
        cost = cost - parent_heuristic_cost
        # if current node is equal to grid exit, update current node values in reached and stop search
        if current_node == grid_exit:
            reached[current_node] = (cost, parent_node, current_action)
            return reached
        # if current node not in reached , update current node values in reached -> get the next node values based on the associated action and heuristic cost
        if current_node not in reached:
            reached[current_node] = (cost, parent_node, current_action)
            for code in location_actions[current_node]:
                next_node = tuple(map(lambda i, j: i + j, current_node, action_code_with_cost[code][0]))
                next_node_cost = cost + action_code_with_cost[code][1] + heuristic_diagonal_distance(next_node, grid_exit)
                if (next_node not in reached) or (reached[next_node][0] > next_node_cost):
                    if next_node < bounds:
                        neighbor_node = (next_node_cost, next_node, current_node, code, heuristic_diagonal_distance(next_node, grid_exit))
                        heapq.heappush(priority_queue, neighbor_node)
    return 'FAIL'


# function to route to the right functional call based on the choice of algorithm
def run_algorithm(algorithm, grid_entrance, grid_exit, bounds):
    if algorithm.upper() == 'BFS':
        reached = bfs(grid_entrance, grid_exit, bounds)
    if algorithm.upper() == 'UCS':
        reached = ucs(grid_entrance, grid_exit, bounds)
    if algorithm.upper() == 'A*':
        reached = a_star(grid_entrance, grid_exit, bounds)
    write_to_output(reached, grid_exit, algorithm)


# function to read input file and initialize data
def process_input_file():
    with open("input.txt") as file_input:
        algorithm = file_input.readline().strip()
        bounds = tuple(map(int, file_input.readline().strip().split(" ")))
        grid_entrance = tuple(map(int, file_input.readline().strip().split(" ")))
        grid_exit = tuple(map(int, file_input.readline().strip().split(" ")))
        number_of_grid_with_actions = file_input.readline().strip()
        if number_of_grid_with_actions:
            for i in range(int(number_of_grid_with_actions)):
                grid_with_actions = list(map(int, file_input.readline().strip().split(" ")))
                grid_location = tuple([grid_with_actions[0], grid_with_actions[1], grid_with_actions[2]])
                location_actions[grid_location] = grid_with_actions[3:]
        run_algorithm(algorithm, grid_entrance, grid_exit, bounds)


# dictionary data structure that holds action code(keys) mapped to direction and the cost associated(values)
action_code_with_cost = {1: [[1, 0, 0], 10], 2: [[-1, 0, 0], 10],
                         3: [[0, 1, 0], 10], 4: [[0, -1, 0], 10],
                         5: [[0, 0, 1], 10], 6: [[0, 0, -1], 10],
                         7: [[1, 1, 0], 14], 8: [[1, -1, 0], 14],
                         9: [[-1, 1, 0], 14], 10: [[-1, -1, 0], 14],
                         11: [[1, 0, 1], 14], 12: [[1, 0, -1], 14],
                         13: [[-1, 0, 1], 14], 14: [[-1, 0, -1], 14],
                         15: [[0, 1, 1], 14], 16: [[0, 1, -1], 14],
                         17: [[0, -1, 1], 14], 18: [[0, -1, -1], 14]}

location_actions = {}
process_input_file()
