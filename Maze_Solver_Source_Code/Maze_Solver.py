import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap


def calculate_open_grid(grid_size_x, grid_size_y, blocked_obstacles):
    open_grid = []
    for x in range(grid_size_y):
        for y in range(grid_size_x):
            # coordination = str(x)+","+str(y)  # ##string coordination
            # if coordination in blocked_obstacles: # ##string coordination check
            if ((x), (y)) in blocked_obstacles:
                continue
            else:
                # open_grid.append((str(x)+","+str(y)))
                open_grid.append([int(x), int(y)])
    # print(open_grid)
    return open_grid
# print(calculate_open_grid(grid_size_x, grid_size_y, blocked_obstacles))


def calculate_heuristic_value_Manhattan(open_grid, goal_node):    # ##Manhattan distance
    heuristic = {}
    for value in open_grid:
        heuristic_value = sum(np.abs(value[i] - goal_node[i]) for i in range(len(goal_node)))
        heuristic[str(value[0])+","+str(value[1])] = int(heuristic_value)
    # print(heuristic)
    return heuristic


def calculate_heuristic_value_Diagonal(open_grid, goal_node):
    heuristic = {}
    for value in open_grid:
        heuristic_value = max(abs(value[i]-goal_node[i]) for i in range(len(goal_node)))
        heuristic[str(value[0])+","+str(value[1])] = int(heuristic_value)
    # print(heuristic)
    return heuristic

# def calculate_adjacent_node(x, y, grid_size, blocked_obstacles):
#     adjacent_nodes = []
#     # ##top level
#     if ((x-1) >= 0):
#         if (((y-1) >= 0) and (((x-1), (y-1)) not in blocked_obstacles)):
#             adjacent_nodes.append(((x-1), (y-1)))
#         if ((((x-1), y) not in blocked_obstacles)):
#             adjacent_nodes.append(((x-1), y))
#         if (((y+1) <= (grid_size-1)) and (((x-1), (y+1)) not in blocked_obstacles)):
#             adjacent_nodes.append(((x-1), (y+1)))

#     # ##same level
#     if (((y-1) >= 0) and ((x, (y-1)) not in blocked_obstacles)):
#         adjacent_nodes.append((x, (y-1)))
#     if (((y+1) <= (grid_size-1)) and ((x, (y+1)) not in blocked_obstacles)):
#         adjacent_nodes.append((x, (y+1)))

#     # ##bottom level
#     if ((x+1) <= (grid_size-1)):
#         if (((y-1) >= 0) and (((x+1), (y-1)) not in blocked_obstacles)):
#             adjacent_nodes.append(((x+1), (y-1)))
#         if ((((x+1), y) not in blocked_obstacles)):
#             adjacent_nodes.append(((x+1), y))
#         if (((y+1) <= (grid_size-1)) and (((x+1), (y+1)) not in blocked_obstacles)):
#             adjacent_nodes.append(((x+1), (y+1)))
#     # print(adjacent_nodes)
#     return adjacent_nodes
# print(calculate_adjacent_node(0, 0, grid_size, blocked_obstacles))


def calculate_adjacent_node(node, grid_size_x, grid_size_y, blocked_obstacles):
    adjacent_nodes = []
    # ##top level
    if ((node[0]-1) >= 0):
        if (((node[1]-1) >= 0) and (((node[0]-1), (node[1]-1)) not in blocked_obstacles)):
            adjacent_nodes.append([(node[0]-1), (node[1]-1)])
        if ((((node[0]-1), node[1]) not in blocked_obstacles)):
            adjacent_nodes.append([(node[0]-1), node[1]])
        if (((node[1]+1) <= (grid_size_y-1)) and (((node[0]-1), (node[1]+1)) not in blocked_obstacles)):
            adjacent_nodes.append([(node[0]-1), (node[1]+1)])

    # ##same level
    if (((node[1]-1) >= 0) and ((node[0], (node[1]-1)) not in blocked_obstacles)):
        adjacent_nodes.append([node[0], (node[1]-1)])
    if (((node[1]+1) <= (grid_size_y-1)) and ((node[0], (node[1]+1)) not in blocked_obstacles)):
        adjacent_nodes.append([node[0], (node[1]+1)])

    # ##bottom level
    if ((node[0]+1) <= (grid_size_x-1)):
        if (((node[1]-1) >= 0) and (((node[0]+1), (node[1]-1)) not in blocked_obstacles)):
            adjacent_nodes.append([(node[0]+1), (node[1]-1)])
        if ((((node[0]+1), node[1]) not in blocked_obstacles)):
            adjacent_nodes.append([(node[0]+1), node[1]])
        if (((node[1]+1) <= (grid_size_y-1)) and (((node[0]+1), (node[1]+1)) not in blocked_obstacles)):
            adjacent_nodes.append([(node[0]+1), (node[1]+1)])
    # print(adjacent_nodes)
    return adjacent_nodes


def get_adjacent_nodes(open_grid, grid_size_x, grid_size_y, blocked_obstacles):
    adjacent_nodes = {}
    for value in open_grid:
        adjacent_nodes[str(value[0])+","+str(value[1])] = calculate_adjacent_node(value, grid_size_x, grid_size_y, blocked_obstacles)
    return adjacent_nodes


def calculate_path_cost(start_node, values):
    path_cost_list = []
    # print("\n")
    for value in values:
        calculate_path_cost = np.sqrt(sum(np.power((start_node[i] - value[i]), 2) for i in range(len(start_node))))
        # start_node_array = np.array(start_node)
        # value_array = np.array(value)
        # calculate_path_cost = np.linalg.norm(start_node_array - value_array)
        # calculate_path_cost = np.float64(calculate_path_cost)
        # print("value: ", value, type(value), "start_node",  start_node, type(start_node), "Value: ", round(calculate_path_cost), 1)
        # path_cost_list.append([value, round(float(calculate_path_cost), 1)])
        path_cost_list.append([str(value[0])+","+str(value[1]), round(float(calculate_path_cost), 1)])
    return path_cost_list


def get_path_cost(adjacent_nodes):
    path_cost = {}
    for key, values in adjacent_nodes.items():
        x, y = map(int, key.split(","))
        start_node = [x, y]
        path_costs = calculate_path_cost(start_node, values)
        path_cost[str(x)+","+str(y)] = path_costs
    return path_cost


def node_iteration(start, goal, heuristic_value_dict, path_cost_dict):
    def smallest_node_for_f_of_n(open_node):    # ## this function is for returning smallest cost node
        smallest_node = min(open_node, key=open_node.get)
        return smallest_node

    def cost(path_cost, heuristic_value):
        cost = path_cost + heuristic_value
        return cost

    def request_heuristic_value(node, heuristic_value_dict):
        heuristic_value = heuristic_value_dict.get(node, float('inf'))
        return heuristic_value

    def request_path_cost_dictionary(node, path_cost_dict):
        dict_list = path_cost_dict[node]
        print("##Node: ", node, "List path cost: ", path_cost_dict)
        dict_list_into_dict = dict(dict_list)   # ##basic implementation
        return dict_list_into_dict

    def modify_path_cost_heuristic(my_dict, s_node, d_node, g_of_n, h_of_n):
        if d_node in my_dict:
            previous_f_of_n = my_dict[d_node][0]+my_dict[d_node][1]
            current_f_of_n = g_of_n+h_of_n
            if current_f_of_n < previous_f_of_n:
                my_dict[d_node] = [g_of_n, h_of_n, s_node+">"+d_node]
        # elif s_node in my_dict:
        #     path = my_dict[s_node][2]
        #     my_dict[d_node] = [g_of_n, h_of_n, path+">"+d_node]
        # else:
        #     my_dict[d_node] = [g_of_n, h_of_n, d_node]
        else:
            path = my_dict[s_node][2]
            my_dict[d_node] = [g_of_n, h_of_n, path+">"+d_node]

    start = f"{start[0]},{start[1]}"
    goal = f"{goal[0]},{goal[1]}"

    heuristic_value = request_heuristic_value(start, heuristic_value_dict)
    path_cost_heuristic = {start: [0, heuristic_value, start]}
    # path_cost_heuristic = {}
    # modify_path_cost_heuristic(path_cost_heuristic, start, start, 0, heuristic_value)
    open_node = {start: heuristic_value}
    close_node = {}

    while open_node:
        source_node = smallest_node_for_f_of_n(open_node)
        if source_node == goal:
            path = path_cost_heuristic[goal][2]
            print("Path found")
            return path

        adjacent_path_cost_dict = request_path_cost_dictionary(source_node, path_cost_dict)
        for adjacent_node, path_cost in adjacent_path_cost_dict.items():
            h_of_n = request_heuristic_value(adjacent_node, heuristic_value_dict)
            previous_g_of_n = path_cost_heuristic[source_node][0]
            g_of_n = cost(previous_g_of_n, path_cost)
            f_of_n = cost(h_of_n, g_of_n)   # ##total estimation of cheapest cost. jana nai eitare ki bola jai?
            if adjacent_node not in close_node or f_of_n < open_node.get(adjacent_node, float('inf')):
                open_node[adjacent_node] = f_of_n
                modify_path_cost_heuristic(path_cost_heuristic, source_node, adjacent_node, g_of_n, h_of_n)

        close_node[source_node] = open_node[source_node]
        del open_node[source_node]

    print("Goal not reachable.")
    return "Something went wrong!!!"


# def plot_grid(grid_size, blocked_obstacles, start, goal, path):
#     plt.figure(figsize=(8, 8))

#     # Create the grid
#     grid = np.zeros((grid_size, grid_size))

#     # Mark blocked obstacles
#     for obstacle in blocked_obstacles:
#         grid[obstacle] = 1  # Use -1 to represent blocked cells

#     # Mark the start and goal positions
#     grid[start[0], start[1]] = -1  # Start position
#     grid[goal[0], goal[1]] = -1    # Goal position
#     # grid = np.rot90(grid)

#     # Create a color map
#     cmap = plt.cm.get_cmap("Greens", 3)  # Red for obstacles, Green for paths
#     cmap.set_under('lightgrey')  # Set color for non-obstacle cells
#     cmap.set_over('blue')         # Set color for the path

#     # Plot the grid
#     plt.imshow(np.rot90(grid), cmap=cmap, origin='upper', extent=[0, grid_size, 0, grid_size])

#     # Plot the path if it exists
#     if path:
#         path_coordinates = [tuple(map(int, node.split(','))) for node in path.split('>')]
#         for (x, y) in path_coordinates:
#             plt.text(x + 0.5, y + 0.5, '😊', color='yellow', fontsize=28, ha='center', va='center')

#     # Add labels and legend
#     plt.text(start[0] + 0.5, start[1] + 0.5, 'Start', color='black', fontsize=12, ha='center', va='center')
#     plt.text(goal[0] + 0.5, goal[1] + 0.5, 'Goal', color='black', fontsize=12, ha='center', va='center')

#     plt.xticks(np.arange(grid_size))
#     plt.yticks(np.arange(grid_size))
#     plt.grid()
#     plt.title('A* Pathfinding Visualization')
#     plt.xlabel('X-axis')
#     plt.ylabel('Y-axis')
#     plt.show()


def plot_grid(grid_size_x, grid_size_y, blocked_obstacles, start, goal, path):
    plt.figure(figsize=(8, 6))

    # Create the grid
    grid = np.zeros((grid_size_x, grid_size_y))

    # Mark blocked obstacles
    for obstacle in blocked_obstacles:
        grid[obstacle] = 1  # Use 1 to represent blocked cells

    # Mark the start and goal positions
    grid[start[0], start[1]] = -1  # Start position
    grid[goal[0], goal[1]] = -2    # Goal position

    # Define a custom colormap
    cmap = ListedColormap(['gainsboro', 'gainsboro', 'snow', 'dimgray'])
    # Define the color mapping: -2 for goal, -1 for start, 1 for obstacles, and 0 for open cells.
    # bounds = [-2.5, -1.5, -0.5, 0.5, 1.5, 2.5]
    norm = plt.Normalize(-2, 1)

    # Plot the grid with specified colors
    plt.imshow(np.rot90(grid), cmap=cmap, norm=norm, extent=[0, grid_size_x, 0, grid_size_y])

    # Plot the path if it exists
    if path:
        path_coordinates = [tuple(map(int, node.split(','))) for node in path.split('>')]
        # Increment each coordinate by +0.5 for correct alignment
        path_coordinates = [(x + 0.5, y + 0.5) for (x, y) in path_coordinates]
        path_x, path_y = zip(*path_coordinates)
        plt.plot(path_x, path_y, color='black', linewidth=1, label="Path", zorder=1)
        plt.scatter(path_x, path_y, color='black', s=50, zorder=2)  # Small dots on the path

    # Add labels for start and goal
    plt.scatter(start[0] + 0.5, start[1] + 0.5, color='deepskyblue', s=150, label="Start", zorder=3)
    plt.scatter(goal[0] + 0.5, goal[1] + 0.5, color='chartreuse', s=150, label="Goal", zorder=3)

    plt.legend(loc="upper right")
    plt.xticks(np.arange(grid_size_x))
    plt.yticks(np.arange(grid_size_y))
    plt.grid(color='black')
    plt.title('A* Pathfinding Visualization')
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.show()


def plot_heuristic_grid(grid_size_x, grid_size_y, blocked_obstacles, start, goal, heuristic):
    plt.figure(figsize=(8, 6))

    # Create the grid
    grid = np.zeros((grid_size_x, grid_size_y))

    # Mark blocked obstacles
    for obstacle in blocked_obstacles:
        grid[obstacle] = 1  # Use 1 to represent blocked cells

    # Mark the start and goal positions
    grid[start[0], start[1]] = -1  # Start position
    grid[goal[0], goal[1]] = -2    # Goal position

    # Define a custom colormap
    cmap = ListedColormap(['gainsboro', 'gainsboro', 'snow', 'dimgray'])
    norm = plt.Normalize(-2, 1)
    plt.imshow(np.rot90(grid), cmap=cmap, norm=norm, extent=[0, grid_size_x, 0, grid_size_y])

    for key, value in heuristic.items():
        y, x = map(int, key.split(","))
        plt.text(x + 0.2, y + 0.9, key, color='black', fontsize=9, ha='right', va='top')
        plt.text(x + 0.5, y + 0.5, f"h(n)={value}", color='black', fontsize=10, ha='center', va='center')

    # Add labels for start and goal
    # plt.scatter(start[0] + 0.5, start[1] + 0.5, color='deepskyblue', s=150, label="Start", zorder=3)
    # plt.scatter(goal[0] + 0.5, goal[1] + 0.5, color='chartreuse', s=150, label="Goal", zorder=3)
    plt.xticks(np.arange(grid_size_x))
    plt.yticks(np.arange(grid_size_y))
    plt.grid(color='black')
    plt.title('A* Pathfinding Visualization')
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.show()


def main():
    grid_size_x = int(input("Enter the size of the grid in horizontal(x) direction: "))
    grid_size_y = int(input("Enter the size of the grid in vertical(y) direction: "))

    number_of_blocked_obstacles = int(input("Enter the number of blocked obstacles: "))
    blocked_obstacles = []

    for i in range(number_of_blocked_obstacles):
        obstacle = input("Enter obstacle "+str(i)+" coordination: ")
        # blocked_obstacles.append(obstacle)    # ##this is string list
        x, y = map(int, obstacle.split(","))
        blocked_obstacles.append((int(x), int(y)))  # ##integer representation is implemented
    # print(blocked_obstacles)

    start = input("** Enter start node coordination: ")
    x, y = map(int, start.split(","))
    start = [x, y]

    goal = input("## Enter goal node coordination: ")
    x, y = map(int, goal.split(","))
    goal = [x, y]

    open_grid = calculate_open_grid(grid_size_x, grid_size_y, blocked_obstacles)
    # print("\n")
    # print(open_grid)

    heuristic_value_Manhattan = calculate_heuristic_value_Manhattan(open_grid, goal)
    print("\n")
    print(heuristic_value_Manhattan)
    plot_heuristic_grid(grid_size_x, grid_size_y, blocked_obstacles, start, goal, heuristic_value_Manhattan)

    heuristic_value_Diagonal = calculate_heuristic_value_Diagonal(open_grid, goal)
    print("\n")
    print(heuristic_value_Diagonal)

    # node = [1, 1]
    # print("\n")
    # print(calculate_adjacent_node(node, grid_size, blocked_obstacles))
    plot_heuristic_grid(grid_size_x, grid_size_y, blocked_obstacles, start, goal, heuristic_value_Diagonal)

    adjacent_nodes = get_adjacent_nodes(open_grid, grid_size_x, grid_size_y, blocked_obstacles)
    # print("\n")
    # print(adjacent_nodes)

    path_cost = get_path_cost(adjacent_nodes)
    print("\n")
    print(path_cost)

    output = node_iteration(start, goal, heuristic_value_Manhattan, path_cost)
    print("\n")
    print(output)
    plot_grid(grid_size_x, grid_size_y, blocked_obstacles, start, goal, output)

    # Call the plot function after finding the path
    output = node_iteration(start, goal, heuristic_value_Diagonal, path_cost)
    print("\nPath found: ", output)
    # Plotting the grid with the path
    plot_grid(grid_size_x, grid_size_y, blocked_obstacles, start, goal, output)


if __name__ == "__main__":
    main()


# output:
# 10
# 10
# 20
# 2,8
# 3,8
# 5,7
# 6,7
# 8,7
# 3,6
# 4,6
# 8,5
# 1,4
# 3,4
# 5,4
# 7,4
# 8,4
# 3,3
# 1,2
# 6,2
# 3,1
# 5,1
# 6,1
# 8,1
# 4,8
# 8,0

# 10
# 10
# 20
# 2,0
# 4,0
# 9,0
# 6,1
# 8,1
# 2,2
# 4,3
# 8,3
# 1,4
# 4,4
# 6,4
# 7,5
# 0,6
# 3,6
# 9,6
# 0,7
# 7,7
# 3,8
# 1,9
# 5,9
# 0,0
# 9,9
