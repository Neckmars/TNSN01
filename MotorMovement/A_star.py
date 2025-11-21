import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np

class Node:
    """
    Class that saves the 
    """

    def __init__(self, position, father = None):
        """
        Atributes:
            position: 2D array of the posittion of the node (x, y)
            father: Node object 
        """
        self.position = position
        self.father = father

        self.g = float('inf')
        self.h = float('inf')
        self.f = float('inf')
    

def HeuristicCalculation(position, goal):
    """
    Parameters:
        position:   2D array of the actual position
        goal:       2D array of the desired position

    Returns:
        Distance estimation, based on the Euclidean distance.
    """
    return np.sqrt((position[0] - goal[0])**2 + (position[1] - goal[1])**2)

def A_star(map, start, objectives, step=100):
    """
    Parameters:
        map:        2D Matrix that simulates the shape of the maze
        start:      2D array of the initial position
        objectives: List of 2D arrays of people position.

    Returns:
        List of the nodes that can solve the maze in the optimal way. 
    """
    
    open_nodes = []
    closed_nodes = set()
    explored_nodes = []

    start_node = Node(start)
    start_node.g = 0
    start_node.h = HeuristicCalculation(start, objectives)
    start_node.f = start_node.g + start_node.h

    pass

grid_size = 300
size = 7*grid_size
my_map = np.zeros((size, size))


# Exterior boundaries
my_map[(size-1), :3*grid_size] = 1
my_map[(size-1), 4*grid_size:] = 1
my_map[0, :(size-1)] = 1
my_map[0:, (size-1)] = 1
my_map[0:, 0] = 1

# Vertical boundaries
my_map[1*grid_size:5*grid_size, 1*grid_size] = 1

my_map[0*grid_size:1*grid_size, 2*grid_size] = 1
my_map[3*grid_size:5*grid_size, 2*grid_size] = 1
my_map[6*grid_size:, 2*grid_size] = 1
my_map[1*grid_size:3*grid_size, 3*grid_size] = 1
my_map[1*grid_size:2*grid_size, 4*grid_size] = 1
my_map[5*grid_size:7*grid_size, 4*grid_size] = 1
my_map[0*grid_size:1*grid_size, 5*grid_size] = 1
my_map[2*grid_size:4*grid_size, 5*grid_size] = 1
my_map[4*grid_size:6*grid_size, 6*grid_size] = 1

my_map[6*grid_size, 0*grid_size:1*grid_size] = 1
my_map[6*grid_size, 3*grid_size:4*grid_size] = 1
my_map[6*grid_size, 5*grid_size:6*grid_size] = 1
my_map[5*grid_size, 2*grid_size:3*grid_size] = 1
my_map[5*grid_size, 4*grid_size:5*grid_size] = 1
my_map[4*grid_size, 2*grid_size:4*grid_size] = 1
my_map[4*grid_size, 5*grid_size:6*grid_size] = 1
my_map[3*grid_size, 3*grid_size:4*grid_size] = 1
my_map[3*grid_size, 6*grid_size:7*grid_size] = 1
my_map[2*grid_size, 1*grid_size:3*grid_size] = 1
my_map[2*grid_size, 5*grid_size:7*grid_size] = 1
my_map[1*grid_size, 4*grid_size:6*grid_size] = 1


fig = plt.figure(figsize=(15,10))
gs = gridspec.GridSpec(1, 2, width_ratios=[1, 1]) 

ax0 = plt.subplot(gs[0])
ax0.imshow(my_map, cmap='Greys', origin='upper')
ax0.set_title('Original Map')
ax0.axis('off')
plt.tight_layout()
plt.show()