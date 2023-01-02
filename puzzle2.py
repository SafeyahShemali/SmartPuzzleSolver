from __future__ import division
from __future__ import print_function

import sys
import math
import time
import heapq
import resource
from collections import deque


## The Class that Represents the Puzzle
class PuzzleState(object):
    """
        The PuzzleState stores a board configuration and implements
        movement instructions to generate valid children.
    """
    def __init__(self, config, n, parent=None, action="Initial", cost=0):
        """
        :param config->List : Represents the n*n board, for e.g.
[0,1,2,3,4,5,6,7,8] represents the goal state.
        :param n->int : Size of the board
        :param parent->PuzzleState
        :param action->string
        :param cost->int
        """
        if n * n != len(config) or n < 2:
            raise Exception("The length of config is not correct!")
        if set(config) != set(range(n * n)):
            raise Exception("Config contains invalid/duplicate entries : ", config)
        self.n = n
        self.cost = cost
        self.parent = parent
        self.action = action
        self.config = config
        self.children = []
        # Get the index and (row, col) of empty block
        self.blank_index = self.config.index(0)

    def __lt__(self, other):
        return calculate_total_cost(self) < calculate_total_cost(other)

    def display(self):
        """ Display this Puzzle state as a n*n board """
        for i in range(self.n):
            print(self.config[3 * i: 3 * (i + 1)])

    def move_up(self):
        """
        Moves the blank tile one row up.
        :return a PuzzleState with the new configuration
        """
        # to move a tile to up -> subtract 3 (size of the board side)
        # to swap the empty blank with the up tile,
        # we need to preserve the value of the other tile first
        if self.blank_index-3 >= 0 and (self.blank_index != 0 and self.blank_index != 1 and self.blank_index != 2):
            # get the data of the parent
            parent_blankIndex = self.blank_index
            parent_list = self.config[:]
            parent_size = self.n
            # create new state with a original list
            new_state = PuzzleState(parent_list, parent_size)
            # set the parent for the new state
            new_state.parent = self
            # take a copy of the tile
            temp = parent_list[parent_blankIndex-3]
            # swap
            new_state.config[parent_blankIndex - 3] = 0
            # put the blank index in new position
            new_state.config[parent_blankIndex] = temp
            # set the action for the new set
            new_state.action = "Up"
            # again get the index of the empty block
            new_state.blank_index = new_state.config.index(0)
            #Increment the sodt of the node
            new_state.cost = self.cost+1

            return new_state

    def move_down(self):
        """
        Moves the blank tile one row down.
        :return a PuzzleState with the new configuration
        """
        # to move a tile to down -> add 3 (size of the board side)
        # to swap the empty blank with the up tile,
        # we need to preserve the value of the other tile first
        if self.blank_index + 3 < 9 and (self.blank_index != 6 and self.blank_index != 7 and self.blank_index != 8):
            # get the data of the parent
            parent_blankIndex = self.blank_index
            parent_list = self.config[:]
            parent_size = self.n
            # create new state with a original list
            new_state = PuzzleState(parent_list, parent_size)
            # set the parent for the new state
            new_state.parent = self
            # take a copy of the tile
            temp = parent_list[parent_blankIndex + 3]
            # swap
            new_state.config[parent_blankIndex + 3] = 0
            # put the blank index in the new position
            new_state.config[parent_blankIndex] = temp
            # set the action
            new_state.action = "Down"
            # Increment the cost
            new_state.cost = self.cost + 1
            # again get the index of the empty block
            new_state.blank_index = new_state.config.index(0)

            return new_state

    def move_left(self):
        """
        Moves the blank tile one column to the left.
        :return a PuzzleState with the new configuration
        """
        # to move a tile to left -> sub 1
        # to swap the empty blank with the up tile,
        # we need to preserve the value of the other tile first
        if self.blank_index-1 >= 0 and (self.blank_index != 0 and self.blank_index != 3 and self.blank_index != 6):
            # get the data of the parent
            parent_blankIndex = self.blank_index
            parent_list = self.config[:]
            parent_size = self.n
            # create new state with a original list
            new_state = PuzzleState(parent_list, parent_size)
            # set the parent for the new state
            new_state.parent = self
            # take a copy of the tile
            temp = parent_list[parent_blankIndex - 1]
            # swap
            new_state.config[parent_blankIndex - 1] = 0
            # put the blank index in the new position
            new_state.config[parent_blankIndex] = temp
            # set the action
            new_state.action = "Left"
            # Increment the cost of the state
            new_state.cost = self.cost + 1

            # again get the index of the empty block
            new_state.blank_index = new_state.config.index(0)

            return new_state

    def move_right(self):
        """
        Moves the blank tile one column to the right.
        :return a PuzzleState with the new configuration
        """
        # to move a tile to right -> add 1
        # to swap the empty blank with the up tile,
        # we need to preserve the value of the other tile first

        if self.blank_index+1 < 9 and (self.blank_index != 2 and self.blank_index != 5 and self.blank_index != 8):
            # get the data of the parent
            parent_blankIndex = self.blank_index
            parent_list = self.config[:]
            parent_size = self.n
            # create new state with a original list
            new_state = PuzzleState(parent_list, parent_size)
            # set the parent for the new state
            new_state.parent = self
            # take a copy of the tile
            temp = parent_list[parent_blankIndex + 1]
            # swap
            new_state.config[parent_blankIndex + 1] = 0
            # put the blank index in the new position
            new_state.config[parent_blankIndex] = temp
            # set the action
            new_state.action = "Right"
            # Increment the cost of state
            new_state.cost = self.cost + 1
            # again get the index of the empty block
            new_state.blank_index = new_state.config.index(0)

            return new_state

    def expand(self):
        """ Generate the child nodes of this node """
        # Node has already been expanded
        if len(self.children) != 0:
            return self.children

        # Add child nodes in order of UDLR
        children = [
            self.move_up(),
            self.move_down(),
            self.move_left(),
            self.move_right()]
        # Compose self.children of all non-None children states
        self.children = [state for state in children if state is not None]
        return self.children

# Function that Writes to output.txt
def writeOutput(path,cost,explored,max_search_depth, run_time):
    path_str = '[' + ','.join(path) + ']'
    run_time_str = "%.8f" % run_time

    max_ram = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    ram_usage = (max_ram) / 2**20
    ram_usage_str = "%.8f" % ram_usage

    with open('output.txt', 'w') as f:
        f.write('path_to_goal: ')
        f.write(path_str)
        f.write('\n')
        f.write('cost_of_path: ')
        f.write(str(cost))
        f.write('\n')
        f.write('nodes_expanded: ')
        f.write(str(len(explored) - 1))
        f.write('\n')
        f.write('search_depth: ')
        f.write(str(cost))
        f.write('\n')
        f.write('max_search_depth: ')
        f.write(str(max_search_depth))
        f.write('\n')
        f.write('running_time: ')
        f.write(run_time_str)
        f.write('\n')
        f.write('max_ram_usage: ')
        f.write(ram_usage_str)
        f.write('\n')
    pass

def bfs_search(initial_state):
    """BFS search"""
    #start_ram = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    # Create the frontier
    frontier = deque()
    frontier.append(initial_state)
    frontierSet = {tuple(initial_state.config)}

    #Expolred set
    explored = set()

    #max_search_depth
    max_search_depth = 0

    while len(frontier) != 0:
        # dequque from the frontier
        state = frontier.popleft()
        if tuple(state.config) in frontierSet:
            frontierSet.remove(tuple(state.config))
        explored.add(tuple(state.config))
        print(len(explored))

        if test_goal(state):
            path = find_path(state)
            return [path,state.cost, explored, max_search_depth]

        #expand the children state and add to the parent state (called state now)
        state.children = state.expand()

        for neighbour in state.children:
            if tuple(neighbour.config) not in frontierSet:
                if tuple(neighbour.config) not in explored:
                    frontier.append(neighbour)
                    frontierSet.add(tuple(neighbour.config))
                    max_search_depth = max(max_search_depth, neighbour.cost)

    return False

def dfs_search(initial_state):
    """DFS search"""
    #start_ram = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    # Create the frontier
    frontier = deque()
    frontier.append(initial_state)
    frontierSet = {tuple(initial_state.config)}

    # Expolred set
    explored = set()

    # max_search_depth
    max_search_depth = 0

    while len(frontier) != 0:
        # pop from the frontier
        state = frontier.pop()
        if tuple(state.config) in frontierSet:
            frontierSet.remove(tuple(state.config))
        explored.add(tuple(state.config))
        #print(len(explored))


        if test_goal(state):
            path = find_path(state)
            return [path,state.cost, explored, max_search_depth]

        # expand the children state and add to the parent state (called state now)
        state.children = state.expand()

        for neighbour in reversed(state.children):
            if tuple(neighbour.config) not in frontierSet:
                if tuple(neighbour.config) not in explored:
                    frontier.append(neighbour)
                    frontierSet.add(tuple(neighbour.config))
                    max_search_depth = max(max_search_depth, neighbour.cost)


    return False

def A_star_search(initial_state):
    """A * search"""
    start_ram = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    # Create the frontier
    frontier = []
    heapq.heappush(frontier, (0, initial_state))
    frontierSet = {tuple(initial_state.config)}

    # Expolred set
    explored = []

    # max_search_depth
    max_search_depth = 0

    while len(frontier) != 0:
        # dequeue from the frontier
        state = heapq.heappop(frontier)[1]
        if tuple(state.config) in frontierSet:
            frontierSet.remove(tuple(state.config))
        explored.append(state.config)
        #print(len(explored))

        if test_goal(state):
            path = find_path(state)
            return [path,state.cost, explored, max_search_depth]

        # expand the children state and add to the parent state (called state now)
        state.children = state.expand()

        for neighbour in state.children:
            if tuple(neighbour.config) not in frontierSet:
                if tuple(neighbour.config) not in explored:
                    total_cost = calculate_total_cost(neighbour)
                    heapq.heappush(frontier,(total_cost,neighbour))
                    frontierSet.add(tuple(neighbour.config))
                    max_search_depth = max(max_search_depth, neighbour.cost)

    return False

def calculate_total_cost(state):
    """calculate the total estimated cost of a state"""
    # The total cost is f(state) = g(state) + h(state)
    # g(state) is the cost from the root to the state which we are standing on
    # h(n) is the manhattan cost of each tile in the state to the goal position

    man_cost = 0
    # empty space not consider as a tile
    for i in range(1,8):
        idx = state.config.index(i)
        man_cost += calculate_manhattan_dist(idx ,i,state.n)

    return state.cost + man_cost

def calculate_manhattan_dist(idx, value, n):
    """calculate the manhattan distance of a tile"""
    # Current Location
    col = idx % 3
    row = idx / 3

    # Goal Location
    colg = value % 3
    rowg = value / 3

    # Horizontal Move
    di_h = abs(row - rowg)
    # Vertical Move
    di_v = abs(col - colg)

    return di_h + di_v

def test_goal(puzzle_state):
    """test the state is the goal state or not"""
    if puzzle_state.config == [0, 1, 2, 3, 4, 5, 6, 7, 8]:
        return True
    else:
        return False

def find_path(state):
    path = []
    current_state = state

    while current_state.parent != None:
        path.insert(0,current_state.action)
        current_state = current_state.parent

    return path

# Main Function that reads in Input and Runs corresponding Algorithm
def main():

  	#Reading Inputs
    search_mode = sys.argv[1].lower()
    begin_state = sys.argv[2].split(",")
    begin_state = list(map(int, begin_state))
    board_size = int(math.sqrt(len(begin_state)))
    
    #Create the puzzle configuration and board 
    hard_state = PuzzleState(begin_state, board_size)
    
    #Running Algorithms 
    start_time = time.time()
    r = []
    if search_mode == "bfs":
        r = bfs_search(hard_state)
    elif search_mode == "dfs":
        r = dfs_search(hard_state)
    elif search_mode == "ast":
        r = A_star_search(hard_state)
    else:
        print("Enter valid command arguments !")
    end_time = time.time()

    # Display the output 
    writeOutput(r[0], r[1], r[2], r[3],  end_time - start_time)
    print("Program completed in %.3f second(s)" % (end_time - start_time))

if __name__ == '__main__':
    main()
