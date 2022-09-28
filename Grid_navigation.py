
import copy
import math
from queue import PriorityQueue
import random



############################################################
#  Grid Navigation
############################################################

# create the Grid Navigation class/object
class GridNavigation(object):

    def __init__(self, start, goal, scene):
        
        # define all the attributes
        self.start = start
        self.goal = goal
        self.scene = scene

        # also calculate the size of the scene
        self.row = len(scene)
        self.col = len(scene[0])

    def cal_distance(self):

        #get the index position for starting point
        x_start = self.start[0]
        y_start = self.start[1]

        #get the index position for the goal point
        x_goal = self.goal[0]
        y_goal = self.goal[1]

        #then calculate the distance
        distance = math.sqrt((x_start - x_goal)**(2) + (y_start - y_goal)**(2))

        return distance


    def successors(self):
        #define all the possible movement directions
        directions = [(0,-1),(0,1),
        (1,-1),(1,0),(1,1),
        (-1,-1),(-1,0),(-1,1)]

        for direction_to_move in directions:
            
            #get the indexes of x and y after the movement
            x_index = direction_to_move[0] + self.start[0]
            y_index = direction_to_move[1] + self.start[1]

            #make sure it's within the boundary
            if (0 <= x_index < self.row) and (0 <= y_index < self.col) and (self.scene[x_index][y_index] is not True):
                #if passed all the validation, then we can perfom the movement
                yield (direction_to_move, (x_index,y_index))

    def find_solution_a_star(self):

        #follow the similar structure as part 1
        frontier = PriorityQueue() #use priority queue since we need to sort the elements
        #put the initial condition element into the queue
        frontier.put((self.cal_distance(), 0, [self.start], self))

        #create an empty visited list first
        visited = {}
        visited[self.start] = 0

        while not frontier.empty():
            #get all the elements from the frontier
            fn, gn, start, board  = frontier.get()
            #loop through to check each components
            for move_to_check, start_to_check in board.successors():
                
                #get the possible solution
                distance = math.sqrt((move_to_check[0])**(2) + (move_to_check[1])**(2))
                current_gn = distance + gn
                current_solution = start + [start_to_check]

                #check if the current possible solution is a real solution
                if start_to_check == board.goal:
                    return current_solution
                # if not, we should add it to the visited list
                else:
                    if start_to_check not in visited:
                        visited[start_to_check] = current_gn
                        frontier.put((current_gn + GridNavigation(start_to_check,self.goal,self.scene).cal_distance(), current_gn, current_solution, GridNavigation(start_to_check,self.goal,self.scene)))
                    elif current_gn < visited[start_to_check]:
                        visited[start_to_check] = current_gn
                        frontier.put((current_gn + GridNavigation(start_to_check,self.goal,self.scene).cal_distance(), current_gn, current_solution, GridNavigation(start_to_check,self.goal,self.scene)))
        return None
        

def find_path(start, goal, scene):
    #call the recursive function
    return GridNavigation(start, goal, scene).find_solution_a_star()
############################################################
# Section 3: Linear Disk Movement, Revisited
############################################################
class LinearDiskMovement(object):

    def __init__(self, board):
        self.board = board
        self.len = len(board)


        # get the total of elements on the board
        n = 0 # initialize
        # loop through all the elements in the board
        for i in board:
            if i != 0:  #if its not 0
                n = n + 1 # count the number

        self.total_element = n

    #use this method to determine if a board is solved or not
    def is_solved(self, board):
        
        total_element = self.total_element
        
        #create the same length of all 0 board
        solution  = [0]*len(board)

        #assign all the numbers in place
        for i in range(total_element):
            solution[i] = i + 1
        solution.reverse() #reverse it to get the correct answer

        if board == solution:
            return True
        else:
            return False

    def get_board(self):
        return self.board

    def successors(self):
        #copy the main idea from hw2 here
        
        for i in range(self.len):
            # when the current location is not 0, the next 1 location is 0, and it's within the boundary
            if ((i+1)<self.len) and self.board[i] != 0 and self.board[i+1] == 0:
                new_board = copy.deepcopy(self.board)
                new_board[i+1] = new_board[i]
                new_board[i] = 0
                yield ((i,i+1), LinearDiskMovement(new_board)) 

            # when the current location is not 0, the next 1 location is also not 0, then make sure the next 2 location is 0, and it's within boundary
            if ((i+2)<self.len) and self.board[i] != 0 and self.board[i+1] != 0 and self.board[i+2] == 0 and i+2< len(self.board):
                new_board = copy.deepcopy(self.board)
                new_board[i+2] = new_board[i]
                new_board[i] = 0
                yield ((i, i+2), LinearDiskMovement(new_board))

            # when the current location is not 0, the previous location is 0, and it's within the boundary
            if ((i-1)>=0) and self.board[i] != 0  and self.board[i-1] == 0 :
                new_board = copy.deepcopy(self.board)
                new_board[i-1] = new_board[i]
                new_board[i] = 0
                yield ((i, i-1), LinearDiskMovement(new_board))

            # when the current location is not 0, the previous 1 location is also not 0, then make sure the previous 2 location is 0, and it's within boundary
            if ((i-2)>=0) and self.board[i] != 0 and self.board[i-1] != 0 and self.board[i-2] == 0:
                new_board = copy.deepcopy(self.board)
                new_board[i-2] = new_board[i]
                new_board[i] = 0
                yield ((i, i-2), LinearDiskMovement(new_board))

    # calculate the distance for each element on the board
    def cal_distance(self):

        total_element = self.total_element
        #create the same length of all 0 board
        solution  = [0]*self.len
        
        #assign all the numbers in place
        for i in range(total_element):
            solution[i] = i + 1
        solution.reverse() #reverse it to get the correct answer

        #use double pointers to check and get the indexes for both the current board and where it should be
        distance = 0
        for i in range(self.len):
            for j in range(self.len):
                if self.board[i] == solution[j] and self.board[i] !=0:
                    distance = distance +  abs(j-i)
        return distance /2 #divide the result by 2


def solve_distinct_disks(length, n):
    #initialize the board according to the inputs
    board = [0]*length

    for i in range(n):
        board[i] = i + 1
    #create the object 
    disk = LinearDiskMovement(board)

    frontier = PriorityQueue()
    frontier.put((disk.cal_distance(),list(), disk))
    visited = {}
    visited[tuple(disk.board)] = 0
    while frontier:

        #pop out of first element
        distance, move, board = frontier.get()

        #get all the corresponding successors to do the search
        for new_solution, new_board in board.successors():
            
            #current solution:
            current_solution = move + [new_solution]
            #if this board is solved, then return the current sulution
            if disk.is_solved(new_board.board):
                return current_solution
            #otherwise compile to get the next search
            else:
                # if it's not in the visited routines, or it's less than the biggest distance
                if tuple(new_board.board) not in visited or len(current_solution) < visited[tuple(new_board.board)]:
                    visited[tuple(new_board.board)] = len(current_solution)

                    #add the distnace, movement list and disk/board layout
                    frontier.put((new_board.cal_distance()+ len(current_solution), current_solution, new_board))
    return None

############################################################
# Section 4: Feedback
############################################################

# Just an approximation is fine.
feedback_question_1 = 50

feedback_question_2 = """
The Iddfs set-up part was REALLY hard for me.
Since all the parts to prepare the setting up does not count any credit, 
I had no idea if thery were correct or not.

Turns out that I had a minor typo which caused me almost a whole night to debug...
"""

feedback_question_3 = """
I like it also provides the GUI after I completed each question.

It's really rewarding to see my implementation actually works! :D
"""
