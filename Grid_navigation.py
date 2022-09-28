
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
