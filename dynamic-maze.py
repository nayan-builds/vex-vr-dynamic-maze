#region VEXcode Generated Robot Configuration
import math
import random
from vexcode_vr import *

# Brain should be defined by default
brain=Brain()

drivetrain = Drivetrain("drivetrain", 0)
pen = Pen("pen", 8)
pen.set_pen_width(THIN)
left_bumper = Bumper("leftBumper", 2)
right_bumper = Bumper("rightBumper", 3)
front_eye = EyeSensor("frontEye", 4)
down_eye = EyeSensor("downEye", 5)
front_distance = Distance("frontdistance", 6)
distance = front_distance
magnet = Electromagnet("magnet", 7)
location = Location("location", 9)

#endregion VEXcode Generated Robot Configuration
# ------------------------------------------
# 
# 	Project:      VEXcode Project
#	Author:       VEX
#	Created:
#	Description:  VEXcode VR Python Project
# 
# ------------------------------------------

from collections import deque

# Add project code in "main"
def main():
    drivetrain.set_drive_velocity(100, PERCENT)
    drivetrain.set_turn_velocity(100, PERCENT)
    pen.set_pen_color(GREEN)
    pen.move(DOWN)
    solve_maze()

def solve_maze():
    # graph stores the mapped maze as an undirected, unweighted graph data structure
    # The graph data structure is a dictionary
    # Keys are nodes as strings in format "(x,y)"
    # Values are a list of connected nodes
    graph = Graph()
    robot = Robot()

    paths_from_start = 0
    start_visited = 0

    # Calculate number of routes from start, when this equals 
    # the number of times the start has been visited, then the mapping is complete
    # and the start has been returned to
    for i in range(0, 3):
        if not robot.wall_or_exit():
            paths_from_start += 1
        robot.turn_right()
    if not robot.wall_or_exit():
        paths_from_start += 1

    graph.add_node(robot.position.to_string())
    end_node = None

    # Map the maze by hugging left wall
    while paths_from_start > start_visited:
        if robot.wall_or_exit():
            robot.turn_right()
        else:
            robot.drive_forward()
            if down_eye.detect(RED): # Set end_node when the red square has been reached
                end_node = robot.position.to_string()
            
            # Add node and connection to previous node after moving
            # (won't be added if the node or connection already exists)
            graph.add_node(robot.position.to_string())
            graph.add_connection(robot.position.to_string(), robot.previous_position.to_string())
            if robot.position.to_string() == "(0,0)":
                start_visited += 1
            robot.turn_left()
    
    drivetrain.turn_to_heading(0, DEGREES)

    brain.print(graph.graph)

    # Calculate and travel shortest route from start to end
    pen.set_pen_color(BLUE)
    robot.travel_path(graph.shortest_path("(0,0)", end_node))

    wait(2, SECONDS)

    # Calculate and travel shortest route from end to start
    pen.set_pen_color(RED)
    robot.travel_path(graph.shortest_path(end_node, "(0,0)"))
        


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def to_string(self):
        return "(" + str(self.x) + "," + str(self.y) + ")"

class Robot:
    def __init__(self):

        self.position = Point(0, 0)
    
    def drive_forward(self, units = 1):
        drivetrain.drive_for(FORWARD, 250 * units, MM)
        self.previous_position = self.position

        if drivetrain.heading(DEGREES) == 0:
            self.position = Point(self.position.x, self.position.y + units)
        elif drivetrain.heading(DEGREES) == 90:
            self.position = Point(self.position.x + units, self.position.y)
        elif drivetrain.heading(DEGREES) == 180:
            self.position = Point(self.position.x, self.position.y - units)
        elif drivetrain.heading(DEGREES) == 270:
            self.position = Point(self.position.x - units, self.position.y)
    
    def turn_right(self):
        drivetrain.turn_for(RIGHT, 90, DEGREES)

    def turn_left(self):
        drivetrain.turn_for(LEFT, 90, DEGREES)
    
    def wall_or_exit(self):
        wall = front_distance.get_distance(MM) < 70
        at_start = self.position.to_string() == "(0,0)"
        at_end = down_eye.detect(RED)
        void = front_distance.get_distance(MM) > 2900


        # Treat the entrance and exit holes in the walls (void) as walls

        return wall or (void and (at_start or at_end))


    def travel_path(self, path):
        if path[0] != self.position.to_string():
            brain.print("Not at start position")
            return
        forward_moves = 1
        prev_heading = -1
        current_heading = 0

        prev_pos_x = self.position.x
        prev_pos_y = self.position.y
        for i in range(1, len(path)):

            # Extract x, y from node string
            next_pos = path[i]
            next_pos = next_pos.strip("()")
            next_pos_x_str, next_pos_y_str = next_pos.split(",")

            next_pos_x = int(next_pos_x_str)
            next_pos_y = int(next_pos_y_str)
            if next_pos_x > prev_pos_x:
                #right
                current_heading = 90
            elif next_pos_x < prev_pos_x:
                #left
                current_heading = 270
            elif next_pos_y > prev_pos_y:
                #up
                current_heading = 0
            elif next_pos_y < prev_pos_y:
                #down
                current_heading = 180


            if current_heading == prev_heading:
                # If the travel direction is same as previous,
                # add 1 forward move
                # (to be moved when this is no longer true)
                forward_moves += 1
            elif i == 1:
                # Doesn't move on first node as it needs to check
                # if the next node is going in the same direction
                prev_heading = current_heading
                drivetrain.turn_to_heading(current_heading, DEGREES)
            else:
                # If the current node is going in a different direction
                # to previous, drive to previous node
                self.drive_forward(forward_moves)
                drivetrain.turn_to_heading(current_heading, DEGREES)
                forward_moves = 1
                prev_heading = current_heading
            if i == len(path) - 1:
                # If it is last node, move to it
                self.drive_forward(forward_moves)
            prev_pos_x = next_pos_x
            prev_pos_y = next_pos_y




class Graph:
    def __init__(self):
        self.graph = {}

    def add_connection(self, node1, node2):
        if node2 not in self.graph[node1]:
            self.graph[node1].append(node2)
        if node1 not in self.graph[node2]:
            self.graph[node2].append(node1)

    def add_node(self, node):
        if node not in self.graph:
            self.graph[node] = []
        
    def shortest_path(self, start, end):
        # Finds the shortest path via a breadth-first traversal from start to end
        graph = self.graph

        # Stores whether each node has been visited
        visited = {node: False for node in graph}
        visited[start] = True
        
        # Stores the previous node for each node
        predecessors = {node: None for node in graph}

        # Stores the furthest reached nodes
        queue = deque([start])

        while queue:
            # Check the neighbours of the next furthest reached node
            current = queue.popleft()
            for neighbour in graph[current]:
                if not visited[neighbour]: # Not yet visited
                    visited[neighbour] = True
                    predecessors[neighbour] = current

                    # Add the neighbour node to the queue of furthest
                    # reached nodes
                    queue.append(neighbour)

                    # Stop if end has been found
                    if neighbour == end:
                        break
        
        # Reconstruct the shortest path from predecessors backwards
        path = []
        at = end
        while at is not None:
            # Stops when the node has no predecessor (start)
            path.append(at)
            at = predecessors[at]

        # Reverse path (for start to end)
        path.reverse()

        # Return empty list if no path found
        return path if path[0] == start else []


            

            
# VR threads — Do not delete
vr_thread(main)
