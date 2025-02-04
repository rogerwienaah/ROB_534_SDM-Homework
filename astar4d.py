from maze import Maze4D
from priority_queue import PriorityQueue
import time
import math

class AStar4D:
    def __init__(self, maze, epsilon=10, max_time=0.05):
        self.maze = maze
        self.epsilon = epsilon
        self.max_time = max_time
    
    # def heuristic_manhattan(self, state1, state2):
    #     x1, y1, dx1, dy1 = state1
    #     x2, y2, dx2, dy2 = state2
    #     return (abs(x1 - x2) + abs(y1 - y2)) / max(1, self.maze.max_vel)
    
    def heuristic(self, state1, state2):
        x1, y1, dx1, dy1 = state1
        x2, y2, dx2, dy2 = state2
        distance = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
        velocity_factor = max(math.sqrt(dx1 ** 2 + dy1 ** 2)+1, 1, self.maze.max_vel) 
        return distance / velocity_factor # time travelled
    
    def just_astar(self):
        start = tuple(self.maze.start_state)
        goal = tuple(self.maze.goal_state)
        
        g_score = {}
        f_score = {}
        open_set = PriorityQueue()
        
        
        g_score[start] = 0
        f_score[start] = self.heuristic(start, goal)
        open_set.insert(start, f_score[start])
        
        came_from = {}
        
        while open_set:
            current = open_set.pop()
            
            if current[:2] == goal[:2] and current[2:] == (0, 0):  
                return self.reconstruct_path(came_from, current)
            
            state_id = self.maze.index_from_state(current)
            for neighbor in self.maze.get_neighbors(state_id):
                child = tuple(self.maze.state_from_index(neighbor))
                temp_g_score = g_score[current] + 1 
                temp_f_score = temp_g_score + self.heuristic(child, goal)
                
                if child not in f_score or temp_f_score < f_score[child]:
                    g_score[child] = temp_g_score
                    f_score[child] = temp_f_score
                    open_set.insert(child, temp_f_score)
                    came_from[child] = current
        
        return [] 
    
    def astar_with_eps(self):
        
        start_time = time.time()

        while self.epsilon > 1:
            if self.epsilon < 1.001:
                self.epsilon = 1

            
            start = tuple(self.maze.start_state)
            goal = tuple(self.maze.goal_state)
            
            g_score = {}
            f_score = {}
            open_set = PriorityQueue()
            
            g_score[start] = 0
            f_score[start] = self.heuristic(start, goal) * self.epsilon
            open_set.insert(start, f_score[start])
            
            came_from = {}
            nodes_expanded = 0
            path_found = False
            final_path = []
            
            while open_set:
                current = open_set.pop()
                
                if current[:2] == goal[:2] and current[2:] == (0, 0):
                    path_found = True
                    final_path = self.reconstruct_path(came_from, current)
                    # maze.plot_path(final_path, "Maze2D")     # --plot maze for Greedy A*
                    break
                
                _time = time.time()
                if _time > start_time + self.max_time:
                    print("Did not complete")
                    break
                
                state_id = self.maze.index_from_state(current)
                for neighbor in self.maze.get_neighbors(state_id):
                    nodes_expanded += 1
                    child = tuple(self.maze.state_from_index(neighbor))
                    temp_g_score = g_score[current] + 1
                    temp_f_score = temp_g_score + self.heuristic(child, goal) * self.epsilon
                    
                    if child not in f_score or temp_f_score < f_score[child]:
                        g_score[child] = temp_g_score
                        f_score[child] = temp_f_score
                        open_set.insert(child, temp_f_score)
                        came_from[child] = current
            
            if path_found:
                print(f"EPSILON: {self.epsilon}\nNODES EXPANDED: {nodes_expanded}\nPATH LENGTH: {len(final_path)} \n")
                # self.maze.plot_path(final_path, 'Maze2D')
                # print(f"start_time: {start_time}")
                # print(f"time: {_time}")
                print("-------------")
            
            self.epsilon -= 0.5 * (self.epsilon - 1)
    
    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]

if __name__ == "__main__":
    maze = Maze4D.from_pgm('maze2.pgm')
    astar = AStar4D(maze, epsilon=10, max_time=1)
    
    # Just 4D A*
    # path = astar.just_astar()
    # maze.plot_path(path, "Maze4D")
    
    # 4D A* with deflating epsilon
    astar.astar_with_eps()
