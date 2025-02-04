from maze import Maze2D
from queue import PriorityQueue
import time

class AStar:
    def __init__(self, maze, epsilon=10, max_time=0.05):
        self.maze = maze
        self.epsilon = epsilon
        self.max_time = max_time
    
    # def heuristic(self, cell1, cell2):
    #     # Manhattan distance
    #     x1, y1 = cell1
    #     x2, y2 = cell2

    #     return abs(x1 - x2) + abs(y1 - y2)

    def heuristic(self, cell1, cell2):
        #Euclidean distance
        x1, y1 = cell1
        x2, y2 = cell2
        return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

    
    def just_astar(self):
        start = self.maze.start_state
        goal = self.maze.goal_state

        # print(start)
        
        grid = [(x, y) for x in range(self.maze.rows) for y in range(self.maze.cols)]
        g_score = {cell: float("inf") for cell in grid}
        f_score = {cell: float("inf") for cell in grid}
        
        g_score[start] = 0
        f_score[start] = self.heuristic(start, goal)
        open_set = PriorityQueue()
        open_set.put((f_score[start], self.heuristic(start, goal), start))
        
        came_from = {}
        
        while not open_set.empty():
            current = open_set.get()[2]
            if current == goal:
                return self.reconstruct_path(came_from, current)
            
            # get neighbours
            state_id = self.maze.index_from_state(current)
            for neighbor in self.maze.get_neighbors(state_id):
                child = self.maze.state_from_index(neighbor)
                temp_g_score = g_score[current] + 1
                temp_f_score = temp_g_score + self.heuristic(child, goal)
                
                if temp_f_score < f_score[child]:
                    g_score[child] = temp_g_score
                    f_score[child] = temp_f_score
                    open_set.put((temp_f_score, self.heuristic(child, goal), child))
                    came_from[child] = current
        
        return []  

    def astar_with_eps(self):
        #greedy A*
        start_time = time.time()

        while self.epsilon > 1:
            if self.epsilon < 1.001:
                self.epsilon = 1
            
            

            start = self.maze.start_state
            goal = self.maze.goal_state
            
            grid = [(x, y) for x in range(self.maze.rows) for y in range(self.maze.cols)]
            g_score = {cell: float("inf") for cell in grid}
            f_score = {cell: float("inf") for cell in grid}
            
            g_score[start] = 0
            f_score[start] = self.heuristic(start, goal) * self.epsilon
            
            open_set = PriorityQueue()
            open_set.put((f_score[start], self.heuristic(start, goal), start))
            
            came_from = {}
            nodes_expanded = 0
            
            
            path_found = False
            final_path = []


            while not open_set.empty():
                current = open_set.get()[2]
                
                if current == goal:
                    path_found = True
                    final_path = self.reconstruct_path(came_from, current)
                    # maze.plot_path(final_path, "Maze2D")  #plot maze for greedy A*
                    break

                _time = time.time()

                if _time > start_time + self.max_time:
                    print("Did not complete")
                    break
                
                state_id = self.maze.index_from_state(current)
                for neighbor in self.maze.get_neighbors(state_id):
                    nodes_expanded += 1
                    child = self.maze.state_from_index(neighbor)
                    temp_g_score = g_score[current] + 1
                    temp_f_score = temp_g_score + self.heuristic(child, goal) * self.epsilon
                    
                    if temp_f_score < f_score[child]:
                        g_score[child] = temp_g_score
                        f_score[child] = temp_f_score
                        open_set.put((temp_f_score, self.heuristic(child, goal), child))
                        came_from[child] = current
                
            
            if path_found:
                print(f"EPSILON: {self.epsilon}\nNODES EXPANDED: {nodes_expanded}\nPATH LENGTH: {len(final_path)} \n")
                # self.maze.plot_path(final_path, 'Maze2D')
                # print(f"start_time: {start_time}")
                # print(f"time: {_time}")
                print("-------------")
            
            self.epsilon -= 0.5 * (self.epsilon - 1)
        
    def reconstruct_path(self, came_from, current):
        #path from start to goal
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]

if __name__ == "__main__":
    maze = Maze2D.from_pgm('maze2.pgm')
    print(maze)
    astr = AStar(maze, epsilon=10, max_time=1)

    #1st astar - Just A*
    # path = astr.just_astar()
    # print(path)
    # maze.plot_path(path, "Maze2D")

    # 2nd astar - A* with epsilon
    astr.astar_with_eps()
    

    