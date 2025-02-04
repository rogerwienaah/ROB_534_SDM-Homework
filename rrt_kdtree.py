import numpy as np
import matplotlib.pyplot as plt
import time
from scipy.spatial import KDTree
from maze import Maze2D

class RRT:
    def __init__(self, maze, max_iter=10000, step_size=5):
        self.maze = maze
        self.max_iter = max_iter
        self.step_size = step_size
        self.nodes = [maze.start_state]  # Store nodes in a list
        self.parents = {maze.start_state: None}  # Parent mapping
        self.kdtree = KDTree([maze.start_state])  # Initialize KDTree
        self.goal = maze.goal_state

    def distance(self, p1, p2):
        return np.linalg.norm(np.array(p1) - np.array(p2))

    def nearest_neighbor(self, random_point):
        _, index = self.kdtree.query(random_point)
        return self.nodes[index]

    def steer(self, from_node, to_node):
        direction = np.array(to_node) - np.array(from_node)
        length = np.linalg.norm(direction)
        if length > self.step_size:
            direction = (direction / length) * self.step_size
        new_node = tuple(np.round(np.array(from_node) + direction).astype(int))
        return new_node if not self.maze.check_hit(from_node, direction) else None
    

    def generate_random_point(self):
        if np.random.rand() < 0.1:  # Goal biasing
            return self.goal
        return (np.random.randint(0, self.maze.cols), np.random.randint(0, self.maze.rows))
    

    def find_path(self):
        start_time = time.time()
        for _ in range(self.max_iter):
            random_point = self.generate_random_point()
            nearest = self.nearest_neighbor(random_point)
            new_node = self.steer(nearest, random_point)

            if new_node and new_node not in self.parents:
                self.nodes.append(new_node)
                self.parents[new_node] = nearest
                self.kdtree = KDTree(self.nodes)  # Update KDTree

                if self.distance(new_node, self.goal) < 1:  # Goal reached
                    return self.reconstruct_path(new_node), time.time() - start_time

        return None, None

    def reconstruct_path(self, end_node):
        path = []
        while end_node is not None:
            path.append(end_node)
            end_node = self.parents[end_node]
        return path[::-1]

def run_rrt_on_maze(maze):
    rrt_solver = RRT(maze)
    path, runtime = rrt_solver.find_path()
    if path:
        path_length = sum(np.linalg.norm(np.array(path[i]) - np.array(path[i - 1])) for i in range(1, len(path)))
        maze.plot_path(path, title_name="RRT Path")
        return path_length, runtime
    else:
        return None, None

if __name__ == "__main__":

    maze1 = Maze2D.from_pgm('maze1.pgm')
    maze2 = Maze2D.from_pgm('maze2.pgm')
    
    # Run RRT on both mazes
    path_length1, runtime1 = run_rrt_on_maze(maze1)
    path_length2, runtime2 = run_rrt_on_maze(maze2)

    print("Maze Results:")
    print(f"Maze 1 - Path Length: {path_length1}, Runtime: {runtime1} seconds")
    print(f"Maze 2 - Path Length: {path_length2}, Runtime: {runtime2} seconds")
