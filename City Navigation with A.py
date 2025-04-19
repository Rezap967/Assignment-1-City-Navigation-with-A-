import heapq
import time
import math

cities = {
    "A": (0, 0),
    "B": (2, 1),
    "C": (4, 2),
    "D": (5, 5),
    "E": (1, 4)
}

roads = {
    "A": ["B", "E"],
    "B": ["A", "C"],
    "C": ["B", "D"],
    "D": ["C"],
    "E": ["A", "D"]
}

def euclidean(a, b):
    ax, ay = cities[a]
    bx, by = cities[b]
    return math.hypot(ax - bx, ay - by)

def reconstruct_path(came_from, start, goal):
    current = goal
    path = [current]
    while current != start:
        current = came_from.get(current)
        if current is None:
            return []
        path.append(current)
    path.reverse()
    return path

def gbfs(start, goal):
    open_set = [(euclidean(start, goal), start)]
    came_from = {}
    visited = set()
    nodes_explored = 0

    while open_set:
        _, current = heapq.heappop(open_set)
        nodes_explored += 1
        if current == goal:
            break
        visited.add(current)
        for neighbor in roads[current]:
            if neighbor not in visited:
                came_from[neighbor] = current
                heapq.heappush(open_set, (euclidean(neighbor, goal), neighbor))
    return reconstruct_path(came_from, start, goal), nodes_explored

def astar(start, goal):
    open_set = [(0 + euclidean(start, goal), 0, start)]
    came_from = {}
    g_score = {start: 0}
    nodes_explored = 0

    while open_set:
        _, cost, current = heapq.heappop(open_set)
        nodes_explored += 1
        if current == goal:
            break
        for neighbor in roads[current]:
            tentative_g = g_score[current] + euclidean(current, neighbor)
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f = tentative_g + euclidean(neighbor, goal)
                heapq.heappush(open_set, (f, tentative_g, neighbor))
    return reconstruct_path(came_from, start, goal), nodes_explored

start, goal = "A", "D"
start_time = time.time()
gbfs_path, gbfs_nodes = gbfs(start, goal)
gbfs_time = (time.time() - start_time) * 1000

start_time = time.time()
astar_path, astar_nodes = astar(start, goal)
astar_time = (time.time() - start_time) * 1000

print("Comparing result (based on Time in millisecond)")
print("Assignment\tGBFS\t\tA star")
print(f"1\t\t{gbfs_time:.2f} ms\t{astar_time:.2f} ms\n")

print("Comparing result (based on number of nodes)")
print("Assignment\tGBFS\tA star")
print(f"1\t\t{gbfs_nodes}\t{astar_nodes}")