from math import cos, exp, hypot, pi, sin
from random import Random
from .graph import DirectedGraph
from .models import WindEstimate

def build_grid_graph(size=6, spacing_m=150.0):
    graph = DirectedGraph()
    for i in range(size):
        for j in range(size):
            graph.add_node(f"n{i}_{j}", i * spacing_m, j * spacing_m)
    for i in range(size):
        for j in range(size):
            src = f"n{i}_{j}"
            for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, -1), (1, -1), (-1, 1)):
                ni = i + di
                nj = j + dj
                if 0 <= ni < size and 0 <= nj < size:
                    graph.add_edge(src, f"n{ni}_{nj}")
    return graph


