import numpy as np


def neighbors(x,y):
    dx = np.asarray([-1, -1, -1, 0, 0, 1, 1, 1])
    dy = np.asarray([-1, 0, 1, -1, 1, -1, 0, 1])
    xs = dx + x
    ys = dy + y

    return xs, ys

# grid is: -1 if inaccessible. 0 otherwise
def flood_fill(x, y, grid):
    grid[x,y] = 1
    xs, ys = neighbors(x, y)
    for i in range(len(xs)):
        x = xs[i]
        y = ys[i]
        if 0 <= x < grid.shape[0] and 0 <= y < grid.shape[1] and grid[x,y] == 0:
            flood_fill(x,y,grid)

