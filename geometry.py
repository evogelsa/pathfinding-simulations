import random
import numpy as np
import itertools
from operator import itemgetter
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder

class Square():
    '''
    Square is a class that defines a square. x,y is the top left corner and
    length is the side of each length.
    '''
    def __init__(self, x, y, length, angle = 0, color=(255,255,255)):
        self.x = x
        self.y = y
        self.length = length
        self.color = color
        self.angle = angle
        offset = int(np.sin(np.radians(angle)) * length)
        self.vertices = [self.x, self.y-offset,
                         self.x+self.length+offset, self.y,
                         self.x+self.length, self.y+self.length+offset,
                         self.x-offset, self.y+self.length]
    def draw(self, renderer):
        points = self.vertices
        points.extend((points[0], points[1]))
        renderer.draw_line(points, self.color)

class EquilateralTriangle():
    '''
    Triangle is a class that defines a triangle with the bottom aligned with the
    x axis. x,y is the bottom left corner and length is the length of each side.
    '''
    def __init__(self, x, y, length, angle = 0, color=(255,255,255)):
        self.x = x
        self.y = y
        self.length = length
        self.color = color
        self.vertices = [x, y,
                         int(x + length*np.cos(np.radians(60))),
                         int(y - length*np.sin(np.radians(60))),
                         x + length, y]
    def draw(self, renderer):
        points = self.vertices
        points.extend((points[0], points[1]))
        renderer.draw_line(self.vertices, self.color)

class Line():
    '''
    Line is a class that defines the endpoints of a line. This object will work
    with the SAT collision detection method
    '''
    def __init__(self, x1, y1, x2, y2, color=(255,255,255)):
        self.x = x1
        self.y = y1
        self.color = color
        self.vertices = [x1, y1, x2, y2]
    def draw(self, renderer):
        renderer.draw_line(self.vertices, self.color)

class LargePoint():
    '''
    LargePoint is a class that defines a point on the screen, but the point is
    exaggerated for drawing. When using draw it creates a solid rectangle that
    is centered on the location of the point
    '''
    def __init__(self, x, y, color=(255,255,255)):
        self.x = x
        self.y = y
        self.color = color
        self.vertices = [x, y]
    def draw(self, renderer):
        renderer.fill((self.x-4, self.y-4, 8, 8), color=self.color)


class RoundedRectangle():
    '''
    Rounded rectangle is class that defined a rectangle with corners rounded
    by the specified radius. x1,y1 sepcify the top right corner, and x2,y2
    specify the bottom right corner. r is the radius of the circle which makes
    the rounded corners.
    '''
    def __init__(self, x1, x2, y1, y2, r, color=(255,255,255)):
        self.x1 = x1
        self.x2 = x2
        self.y1 = y1
        self.y2 = y2
        self.r = r
        self.color = color
        self.center_line = []
        self.R = None
        self.pixels = []
        self.make()
    def make(self):
        centers = []
        # center for top left corner
        x = self.x1+self.r
        y = self.y1+self.r
        centers.append((x, y, self.x1, self.y1, x, y))

        # center for bottom left corner
        x = self.x1+self.r
        y = self.y2-self.r
        centers.append((x, y, self.x1, y, x, self.y2))

        # center for top right
        x = self.x2-self.r
        y = self.y1+self.r
        centers.append((x, y, x, self.y1, self.x2, y))

        x = self.x2-self.r
        y = self.y2-self.r
        centers.append((x, y, x, y, self.x2, self.y2))

        pixels = []
        for x, y, x1, y1, x2, y2 in centers:
            for i in range(y1,y2):
                for j in range(x1,x2):
                    if -1 <= distance(j, x, i, y) - self.r <= 1:
                        pixels.extend((j, i))

        # top straight line
        for x in range(self.x1+self.r, self.x2-self.r):
            pixels.extend((x, self.y1))

        # bottom straight line
        for x in range(self.x1+self.r, self.x2-self.r):
            pixels.extend((x, self.y2))

        # left straight line
        for y in range(self.y1+self.r, self.y2-self.r):
            pixels.extend((self.x1, y))

        # right straight line
        for y in range(self.y1+self.r, self.y2-self.r):
            pixels.extend((self.x2, y))

        R = (self.y2-self.y1)/2
        self.R = R
        clx1 = int(self.x1+R)
        clx2 = int(self.x2-R)
        for x in range(clx1, clx2):
            self.center_line.extend((x, self.y1+R))

        # pixels is the graphic pixels needed
        self.pixels = pixels
    def draw(self, renderer):
        renderer.draw_point(self.pixels, self.color)

def distance(x1, x2, y1, y2):
    return np.sqrt(np.power(x2-x1, 2) + np.power(y2-y1, 2))

def project(verts, axis):
    '''
    Take the dot product of vertices and an axis and return the endpoints of
    that projection. Used to determine if there is there is a separating axis
    '''
    dots = np.dot(verts, axis)
    return (min(dots), max(dots))

def detect_collision(shape0, shape1):
    '''
    Takes two objects which have a property named vertices and looks for
    collisions using the separating axis theorem. Returns true if colliding,
    false otherwise
    '''
    # create a list of tuples containing each vertex
    verts0 = list(zip(*[iter(shape0.vertices)]*2))
    verts1 = list(zip(*[iter(shape1.vertices)]*2))

    # calculate vectors connecting each vertex
    edges0 = []
    for v in range(len(verts0)):
        edges0.append((np.subtract(verts0[(v+1)%len(verts0)],verts0[v])))

    edges1 = []
    for v in range(len(verts1)):
        edges1.append((np.subtract(verts1[(v+1)%len(verts1)],verts1[v])))

    # calculate perpendicular vectors
    edges = edges0 + edges1
    axes = []
    for v in range(len(edges)):
        axes.append((edges[v][1], -edges[v][0]))

    # project each vertex onto each axis
    for a in range(len(axes)):
        v = project(verts0, axes[a])
        w = project(verts1, axes[a])
        # check if there is overlap between the projections, no overlap means
        # there is a separating axis and the objects are not colliding, some
        # overlap is inconclusive so we continue until all projections have been
        # tested. If no gap in all projections then objects are colliding
        if (v[0] < w[1] and v[0] > w[0]) or (w[0] < v[1] and w[0] > v[0]):
            continue
        else:
            return False
    return True

def generate_obstacles(
        n_obstacles,
        color,
        win_width,
        win_height,
        shape = 'both'):
    '''
    Create a list of n_obstacles with the specified color. The returned
    obstacles will be constrained within the window and will not overlap with
    each other.
    '''
    # create randomized obstacles
    obstacles = []

    if shape == 'square':
        shapes = [Square]
    elif shape == 'triangle':
        shapes = [EquilateralTriangle]
    else:
        shapes = [Square, EquilateralTriangle]

    for _ in range(n_obstacles):
        # assume obstacle collides until checked that it doesn't
        collides = True
        while collides:
            x = random.randrange(0, win_width, 1)
            y = random.randrange(0, win_height, 1)
            l = random.randrange(40, 80, 1)
            #  angle = random.randrange(-90,90,1)
            angle=0
            obstacle = random.choice(shapes)(x, y, l, angle, color=color)

            # check for obstacles not overlapping
            collides = False
            for each in obstacles:
                if detect_collision(obstacle, each):
                    collides = True

            # object may be colliding with edges of window so check for that
            vertices = obstacle.vertices
            vertices = list(zip(*[iter(vertices)]*2))
            for vertex in vertices:
                if ((vertex[0] >= win_width-20) or (vertex[1] >= win_height-20)
                        or (vertex[0] < 20) or (vertex[1] < 20)):
                    collides = True

        # if we got here then the obstacle didn't collide and we can continue
        obstacles.append(obstacle)
    return obstacles

def generate_narrow_obstacles(color):
    '''
    Create a obstacle filled path with narrow opening in the center
    '''
    top_square = Square(275, 0 , 250, 0, color=color)
    bot_square = Square(275, 300, 250, 0, color=color)
    obstacles = [top_square, bot_square]
    return obstacles

def generate_start_end_points(start_color, end_color, win_width, win_height):
    '''
    Creates two "large points" at 20,20 and width-20 height-20, so the top left
    and bottom right corners. The start and ends are actually points but are
    drawn as solid rectangles for visibility.
    '''
    x1, y1 = (20, 20)
    x2, y2 = (win_width-20, win_height-20)
    start = LargePoint(x1, y1, start_color)
    end = LargePoint(x2, y2, end_color)
    return start,end


def generate_visibility_graph(obstacles, color):
    '''
    Creates a list of lines connecting all the vertices of the passed in
    obstacles.
    '''
    # create a list of vertices from each obstacle, this is used to generate the
    # lines for the visibility graph
    vertices = [vertex for obstacle in obstacles for vertex in obstacle.vertices]
    vertices = list(zip(*[iter(vertices)]*2))

    # create a list of edges from each obstacle which will be used to detect
    # which lines of the visibilty graph are intersecting an edge from an
    # obstacle
    edges = []
    for obstacle in obstacles:
        vert_pairs = list(zip(*[iter(obstacle.vertices)]*2))
        for i in range(len(vert_pairs)):
            edges.append((vert_pairs[i],vert_pairs[(i+1)%len(vert_pairs)]))
    edge_lines = []
    for edge in edges:
        edge_lines.append(Line(*np.array(edge).ravel()))

    # create lines connecting every unique combination of vertices
    lines = []
    for i in range(len(vertices)):
        for j in range(len(vertices)-1, i, -1):
            x1, y1 = vertices[i]
            x2, y2 = vertices[j]
            line = Line(x1, y1, x2, y2, color=color)
            # only accept the line and add to list if it does not collide with
            # and of the edges of the obstacles
            collides = False
            for edge in edge_lines:
                if detect_collision(line, edge):
                    collides = True
            if not collides:
                lines.append(line)

    # my simple axis theorem doesn't detect the edge case where the line segment
    # is connecting interior corners of a square, so remove those manually by
    # checking if the line is made up of vertices which come from opposing
    # corners of the square
    for line in lines:
        for obstacle in obstacles:
            obstacle_vert_pairs = list(zip(*[iter(obstacle.vertices)]*2))
            line_vert_pairs = list(zip(*[iter(line.vertices)]*2))
            # need to ensure that lines which traverse the edge of an obstacle
            # are not removed, so check that the difference between x and y of
            # each vertex is equal to the length of the obstacle
            diff = np.abs(np.subtract(line_vert_pairs[1],line_vert_pairs[0]))
            if ((line_vert_pairs[0] in obstacle_vert_pairs)
                    and (line_vert_pairs[1] in obstacle_vert_pairs)
                    and ((diff[0] == obstacle.length)
                    and (diff[1] == obstacle.length))):
                lines.remove(line)

    lines.extend(edge_lines)

    return lines

def prm(obstacles,   # list of obstacles in the environment
        n_samples,   # number of samples per construction step
        min_samples, # minimum amount of construction samples before checking
        k,           # number of local points to connect to
        start,       # start point
        end,         # end point
        robot_dist,  # distance needed from obstacles for robot clearance
        color,       # color of points and lines
        win_width,
        win_height):

    # create a list of edges from each obstacle which will be used to detect
    # which lines of the graph are intersecting an edge from an obstacle
    edges = []
    for obstacle in obstacles:
        vert_pairs = list(zip(*[iter(obstacle.vertices)]*2))
        for i in range(len(vert_pairs)):
            edges.append((vert_pairs[i],vert_pairs[(i+1)%len(vert_pairs)]))
    edge_lines = []
    for edge in edges:
        edge_lines.append(Line(*np.array(edge).ravel()))

    # add start and end to coord list
    coordinates = [start, end]

    # add minimum amount of samples
    for _ in range(min_samples):
        # create a point
        x = random.randint(0, win_width)
        y = random.randint(0, win_height)
        point = LargePoint(x, y, color = color)
        # check if it is inside an obstacle
        for obstacle in obstacles:
            verts = extend_square(obstacle, robot_dist)
            inside = point_inside_square(verts, point)
            if inside == True:
                break
        # if it isn't inside an obstacle then add to list
        if not inside:
            coordinates.append(point)

    # start sampling n points at a time until solution found
    solution_found = False
    iters = 0
    while not solution_found:
        # init list holding lines connecting points to nearest neighbors
        lines = []

        # find nearest neighbors to each point and connect
        for i in range(len(coordinates)):
            point = coordinates[i]
            others = coordinates[:i] + coordinates[i+1:]
            nn = point_nearest_neighbors(point, others, k)

            # create lines from each point to its nearest neighbors
            for i in range(len(nn)):
                for j in range(len(nn)-1, i, -1):
                    _, p1 = nn[i]
                    _, p2 = nn[j]
                    x1, y1 = p1.x, p1.y
                    x2, y2 = p2.x, p2.y
                    line = Line(x1, y1, x2, y2, color = color)
                    # only accept the line and add to list if it does not
                    # collide with and of the edges of the obstacles
                    collides = False
                    for edge in edge_lines:
                        if detect_collision(line, edge):
                            collides = True
                    if not collides:
                        lines.append(line)

        # check if solution found
        grid_mat = generate_pathfinding_grid(lines, win_width, win_height)
        grid = Grid(matrix=grid_mat)
        start = grid.node(start.x, start.y)
        end = grid.node(end.x, end.y)
        # create A* finder
        finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
        # and calculate the path along the provided grid
        path, _ = finder.find_path(start, end, grid)
        if len(path) > 0:
            return coordinates, lines, path

        # otherwise add n_samples and check again
        for _ in range(n_samples):
            # create a point
            x = random.randint(0, win_width)
            y = random.randint(0, win_height)
            point = LargePoint(x, y, color = color)
            # check if it is inside an obstacle
            for obstacle in obstacles:
                verts = extend_square(obstacle, robot_dist)
                inside = point_inside_square(verts, point)
                if inside == True:
                    break
            # if it isn't inside an obstacle then add to list
            if not inside:
                coordinates.append(point)

        # incr iter
        iters += 1

    return coordinates, lines, path

def extend_square(square, dist):
    verts = []
    verts.extend(square.vertices)
    verts[0] -= dist
    verts[1] -= dist
    verts[2] += dist
    verts[3] -= dist
    verts[4] += dist
    verts[5] += dist
    verts[6] -= dist
    verts[7] += dist
    return verts

def point_inside_square(verts, point):
    x = point.x
    y = point.y
    return x > verts[0] and x < verts[2] and y > verts[1] and y < verts[7]

def point_nearest_neighbors(point, others, k):
    points = []
    # find distances to each point
    for other in others:
        d = distance(point.x, other.x, point.y, other.y)
        points.append((d, other))
    # sort list by smallest point
    points.sort(key=itemgetter(0))
    # return k nearest neighbors
    return points[:k]


def generate_pathfinding_grid(lines, win_width, win_height):
    # create a grid for the pathfinding
    grid = np.zeros((win_height, win_width))
    for line in lines:
        x1, y1, x2, y2 = line.vertices
        # special case for vertical line
        if (x2-x1) == 0:
            # make sure range is valid by checking whhich "direction" the line
            # is going
            if y2 > y1:
                for y in range(y1, y2):
                    grid[y, x1] = 1
            else:
                for y in range(y2, y1):
                    grid[y, x1] = 1
        # typical line, so calculate slope
        else:
            slope = (y2-y1)/(x2-x1)
            fy = lambda x: y1 + (x - x1)*slope
            for x in range(x1, x2):
                y = int(fy(x)+.5)
                grid[y,x] = 1
            for x in range(x2, x1):
                y = int(fy(x)+.5)
                grid[y,x] = 1
            fx = lambda y: (y - y1)/slope + x1
            for y in range(y1, y2):
                x = int(fx(y)+.5)
                grid[y,x] = 1
            for y in range(y2, y1):
                x = int(fx(y)+.5)
                grid[y,x] = 1
        # need to add end points
        grid[y1,x1] = 1
        grid[y2,x2] = 1
    return grid
