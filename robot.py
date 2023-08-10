import numpy as np
import time
import sys
import sdl2
import random
import ctypes
import matplotlib.pyplot as plt
import pathfinding
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder

import particles
import geometry

# seeding random will ensure that the obstacles are generated the same each time
random.seed(5123)

BLACK  = sdl2.ext.Color(0,0,0)
WHITE  = sdl2.ext.Color(255,255,255)
GRAY   = sdl2.ext.Color(121, 121, 121)
RED    = sdl2.ext.Color(255,0,0)
GREEN  = sdl2.ext.Color(0,255,0)
BLUE   = sdl2.ext.Color(0,0,255)
ORANGE = sdl2.ext.Color(249, 108, 7)

RESOURCES = sdl2.ext.Resources(__file__, "resources")

win_width = 800
win_height = 600

elapsed_time = 0

# Robot entity, stores velocity and sprite information
class Robot(sdl2.ext.Entity):
    def __init__(self, world, sprite, posx=0, posy=0, angle=0):
        self.sprite = sprite
        self.sprite.position = (posx, posy)
        self.sprite.true_x = posx
        self.sprite.true_y = posy
        self.sprite.angle = angle
        self.velocity = Velocity()
    def set_pos(self, posx, posy, angle=0):
        self.sprite.true_x = posx
        self.sprite.true_y = posy
        self.sprite.position = int(posx), int(posy)
        self.sprite.angle = angle
    def update_velocity(self):
        self.velocity.v += self.velocity.dv
        self.velocity.w += self.velocity.dw

        v = self.velocity.v
        w = self.velocity.w

        vx = v * np.cos(np.radians(self.sprite.angle+90))
        vy = v * np.sin(np.radians(self.sprite.angle+90))
        vr = w

        self.velocity.vx = int(vx)
        self.velocity.vy = int(vy)
        self.velocity.vr = vr
    def track_error(self, track):
        min_dist = float('inf')
        x1, y1 = self.sprite.position
        w, h = self.sprite.size
        x1 += int(w/2)
        y1 += int(h/2)

        pixels = track.center_line

        for x2, y2 in zip(*[iter(pixels)]*2):
            d = geometry.distance(x1, x2, y1, y2)
            if d < min_dist:
                min_dist = d

        d = min_dist - track.R
        return d

# Render system to handle rendering texture sprites (the robot)
class TextureRenderSystem(sdl2.ext.TextureSpriteRenderSystem):
    def __init__(self, renderer):
        super(TextureRenderSystem, self).__init__(renderer)
        self.renderer = renderer

    # render method called by world process
    def render(self, components):
        #  tmp = self.renderer.color
        #  self.renderer.color = BLACK
        #  self.renderer.clear()
        #  self.renderer.color = tmp
        for component in components:
            r = sdl2.SDL_Rect()
            r.x, r.y = component.x, component.y
            r.w = component.area[2] - r.x
            r.h = component.area[3] - r.y
            angle = component.angle
            renderer = self.renderer.sdlrenderer
            tex = component.texture
            sdl2.SDL_RenderCopyEx(renderer, tex, None, r, angle, None, 0)
        #  super(TextureRenderSystem, self).render(components)

# velocity type storing linear and angular velocity as well as their derivatives
# and the components of velocity
class Velocity(object):
    def __init__(self):
        super(Velocity, self).__init__()
        self.vx = 0
        self.vy = 0
        self.vr = 0
        self.v = 0
        self.w = 0
        self.dv = 0
        self.dw = 0

# handles updating robot position
class MovementSystem(sdl2.ext.Applicator):
    def __init__(self, minx, miny, maxx, maxy):
        super(MovementSystem, self).__init__()
        self.componenttypes = Velocity, sdl2.ext.Sprite
        self.minx = minx
        self.miny = miny
        self.maxx = maxx
        self.maxy = maxy

    # process fucntion called by world
    def process(self, world, componentsets):
        for velocity, sprite in componentsets:
            swidth, sheight = sprite.size

            sprite.true_x += velocity.vx * elapsed_time
            sprite.true_y += velocity.vy * elapsed_time

            sprite.x = int(sprite.true_x+.5)
            sprite.y = int(sprite.true_y+.5)
            sprite.x = max(self.minx, sprite.x)
            sprite.y = max(self.miny, sprite.y)
            sprite.angle += velocity.vr * elapsed_time
            rmaxx = sprite.x + swidth
            rmaxy = sprite.y + sheight
            if rmaxx > self.maxx:
                sprite.x = self.maxx - swidth
            if rmaxy > self.maxy:
                sprite.y = self.maxy - sheight

class PIDController():
    def __init__(self, P, I, D):
        self.p = P
        self.i = I
        self.d = D
        self.last_errors = np.zeros(5)
        self.error_sum = 0
    def update(self, error):
        self.error_sum += error
        self.last_errors = np.roll(self.last_errors, 1)
        self.last_errors[0] = error
        derror = -1*sum(self.last_errors)/len(self.last_errors)
        u = self.p*error + self.i*self.error_sum + self.d*(derror)
        if '-debug' in sys.argv:
            print(
                   '{:6.2f} {:6.2f} {:6.2f}'.format(
                   self.p*error,
                   self.i*self.error_sum,
                   self.d*(derror)
                   )
               )
        self.last_error = error
        return u

# run simulation
def run():
    # init sdl systems
    sdl2.ext.init()

    # create window, renderer, and the factory to create sprites
    window = sdl2.ext.Window("2D Robot Motion", size=(win_width,win_height))
    renderer = sdl2.ext.Renderer(window)
    factory = sdl2.ext.SpriteFactory(sdl2.ext.TEXTURE, renderer=renderer)

    # create world
    world = sdl2.ext.World()

    # defines movement system with bounds of window dimensions
    movement = MovementSystem(0, 0, win_width, win_height)

    # create sprite renderer with the texture system
    spriterenderer = TextureRenderSystem(renderer)

    # create a particle engine
    particleengine = particles.make_particle_engine()

    # load rectangle image for the particle trail
    images = [factory.from_image(RESOURCES.get_path("small-square.png"))]
    particlerenderer = particles.ParticleRenderSystem(renderer, images)

    # add the various systems to the world
    world.add_system(movement)
    world.add_system(particleengine)
    world.add_system(spriterenderer)
    world.add_system(particlerenderer)

    # define robot position for particle system to know where to spawn
    world.robotx = int(win_width/2)
    world.roboty = int(win_height/2)

    # create a sprite for the robot
    sp_robot = factory.from_color(WHITE, size=(15,15))
    # initialize the robot in the center of the window
    robot = Robot(world, sp_robot, 400, 100, -90)

    # process arguments to define initial conditions
    if '-v' in sys.argv:
        v = float(sys.argv[sys.argv.index('-v')+1])
    else:
        v = 300

    if '-w' in sys.argv:
        w = float(sys.argv[sys.argv.index('-w')+1])
    else:
        w = 300

    if '-dv' in sys.argv:
        dv = float(sys.argv[sys.argv.index('-dv')+1])
    else:
        dv = 0

    if '-dw' in sys.argv:
        dw = float(sys.argv[sys.argv.index('-dw')+1])
    else:
        dw = 0

    show_track = False
    if '-track' in sys.argv:
        show_track = True

    if '-p' in sys.argv:
        P = float(sys.argv[sys.argv.index('-p')+1])
    else:
        P = 0

    if '-i' in sys.argv:
        I = float(sys.argv[sys.argv.index('-i')+1])
    else:
        I = 0

    if '-d' in sys.argv:
        D = float(sys.argv[sys.argv.index('-d')+1])
    else:
        D = 0

    if '-keyboard' not in sys.argv:
        # set initial conditions
        robot.velocity.v = v
        robot.velocity.w = w
        robot.velocity.dv = dv
        robot.velocity.dw = dw
        keyboard = False
    else:
        keyboard = True

    if '-obstacles' in sys.argv:
        n_obstacles = int(sys.argv[sys.argv.index('-obstacles')+1])
    else:
        n_obstacles = 8

    if '-pathfinding' in sys.argv:
        # create navigation obstacles for robot
        obstacles = geometry.generate_obstacles(n_obstacles,
                                                ORANGE,
                                                win_width,
                                                win_height)
        # define the start and end point
        start_point, end_point = geometry.generate_start_end_points(GREEN,
            RED, win_width, win_height)
        # add start and end point to the obstacles list for the vis graph
        obstacles.extend((start_point, end_point))
        # generate the visibility graph
        lines = geometry.generate_visibility_graph(obstacles, GRAY)

        # generate grid for A* pathfinding by descretizing the lines into each
        # point along the line. The result is a matrix with each pixel
        # representing a node following the visibility graph
        grid_mat = geometry.generate_pathfinding_grid(lines,
                                                      win_width,
                                                      win_height)

        # uncomment the next two lines to see what the conversion from lines to
        # pixels looks like

        #  plt.imshow(grid_mat)
        #  plt.show()

        grid = Grid(matrix=grid_mat)
        start = grid.node(start_point.x, start_point.y)
        end = grid.node(end_point.x, end_point.y)
        # create A* finder
        finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
        # and calculate the path along the provided grid
        path, runs = finder.find_path(start, end, grid)
        if '-debug' in sys.argv:
            print('operations:', runs, 'path length:', len(path))

        # while pathfinding the robot is just animated along the route and is
        # not actually controlled
        frame_step = 0
        x, y = path[frame_step]
        x -= int(robot.sprite.size[0]/2)
        y -= int(robot.sprite.size[1]/2)
        robot.set_pos(x, y)
        robot.velocity.v = 0
        robot.velocity.w = 0
        robot.velocity.dv = 0
        robot.velocity.dw = 0

    if '-prmscattered' in sys.argv:
        # draw random obstacles like in first example
        obstacles = geometry.generate_obstacles(8, ORANGE,
                                                win_width, win_height,
                                                'square')
        start_point, end_point = geometry.generate_start_end_points(GREEN,
            RED, win_width, win_height)
        points, lines, path = geometry.prm(obstacles, 20, 100, 4, start_point,
                                           end_point, 30, GRAY,
                                           win_width, win_height)
        frame_step = 0
        x, y = path[frame_step]
        x -= int(robot.sprite.size[0]/2)
        y -= int(robot.sprite.size[1]/2)
        robot.set_pos(x, y)
        robot.velocity.v = 0
        robot.velocity.w = 0
        robot.velocity.dv = 0
        robot.velocity.dw = 0

    if '-prmnarrow' in sys.argv:
        obstacles = geometry.generate_narrow_obstacles(ORANGE)
        start_point, end_point = geometry.generate_start_end_points(GREEN,
            RED, win_width, win_height)
        points, lines, path = geometry.prm(obstacles, 20, 100, 4, start_point,
                                           end_point, 30, GRAY,
                                           win_width, win_height)
        frame_step = 0
        x, y = path[frame_step]
        x -= int(robot.sprite.size[0]/2)
        y -= int(robot.sprite.size[1]/2)
        robot.set_pos(x, y)
        robot.velocity.v = 0
        robot.velocity.w = 0
        robot.velocity.dv = 0
        robot.velocity.dw = 0


    track = geometry.RoundedRectangle(100, 700, 100, 500, 200)
    PID = PIDController(P, I, D)

    # make the window visible and start simulation loop
    window.show()
    running = True
    while running:
        # time frame start
        frame_start = time.monotonic()

        # check for window being closed or keypresses and keyboard control
        events = sdl2.ext.get_events()
        for event in events:
            if event.type == sdl2.SDL_QUIT:
                running = False
                break
            if event.type == sdl2.SDL_KEYDOWN and keyboard:
                if event.key.keysym.sym == sdl2.SDLK_UP:
                    robot.velocity.v = v
                if event.key.keysym.sym == sdl2.SDLK_DOWN:
                    robot.velocity.v = -v
                if event.key.keysym.sym == sdl2.SDLK_LEFT:
                    robot.velocity.w = -w
                if event.key.keysym.sym == sdl2.SDLK_RIGHT:
                    robot.velocity.w = w
            elif event.type == sdl2.SDL_KEYUP and keyboard:
                if event.key.keysym.sym in (sdl2.SDLK_UP, sdl2.SDLK_DOWN):
                    robot.velocity.v = 0
                if event.key.keysym.sym in (sdl2.SDLK_LEFT, sdl2.SDLK_RIGHT):
                    robot.velocity.w = 0

        # clear old graphics and update state for next frame
        renderer.clear()

        # calculate new robot velocities
        if ('-pathfinding' in sys.argv or '-prmscattered' in sys.argv
                or '-prmnarrow' in sys.argv):
            frame_step +=1
            x, y = path[frame_step%len(path)]
            x -= int(robot.sprite.size[0]/2)
            y -= int(robot.sprite.size[1]/2)
            robot.set_pos(x, y)
        else:
            robot.update_velocity()

        # create trail
        particles.createparticles(world, None, 1)

        # draw track
        if show_track:
            track.draw(renderer)
        if '-pathfinding' in sys.argv:
            for line in lines:
                line.draw(renderer)
            for obstacle in obstacles:
                obstacle.draw(renderer)
        if '-prmscattered' in sys.argv or '-prmnarrow' in sys.argv:
            for obstacle in obstacles:
                obstacle.draw(renderer)
            for point in points:
                point.draw(renderer)
            for line in lines:
                line.draw(renderer)

        global elapsed_time
        # update error
        if P > 0 or I > 0 or D > 0:
            err = robot.track_error(track)
            u = PID.update(err)
            if not keyboard:
                robot.velocity.w = u

        # update simulation
        world.process()
        renderer.present()

        # update robot coordinate tracker
        world.robotx = robot.sprite.x + int(robot.sprite.size[0]/2)
        world.roboty = robot.sprite.y + int(robot.sprite.size[1]/2)

        # preserve physics by calculating elapsed time and delaying if necessary
        # to limit to only 60 FPS. This prevents the robot velocity being
        # dependent on the frame rate of the simulation
        elapsed_time = time.monotonic() - frame_start
        if elapsed_time < 1/60:
            sdl2.SDL_Delay(int((1/60*1000)-(elapsed_time*1000)))
            elapsed_time = time.monotonic() - frame_start

if __name__ == "__main__":
    sys.exit(run())
