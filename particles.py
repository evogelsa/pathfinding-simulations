import sys
import sdl2
import sdl2.ext.particles


# particle class for leaving a trail
class CParticle(sdl2.ext.particles.Particle):
    def __init__(self, entity, x, y, vx, vy, ptype, life):
        super(CParticle, self).__init__(x, y, life)
        self.entity = entity
        self.type = ptype
        self.vx = vx
        self.vy = vy

# entity super class of cparticle
class EParticle(sdl2.ext.Entity):
    def __init__(self, world, x, y, vx, vy, ptype, life):
        self.cparticle = CParticle(self, x, y, vx, vy, ptype, life)

# render system which handles the particle trail
class ParticleRenderSystem(sdl2.ext.particles.ParticleEngine):
    def __init__(self, renderer, images):
        super(ParticleRenderSystem, self).__init__()
        self.componenttypes = (CParticle,)
        self.renderer = renderer
        self.images = images

    # process function called by world
    def process(self, world, components):
        r = sdl2.SDL_Rect()
        dorender = sdl2.SDL_RenderCopy

        sdlrenderer = self.renderer.sdlrenderer
        images = self.images

        #  self.renderer.clear(0x0)
        for particle in components:
            r.x = int(particle.x)
            r.y = int(particle.y)

            img = images[particle.type]
            r.w, r.h = img.size
            dorender(sdlrenderer, img.texture, None, r)
        #  self.renderer.present()

# creates a new particle
def createparticles(world, deadones, count=None):
    if deadones is not None:
        count = len(deadones)
    for _ in range(count):
        x = world.robotx
        y = world.roboty
        vx = 0
        vy = 0
        if '-plife' in sys.argv:
            life = int(sys.argv[sys.argv.index('-plife')+1])
        else:
            life = 250
        ptype = 0
        EParticle(world, x, y, vx, vy, ptype, life)

# updates particle position if velocity is given
def updateparticles(world, particles):
    for p in particles:
        p.x += p.vx
        p.y += p.vy

# deletes particles with expired lifes
def deleteparticles(world, deadones):
    world.delete_entities(p.entity for p in deadones)

def make_particle_engine():
    # create a particle engine to handle the particle trail
    particleengine = sdl2.ext.particles.ParticleEngine()
    particleengine.createfunc = createparticles
    particleengine.updatefunc = updateparticles
    particleengine.deletefunc = deleteparticles

    return particleengine
