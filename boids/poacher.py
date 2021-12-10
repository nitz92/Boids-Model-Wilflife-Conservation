# -*- coding: utf-8 -*-

import math
from pyglet.gl import (
    glPushMatrix, glPopMatrix, glBegin, glEnd, glColor3f,
    glVertex2f, glTranslatef, glRotatef,
    GL_LINE_LOOP, GL_LINES, GL_TRIANGLES)

from . import vector

_POACHER_RANGE = 250.0
_POACHER_VIEW_ANGLE = 320
_POACHER_COLLISION_DISTANCE = 45.0
_OBSTACLE_COLLISION_DISTANCE = 250.0
_MAX_COLLISION_VELOCITY = 1.0
_CHANGE_VECTOR_LENGTH = 15.0
_MAX_SPEED = 400.0
_MIN_SPEED = 200.0
_BOUNDARY_SLOP = 50.0

_COHESION_FACTOR = 0.03
_ALIGNMENT_FACTOR = 0.045
_POACHER_AVOIDANCE_FACTOR = 5.5
_OBSTACLE_AVOIDANCE_FACTOR = 1000.0
_ATTRACTOR_FACTOR = 0.01

class Poacher:
    def __init__(self,
                 position=[100.0, 100.0],
                 bounds=[500, 500],
                 velocity=[0.0, 0.0],
                 size=15.0,
                 color=[1.0, 0.0, 0.0]):
        self.position = position
        self.wrap_bounds = [i + _BOUNDARY_SLOP for i in bounds]
        self.velocity = velocity
        self.size = size
        self.color = color
        self.change_vectors = []
        self.visualise = True

    def __repr__(self):
        return "Poacher: position={}, velocity={}, color={}".format(
            self.position, self.velocity, self.color)

    def render_velocity(self):
        glColor3f(0.6, 0.6, 0.6)
        glBegin(GL_LINES)
        glVertex2f(0.0, 0.0)
        glVertex2f(0.0, _POACHER_RANGE)
        glEnd()


    def render_view(self):
        glColor3f(0.6, 0.5, 0.1)
        glBegin(GL_LINE_LOOP)

        step = 10
        # render a circle for the poacher's view
        for i in range(-_POACHER_VIEW_ANGLE, _POACHER_VIEW_ANGLE + step, step):
            glVertex2f(_POACHER_RANGE * math.sin(math.radians(i)),
                       (_POACHER_RANGE * math.cos(math.radians(i))))
        glVertex2f(0.0, 0.0)
        glEnd()


    def render_change_vectors(self):
        glBegin(GL_LINES)

        color = [0.0, 0.0, 0.0]
        for i, (factor, vec) in enumerate(self.change_vectors):
            color[i % 3] = 1.0
            glColor3f(*color)
            glVertex2f(0.0, 0.0)
            glVertex2f(*[i * factor * _CHANGE_VECTOR_LENGTH for i in vec])
            color[i % 3] = 0.0
        glEnd()


    def render_poacher(self):
        glBegin(GL_TRIANGLES)
        glColor3f(*self.color)
        glVertex2f(-(self.size), 0.0)
        glVertex2f(self.size, 0.0)
        glVertex2f(0.0, self.size * 3.0)
        glEnd()


    def draw(self, show_velocity=False, show_view=False, show_vectors=False):
        glPushMatrix()
        # apply the transformation for the poacher
        glTranslatef(self.position[0], self.position[1], 0.0)
        if show_vectors:
            self.render_change_vectors()

        glRotatef(math.degrees(math.atan2(self.velocity[0], self.velocity[1])), 0.0, 0.0, -1.0)

        # render the poacher's velocity
        if show_velocity:
            self.render_velocity()

        # render the poacher's view
        if show_view:
            self.render_view()

        # render the poacher itself
        self.render_poacher()
        glPopMatrix()


    def determine_nearby_poachers(self, all_poachers):
        """Note, this can be done more efficiently if performed globally,
        rather than for each individual poacher.
        """

        for poacher in all_poachers:
            diff = (poacher.position[0] - self.position[0], poacher.position[1] - self.position[1])
            if (poacher != self and
                    vector.magnitude(*diff) <= _POACHER_RANGE and
                    vector.angle_between(self.velocity, diff) <= _POACHER_VIEW_ANGLE):
                yield poacher
        return


    def average_position(self, nearby_poachers):
        # take the average position of all nearby poachers, and move the poacher towards that point
        if len(nearby_poachers) > 0:
            sum_x, sum_y = 0.0, 0.0
            for poacher in nearby_poachers:
                sum_x += poacher.position[0]
                sum_y += poacher.position[1]

            average_x, average_y = (sum_x / len(nearby_poachers), sum_y / len(nearby_poachers))
            return [average_x - self.position[0], average_y - self.position[1]]
        else:
            return [0.0, 0.0]

    def average_velocity(self, nearby_poachers):
        # take the average velocity of all nearby poachers
        if len(nearby_poachers) > 0:
            sum_x, sum_y = 0.0, 0.0
            for poacher in nearby_poachers:
                sum_x += poacher.velocity[0]
                sum_y += poacher.velocity[1]

            average_x, average_y = (sum_x / len(nearby_poachers), sum_y / len(nearby_poachers))
            return [average_x - self.velocity[0], average_y - self.velocity[1]]
        else:
            return [0.0, 0.0]


    def avoid_collisions(self, objs, collision_distance):
        # determine nearby objs using distance only
        nearby_objs = (
            obj for obj in objs
            if (obj != self and
                vector.magnitude(obj.position[0] - self.position[0],
                                 obj.position[1] - self.position[1])
                - self.size <= collision_distance))

        c = [0.0, 0.0]
        for obj in nearby_objs:
            diff = obj.position[0] - self.position[0], obj.position[1] - self.position[1]
            inv_sqr_magnitude = 1 / ((vector.magnitude(*diff) - self.size) ** 2)

            c[0] = c[0] - inv_sqr_magnitude * diff[0]
            c[1] = c[1] - inv_sqr_magnitude * diff[1]
        return vector.limit_magnitude(c, _MAX_COLLISION_VELOCITY)

    def attraction(self, attractors):
        # generate a vector that moves the poacher towards the attractors
        a = [0.0, 0.0]
        if not attractors:
            return a

        attractors = [x for x in attractors if vector.magnitude(x.position[0] - self.position[0],
                         x.position[1] - self.position[1]) < 500]

        # a = [1, 2, 3, 4, 5]
        # b = [x for x in a if x % 2 == 0]
        # [2, 4]

        for attractor in attractors:
            a[0] += attractor.position[0] - self.position[0]
            a[1] += attractor.position[1] - self.position[1]
            if vector.magnitude(attractor.position[0] - self.position[0],
                             attractor.position[1] - self.position[1]) < 35:
                attractor.visualise = False

        return a


    def update(self, dt, all_poachers, attractors, rangers):
        nearby_poachers = list(self.determine_nearby_poachers(all_poachers))

        #update the poacher's direction based on several behavioural rules
        cohesion_vector = self.average_position(nearby_poachers)
        alignment_vector = self.average_velocity(nearby_poachers)
        attractor_vector = self.attraction(attractors)
        poacher_avoidance_vector = self.avoid_collisions(all_poachers, _POACHER_COLLISION_DISTANCE)
        obstacle_avoidance_vector = self.avoid_collisions(rangers, _OBSTACLE_COLLISION_DISTANCE)

        self.change_vectors = [
            (_COHESION_FACTOR, cohesion_vector),
            (_ALIGNMENT_FACTOR, alignment_vector),
            (_ATTRACTOR_FACTOR, attractor_vector),
            (_POACHER_AVOIDANCE_FACTOR, poacher_avoidance_vector),
            (_OBSTACLE_AVOIDANCE_FACTOR, obstacle_avoidance_vector)]

        for factor, vec in self.change_vectors:
            self.velocity[0] += factor *vec[0]
            self.velocity[1] += factor *vec[1]

        # ensure that the poacher's velocity is <= _MAX_SPEED
        self.velocity = vector.limit_magnitude(self.velocity, _MAX_SPEED, _MIN_SPEED)

        # move the poacher to its new position, given its current velocity,
        # taking into account the world boundaries
        for i in range(0, len(self.position)):
            self.position[i] += dt * self.velocity[i]
            if self.position[i] >= self.wrap_bounds[i]:
                self.position[i] = (self.position[i] % self.wrap_bounds[i]) - _BOUNDARY_SLOP
            elif self.position[i] < -_BOUNDARY_SLOP:
                self.position[i] = self.position[i] + self.wrap_bounds[i] + _BOUNDARY_SLOP
