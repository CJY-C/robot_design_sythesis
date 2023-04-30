from OpenGL.GL import *
import OpenGL.GL as gl
import numpy as np
import pygame
import math
import glm


class Camera:
    def __init__(self, pitch=30.0, yaw=45.0, distance=10.0):
        self.pitch = pitch
        self.yaw = yaw
        self.distance = distance

    def rotate(self, delta_pitch, delta_yaw):
        self.pitch += delta_pitch
        self.yaw += delta_yaw
        self.pitch = max(min(self.pitch, 80.0), -80.0)  # Clamp pitch between -80 and 80 degrees


class Scene:
    def __init__(self, config):
        self.config = config
        self.camera = Camera()

    def draw_cube(self, x, y, z, width, height, length):
        vertices = [
            (x, y, z),
            (x + width, y, z),
            (x + width, y + height, z),
            (x, y + height, z),
            (x, y, z + length),
            (x + width, y, z + length),
            (x + width, y + height, z + length),
            (x, y + height, z + length)
        ]

        edges = (
            (0, 1), (1, 2), (2, 3), (3, 0),  # Bottom
            (4, 5), (5, 6), (6, 7), (7, 4),  # Top
            (0, 4), (1, 5), (2, 6), (3, 7)   # Sides
        )

        faces = [
            (0, 1, 2, 3),
            (4, 5, 6, 7),
            (0, 1, 5, 4),
            (1, 2, 6, 5),
            (2, 3, 7, 6),
            (3, 0, 4, 7)
        ]

        glBegin(GL_QUADS)
        for face in faces:
            for vertex in face:
                glVertex3fv(vertices[vertex])
        glEnd()

        glColor3f(0, 0, 0)
        glBegin(GL_LINES)
        for edge in edges:
            for vertex in edge:
                glVertex3fv(vertices[vertex])
        glEnd()

    # def draw_cube(self, x, y, z, size):
    #     vertices = np.array([
    #         [x, y, z],
    #         [x+size, y, z],
    #         [x+size, y+size, z],
    #         [x, y+size, z],
    #         [x, y, z+size],
    #         [x+size, y, z+size],
    #         [x+size, y+size, z+size],
    #         [x, y+size, z+size],
    #     ], dtype=np.float32)

    #     edges = [
    #         (0, 1), (1, 2), (2, 3), (3, 0),
    #         (4, 5), (5, 6), (6, 7), (7, 4),
    #         (0, 4), (1, 5), (2, 6), (3, 7),
    #     ]

    #     gl.glLineWidth(2)
    #     gl.glBegin(gl.GL_LINES)
    #     for edge in edges:
    #         for vertex in edge:
    #             gl.glVertex3fv(vertices[vertex])
    #     gl.glEnd()


    def render(self):
        gl.glClearColor(0.2, 0.3, 0.3, 1.0)
        gl.glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        width, height = pygame.display.get_surface().get_size()
        gl.glMatrixMode(GL_PROJECTION)
        gl.glLoadIdentity()
        projection_matrix = glm.perspective(glm.radians(45.0), width / height, 0.1, 50.0)
        gl.glMultMatrixf(np.array(projection_matrix, dtype=np.float32))
        # gl.glMatrixMode(GL_PROJECTION)
        # gl.glLoadIdentity()
        # projection_matrix = glm.perspective(glm.radians(45.0), width / height, 0.1, 50.0)
        # gl.glMultMatrixf(glm.value_ptr(projection_matrix))

        camera_position = glm.vec3(self.camera.distance * math.sin(math.radians(self.camera.pitch)) * math.cos(math.radians(self.camera.yaw)),
                                   self.camera.distance * math.sin(math.radians(self.camera.pitch)) * math.sin(math.radians(self.camera.yaw)),
                                   self.camera.distance * math.cos(math.radians(self.camera.pitch)))
        view_matrix = glm.lookAt(camera_position, glm.vec3(0, 0, 0), glm.vec3(0, 0, 1))
        gl.glMatrixMode(GL_MODELVIEW)
        gl.glLoadIdentity()
        gl.glMultMatrixf(np.array(view_matrix, dtype=np.float32))
        # gl.glMatrixMode(GL_MODELVIEW)
        # gl.glLoadIdentity()
        # gl.glMultMatrixf(glm.value_ptr(view_matrix))

        # Draw obstacles
        glColor3f(1.0, 0.0, 0.0)  # Red color for obstacles
        nz = len(self.config.obstacle_matrix)
        ny = len(self.config.obstacle_matrix[0])
        nx = len(self.config.obstacle_matrix[0][0])
        for z in range(nz):
            for y in range(ny):
                for x in range(nx):
                    if self.config.obstacle_matrix[z][y][x] == 1:
                        pos_x, pos_y, pos_z = x * self.config.obstacle["width"], y * self.config.obstacle["height"], z * self.config.obstacle["length"]
                        size_x, size_y, size_z = self.config.obstacle["width"], self.config.obstacle["height"], self.config.obstacle["length"]
                        self.draw_cube(pos_x, pos_y, pos_z, size_x, size_y, size_z)

        # Draw target points
        glColor3f(0.0, 1.0, 0.0)  # Green color for target points
        for target in self.config.target_points:
            x, y, z = target['position']
            size_x, size_y, size_z = 0.1, 0.1, 0.1  # Fixed size for target points
            self.draw_cube(x, y, z, size_x, size_y, size_z)

        # pygame.display.flip()
