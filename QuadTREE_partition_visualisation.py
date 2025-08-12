#quadtree
import pygame
import random
import math

# Configuration
WIDTH, HEIGHT = 800, 600
WHITE = (255, 255, 255)
GRAY = (180, 180, 180)
BLUE = (50, 100, 255)
GREEN = (50, 255, 50)
DRONE_RADIUS = 5

# QuadTree Classes
class Point:
    def __init__(self, x, y, drone=None):
        self.x = x
        self.y = y
        self.drone = drone

class Boundary:
    def __init__(self, x, y, w, h):
        self.x = x
        self.y = y
        self.w = w
        self.h = h

    def contains(self, point):
        return (self.x - self.w <= point.x <= self.x + self.w and
                self.y - self.h <= point.y <= self.y + self.h)

    def intersects(self, range):
        return not (range.x - range.w > self.x + self.w or
                    range.x + range.w < self.x - self.w or
                    range.y - range.h > self.y + self.h or
                    range.y + range.h < self.y - self.h)

class QuadTree:
    def __init__(self, boundary, capacity):
        self.boundary = boundary
        self.capacity = capacity
        self.points = []
        self.divided = False

    def subdivide(self):
        x, y, w, h = self.boundary.x, self.boundary.y, self.boundary.w, self.boundary.h
        self.northeast = QuadTree(Boundary(x + w/2, y - h/2, w/2, h/2), self.capacity)
        self.northwest = QuadTree(Boundary(x - w/2, y - h/2, w/2, h/2), self.capacity)
        self.southeast = QuadTree(Boundary(x + w/2, y + h/2, w/2, h/2), self.capacity)
        self.southwest = QuadTree(Boundary(x - w/2, y + h/2, w/2, h/2), self.capacity)
        self.divided = True

    def insert(self, point):
        if not self.boundary.contains(point):
            return False

        if len(self.points) < self.capacity:
            self.points.append(point)
            return True
        else:
            if not self.divided:
                self.subdivide()

            return (self.northeast.insert(point) or
                    self.northwest.insert(point) or
                    self.southeast.insert(point) or
                    self.southwest.insert(point))

    def draw(self, screen):
        rect = pygame.Rect(self.boundary.x - self.boundary.w, self.boundary.y - self.boundary.h,
                           self.boundary.w * 2, self.boundary.h * 2)
        pygame.draw.rect(screen, GREEN, rect, 1)

        if self.divided:
            self.northeast.draw(screen)
            self.northwest.draw(screen)
            self.southeast.draw(screen)
            self.southwest.draw(screen)

# Main Visualization
def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("QuadTree Partition Visualization")
    clock = pygame.time.Clock()

    points = [Point(random.randint(0, WIDTH), random.randint(0, HEIGHT)) for _ in range(100)]
    boundary = Boundary(WIDTH / 2, HEIGHT / 2, WIDTH / 2, HEIGHT / 2)
    qtree = QuadTree(boundary, 4)

    for point in points:
        qtree.insert(point)

    running = True
    while running:
        screen.fill(WHITE)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        qtree.draw(screen)
        for point in points:
            pygame.draw.circle(screen, BLUE, (int(point.x), int(point.y)), DRONE_RADIUS)

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":
    main()
