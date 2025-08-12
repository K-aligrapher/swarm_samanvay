import pygame
import random
import math

# Configuration
WIDTH, HEIGHT = 800, 600
NUM_DRONES = 11  # excluding master
DRONE_RADIUS = 10
MASTER_RADIUS = 15
CLICK_RADIUS = 15
MOVE_SPEED = 2

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
BLUE = (50, 100, 255)
RED = (255, 50, 50)
GREEN = (50, 200, 50)
GRAY = (150, 150, 150)

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Drones with Master and Simulation Button")
clock = pygame.time.Clock()

font_small = pygame.font.SysFont(None, 20)
font_medium = pygame.font.SysFont(None, 30)

class Broker:
    def __init__(self):
        self.subscribers = {}
        self.message_queue = []

    def subscribe(self, drone, topic):
        if topic not in self.subscribers:
            self.subscribers[topic] = []
        if drone not in self.subscribers[topic]:
            self.subscribers[topic].append(drone)

    def unsubscribe(self, drone, topic):
        if topic in self.subscribers and drone in self.subscribers[topic]:
            self.subscribers[topic].remove(drone)

    def publish(self, topic, message):
        self.message_queue.append((topic, message))

    def distribute(self):
        for topic, message in self.message_queue:
            for drone in self.subscribers.get(topic, []):
                drone.receive_message(topic, message)
        self.message_queue.clear()

broker = Broker()

class Simulation:
    simulating = False

    @staticmethod
    def draw_button(screen, rect, text, active=True):
        color = GRAY if not active else GREEN
        pygame.draw.rect(screen, color, rect)
        label = font_medium.render(text, True, BLACK)
        text_rect = label.get_rect(center=rect.center)
        screen.blit(label, text_rect)

    @staticmethod
    def button_clicked(rect, mouse_pos):
        return rect.collidepoint(mouse_pos)

class Drone:
    def __init__(self, idx, is_master=False):
        self.id = idx
        self.is_master = is_master
        if self.is_master:
            # Master fixed top center
            self.x = WIDTH // 2
            self.y = HEIGHT // 10
        else:
            self.x = random.randint(50, WIDTH - 50)
            self.y = random.randint(HEIGHT // 2, HEIGHT - 50)
        self.target_x = self.x
        self.target_y = self.y
        self.subscribed_topic = "formation/line" if not is_master else None
        self.group_index = 0
        self.group_size = 1
        if not is_master:
            broker.subscribe(self, self.subscribed_topic)

    def toggle_subscription(self):
        if self.is_master:
            return
        broker.unsubscribe(self, self.subscribed_topic)
        if self.subscribed_topic == "formation/line":
            self.subscribed_topic = "formation/circle"
        else:
            self.subscribed_topic = "formation/line"
        broker.subscribe(self, self.subscribed_topic)

    def receive_message(self, topic, message):
        if self.is_master:
            return
        if topic == self.subscribed_topic and message == "arrange" and Simulation.simulating:
            self.set_target_position()

    def set_target_position(self):
        if self.subscribed_topic == "formation/line":
            spacing = WIDTH // (self.group_size + 1)
            self.target_x = spacing * (self.group_index + 1)
            self.target_y = HEIGHT // 3
        elif self.subscribed_topic == "formation/circle":
            center_x = WIDTH // 2
            center_y = 2 * HEIGHT // 3
            radius = 100
            angle = (2 * math.pi / self.group_size) * self.group_index
            self.target_x = center_x + radius * math.cos(angle)
            self.target_y = center_y + radius * math.sin(angle)

    def move_towards_target(self):
        if self.is_master:
            return
        dx = self.target_x - self.x
        dy = self.target_y - self.y
        dist = math.hypot(dx, dy)
        if dist < MOVE_SPEED or dist == 0:
            self.x, self.y = self.target_x, self.target_y
        else:
            self.x += MOVE_SPEED * dx / dist
            self.y += MOVE_SPEED * dy / dist

    def draw(self, screen):
        if self.is_master:
            pygame.draw.circle(screen, GREEN, (int(self.x), int(self.y)), MASTER_RADIUS)
            label = font_small.render("Master", True, BLACK)
            screen.blit(label, (int(self.x) - MASTER_RADIUS, int(self.y) - MASTER_RADIUS - 20))
        else:
            color = BLUE if self.subscribed_topic == "formation/line" else RED
            pygame.draw.circle(screen, color, (int(self.x), int(self.y)), DRONE_RADIUS)
            label = font_small.render(str(self.id), True, BLACK)
            screen.blit(label, (int(self.x) - DRONE_RADIUS//2, int(self.y) - DRONE_RADIUS//2))

    def is_clicked(self, pos):
     if self.is_master:
        return False
     px, py = pos
    # Slightly larger than visible radius for easier clicks
     click_area = DRONE_RADIUS + 5
     dist = math.hypot(self.x - px, self.y - py)
     return dist <= click_area


def update_group_indices_and_sizes(drones):
    line_group = [d for d in drones if d.subscribed_topic == "formation/line"]
    circle_group = [d for d in drones if d.subscribed_topic == "formation/circle"]

    for i, d in enumerate(line_group):
        d.group_index = i
        d.group_size = len(line_group)
    for i, d in enumerate(circle_group):
        d.group_index = i
        d.group_size = len(circle_group)

    return line_group, circle_group

# Create master drone + children
master_drone = Drone(0, is_master=True)
drones = [master_drone] + [Drone(i) for i in range(1, NUM_DRONES + 1)]

line_group, circle_group = update_group_indices_and_sizes(drones)

# Button rect
button_rect = pygame.Rect(WIDTH - 160, HEIGHT - 60, 140, 40)

running = True
while running:
    screen.fill(WHITE)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            mouse_pos = pygame.mouse.get_pos()

            if Simulation.button_clicked(button_rect, mouse_pos):
                if not Simulation.simulating:
                    # Start simulation: assign targets and start moving
                    line_group, circle_group = update_group_indices_and_sizes(drones)
                    Simulation.simulating = True
                    # Publish arrange message to set targets
                    broker.publish("formation/line", "arrange")
                    broker.publish("formation/circle", "arrange")

            else:
                for d in drones:
                    if d.is_clicked(mouse_pos):
                        d.toggle_subscription()
                        if not Simulation.simulating:
                            line_group, circle_group = update_group_indices_and_sizes(drones)
                        break

        if event.type == pygame.KEYDOWN and Simulation.simulating:
            if event.key == pygame.K_l:
                line_group, circle_group = update_group_indices_and_sizes(drones)
                broker.publish("formation/line", "arrange")
            elif event.key == pygame.K_c:
                line_group, circle_group = update_group_indices_and_sizes(drones)
                broker.publish("formation/circle", "arrange")

    broker.distribute()

    if Simulation.simulating:
        for d in drones:
            d.move_towards_target()

    for d in drones:
        d.draw(screen)

    # Draw UI info
    info_y = 10
    screen.blit(font_medium.render("Click drones to toggle subscription", True, BLACK), (10, info_y))
    screen.blit(font_small.render("Blue = line formation", True, BLUE), (10, info_y + 30))
    screen.blit(font_small.render("Red = circle formation", True, RED), (10, info_y + 50))

    if not Simulation.simulating:
        Simulation.draw_button(screen, button_rect, "Simulate")
    else:
        screen.blit(font_medium.render("Simulation Running", True, GREEN), (WIDTH - 250, HEIGHT - 50))
        #screen.blit(font_small.render("Press L for Line formation", True, BLUE), (WIDTH - 250, HEIGHT - 80))
        #screen.blit(font_small.render("Press C for Circle formation", True, RED), (WIDTH - 250, HEIGHT - 100))

    pygame.display.flip()
    clock.tick(60)

pygame.quit()
