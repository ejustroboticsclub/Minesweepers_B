#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
import pygame

# Initialize Pygame
pygame.init()

# Constants
GRID_SIZE = 10
CELL_SIZE = 50  # 50x50 pixels per cell
SCREEN_SIZE = GRID_SIZE * CELL_SIZE
screen = pygame.display.set_mode((SCREEN_SIZE, SCREEN_SIZE))

# Global list to store mine data
mines = []

def mine_callback(msg):
    global mines
    # Extract the x, y position and the boolean (above or below ground)
    x_idx, y_idx = int(msg.x), int(msg.y)
    above = msg.z > 0  # Assuming z > 0 means above ground; otherwise below
    mines.append({'x': x_idx, 'y': y_idx, 'above': above})

def get_grid_index(x, y):
    return int(x), int(y)

def main():
    rospy.init_node('minesweeper_grid', anonymous=True)
    
    # Subscribe to the topic (change 'mine_topic' to your actual topic name)
    rospy.Subscriber("mine_topic", Point, mine_callback)

    running = True
    while not rospy.is_shutdown() and running:
        screen.fill((255, 255, 255))  # Clear screen

        for mine in mines:
            x_idx, y_idx = get_grid_index(mine['x'], mine['y'])
            color = (255, 255, 0) if mine['above'] else (255, 0, 0)
            pygame.draw.rect(screen, color, pygame.Rect(x_idx * CELL_SIZE, y_idx * CELL_SIZE, CELL_SIZE, CELL_SIZE))

        # Draw grid lines
        for x in range(0, SCREEN_SIZE, CELL_SIZE):
            pygame.draw.line(screen, (0, 0, 0), (x, 0), (x, SCREEN_SIZE))
            pygame.draw.line(screen, (0, 0, 0), (0, x), (SCREEN_SIZE, x))

        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        pygame.display.flip()

    pygame.quit()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
