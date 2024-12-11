#!/usr/bin/env python3

# This assignment lets you both define a strategy for picking the next point to explore and determine how you
#  want to chop up a full path into way points. You'll need path_planning.py as well (for calculating the paths)
#
# Note that there isn't a "right" answer for either of these. This is (mostly) a light-weight way to check
#  your code for obvious problems before trying it in ROS. It's set up to make it easy to download a map and
#  try some robot starting/ending points
#
# Given to you:
#   Image handling
#   plotting
#   Some structure for keeping/changing waypoints and converting to/from the map to the robot's coordinate space
#
# Slides

# The ever-present numpy
import numpy as np

# Your path planning code
import path_planning as path_planning
# Our priority queue
import heapq

# Using imageio to read in the image
import imageio
import math

# -------------- Showing start and end and path ---------------
def plot_with_explore_points(im_threshhold, zoom=1.0, robot_loc=None, explore_points=None, best_pt=None):
    """Show the map plus, optionally, the robot location and points marked as ones to explore/use as end-points
    @param im - the image of the SLAM map
    @param im_threshhold - the image of the SLAM map
    @param robot_loc - the location of the robot in pixel coordinates
    @param best_pt - The best explore point (tuple, i,j)
    @param explore_points - the proposed places to explore, as a list"""

    # Putting this in here to avoid messing up ROS
    import matplotlib.pyplot as plt

    fig, axs = plt.subplots(1, 2)
    axs[0].imshow(im_threshhold, origin='lower', cmap="gist_gray")
    axs[0].set_title("original image")
    axs[1].imshow(im_threshhold, origin='lower', cmap="gist_gray")
    axs[1].set_title("threshold image")
    """
    # Used to double check that the is_xxx routines work correctly
    for i in range(0, im_threshhold.shape[1]-1, 10):
        for j in range(0, im_threshhold.shape[0]-1, 2):
            if is_reachable(im_thresh, (i, j)):
                axs[1].plot(i, j, '.b')
    """

    # Show original and thresholded image
    if explore_points is not None:
        for p in explore_points:
            axs[1].plot(p[0], p[1], '.b', markersize=2)

    for i in range(0, 2):
        if robot_loc is not None:
            axs[i].plot(robot_loc[0], robot_loc[1], '+r', markersize=10)
        if best_pt is not None:
            axs[i].plot(best_pt[0], best_pt[1], '*y', markersize=10)
        axs[i].axis('equal')

    for i in range(0, 2):
        # Implements a zoom - set zoom to 1.0 if no zoom
        width = im_threshhold.shape[1]
        height = im_threshhold.shape[0]

        axs[i].set_xlim(width / 2 - zoom * width / 2, width / 2 + zoom * width / 2)
        axs[i].set_ylim(height / 2 - zoom * height / 2, height / 2 + zoom * height / 2)

    plt.show()

# -------------- For converting to the map and back ---------------
def convert_pix_to_x_y(im_size, pix, size_pix):
    """Convert a pixel location [0..W-1, 0..H-1] to a map location (see slides)
    Note: Checks if pix is valid (in map)
    @param im_size - width, height of image
    @param pix - tuple with i, j in [0..W-1, 0..H-1]
    @param size_pix - size of pixel in meters
    @return x,y """
    if not (0 <= pix[0] <= im_size[1]) or not (0 <= pix[1] <= im_size[0]):
        raise ValueError(f"Pixel {pix} not in image, image size {im_size}")

    return [size_pix * pix[i] / im_size[1-i] for i in range(0, 2)]


def convert_x_y_to_pix(im_size, x_y, size_pix):
    """Convert a map location to a pixel location [0..W-1, 0..H-1] in the image/map
    Note: Checks if x_y is valid (in map)
    @param im_size - width, height of image
    @param x_y - tuple with x,y in meters
    @param size_pix - size of pixel in meters
    @return i, j (integers) """
    pix = [int(x_y[i] * im_size[1-i] / size_pix) for i in range(0, 2)]

    if not (0 <= pix[0] <= im_size[1]) or not (0 <= pix[1] <= im_size[0]):
        raise ValueError(f"Loc {x_y} not in image, image size {im_size}")
    return pix


def is_reachable(im, pix):
    """ Is the pixel reachable, i.e., has a neighbor that is free?
    Used for
    @param im - the image
    @param pix - the pixel i,j"""

    # Returns True (the pixel is adjacent to a pixel that is free)
    #  False otherwise
    # You can use four or eight connected - eight will return more points
    # YOUR CODE HERE
    # NOTE it's backwards the im.shape(y, x)
    map_width = im.shape[1]
    map_height = im.shape[0]
    #print(im.shape)
    for neighbor in path_planning.eight_connected(pix):
        x_pix = neighbor[0]
        y_pix = neighbor[1]
        # make sure neighbor pixel is in bounds and is not a wall or unknown space
        if 0 <= x_pix < map_width and 0 <= y_pix < map_height and path_planning.is_free(im, neighbor):
            return True
    return False

# Find all unseen pixels that are adjacent to free pixels using a convolution.
def find_all_possible_goals(im):
    """
    Find all of the places where you have a pixel that is unseen next to a pixel that is free
    It is probably easier to do this, THEN cull it down to some reasonable places to try
    This is because of noise in the map - there may be some isolated pixels
    @param im - thresholded image
    @return - a list of possible goal points (tuples)
    """ 
    # YOUR CODE HERE
    im_threshold = np.array(im)
    # Kernel for 8-connected neighbors
    kernel = np.array([[1, 1, 1],
                       [1, 0, 1],
                       [1, 1, 1]])

    # Create a binary mask of free pixels where 255 in the occupancy grid is 1 and everything else is 0
    free_mask = (im_threshold == 255)

    # Pad the free_mask to handle edge cases during convolution
    padded_mask = np.pad(free_mask, pad_width=1, mode='constant', constant_values=0)

    # Initialize an zero array to store the neighbor counts
    neighbor_count = np.zeros_like(free_mask)

    # Perform the convolution across every pixel
    for i in range(kernel.shape[0]):      # Kernel rows
        for j in range(kernel.shape[1]):  # Kernel columns
            if kernel[i, j] == 1:         # Only consider positions with 1 in the kernel
                row_start = i
                row_end = i + free_mask.shape[0]
                col_start = j
                col_end = j + free_mask.shape[1]
                # slice out top left , top middle, top right; then middle left, middle right; then bottom left, bottom midle, bottom right
                # add the padded mask counts of the above slices to the neighbor array
                # same thing as dot product between the 8 way kernel array across every pixel in the padded array
                neighbor_count += padded_mask[row_start:row_end, col_start:col_end]

    # Identify unseen pixels=128 that are adjacent to at least one free pixel=255
    unseen_mask = (im_threshold == 128)
    adjacent_to_free = (neighbor_count > 0) & unseen_mask

    # swap pixel values 
    possible_goals = np.argwhere(adjacent_to_free)
    return [tuple(p[::-1]) for p in possible_goals]  

# Slow method/Naive way
# def find_all_possible_goals(im):
#     """Find all unseen pixels that are adjacent to free pixels.
#     @param im - thresholded image
#     @return - a list of possible goal points (tuples)"""

#     possible_goals = []

#     # Loop through every pixel in the image (Note: j is the rows and i is the columns)
#     for j in range(im.shape[0]):
#         for i in range(im.shape[1]):
#             pix = (i, j)
#             # Check if the pixel is unseen and reachable
#             if path_planning.is_unseen(im, pix) and is_reachable(im, pix):
#                 possible_goals.append(pix)

#     return possible_goals


def find_best_point(im, possible_points, robot_loc):
    """ Pick one of the unseen points to go to
    @param im - thresholded image
    @param possible_points - possible points to chose from
    @param robot_loc - location of the robot (in case you want to factor that in)
    @return a single (x,y) of all the possible possible points
    """
    # YOUR CODE HERE
    if not possible_points:
        return None  # No possible points to explore

    # Use L2 norm to find the farthest point
    best_point = None
    max_dist = -math.inf
    for point in possible_points:
        distance = math.sqrt((point[0] - robot_loc[0])**2 + (point[1] - robot_loc[1])**2)
        if distance > max_dist:
            max_dist = distance
            best_point = point
    return best_point


def find_waypoints(im, path):
    """ Place waypoints along the path
    @param im - the thresholded image
    @param path - the initial path
    @return - a new path"""
    # Again, no right answer here
    # YOUR CODE HERE
    waypoints = []

    # Always include the start and end points
    waypoints.append(path[0])

    # Add intermediate points at "turns" or a regular interval
    for i in range(1, len(path) - 1):
        prev = path[i - 1]
        curr = path[i]
        next_ = path[i + 1]

        # Check if the direction changes
        if (curr[0] - prev[0], curr[1] - prev[1]) != (next_[0] - curr[0], next_[1] - curr[1]):
            buffer_dist = 4
            safe_point = move_away_from_wall(im, curr, buffer_dist)
            waypoints.append(safe_point)

    # Always include the end point
    waypoints.append(path[-1])

    return waypoints

def move_away_from_wall(im, point, buffer_distance=5):
    """
    Moves a point away from walls by a fixed offset.
    @param im: 2D numpy array of the thresholded map.
    @param point: A tuple (x, y) representing the waypoint.
    @param buffer_distance: Minimum distance from walls.
    @return: Adjusted waypoint (x, y) or original if already safe.
    """
    x, y = point
    map_width, map_height = im.shape

    # Check surrounding cells within buffer_distance
    for i in range(-buffer_distance, buffer_distance + 1):
        for j in range(-buffer_distance, buffer_distance + 1):
            nx, ny = x + i, y + j
            if 0 <= nx < map_width and 0 <= ny < map_height:
                if im[ny, nx] == 0:  # Wall detected
                    # Shift the point directly away from the wall
                    dx = -i if i != 0 else 0
                    dy = -j if j != 0 else 0
                    x += dx
                    y += dy
                    break
    return (x, y)

if __name__ == '__main__':
    # Doing this here because it is a different yaml than JN
    #import yaml_1 as yaml
    import yaml
    im, im_thresh = path_planning.open_image("map.pgm")

    robot_start_loc = (1940, 1953)
    all_unseen = find_all_possible_goals(im_thresh)

    best_unseen = find_best_point(im_thresh, all_unseen, robot_loc=robot_start_loc)
    #best_unseen = (1907, 2130)
    plot_with_explore_points(im_thresh, zoom=0.1, robot_loc=robot_start_loc, explore_points=all_unseen, best_pt=best_unseen)

    path = path_planning.dijkstra(im_thresh, robot_start_loc, best_unseen)
    waypoints = find_waypoints(im_thresh, path)
    path_planning.plot_with_path(im, im_thresh, zoom=0.1, robot_loc=robot_start_loc, goal_loc=best_unseen, path=waypoints)

    # Depending on if your mac, windows, linux, and if interactive is true, you may need to call this to get the plt
    #plt.show()

    print("Done")