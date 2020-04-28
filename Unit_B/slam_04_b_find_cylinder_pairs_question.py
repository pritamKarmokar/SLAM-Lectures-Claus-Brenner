# For each cylinder in the scan, find its cartesian coordinates,
# in the world coordinate system. Then, find the closest point
# in the reference cylinder dataset and output it.
# 04_b_find_cylinder_pairs
# Claus Brenner, 14 NOV 2012
from lego_robot import *
from slam_b_library import filter_step
from slam_04_a_project_landmarks import\
     compute_scanner_cylinders, write_cylinders

import logging 
logging.basicConfig(filename='debug.log',filemode='w', level=logging.DEBUG)

def l_print(line):
    logging.debug(line)

# Given a list of cylinders (points) and reference_cylinders:
# For every cylinder, find the closest reference_cylinder and add
# the index pair (i, j), where i is the index of the cylinder, and
# j is the index of the reference_cylinder, to the result list.
def find_cylinder_pairs(cylinders, reference_cylinders, max_radius):
    cylinder_pairs = []

    # --->>> Enter your code here.
    # Make a loop over all cylinders and reference_cylinders.
    # In the loop, if cylinders[i] is closest to reference_cylinders[j],
    # and their distance is below max_radius, then add the
    # tuple (i,j) to cylinder_pairs, i.e., cylinder_pairs.append( (i,j) ).
    # cylinders : scanned cylinders in world coordinate frame
    # reference_cylinders : (1291.0, 1881.0),(482.0, 682.0),(1191.0, 747.0),(1693.0, 1043.0),(383.0, 1458.0),(1805.0, 190.0)
    paired = [0 for i in range(len(cylinders))]
    for j, c in enumerate(reference_cylinders):
        dist = max_radius        
        closest = None
        for i, rc in enumerate(cylinders):
            if paired[i]==1:
                continue
            c_rc_dist = distance(c, rc)
            if c_rc_dist < dist:
                dist = c_rc_dist
                closest = i
        
        if closest is not None and dist < max_radius:
            paired[closest] = 1
            cylinder_pairs.append((closest, j))

    return cylinder_pairs



def distance(a,b):
    d = 0
    for i in range(len(a)):
        d += (a[i] - b[i] )**2
    return d**0.5

if __name__ == '__main__':
    # The constants we used for the filter_step.
    scanner_displacement = 30.0
    ticks_to_mm = 0.349
    robot_width = 150.0

    # The constants we used for the cylinder detection in our scan.    
    minimum_valid_distance = 20.0
    depth_jump = 100.0
    cylinder_offset = 90.0

    # The maximum distance allowed for cylinder assignment.
    max_cylinder_distance = 300.0

    # The start pose we obtained miraculously.
    pose = (1850.0, 1897.0, 3.717551306747922)

    # Read the logfile which contains all scans.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")
    logfile.read("robot4_scan.txt")

    # Also read the reference cylinders (the map).
    logfile.read("robot_arena_landmarks.txt")
    reference_cylinders = [l[1:3] for l in logfile.landmarks]

    # Iterate over all positions.
    out_file = open("find_cylinder_pairs.txt", "w")
    for i in range(len(logfile.scan_data)):
        # Compute the new pose.
        pose = filter_step(pose, logfile.motor_ticks[i],
                           ticks_to_mm, robot_width,
                           scanner_displacement)

        # Extract cylinders, also convert them to world coordinates.
        cartesian_cylinders = compute_scanner_cylinders(
            logfile.scan_data[i],
            depth_jump, minimum_valid_distance, cylinder_offset)
        world_cylinders = [LegoLogfile.scanner_to_world(pose, c)
                           for c in cartesian_cylinders]

        # For every cylinder, find the closest reference cylinder.
        cylinder_pairs = find_cylinder_pairs(
            world_cylinders, reference_cylinders, max_cylinder_distance)
        print (f'{i}, {cylinder_pairs}\n')
        # Write to file.
        # The pose.
        out_file.write(f"F {pose[0]} {pose[1]} {pose[2]}\n")
        # The detected cylinders in the scanner's coordinate system.
        write_cylinders(out_file, "D C", cartesian_cylinders)
        # The reference cylinders which were part of a cylinder pair.
        write_cylinders(out_file, "W C",
            [reference_cylinders[j[1]] for j in cylinder_pairs])

    out_file.close()
