#!/usr/bin/env python3

import argparse
import getch as g

from as2_python_api.drone_interface import DroneInterface
from as2_msgs.msg import ControlMode
import rclpy

TAKE_OFF_HEIGHT = 0.25  # Height in meters
TAKE_OFF_SPEED = 1.0  # Max speed in m/s
SLEEP_TIME = 0.5  # Sleep time between behaviors in seconds
SPEED = 0.5  # Max speed in m/s
HEIGHT = 3.0 # TAKE_OFF_HEIGHT - INIT_Z  # Height in meters
LAND_SPEED = 0.5  # Max speed in m/s

POS_DIFF = 0.25

def drone_start(drone_interface: DroneInterface) -> bool:
    """
    Take off the drone.

    :param drone_interface: DroneInterface object
    :return: Bool indicating if the take off was successful
    """
    print('Start mission')

    # Arm
    print('Arm')
    success = drone_interface.arm()
    print(f'Arm success: {success}')

    # Offboard
    print('Offboard')
    success = drone_interface.offboard()
    print(f'Offboard success: {success}')
    
    # Take Off
    print('Take Off')
    success = drone_interface.takeoff(height=TAKE_OFF_HEIGHT, speed=TAKE_OFF_SPEED)
    print(f'Take Off success: {success}')

    return success


def drone_teleop(drone_interface: DroneInterface) -> bool:
    """
    Read input keys from keyboard and map them into drone commands

    :param drone_interface: DroneInterface object
    :return: Bool indicating if the mission was successful
    """

    print('Reading key from user')
    print('''
          w ⭡ [key] : move positive in X-axis
          s ⭣ [key] : move negative in X-axis
          a ⭠ [key] : move positive in Y-axis
          d ⭢ [key] : move negative in Y-axis
          u : move positive in Z-axis
          j : move negative in Z-axis
          p : exit
          ''')

    TOTAL_X = 0.0
    TOTAL_Y = 0.0
    TOTAL_Z = TAKE_OFF_HEIGHT


    while True:
        key_in = g.getch()
        print(f"Key pressed: {key_in}")

        if key_in == 'p':
            print("Exiting teleop...")
            break

        if key_in == 'w':
            TOTAL_X += POS_DIFF
        elif key_in == 's':
            TOTAL_X -= POS_DIFF
        elif key_in == 'a':
            TOTAL_Y += POS_DIFF
        elif key_in == 'd':
            TOTAL_Y -= POS_DIFF
        elif key_in == 'u':
            TOTAL_Z += POS_DIFF
        elif key_in == 'j':
            TOTAL_Z -= POS_DIFF
        else:
            continue
        
        goal = [TOTAL_X, TOTAL_Y, TOTAL_Z]

        success = drone_interface.go_to.go_to_point(goal, speed=SPEED)
        print(f'Go to success: {success}')
        if not success:
            print('Failed Go To')
            return success

    return True

def drone_end(drone_interface: DroneInterface) -> bool:
    """
    End the mission for a single drone.

    :param drone_interface: DroneInterface object
    :return: Bool indicating if the land was successful
    """
    print('End mission')

    # Land
    print('Land')
    success = drone_interface.land(speed=LAND_SPEED)
    print(f'Land success: {success}')
    if not success:
        return success

    # Manual
    print('Manual')
    success = drone_interface.manual()
    print(f'Manual success: {success}')

    return success


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Single drone mission')

    parser.add_argument('-n', '--namespace',
                        type=str,
                        default='drone0',
                        help='ID of the drone to be used in the mission')
    parser.add_argument('-v', '--verbose',
                        action='store_true',
                        default=False,
                        help='Enable verbose output')
    parser.add_argument('-s', '--use_sim_time',
                        action='store_true',
                        default=True,
                        help='Use simulation time')

    args = parser.parse_args()
    drone_namespace = args.namespace
    verbosity = args.verbose
    use_sim_time = args.use_sim_time

    print(f'Running mission for drone {drone_namespace}')

    rclpy.init()

    uav = DroneInterface(
        drone_id=drone_namespace,
        use_sim_time=use_sim_time,
        verbose=verbosity)

    success = drone_start(uav)
    if success:
        success = drone_teleop(uav)
    success = drone_end(uav)

    uav.shutdown()
    rclpy.shutdown()
    print('Clean exit')
    exit(0)
