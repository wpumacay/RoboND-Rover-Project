import numpy as np

from RoverAI import *

g_roverController = RoverMotionController()


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step( Rover ):

    global g_roverController

    if Rover is not None :
        g_roverController.update( Rover.time_struct['delta'], Rover )

    if Rover.near_sample and not Rover.picking_up :
        print( 'sample in range!!!' )

    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample  and not Rover.picking_up:
        g_roverController.ai.navpath.restartTimers();
        Rover.send_pickup = True

    return Rover

