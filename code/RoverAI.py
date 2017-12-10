

import numpy as np

from RoverAI_States import *
from RoverNavUtils import *

class RoverAI :

    ST_LOOKING_FOR_PATH = 'lookingForPath'
    ST_FORWARD          = 'forward'
    ST_BRAKING          = 'braking'
    ST_TEST             = 'test'

    def __init__( self, robot ) :

        self.m_robot = robot
        self.m_data  = None

        self.m_states = {}
        self.m_states[RoverAI.ST_LOOKING_FOR_PATH]  = STLookingForPath( self, robot )
        self.m_states[RoverAI.ST_FORWARD]           = STForward( self, robot )
        self.m_states[RoverAI.ST_BRAKING]           = STBraking( self, robot )
        self.m_states[RoverAI.ST_TEST]              = STTest( self, robot )
        
        self.m_currentState = None
        self.m_currentStateId = ''
        self.m_fsm_change_successful = True

        self.navpath = RoverNavPath()

        self.setCurrentState( RoverAI.ST_LOOKING_FOR_PATH )

    def setCurrentState( self, stateId ) :

        if ( self.m_currentState != None ) :
            self.m_currentState.onExit()
        else :
            print( 'RoverAI::setCurrentState> tried to exit a non existing state' )

        if ( stateId not in self.m_states ) :
            print( 'RoverAI::setCurrentState> set a state that doesnt exist' )

        self.m_currentState = self.m_states[stateId]
        self.m_currentStateId = stateId
        self.m_currentState.onEnter()

        self.m_fsm_change_successful = True

    def update( self, dt, data ) :

        self.m_data = data

        if self.m_currentState == None :

            print( 'RoverAI::update> error - it seems there is no state registered' )
            return

        self.m_currentState.update( dt, data )
        self.navpath.update( dt, data )

        if ( self.m_currentState.state == RoverFSMState.ST_FINISHED ) :
                
            self.m_fsm_change_successful = False

            if self.m_currentStateId == RoverAI.ST_LOOKING_FOR_PATH :

                if self.m_currentState.status == 'found_navigable_area' :
                    self.setCurrentState( RoverAI.ST_FORWARD )

                else :
                    self.setCurrentState( RoverAI.ST_LOOKING_FOR_PATH )

            elif self.m_currentStateId == RoverAI.ST_FORWARD :

                if self.m_currentState.status == 'no_navigable_area' or \
                   self.m_currentState.status == 'timeout' :

                    self.setCurrentState( RoverAI.ST_BRAKING )

            elif self.m_currentStateId == RoverAI.ST_BRAKING :

                if self.m_currentState.status == 'fully_stopped' :
                    self.setCurrentState( RoverAI.ST_LOOKING_FOR_PATH )

            if not self.m_fsm_change_successful :
                # an error occurred in our fsm logic, check which state was it
                print( 'RoverAI::update> error - unsuccessful state change: ', 
                       self.m_currentStateId, ' - ', self.m_currentState.status )


class PIDController :

    def __init__( self, Kp = 5.0, Kd = 4.0, Ki = 0.001 ) :

        self.epv = 0.0
        self.eiv = 0.0
        self.edv = 0.0

        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki

    def reset( self ) :
        self.epv = 0.0
        self.eiv = 0.0
        self.edv = 0.0

    def calculate( self, x, xRef, verbose = False ) :
        _epv = x - xRef
        self.edv = _epv - self.epv
        self.epv = _epv
        self.eiv += _epv
        _u = -( self.Kp * self.epv + self.Kd * self.edv + self.Ki * self.eiv )
        if ( verbose ) :
            print( 'x,xRef: ', x, xRef, ' u: ', _u )

        return _u

class RoverMotionController :

    def __init__( self ) :

        self.m_speedController = PIDController()
        self.m_steerController = PIDController( 15.0, 15.0, 0.0 )
        self.ai = RoverAI( self )

    def update( self, dt, roverData ) :
        self.ai.update( dt, roverData )

    def restartNavigationController( self ) :
        self.m_speedController.reset()

    def restartSteerController( self ):
        self.m_steerController.reset()

    def navigationController( self, v, theta, vRef, thetaRef ) :

        u_throttle = self.m_speedController.calculate( v, vRef, False )
        u_brake = 0

        if u_throttle < 0 :
            u_brake = np.clip( -u_throttle ,0, 10 )

        u_throttle = np.clip( u_throttle, 0, 0.2 )
        u_steer = np.clip( thetaRef, -15, 15 )

        return [ u_throttle, u_brake, u_steer ]

    def steerController( self, theta, thetaRef ) :
        u_steer = self.m_steerController.calculate( theta, thetaRef, True )
        u_steer = np.clip( u_steer, -15, 15 )

        return [ 0, 0, u_steer ]

    def positionController( self, xRef, yRef ) :

        u_throttle = 0
        u_steer = 0

        return [u_throttle,u_brake,u_steer]