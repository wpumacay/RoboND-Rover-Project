
import numpy as np
import math
from AppParams import *


class RoverFSMState :

    ST_IDLE = 'idle'
    ST_RUNNING = 'running'
    ST_FINISHED = 'finished'

    def __init__( self, ai, robot ) :

        self.m_ai = ai
        self.m_robot = robot

        self.state = RoverFSMState.ST_IDLE
        self.status = ''

    def onEnter( self ) :
        self.state = RoverFSMState.ST_RUNNING

    def onExit( self ) :
        self.state = RoverFSMState.ST_IDLE

    def update( self, dt, roverData ) :
        ## override this
        pass


class STLookingForPath ( RoverFSMState ) :

    def __init__( self, ai, robot ) :

        super( STLookingForPath, self ).__init__( ai, robot )
        self.m_timer = 0.0

    def onEnter( self ) :
        super( STLookingForPath, self ).onEnter()
        self.m_timer = 0.0

    def update( self, dt, roverData ) :

        roverData.throttle = 0
        roverData.brake = 0
        roverData.steer = -15

        if self.m_ai.navpath.hasTimeoutExpired() and \
           self.m_timer <= RoverParams.ESCAPE_BLOCK_TIME :
            self.m_timer += dt

        elif ( len( roverData.nav_angles ) >= RoverParams.THRESHOLD_GO_FORWARD ) :
            self.status = 'found_navigable_area'
            self.state = RoverFSMState.ST_FINISHED

            if self.m_ai.navpath.hasTimeoutExpired() :
                self.m_ai.navpath.restartTimers()


class STForward ( RoverFSMState ) :

    def __init__( self, ai, robot ) :
        super( STForward, self ).__init__( ai, robot )
        self.m_timeout = 0.0
        self.m_rockTimer = 0.0
        self.m_engageForRock = False
        self.m_lastRef = 0.0

    def onEnter( self ) :
        super( STForward, self ).onEnter()
        self.m_robot.restartNavigationController()
        self.m_timeout = 0.0
        self.m_ai.navpath.setEnable( True )
        self.m_rockTimer = 0.0
        self.m_engageForRock = False
        self.m_lastRef = 0.0

    def update( self, dt, roverData ) :

        if len( roverData.nav_angles ) < roverData.stop_forward and not self.m_engageForRock :
            self.status = 'no_navigable_area'
            self.state = RoverFSMState.ST_FINISHED

            return

        if self.m_ai.navpath.hasTimeoutExpired() :
            self.status = 'timeout'
            self.state = RoverFSMState.ST_FINISHED
            print( 'timeout expired :(' )
            return

        if roverData.picking_up :
            return

        self.m_timeout += dt
        _v = roverData.vel
        _theta = roverData.yaw

        _vref = RoverParams.CRUISE_SPEED

        if roverData.sample_in_range['exists'] :

            print( 'going for the rock' )
            _thetaref = roverData.sample_in_range['dir'] * 180 / np.pi
            _vref = RoverParams.APPROACH_SPEED
            self.m_engageForRock = True
            self.m_rockTimer = 0
            self.m_lastRef = _thetaref

        else :
            if self.m_engageForRock and self.m_rockTimer <= RoverParams.PICK_ROCK_TIMEOUT :
                self.m_rockTimer += dt
                _thetaref = self.m_lastRef
                if self.m_rockTimer > RoverParams.PICK_ROCK_TIMEOUT :
                    self.m_rockTimer = 0
                    self.m_engageForRock = False
            else :
                _thetaref = np.mean( roverData.nav_angles * 180 / np.pi ) + RoverParams.OFFSET_WALL_LEFT
            
        [u_throttle, u_brake, u_steer] = self.m_robot.navigationController( _v, _theta , _vref, _thetaref )
        roverData.throttle = u_throttle
        roverData.steer = u_steer
        roverData.brake = u_brake

class STBraking ( RoverFSMState ) :

    def __init__( self, ai, robot ) :
        super( STBraking, self ).__init__( ai, robot )
        self.m_timer = 0

    def onEnter( self ) :
        super( STBraking, self ).onEnter()
        self.m_timer = 0
        self.m_ai.navpath.setEnable( False )

    def update( self, dt, roverData ) :
        if roverData.vel > 0.2 :
            _v = roverData.vel
            _vref = 0.0
            [u_throttle, u_brake, u_steer] = self.m_robot.navigationController( _v, 0 , _vref, 0 )
            roverData.throttle = u_throttle
            roverData.steer = 0
            roverData.brake = u_brake

        else :
            roverData.throttle = 0
            roverData.brake = roverData.brake_set
            roverData.steer = 0
            if ( roverData.vel < 0.1 ) :
                self.m_timer += dt
                if self.m_timer > 3.0 :
                    self.status = 'fully_stopped'
                    self.state = RoverFSMState.ST_FINISHED


class STTest ( RoverFSMState ) :

    def __init__( self, ai, robot ) :
        super( STTest, self ).__init__( ai, robot )
        self.m_metaState = ''
        self.m_timer = 0
        self.m_baseTimer = 0
        self.vApproach = 0.25

    def onEnter( self ) :
        super( STTest, self ).onEnter()
        self.m_metaState = STReachingRock.MST_STOPPING
        self.m_timer = 0
        self.m_baseTimer = 0

        self._trick = False
        self._trickPos = [0, 0]

    def update( self, dt, roverData ) :
        ## if roverData.sample_in_range['exists'] == False :
        ##     self.status = 'rock_out_of_range'
        ##     self.state = RoverFSMState.ST_FINISHED
        ##     return

        if not self._trick :
            self._trickPos[0] = roverData.pos[0] + 10
            self._trickPos[1] = roverData.pos[1]
            self._trick = True

        if roverData.near_sample :
            self.status = 'rock_reachable'
            self.state = RoverFSMState.ST_FINISHED
            return

        _angle = math.atan2( self._trickPos[1] - roverData.pos[1],
                             self._trickPos[0] - roverData.pos[0] )
        _dist = math.sqrt( ( self._trickPos[0] - roverData.pos[0] ) ** 2 +
                           ( self._trickPos[1] - roverData.pos[1] ) ** 2 )

        if ( self.m_metaState == STReachingRock.MST_STOPPING ) :
            roverData.throttle = 0
            roverData.brake = roverData.brake_set
            roverData.steer = 0
            if ( roverData.vel < 0.1 ) :
                self.m_timer += dt
                if self.m_timer > 2.0 :
                    self.m_metaState = STReachingRock.MST_TURNING
                    print( 'changed state to: ', 'turning' )

        elif ( self.m_metaState == STReachingRock.MST_TURNING ) :
            [u_throttle, u_brake, u_steer] = self.m_robot.steerController( _angle, 
                                                                         np.radians( roverData.yaw ) )
            roverData.throttle = u_throttle
            roverData.steer = -u_steer
            roverData.brake = u_brake
            if np.abs( _angle - np.radians( roverData.yaw ) ) < 0.1 :
                self.m_metaState = STReachingRock.MST_APPROACHING
                self.m_baseTimer = 0.5 * ( ( _dist / self.vApproach ) + 0.5 )
                self.m_timer = 0
                print( 'changed state to: ', 'approaching' )

        elif ( self.m_metaState == STReachingRock.MST_APPROACHING ) :
            _v = roverData.vel
            _vref = self.vApproach
            [u_throttle, u_brake, u_steer] = self.m_robot.navigationController( _v, 0 , _vref, 0 )
            roverData.throttle = u_throttle
            roverData.steer = u_steer
            roverData.brake = u_brake
            self.m_timer += dt
            if ( self.m_baseTimer < self.m_timer ) :
                self.m_metaState = STReachingRock.MST_STOPPING
                print( 'changed state to: ', 'stopping' )