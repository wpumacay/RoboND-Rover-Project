

import numpy as np

from AppParams import *



class RoverNavPath :

    def __init__( self ) :

        self.m_sampleTimer = 0
        self.m_ptsPath = []
        self.m_lastPt = None
        self.m_timeoutExpired = False
        self.m_enabled = True

    def getPoints( self ) :
        return self.m_ptsPath

    def dist( self, p1, p2 ) :
        return np.sqrt( ( p1[0] - p2[0] ) ** 2 + ( p1[1] - p2[1] ) ** 2 )

    def hasTimeoutExpired( self ) :
        return self.m_timeoutExpired

    def restartTimers( self ) :
        self.m_sampleTimer = 0
        self.m_timeoutExpired = False

    def setEnable( self, val ) :
        self.m_enabled = val

    def update( self, dt, roverData ) :

        if not self.m_enabled :
            return

        self.m_sampleTimer += dt

        if self.m_sampleTimer > RoverParams.NAVIGATION_SAMPLE_TIME :

            _newPt = ( roverData.pos[0], roverData.pos[1] )

            if self.m_lastPt != None :
                _dist = self.dist( self.m_lastPt, _newPt )
                #print( '_dist> ', _dist )

            if ( self.m_lastPt == None ) or \
               ( self.dist( self.m_lastPt, _newPt ) > RoverParams.NAVIGATION_DIST_THRESHOLD ):

                self.m_lastPt = ( _newPt[0], _newPt[1] )
                self.m_ptsPath.append( _newPt )
                self.m_sampleTimer = 0
                self.m_timeoutExpired = False

                #print( 'added: ', self.m_lastPt )

            elif self.m_sampleTimer > RoverParams.NAVIGATION_TIMEOUT :

                self.m_timeoutExpired = True

