

import numpy as np

from RoverAI_States import *

class RoverAI :

    ST_LOOKING_FOR_PATH = 'lookingForPath'
    ST_FORWARD          = 'forward'
    ST_BRAKING          = 'braking'
    ST_REACHING_ROCK    = 'reachingRock'
    ST_PICKING_ROCK     = 'pickingRock'
	ST_TEST = 'test'

	def __init__( self, robot ) :

		self.m_robot = robot
		self.m_data  = None

		self.m_states = {}
        self.m_states[RoverAI_FSM.ST_LOOKING_FOR_PATH]  = STLookingForPath( self, robot )
        self.m_states[RoverAI_FSM.ST_FORWARD]           = STForward( self, robot )
        self.m_states[RoverAI_FSM.ST_BRAKING]           = STBraking( self, robot )
        self.m_states[RoverAI_FSM.ST_REACHING_ROCK]     = STReachingRock( self, robot )
        self.m_states[RoverAI_FSM.ST_PICKING_ROCK]      = STPickingRock( self, robot )
        self.m_states[RoverAI_FSM.ST_TEST]              = STTest( self, robot )
        
        self.m_currentState = None
		self.m_currentStateId = ''
		self.m_fsm_change_successful = True

		self.setCurrentState( RoverAI_FSM.ST_LOOKING_FOR_PATH )

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

        self.m_currentState.update( dt, roverData )

		if ( self.m_currentState.state == RoverFSMState.ST_FINISHED ) :
                
            self.m_fsm_change_successful = False

			if self.m_currentStateId == RoverAI_FSM.ST_LOOKING_FOR_PATH :

				if self.m_currentState.status == 'found_navigable_area' :
	                self.setCurrentState( RoverAI_FSM.ST_FORWARD )

	            else :
	                self.setCurrentState( RoverAI_FSM.ST_LOOKING_FOR_PATH )

	        elif self.m_currentStateId == RoverAI_FSM.ST_FORWARD :

	            if self.m_currentState.status == 'no_navigable_area' :
	                self.setCurrentState( RoverAI_FSM.ST_BRAKING )

	            #elif self.m_currentState.status == 'rock_in_area' :
	            #    self.setCurrentState( RoverAI_FSM.ST_REACHING_ROCK )

	        elif self.m_currentStateId == RoverAI_FSM.ST_BRAKING :

	            if self.m_currentState.status == 'fully_stopped' :
	                self.setCurrentState( RoverAI_FSM.ST_LOOKING_FOR_PATH )

	        #elif self.m_currentStateId == RoverAI_FSM.ST_REACHING_ROCK :

	        #    if self.m_currentState.status == 'rock_reachable' :
	        #        self.setCurrentState( RoverAI_FSM.ST_PICKING_ROCK )

	        #    elif self.m_currentState.status == 'rock_out_of_range' :
	        #        self.setCurrentState( RoverAI_FSM.ST_LOOKING_FOR_PATH )

	        #elif self.m_currentStateId == RoverAI_FSM.ST_PICKING_ROCK :

	        #    if self.m_currentState.status == 'rock_picked' :
			#		self.setCurrentState( RoverAI_FSM.ST_LOOKING_FOR_PATH )

			if not self.m_fsm_change_successful :
				# an error occurred in our fsm logic, check which state was it
				print( 'RoverAI::update> error - unsuccessful state change: ', 
					   self.m_currentStateId, ' - ', self.m_currentState.status )
