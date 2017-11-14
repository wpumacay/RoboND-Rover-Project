# Do the necessary imports

import socketio
import eventlet
import eventlet.wsgi
from flask import Flask
from io import BytesIO, StringIO


sio = socketio.Server()
app = Flask(__name__)

g_cppBrain_sid = -1

@sio.on( 'cppBrainConnect' )
def cppBrainConnect( sid, pData1, pData2 ) :
    global g_cppBrain_sid

    g_cppBrain_sid = sid
    print( 'cppBrain connected: ', pData1, ' - ', pData2 )

    # start asking for samples
    sio.emit( 'get_samples', data = {}, skip_sid = g_cppBrain_sid )

@sio.on( 'connect' )
def connect( sid, environ ) :
    global g_cppBrain_sid

    print( 'connect ', sid )

    if ( g_cppBrain_sid != -1 ) :
        sio.emit( 'get_samples', data = {}, skip_sid = g_cppBrain_sid )
    else :
        # request for a cppBrainConnect
        sio.emit( 'connect', data = {}, skip_sid = True )

@sio.on( 'telemetry' )
def telemetry( sid, data ) :

    global g_cppBrain_sid
    
    print( 'data: ', data )

    if ( g_cppBrain_sid == -1 ) :

        sio.emit( 'manual', data = {}, skip_sid = True )
    else :

        sio.emit( 'telemetry', data = data, skip_sid = sid )

@sio.on( 'cppBrainControls' )
def cppBrainControls( sid, pData ) :
    global g_cppBrain_sid

    sio.emit( 'data', data = pData, skip_sid = g_cppBrain_sid )


if __name__ == '__main__':

    # wrap Flask application with socketio's middleware
    app = socketio.Middleware( sio, app )

    # deploy as an eventlet WSGI server
    eventlet.wsgi.server( eventlet.listen( ( '', 4567 ) ), app )