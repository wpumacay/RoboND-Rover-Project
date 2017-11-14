
#include "../build/include/sio_client.h"

#include <iostream>
#include <string>

using namespace std;

sio::client g_h;


void onConnect( sio::event &ev )
{
	cout << "connect!" << endl;

	sio::message::list _li( "sports" );
	_li.push( sio::string_message::create( "economics" ) );

	g_h.socket()->emit( "cppBrainConnect", _li );
}

void onTelemetry( sio::event &ev )
{
	cout << "telemetry received" << endl;

}

int main()
{
	g_h.connect( "http://127.0.0.1:4567" );

	g_h.socket()->on( "connect", &onConnect );
	g_h.socket()->on( "telemetry", &onTelemetry );

	while ( true );
	
	return 0;
}
