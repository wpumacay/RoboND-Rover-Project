

#include "perception/LPerceptionHandler.h"

using namespace std;

int main()
{

	robo::perception::LPerceptionHandler _pHandler;

	//_pHandler.test_perspectiveTransform();
	//_pHandler.test_color_thresholding();
	//_pHandler.test_coord_transformation();
	_pHandler.test_pick_ranges();

	return 0;

}