

#include "perception/LImageLoader.h"

using namespace std;

int main()
{

	cout << "initializing image loader" << endl;

	robo::perception::LImageLoader _imgLoader( string( PATH_TEST_DATASET ), string( PATH_CALIBRATION ) );

	cout << "done" << endl;

	cv::namedWindow( "SampleImg" );
	cv::imshow( "SampleImg", _imgLoader.getRandomImage() );
	cv::waitKey( 0 );

	return 0;

}