

#include <opencv2/opencv.hpp>

#include <iostream>
#include <string>
#include <cstdlib>

using namespace std;

//#define RAND( X ) rand() % X

int main()
{

	string _path = "../../test_dataset/IMG/robocam_2017_05_02_11_16_21_421.jpg";

	cv::Mat _img;

	_img = cv::imread( _path, cv::IMREAD_COLOR );
	if ( _img.empty() )
	{
		cout << "there was a problem opening the image" << endl;
		return -1;
	}

	cv::namedWindow( "Sample img" );

	cv::imshow( "Sample img", _img );

	cv::waitKey( 0 );

	return 0;

}