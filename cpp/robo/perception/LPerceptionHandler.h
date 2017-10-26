
#pragma once

#include "../common.h"

#include "LImageLoader.h"

using namespace std;

namespace robo
{


	namespace perception
	{



		class LPerceptionHandler
		{


			private :

			LImageLoader* m_imgLoader;
			vector<cv::Point2f> m_pt_srcPoints;
			vector<cv::Point2f> m_pt_dstPoints;

			cv::Mat m_calImg_grid;
			cv::Mat m_calImg_rock;

			cv::Mat m_matPerspTransform;

			cv::Scalar m_thresholdLow;
			cv::Scalar m_thresholdHigh;


			public :


			LPerceptionHandler()
			{
				m_imgLoader = new LImageLoader( string( PATH_TEST_DATASET ), string( PATH_CALIBRATION ) );

				m_calImg_grid = m_imgLoader->getCalibrationImage_grid();
				m_calImg_rock = m_imgLoader->getCalibrationImage_rock();

				// src points for the perspective transform, calculated manually using test function
				m_pt_srcPoints.push_back( cv::Point2f( 16, 141 ) );
				m_pt_srcPoints.push_back( cv::Point2f( 303, 141 ) );
				m_pt_srcPoints.push_back( cv::Point2f( 201, 98 ) );
				m_pt_srcPoints.push_back( cv::Point2f( 120, 98 ) );

				// dst points for the perspective transform
				int _dst_size = PERSPECTIVE_TRANSFORM_DESTINATION_SIZE;
				int _btm_offset = PERSPECTIVE_TRANSFORM_BOTTOM_OFFSET;
				int _cal_img_h = m_calImg_grid.rows;
				int _cal_img_w = m_calImg_rock.cols;

				// Put the converted grid square in a box of size 2 * _dst_size in the bottom center of the resulting image
				m_pt_dstPoints.push_back( cv::Point2f( _cal_img_w / 2 - _dst_size, _cal_img_h - _btm_offset ) );
				m_pt_dstPoints.push_back( cv::Point2f( _cal_img_w / 2 + _dst_size, _cal_img_h - _btm_offset ) );
				m_pt_dstPoints.push_back( cv::Point2f( _cal_img_w / 2 + _dst_size, _cal_img_h - _btm_offset - 2 * _dst_size ) );
				m_pt_dstPoints.push_back( cv::Point2f( _cal_img_w / 2 - _dst_size, _cal_img_h - _btm_offset - 2 * _dst_size ) );

				m_matPerspTransform = cv::getPerspectiveTransform( m_pt_srcPoints, m_pt_dstPoints );

				m_thresholdLow = cv::Scalar( THRESHOLDING_R, THRESHOLDING_G, THRESHOLDING_B );
				m_thresholdHigh = cv::Scalar( 255, 255, 255 );
			}

			cv::Mat apply_perspective_transform( cv::Mat pImg )
			{
				cv::Mat _res;

				cv::warpPerspective( pImg, _res, m_matPerspTransform, pImg.size() );

				return _res;
			}


			cv::Mat apply_color_thresholding( cv::Mat pImg )
			{
				cv::Mat _res;

				cv::inRange( pImg, m_thresholdLow, m_thresholdHigh, _res );

				return _res;
			}

			vector<cv::Point2f> coord_transform_rover_coords( cv::Mat pImg )
			{
				vector<cv::Point2f> _res;
				vector<cv::Point2i> _nonZero;
				
				cv::findNonZero( pImg, _nonZero );

				float _w = pImg.cols;
				float _h = pImg.rows;

				for ( int q = 0; q < _nonZero.size(); q++ )
				{
					float _x = _nonZero[q].x;
					float _y = _nonZero[q].y;

					_res.push_back( cv::Point2f( -( _y - _h ), -( _x - _w / 2 ) ) );
				}

				return _res;
			}

			void coord_transform_to_polar( const vector<cv::Point2f>& pts, vector<float>& vDist, vector<float>& vAngle )
			{
				for ( int q = 0; q < pts.size(); q++ )
				{
					float _dist = sqrt( pts[q].x * pts[q].x + pts[q].y + pts[q].y );
					float _angle = atan2( pts[q].y, pts[q].x );

					vDist.push_back( _dist );
					vAngle.push_back( _angle );
				}
			}

			// Testing methods

			void test_perspectiveTransformPickPoints()
			{
				cv::Mat _imgGrid = m_imgLoader->getCalibrationImage_grid();

				for ( int q = 0; q < m_pt_srcPoints.size(); q++ )
				{
					cv::circle( _imgGrid, m_pt_srcPoints[q], 5, cv::Scalar( 0, 0, 255 ), 3 );
				}

				cv::namedWindow( "test_grid_pick" );
				cv::setMouseCallback( "test_grid_pick", LPerceptionHandler::test_pickPoints_callback, 0 );
				cv::imshow( "test_grid_pick", _imgGrid );

				cv::waitKey( 0 );
			}

			void test_perspectiveTransform()
			{
				cv::Mat _imgIn = m_imgLoader->getRandomImage();
				cv::Mat _imgOut;

				cv::warpPerspective( _imgIn, _imgOut, m_matPerspTransform, _imgIn.size() );

				cv::namedWindow( "test_perspective_transform" );
				cv::imshow( "test_perspective_transform", _imgOut );

				cv::waitKey( 0 );
			}

			void test_color_thresholding()
			{
				cv::Mat _imgIn = apply_perspective_transform( m_imgLoader->getRandomImage() );
				cv::Mat _imgOut;

				cv::inRange( _imgIn, m_thresholdLow, m_thresholdHigh, _imgOut );

				cv::namedWindow( "test_color_thresholding" );
				cv::imshow( "test_color_thresholding", _imgOut );

				cv::waitKey( 0 );
			}

			void test_coord_transformation()
			{
				cv::Mat _warped = apply_perspective_transform( m_imgLoader->getRandomImage() );
				cv::Mat _threshed = apply_color_thresholding( _warped );

				vector<cv::Point2f> _vNavPoints = coord_transform_rover_coords( _threshed );

				vector<float> _vDists, _vAngles;

				coord_transform_to_polar( _vNavPoints, _vDists, _vAngles );

				float _avgAngle = 0.0f;
				for ( int q = 0; q < _vAngles.size(); q++ )
				{
					_avgAngle += _vAngles[q];
				}

				_avgAngle = _avgAngle / _vAngles.size();

				float _minX = 1000000;
				float _maxX = -1000000;
				float _minY = 1000000;
				float _maxY = -1000000;

				for ( int q = 0; q < _vNavPoints.size(); q++ )
				{
					// cout << "_vNavPoints[q]-> " << _vNavPoints[q].x << " - " << _vNavPoints[q].y << endl;

					if ( _vNavPoints[q].x < _minX )
					{
						_minX = _vNavPoints[q].x;
					}
					if ( _vNavPoints[q].x > _maxX )
					{
						_maxX = _vNavPoints[q].x;
					}

					if ( _vNavPoints[q].y < _minY )
					{
						_minY = _vNavPoints[q].y;
					}
					if ( _vNavPoints[q].y > _maxY )
					{
						_maxY = _vNavPoints[q].y;
					}
				}

				int _dx = _maxX - _minX;
				int _dy = _maxY - _minY;
				
				cv::Mat _imgDisplay = cv::Mat::zeros( _dy, _dx, CV_8UC3 );
				
				for ( int q = 0; q < _vNavPoints.size(); q++ )
				{
					int _x = _vNavPoints[q].x - _minX;
					_x = ( _x < 0 ) ? 0 : _x;
					_x = ( _x > _dx ) ? _dx : _x;
					int _y = _vNavPoints[q].y - _minY;
					_y = ( _y < 0 ) ? 0 : _y;
					_y = ( _y > _dy ) ? _dy : _y;
					cv::circle( _imgDisplay, cv::Point( _x, _y ), 1, cv::Scalar( 255, 0, 0 ) );
				}

				cv::line( _imgDisplay, 
						  cv::Point( -_minX, -_minY ),
						  cv::Point( 100 * cos( _avgAngle ) - _minX, 100 * sin( _avgAngle ) - _minY ),
						  cv::Scalar( 0, 0, 255 ) );

				cv::namedWindow( "test_coord_transformation" );
				cv::imshow( "test_coord_transformation", _imgDisplay );
				cv::waitKey( 0 );
			}

			static void test_pickPoints_callback( int pEvent, int px, int py, int _foo, void* _fun )
			{
				if ( pEvent != CV_EVENT_LBUTTONDOWN )
				{
					return;
				}

				cout << "px: " << px << " - py: " << py << endl;
			}

			void test_pick_obstacles_ranges()
			{

			}

			static int g_threshold_h_min;
			static int g_threshold_h_max;
			static int g_threshold_s_min;
			static int g_threshold_s_max;
			static int g_threshold_v_min;
			static int g_threshold_v_max;

			static cv::Mat* g_img_rock_hsv;

			void test_pick_ranges()
			{
				cv::Mat _imgRock_bgr = m_imgLoader->getCalibrationImage_rock();
				cv::Mat _imgRock_hsv;
				cv::cvtColor( _imgRock_bgr, _imgRock_hsv, CV_BGR2HSV );

				LPerceptionHandler::g_threshold_h_min = 0;
				LPerceptionHandler::g_threshold_h_max = 255;
				LPerceptionHandler::g_threshold_s_min = 0;
				LPerceptionHandler::g_threshold_s_max = 255;
				LPerceptionHandler::g_threshold_v_min = 0;
				LPerceptionHandler::g_threshold_v_max = 255;

				LPerceptionHandler::g_img_rock_hsv = & _imgRock_hsv;

				cv::namedWindow( "test_pick_ranges" );
				cv::namedWindow( "img_rock" );
				cv::imshow( "img_rock", _imgRock_bgr );

				cv::createTrackbar( "hue_min", "test_pick_ranges", 
									&LPerceptionHandler::g_threshold_h_min, 
									255,
									LPerceptionHandler::test_pick_ranges_callback );
				cv::createTrackbar( "hue_max", "test_pick_ranges", 
									&LPerceptionHandler::g_threshold_h_max, 
									255,
									LPerceptionHandler::test_pick_ranges_callback );

				cv::createTrackbar( "sat_min", "test_pick_ranges", 
									&LPerceptionHandler::g_threshold_s_min, 
									255,
									LPerceptionHandler::test_pick_ranges_callback );
				cv::createTrackbar( "sat_max", "test_pick_ranges", 
									&LPerceptionHandler::g_threshold_s_max, 
									255,
									LPerceptionHandler::test_pick_ranges_callback );

				cv::createTrackbar( "val_min", "test_pick_ranges", 
									&LPerceptionHandler::g_threshold_v_min, 
									255,
									LPerceptionHandler::test_pick_ranges_callback );
				cv::createTrackbar( "val_max", "test_pick_ranges", 
									&LPerceptionHandler::g_threshold_v_max, 
									255,
									LPerceptionHandler::test_pick_ranges_callback );

				LPerceptionHandler::test_pick_ranges_callback( -1, NULL );

				while( true )
				{
					int _key = cv::waitKey( 0 );
					if ( ( char ) _key == 27 )
					{
						break;
					}
				}
			}

			static void test_pick_ranges_callback( int _foo, void* _fun )
			{
				cv::Scalar _low( LPerceptionHandler::g_threshold_h_min, 
								 LPerceptionHandler::g_threshold_s_min, 
								 LPerceptionHandler::g_threshold_v_min );
				cv::Scalar _high( LPerceptionHandler::g_threshold_h_max, 
								  LPerceptionHandler::g_threshold_s_max, 
								  LPerceptionHandler::g_threshold_v_max );

				cv::Mat _img_rock_threshed;
				cv::inRange( *LPerceptionHandler::g_img_rock_hsv, _low, _high, _img_rock_threshed );

				cv::imshow( "test_pick_ranges", _img_rock_threshed );
			}

		};



	}


}


int robo::perception::LPerceptionHandler::g_threshold_h_min = 0;
int robo::perception::LPerceptionHandler::g_threshold_h_max = 255;
int robo::perception::LPerceptionHandler::g_threshold_s_min = 0;
int robo::perception::LPerceptionHandler::g_threshold_s_max = 255;
int robo::perception::LPerceptionHandler::g_threshold_v_min = 0;
int robo::perception::LPerceptionHandler::g_threshold_v_max = 255;
cv::Mat* robo::perception::LPerceptionHandler::g_img_rock_hsv = NULL;