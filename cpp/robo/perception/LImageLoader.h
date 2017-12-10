
#pragma once

#include "../common.h"
#include <dirent.h>
#include <string>

using namespace std;

namespace robo
{



	namespace perception
	{



		class LImageLoader
		{

			private :

			vector<string> m_imgsIDs;
			vector<string> m_calImgsRocksIDs;
			vector<string> m_calImgsGridIDs;
			string m_resPath;
			string m_calPath;

			public :


			LImageLoader( string pResPath, string pCalPath )
			{
				m_resPath = pResPath;
				m_calPath = pCalPath;

				// Load all images from the test_dataset folder ***************************

				DIR *_dir;
				dirent *_ent;
				_dir = opendir( m_resPath.c_str() );
				if ( _dir == NULL )
				{
					cout << "LImageLoader> couldnt open dir: " << m_resPath << endl;
				}

				_ent = readdir( _dir );

				while ( _ent != NULL )
				{
					string _fileStr( _ent->d_name );
					// cout << "looking at file: " << _fileStr << endl;

					if ( _fileStr.find( ".jpg" ) != string::npos )
					{
						m_imgsIDs.push_back( string( m_resPath + _fileStr ) );
					}

					_ent = readdir( _dir );
				}
				// ************************************************************************

				// Load all images from the calibration folder ****************************

				_dir = opendir( m_calPath.c_str() );
				if ( _dir == NULL )
				{
					cout << "LImageLoader> couldnt open dir: " << m_calPath << endl;
				}

				delete _ent;

				_ent = readdir( _dir );

				while ( _ent != NULL )
				{
					string _fileStr( _ent->d_name );
					
					if ( _fileStr.find( ".jpg" ) != string::npos )
					{
						if ( _fileStr.find( "grid" ) != string::npos )
						{
							m_calImgsGridIDs.push_back( string( m_calPath + _fileStr ) );
						}
						else if ( _fileStr.find( "rock" ) != string::npos )
						{
							m_calImgsRocksIDs.push_back( string( m_calPath + _fileStr ) );
						}
					}

					_ent = readdir( _dir );
				}

				// ************************************************************************
			};


			void dumpInfo()
			{
				cout << "LImageLoader::dumpInfo> ..." << endl;

				for ( int q = 0; q < m_imgsIDs.size(); q++ )
				{
					cout << "path: " << m_imgsIDs[q] << endl;
				}

				cout << "LImageLoader::dumpInfo> done" << endl;
			}

			cv::Mat getRandomImage()
			{
				cv::Mat _img = cv::imread( m_imgsIDs[RAND( m_imgsIDs.size() )], cv::IMREAD_COLOR );

				return _img;
			}

			cv::Mat getCalibrationImage_rock()
			{
				cv::Mat _img = cv::imread( m_calImgsRocksIDs[1], cv::IMREAD_COLOR );

				return _img;
			}

			cv::Mat getCalibrationImage_grid()
			{
				cv::Mat _img = cv::imread( m_calImgsGridIDs[0], cv::IMREAD_COLOR );

				return _img;
			}

		};









	}








}