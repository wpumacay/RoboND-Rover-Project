
#pragma once

#include <opencv2/opencv.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>
#include <cmath>
#include <algorithm>

#define PATH_TEST_DATASET "../../test_dataset/IMG_2/"
#define PATH_CALIBRATION "../../calibration_images/"

#define PERSPECTIVE_TRANSFORM_DESTINATION_SIZE 5
#define PERSPECTIVE_TRANSFORM_BOTTOM_OFFSET 6

#define THRESHOLDING_R 160
#define THRESHOLDING_G 160
#define THRESHOLDING_B 160

#define RAND( X ) rand() % X