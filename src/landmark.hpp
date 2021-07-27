/**
 * @file landmark.hpp
 * @brief model for a landmark composed of its 3D location and corresponding keypoint
 * @author Manas Mehta, mmehta@wpi.edu
 * @date July 2021
 *
 */

#ifndef __LANDMARK_H__
#define __LANDMARK_H__


#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/features2d.hpp>

/**
 * @class Landmark
 * @brief stores a 3D point and corresponding keypoint 
 */
class Landmark {
public:
    /** @brief Constructor
     */
    Landmark(void) {}

private:
    cv::KeyPoint kpt;       /**< keypoint of the landmark*/
    cv::Point3f loc;            /**< 3D location of the landmark*/
};

#endif