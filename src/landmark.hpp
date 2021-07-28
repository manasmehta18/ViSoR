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
    Landmark(cv::KeyPoint kpt, cv::Point3f locRel, cv::Point3f locGlob) {
        this->kpt = kpt;
        this->locRel = locRel;
        this->locGlob = locGlob;
    }

    cv::KeyPoint getKpt(void) {
        return this->kpt;
    }

    cv::Point3f getLocRel(void) {
        return this->locRel;
    }

    cv::Point3f getLocGlob(void) {
        return this->locGlob;
    }

private:
    cv::KeyPoint kpt;       /**< keypoint of the landmark*/
    cv::Point3f locRel;            /**< 3D location of the landmark relative to the pose*/
    cv::Point3f locGlob;           /**< 3D global location of the landmark*/
};

#endif