/**
 * @file ekfSlam.hpp
 * @brief EKF for implementing SLAM using data from viodom
 * @author Manas Mehta, mmehta@wpi.edu
 * @date July 2021
 *
 */

#ifndef __EKFSLAM_H__
#define __EKFSLAM_H__

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <visor/kpt.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/Image.h>
#include "stereodom.hpp"
#include "robustmatcher.hpp"
#include "landmark.hpp"

// Convenient constants
#define RAD2DEG(X) ( (X) * 57.2957795131)
#define DEG2RAD(X) ( (X) * 0.01745329251)
#define LIN2GRAV(X) ( (X) * 0.10197162)


/**
 * @brief Keypoint comparison auxiliar function to sort the sets of keypoints
 * according to their score
 * @param p1 The first keypoint
 * @param p2 The second keypoint
 * @return Boolean true if p1 > p2
 */
bool score_comparator1(const cv::KeyPoint& p1, const cv::KeyPoint& p2) {
    return p1.response > p2.response;
}

/**
 * @class EkfSlam
 * @brief calculates robot's pose and simultaneously creates a map of the environment
 */
class EkfSlam {

public:
	
    /** @brief Constructor
     * @param T Prediction period (seconds)
     * @param poseTopic Name of the IMU data topic
     * @param calibTime Initial calibration time (seconds)
     */
    EkfSlam(std::string& nodeName, std::string& kptTopic) {
        init_ = false;

        fDetector_ = cv::FastFeatureDetector::create(/*5*/);
        fExtractor_ = cv::xfeatures2d::BriefDescriptorExtractor::create(16 /*32*/);

		// EKF parameters
        biaDev_ = 0.00001;//0.000001;
        biaTh_ = 0.005; //0.001;

        // initialize hyperparameters 
        downsampling_ = 2;
        maxFeatures_ = 100;
		
        // Setup data subscribers
        kptSub_ = nh_.subscribe(kptTopic, 10, &EkfSlam::kptDataCallback, this);

        // Setup data publishers
        posePub_ = nh_.advertise<geometry_msgs::TransformStamped>(nodeName + "/slamPose", 1);
        pcPub_ = nh_.advertise<sensor_msgs::PointCloud2>(nodeName + "/point_cloud", 1);
	}


    /** @brief Initialize EKF
     * Input: visor::kpt::ConstPtr keyframe
     * The user must continue calling this function with new gyro data until it returns true
     */
	bool initialize(const visor::kpt::ConstPtr& msg) {
	
		// Initialize state vector CV_ = [x, y, z, lix, liy ...]
        SV_ = std::vector<double>(203, 0.0);
        // ROS_INFO_STREAM("State Vector: initialized (" << SV_[202] << ")");
		
		// // Initialize covariance matrix
        // P_.setIdentity(6, 6);
        // P_(0,0) = M_PI_2;
        // P_(1,1) = M_PI_2;
        // P_(2,2) = M_PI_2;
        // P_(3,3) = 0.01*0.01;
        // P_(4,4) = 0.01*0.01;
        // P_(5,5) = 0.01*0.01;
        // /*P_(3,3) = biaVar_[0];
        // P_(4,4) = biaVar_[1];
        // P_(5,5) = biaVar_[2];*/
		
        if(msg) {
            sensor_msgs::CameraInfo leftInfo, rightInfo;

            leftInfo = msg->infoL;
            rightInfo = msg->infoR;

            calibInit(leftInfo, rightInfo);
        }

        init_ = true;
        ROS_INFO("SLAM EKF INITIALIZED");

        return true;
	}	

    /** @brief Distribute maxFeatures_ among all buckets.
     * We are considering 6 buckets organized in 2 rows and 3 columns.
     * @param[in] srcKpts Sorted key-points
     * @param[out] dstKpts Output bucketed key-points
     * @param[in] width Image width (cols)
     * @param[in] height Image height (rows)
     */
    void kptsBucketing(std::vector<cv::KeyPoint>& srcKpts, std::vector<cv::KeyPoint>& dstKpts,
        int width, int height) {
        const int maxFeatBuck = maxFeatures_ / 6;
        int buckets[6] = { maxFeatBuck, maxFeatBuck, maxFeatBuck, maxFeatBuck, maxFeatBuck, maxFeatBuck };
        for (size_t i = 0; i < srcKpts.size(); i++) {
            int id;

            // Check keypoint y coord
            if (srcKpts[i].pt.y <= height / 2)
                id = 0;
            else
                id = 3;

            // Check keypoint x coord
            if (srcKpts[i].pt.x <= width / 3)
                id += 0;
            else if (srcKpts[i].pt.x <= 2 * width / 3)
                id += 1;
            else
                id += 2;

            // Assign to corresponding bucket
            if (buckets[id] > 0) {
                buckets[id]--;
                dstKpts.push_back(srcKpts[i]);
            }
        }
    }
	
    /** EKF prediction stage based on pose data from viodom
     * @param[in] gx Raw X gyro data (rad/s)
     */
	bool predict(tf::Transform pose) {

        // SV_[0] = pose.getOrigin().x;
        // SV_[1] = pose.getOrigin().y;
        // SV_[2] = pose.getOrigin().z;

	}

//     /** EKF update stage based on accelerometer information
//      * @param[in] ax Raw X accelerometer data (g)
//      * @param[in] ay Raw Y accelerometer data (g)
//      * @param[in] az Raw Z accelerometer data (g)
//      * @param[in] mx Raw X magnetometer data ()
//      * @param[in] my Raw Y magnetometer data ()
//      * @param[in] mz Raw Z magnetometer data ()
//      */
// 	bool update(double ax, double ay, double az, double mx, double my, double mz) {
// 	}
	

    /** pose data callback
     * @param[in] msg TransformStamped data message
     */
	void kptDataCallback(const visor::kpt::ConstPtr& msg) {

        if(!init_) {
            initialize(msg);
        }

        if(msg) {
            ROS_INFO("KEYPOINT RECEIVED");

            // Get pose data
            geometry_msgs::TransformStamped uPose;
            uPose = msg->pose;

            tf::Quaternion qPose;
            qPose.setX(uPose.transform.rotation.x);
            qPose.setY(uPose.transform.rotation.y);
            qPose.setZ(uPose.transform.rotation.z);
            qPose.setW(uPose.transform.rotation.w);
            tf::Vector3 pPose;
            pPose.setX(uPose.transform.translation.x);
            pPose.setY(uPose.transform.translation.y);
            pPose.setZ(uPose.transform.translation.z);
            tf::Transform odomTemp;
            odomTemp.setRotation(qPose);
            odomTemp.setOrigin(pPose);

                 		
		    // ekf predict step
		    predict(odomTemp);

            // get observations - left and right images
            generateLandmarks(msg->imgL, msg->imgR);

            // ekf update step
            // update(LIN2GRAV(msg->linear_acceleration.z), LIN2GRAV(msg->linear_acceleration.x), LIN2GRAV(msg->linear_acceleration.y));

            // send corrected pose to viodom node
            tf::Transform updatedOdomC;

            updatedOdomC = odomTemp;
            publishSlamTf(updatedOdomC);

        } else {
            ROS_WARN("No keyframe received!");
        }  
    }


    /** @brief generate landmarks from left and right images
     * @param leftImg Left image
     * @param rightImg Right image
     */
    void generateLandmarks(const sensor_msgs::Image& leftImg, const sensor_msgs::Image& rightImg) {
        double flow = 0.0;
        lmrksC_.clear();

        // Convert to OpenCV format and rectify both images
        cv::Mat imgL, imgR;
        if (!convertRectifyImages(leftImg, rightImg, imgL, imgR))
            return;

        cv::imshow("Left Image", imgL);
        cv::imshow("Right Image", imgR);
        cv::waitKey(3);

        // Detect key-points in the images
        kptsLeftC_.clear();
        kptsRightC_.clear();
        selectKeypoints(imgL, imgR, kptsLeftC_, kptsRightC_);
        // std::cout << "Keypoints: left (" << kptsLeftC_.size() << "), right (" << kptsRightC_.size() << ")\n";

        // Extract feature descriptors
        extractDescriptors(imgL, imgR, kptsLeftC_, kptsRightC_, descLeftC_, descRightC_);

        // Stereo matching and 3D point cloud generation
        pclC_.clear();
        stereoMatchesC_.clear();
        if (!stereoMatching(kptsLeftC_, kptsRightC_, descLeftC_, descRightC_, stereoMatchesC_, pclC_))
            return;
        //std::cout << "Stereo matches: " stereoMatchesC_.size() << std::endl;

        publishPointCloud(pclC_, leftImg.header);
    }

    /** @brief Publish 3D point cloud
     * @param[in] header Header for the point cloud message
     */
    void publishPointCloud(const std::vector<cv::Point3f> pcl, const std_msgs::Header header)
    {
        // Fill PointCloud message
        sensor_msgs::PointCloud outCloud;
        outCloud.points.resize(pcl.size());
        outCloud.header = header;
        for (size_t i = 0; i < outCloud.points.size(); i++) {
            outCloud.points[i].x = pcl[i].x;
            outCloud.points[i].y = pcl[i].y;
            outCloud.points[i].z = pcl[i].z;
        }

        // Convert PointCloud to PointCloud2
        sensor_msgs::PointCloud2 outCloud2;
        sensor_msgs::convertPointCloudToPointCloud2(outCloud, outCloud2);

        // Publish point cloud
        pcPub_.publish(outCloud2);
    }


    /** @brief Publish 6DOF updated pose as TF and geometry_msgs::TransformStamped
     * @param odom Odometry transform to be published
     */
    void publishSlamTf(const tf::Transform& odom) {

        // // Publish TF
        // tfBr_.sendTransform(
        //     tf::StampedTransform(odom, ros::Time::now(),
        //         srcFrameId_, tgtFrameId_));

        // Publish pose
        geometry_msgs::TransformStamped outTransform;
        outTransform.header.frame_id = "world";
        outTransform.header.stamp = ros::Time::now();
        tf::Quaternion qPose = odom.getRotation();
        outTransform.transform.rotation.x = qPose.x();
        outTransform.transform.rotation.y = qPose.y();
        outTransform.transform.rotation.z = qPose.z();
        outTransform.transform.rotation.w = qPose.w();
        tf::Vector3 pPose = odom.getOrigin();
        outTransform.transform.translation.x = pPose.x();
        outTransform.transform.translation.y = pPose.y();
        outTransform.transform.translation.z = pPose.z();
        posePub_.publish(outTransform);
    }


    /** @brief Converts images to OpenCV format and rectify both images
     * @param[in] leftImg Left image in ROS format
     * @param[in] rightImg Right image in ROS format
     * @param[out] imgL Rectified left image in OpenCV format
     * @param[out] imgR Rectified right image in OpenCV format
     */
    bool convertRectifyImages(const sensor_msgs::Image& leftImg, const sensor_msgs::Image& rightImg, cv::Mat& imgL, cv::Mat& imgR) {
        // Convert to OpenCV format
        cv_bridge::CvImageConstPtr cvbLeft, cvbRight;
        
        try {
            cvbLeft = cv_bridge::toCvCopy(leftImg);
            cvbRight = cv_bridge::toCvCopy(rightImg);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return false;
        }

        // Rectify both images
        cv::remap(cvbLeft->image, imgL, mapL1_, mapL2_, cv::INTER_LINEAR);
        cv::remap(cvbRight->image, imgR, mapR1_, mapR2_, cv::INTER_LINEAR);
        return true;
    }

    /** @brief Detects key-points in the images
     * Key-points are sorted based on their score to get the best maxFeatures_
     * Features are included in buckets in order to force sparseness
     * @param[in] imgLeft Left image
     * @param[in] imgRight Right image
     * @param[out] keypointsLeft Key-points extracted from left image
     * @param[out] keypointsRight Key-points extracted from right image
     */
    void selectKeypoints(cv::Mat& imgLeft, cv::Mat& imgRight,
        std::vector<cv::KeyPoint>& keypointsLeft,
        std::vector<cv::KeyPoint>& keypointsRight)
    {
        // Detect key-points in both images
        std::vector<cv::KeyPoint> kptsL, kptsR;
        fDetector_->detect(imgLeft, kptsL);
        fDetector_->detect(imgRight, kptsR);

        // Sort keypoints according to their score
        std::sort(kptsL.begin(), kptsL.end(), score_comparator1);
        std::sort(kptsR.begin(), kptsR.end(), score_comparator1);

        // Distribute maxFeatures_ in buckets
        const int width = imgLeft.cols;
        const int height = imgLeft.rows;
        kptsBucketing(kptsL, keypointsLeft, width, height);
        kptsBucketing(kptsR, keypointsRight, width, height);

        // keypointsLeft = kptsL;
        // keypointsRight = kptsR;
    }

    /** @brief Extract feature descriptors from image key-points
     * @param[in] imgLeft Left image
     * @param[in] imgRight Right image
     * @param[in] keypointsLeft Key-points extracted from left image
     * @param[in] keypointsRight Key-points extracted from right image
     * @param[out] descriptorsLeft Feature descriptors for key-points from left image
     * @param[out] descriptorsRight Feature descriptors for key-points from right image
     */
    void extractDescriptors(cv::Mat& imgL, cv::Mat& imgR,
        std::vector<cv::KeyPoint>& keypointsLeft,
        std::vector<cv::KeyPoint>& keypointsRight,
        cv::Mat& descriptorsLeft, cv::Mat& descriptorsRight)
    {
        fExtractor_->compute(imgL, keypointsLeft, descriptorsLeft);
        fExtractor_->compute(imgR, keypointsRight, descriptorsRight);
    }

    /** @brief Match key-points L-R descriptors and generate 3D point cloud
     * @param[in] keypointsLeft Key-points extracted from left image
     * @param[in] keypointsRight Key-points extracted from right image
     * @param[in] descriptorsLeft Feature descriptors for key-points from left image
     * @param[in] descriptorsRight Feature descriptors for key-points from right image
     * @param[out] stereoMatches Resulting matches between features
     * @param[out] pcl Resulting 3D point cloud based on stereo matches between features
     */
    bool stereoMatching(std::vector<cv::KeyPoint>& keypointsLeft,
        std::vector<cv::KeyPoint>& keypointsRight,
        cv::Mat& descriptorsLeft, cv::Mat& descriptorsRight,
        std::vector<cv::DMatch>& stereoMatches, std::vector<cv::Point3f>& pcl)
    {
        std::vector<cv::DMatch> matches;
        try {
            matcher_.match(keypointsRight, descriptorsRight, keypointsLeft, descriptorsLeft,
                matches, false);
        } catch (std::exception e) {
            ROS_ERROR("Matching error!");
            return false;
        }

        // Generate stereo point cloud
        float f = PR_.at<double>(0, 0), cx = PR_.at<double>(0, 2), cy = PR_.at<double>(1, 2), b = -PR_.at<double>(0, 3) / f;
        float d, k1 = 1 / f, k2 = f * b;
        for (size_t i = 0; i < matches.size(); i++) {
            const cv::DMatch& match = matches[i];
            const cv::Point2f& pL = keypointsLeft[match.trainIdx].pt;
            const cv::Point2f& pR = keypointsRight[match.queryIdx].pt;
            d = pL.x - pR.x;
            // If key-points y-coords are close enough (stabilized in roll,pitch)
            // and x-coords are far enough (sufficient disparity), it's a match!
            if (fabs(pR.y - pL.y) <= 5 && d > 2.0) {
                stereoMatches.push_back(match);

                // Calculate 3D point
                cv::Point3f p;
                p.z = k2 / d;
                p.x = (pL.x - cx) * p.z * k1;
                p.y = (pL.y - cy) * p.z * k1;
                pcl.push_back(p);

                cv::Point3f gp;
                gp.x = 0;
                gp.y = 0;
                gp.z = 0;

                lmrksC_.push_back(Landmark(keypointsLeft[match.trainIdx], p, gp));
            }
        }

        // ROS_INFO_STREAM("Number of landmarks (" << lmrksC_.size() << ")");
        // ROS_INFO_STREAM("landmark kpt (" << lmrksC_[0].getKpt().pt << ")");
        // ROS_INFO_STREAM("landmark location (" << lmrksC_[0].getLocRel() << ")");
        // ROS_INFO_STREAM("3D landmark location (" << lmrksC_[0].getLocGlob() << ")");

        return true;
    }

    /** @brief Camera calibration 
     * @param leftInfo Left camera calibration data
     * @param rightInfo Right camera calibration data
     */
    bool calibInit(const sensor_msgs::CameraInfo& leftInfo, const sensor_msgs::CameraInfo& rightInfo) {

        try {

            RL_ = cv::Mat(3, 3, CV_64FC1, (void*)leftInfo.R.elems).clone();
            PL_ = cv::Mat(3, 4, CV_64FC1, (void*)leftInfo.P.elems).clone();
            RR_ = cv::Mat(3, 3, CV_64FC1, (void*)rightInfo.R.elems).clone();
            PR_ = cv::Mat(3, 4, CV_64FC1, (void*)rightInfo.P.elems).clone();

            // Obtain the corresponding PL and PR of the downsampled images
            PL_.at<double>(0, 0) = PL_.at<double>(0, 0) / (float)downsampling_;
            PL_.at<double>(0, 2) = PL_.at<double>(0, 2) / (float)downsampling_;
            PL_.at<double>(0, 3) = PL_.at<double>(0, 3) / (float)downsampling_;
            PL_.at<double>(1, 1) = PL_.at<double>(1, 1) / (float)downsampling_;
            PL_.at<double>(1, 2) = PL_.at<double>(1, 2) / (float)downsampling_;
            PR_.at<double>(0, 0) = PR_.at<double>(0, 0) / (float)downsampling_;
            PR_.at<double>(0, 2) = PR_.at<double>(0, 2) / (float)downsampling_;
            PR_.at<double>(0, 3) = PR_.at<double>(0, 3) / (float)downsampling_;
            PR_.at<double>(1, 1) = PR_.at<double>(1, 1) / (float)downsampling_;
            PR_.at<double>(1, 2) = PR_.at<double>(1, 2) / (float)downsampling_;

            // Initialize left and right image remapping (rectification and undistortion)
            cv::initUndistortRectifyMap(cv::Mat(3, 3, CV_64FC1, (void*)leftInfo.K.elems),
                cv::Mat(4, 1, CV_64FC1, (void*)leftInfo.D.data()),
                RL_, PL_,
                cv::Size(leftInfo.width / (float)downsampling_, leftInfo.height / (float)downsampling_),
                CV_16SC2, mapL1_, mapL2_);
            cv::initUndistortRectifyMap(cv::Mat(3, 3, CV_64FC1, (void*)rightInfo.K.elems),
                cv::Mat(4, 1, CV_64FC1, (void*)rightInfo.D.data()),
                RR_, PR_,
                cv::Size(rightInfo.width / (float)downsampling_, rightInfo.height / (float)downsampling_),
                CV_16SC2, mapR1_, mapR2_);

            // Store single camera calibration after rectification (no distortion)
            KL_ = cv::Mat(3, 3, CV_64FC1);
            KL_.at<double>(0, 0) = PL_.at<double>(0, 0);
            KL_.at<double>(0, 1) = PL_.at<double>(0, 1);
            KL_.at<double>(0, 2) = PL_.at<double>(0, 2);
            KL_.at<double>(1, 0) = PL_.at<double>(1, 0);
            KL_.at<double>(1, 1) = PL_.at<double>(1, 1);
            KL_.at<double>(1, 2) = PL_.at<double>(1, 2);
            KL_.at<double>(2, 0) = PL_.at<double>(2, 0);
            KL_.at<double>(2, 1) = PL_.at<double>(2, 1);
            KL_.at<double>(2, 2) = PL_.at<double>(2, 2);
            KR_ = cv::Mat(3, 3, CV_64FC1);
            KR_.at<double>(0, 0) = PR_.at<double>(0, 0);
            KR_.at<double>(0, 1) = PR_.at<double>(0, 1);
            KR_.at<double>(0, 2) = PR_.at<double>(0, 2);
            KR_.at<double>(1, 0) = PR_.at<double>(1, 0);
            KR_.at<double>(1, 1) = PR_.at<double>(1, 1);
            KR_.at<double>(1, 2) = PR_.at<double>(1, 2);
            KR_.at<double>(2, 0) = PR_.at<double>(2, 0);
            KR_.at<double>(2, 1) = PR_.at<double>(2, 1);
            KR_.at<double>(2, 2) = PR_.at<double>(2, 2);

            ROS_INFO("Slam node calibration succeded!");
            return true;

        } catch (...) {
            ROS_WARN("Slam node calibration failed!");
            return false;
        }

    }


    std::vector<cv::KeyPoint> kptsLeftC_, kptsRightC_; /**< Stored current (C) keypoints for left and right images*/
    cv::Mat descLeftC_, descRightC_; /**< Stored current (C) descriptors for left and right images*/

    cv::Ptr<cv::FastFeatureDetector> fDetector_; /**< Feature detector (FAST)*/
    cv::Ptr<cv::xfeatures2d::BriefDescriptorExtractor> fExtractor_; /**< Feature decriptor extractor (BRIEF)*/

    RobustMatcher matcher_; /**< Matcher*/

    std::vector<cv::DMatch> stereoMatchesC_; /**< Matching results for current(C) stereo pairs*/
    std::vector<cv::Point3f> pclC_; /**< Stored current(C) 3D point cloud*/
	
    std::vector<double> SV_;                /* State vector for pose = x, y, theta & map = x,y for 100 landmarks */                
    Eigen::Matrix<double, 6, 603> CM_;      /**< Covariance matrix*/

    bool init_;                             /**< Flag indicating if EKF has been initialized*/
    bool pred_;                             /**< Flag indicating if prediction step has been done*/

    ros::NodeHandle nh_;                    /**< ROS node handler*/
    ros::Subscriber kptSub_;               /**< VIO keyframe subscriber*/

    ros::Publisher posePub_;                /**< Publisher for predicted pose*/
    ros::Publisher pcPub_;                /**< Publisher for point cloud*/

    std::vector<Landmark> lmrksC_;          /**< current landmarks*/
    std::vector<Landmark> map;              /**< map of landmarks*/

    cv::Mat KL_, PL_, RL_; /**< Stereo camera parameters for left (L) camera*/
    cv::Mat KR_, PR_, RR_; /**< Stereo camera parameters for right (R) camera*/

    int downsampling_; /**< Downsampling factor for the input images*/
    int maxFeatures_; /**< Maximum number of features to find in each bucket*/

    cv::Mat mapL1_, mapL2_, mapR1_, mapR2_; /**< Stereo rectification mappings*/
    
    // double calibTime_;      /**< IMU calibration time*/

    // double T_;                                  /**< IMU KF prediction period*/
    // double T2_;                                 /**< Squared IMU KF prediction period*/
    // double rx_, ry_, rz_, gbx_, gby_, gbz_;     /**< IMU KF state vector x = [rx, ry, rz, gbx, gby, gbz]*/
    // Eigen::MatrixXd P_;                         /**< IMU KF matrix*/
	
    double biaDev_;
    double biaTh_;
};

#endif








