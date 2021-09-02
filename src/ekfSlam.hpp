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
#include <cstdlib>
#include <ctime>
#include <cmath>
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
        // biaDev_ = 0.00001;//0.000001;
        // biaTh_ = 0.005; //0.001;

        // initialize hyperparameters 
        downsampling_ = 2;
        maxFeatures_ = 30;
		
        // Setup data subscribers
        kptSub_ = nh_.subscribe(kptTopic, 10, &EkfSlam::kptDataCallback, this);

        // Setup data publishers
        posePub_ = nh_.advertise<geometry_msgs::TransformStamped>(nodeName + "/slamPose", 1);
        pcPub_ = nh_.advertise<sensor_msgs::PointCloud2>(nodeName + "/point_cloud", 1);

        deltaT_ = 1.0;
        simTh_ = 1.0;
	}


    /** @brief Initialize EKF
     * Input: visor::kpt::ConstPtr keyframe
     * The user must continue calling this function with new gyro data until it returns true
     */
	bool initialize(const visor::kpt::ConstPtr& msg) {
	
		// Initialize state vector CV_
        SV_ = std::vector<double>(314, 0.0);

        // Initialize covariance matrices
        CM_.setIdentity(314, 314);
        CM_.setZero();
        Q_.setIdentity(314, 314);
        Q_.setZero();
        R_.setIdentity(314, 314);
        R_.setZero();
        
        if(msg) {
            sensor_msgs::CameraInfo leftInfo, rightInfo;

            leftInfo = msg->infoL;
            rightInfo = msg->infoR;

            calibInit(leftInfo, rightInfo);
        }

        numLandmarks_ = 0;

        init_ = true;
        ROS_INFO("SLAM EKF INITIALIZED");

        observedL_ = false;

        return true;
	}

    /** @brief generate landmark with global location for the map using the current observed landmarks
     * @param[in] landmarkC current observed landmark
     * @return landmark for storing in the map
     */
    Landmark mapLandmarkCreator(Landmark& landmarkC) {

        // get relative landmark location
        cv::Point3f rp;
        rp.x = landmarkC.getLocRel().x;
        rp.y = landmarkC.getLocRel().y;
        rp.z = landmarkC.getLocRel().z;

        // set global landmark location as gp = relative location + predicted pose
        cv::Point3f gp;
        gp.x = rp.x + SV_[0];
        gp.y = rp.y + SV_[1];
        gp.z = rp.z + SV_[2];

        return Landmark(landmarkC.getKpt(), rp, gp); 
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
     * @param[in] pose Predicted pose from viodom 
     * @param[in] dist Pose change between previous and current frame calculated using PnP-RANSAC 
     */
	bool predict(tf::Transform pose, tf::Transform dist) {

        // ut/t-1 = fv(ut-1/t-1, ct, nt)

        // get deltaT
        ros::Time timeC = ros::Time::now();
        deltaT_ = (timeC - timeP_).toSec();
        timeP_ = timeC;
        
        // pose translation vector
        SV_[0] = pose.getOrigin().getX();
        SV_[1] = pose.getOrigin().getY();
        SV_[2] = pose.getOrigin().getZ();
        
        // pose rotation quaternion
        SV_[3] = pose.getRotation().getW();
        SV_[4] = pose.getRotation().getX();
        SV_[5] = pose.getRotation().getY();
        SV_[6] = pose.getRotation().getZ();

        // linear velocity vector 
        SV_[7] = pose.getOrigin().getX() / deltaT_;
        SV_[8] = pose.getOrigin().getY() / deltaT_;
        SV_[9] = pose.getOrigin().getZ() / deltaT_;

        // angular velocity quaternion
        SV_[10] = pose.getRotation().getW() / deltaT_;
        SV_[11] = pose.getRotation().getX() / deltaT_;
        SV_[12] = pose.getRotation().getY() / deltaT_;
        SV_[13] = pose.getRotation().getZ() / deltaT_;
        
        // Gt = g(ct)
        // motion jacobian

        // temporary bias matrices
        Eigen::MatrixXd B1, B2;
        B1.setIdentity(14, 314);
        B2.setIdentity(314, 14);

        // add random bias to the bias matrices
        std::srand(std::time(0));

        for(int i = 0; i < 14; i++) {
            for(int j = 0; j < 314; j++) {
                B1(i,j) = std::rand() % 100 + 1;
                B2(j,i) = std::rand() % 100 + 1;
            }
        }

        // add bias to bias matrix Q
        Q_.block<14,314>(0,0) = B1;
        Q_.block<314,14>(0,0) = B2;

        // ROS_INFO_STREAM("CM (" << CM_(0,0) << "," << CM_(0,313) << "," << CM_(313,313) << "," << CM_(313,0) << ")");

        // update covariance matrix
        CM_ += Q_;

        ROS_INFO_STREAM("State Vector: new pose (" << SV_[0] << "," << SV_[1] << "," << SV_[2] << ","<< ")");
        // ROS_INFO_STREAM("CM(" << CM_(0,0) << "," << CM_(0,313) << "," << CM_(313,313) << "," << CM_(313,0) << ")");

        return true;
	}

    /** EKF update stage
     */
	bool update() {

        // iterate through every observed landmark
        for(std::vector<Landmark>::iterator lmc = lmrksC_.begin(); lmc != lmrksC_.end(); ++lmc) {

            // index of current landmark in the map and the covariance matrix
            int lmIndex = 0;

            // initialize landmark for the map
            Landmark nlm = mapLandmarkCreator((*lmc));

            // if map is empty, add the first landmark
            if(numLandmarks_ == 0) {

                // add landmark to the map
                map.push_back(nlm);
                
                // add landmark to state vector
                SV_[14] = nlm.getLocGlob().x;
                SV_[15] = nlm.getLocGlob().y;
                SV_[16] = nlm.getLocGlob().z;

                numLandmarks_ = 1;

            } else {

                double minEucDist = 10000.0;
                Landmark minLm = nlm;
                int minIndex = 0;
                int index = 0;

                // iterate through every landmark stored in the map
                for(std::vector<Landmark>::iterator lmm = map.begin(); lmm != map.end(); ++lmm) {

                    // calculate euclidean distance between points
                    double eucDist = std::sqrt((nlm.getLocGlob().x - (*lmm).getLocGlob().x)*(nlm.getLocGlob().x - (*lmm).getLocGlob().x) 
                                                 + (nlm.getLocGlob().y - (*lmm).getLocGlob().y)*(nlm.getLocGlob().y - (*lmm).getLocGlob().y)
                                                 + (nlm.getLocGlob().z - (*lmm).getLocGlob().z)*(nlm.getLocGlob().z - (*lmm).getLocGlob().z));

                    // get min euclidean distance and corresponding landmark
                    if(eucDist < minEucDist) {
                        minEucDist = eucDist;
                        minLm = (*lmm);
                        minIndex = index;
                    } 

                    index += 1;
                }

                // landmark observed before
                if (minEucDist < simTh_) {

                    lmIndex = minIndex;

                    cv::Point3f minLmGL;
                    minLmGL.x = minLm.getLocGlob().x;
                    minLmGL.y = minLm.getLocGlob().y;
                    minLmGL.z = minLm.getLocGlob().z;

                    // update global location of current landmark with global location from the map
                    nlm.setLocGlob(minLmGL);

                } else {

                    // unobserved landmark - add to the map
                    map.push_back(nlm);
                    lmIndex = numLandmarks_;

                    // add landmark to the state vector
                    SV_[lmIndex * 3 + 14] = nlm.getLocGlob().x;
                    SV_[(lmIndex * 3 + 14) + 1] = nlm.getLocGlob().y;
                    SV_[(lmIndex * 3 + 14) + 2] = nlm.getLocGlob().z;

                    numLandmarks_ += 1;
                }
    
                ROS_INFO_STREAM("Yeetus(" << numLandmarks_ << ")");

                // lmIndex = location of current landmark in the map
                // lmIndex * 3 + 14 = location of current landmark in SV and CM
                
                // observation model
                cv::Point3f h;

                h.x = nlm.getLocGlob().x - SV_[0];
                h.y = nlm.getLocGlob().y - SV_[1];
                h.z = nlm.getLocGlob().z - SV_[2];

                // observation jacobian
                Eigen::MatrixXd H;

                // kalman gain
                double K = 0.0;


                // SV update - pose
                SV_[0] +=  K * (nlm.getLocGlob().x - h.x);
                SV_[1] +=  K * (nlm.getLocGlob().y - h.y);
                SV_[2] +=  K * (nlm.getLocGlob().y - h.y);

                SV_[3] +=  K * (nlm.getLocGlob().y - h.y);
                SV_[4] +=  K * (nlm.getLocGlob().y - h.y);
                SV_[5] +=  K * (nlm.getLocGlob().y - h.y);
                SV_[6] +=  K * (nlm.getLocGlob().y - h.y);

                SV_[7] +=  K * (nlm.getLocGlob().y - h.y);
                SV_[8] +=  K * (nlm.getLocGlob().y - h.y);
                SV_[9] +=  K * (nlm.getLocGlob().y - h.y);

                SV_[10] +=  K * (nlm.getLocGlob().y - h.y);
                SV_[11] +=  K * (nlm.getLocGlob().y - h.y);
                SV_[12] +=  K * (nlm.getLocGlob().y - h.y);
                SV_[13] +=  K * (nlm.getLocGlob().y - h.y);

                // SV update - landmark
                SV_[lmIndex * 3 + 14] += K * (nlm.getLocGlob().x - h.x);
                SV_[(lmIndex * 3 + 14) + 1] += K * (nlm.getLocGlob().x - h.x);
                SV_[(lmIndex * 3 + 14) + 2] += K * (nlm.getLocGlob().x - h.x);


                // CM update
                Eigen::MatrixXd lQ;
                lQ.setIdentity(314, 314);
                lQ.setZero();

                // temporary bias matrices
                Eigen::MatrixXd Bp, Bl, Bpl, Blp, B1, B2;
                Bp.setIdentity(14, 14);
                Bl.setIdentity(3, 3);
                Bpl.setIdentity(3, 14);
                Blp.setIdentity(14, 3);
                B1.setIdentity(3, 314);
                B2.setIdentity(314, 3);

                // add random bias to the bias matrices
                std::srand(std::time(0));

                for(int i = 0; i < 14; i++) {
                    for(int j = 0; j < 14; j++) {
                        Bp(i,j) = std::rand() % 10 + 1;

                        if (i < 3 && j < 3) {
                            Bl(j,i) = std::rand() % 100 + 1;
                        }
                    }
                }

                for(int i = 0; i < 3; i++) {
                    for(int j = 0; j < 14; j++) {
                        Bpl(i,j) = std::rand() % 100 + 1;
                        Blp(j,i) = std::rand() % 100 + 1;
                    }
                }

                // add bias to bias matrix lQ
                lQ.block<14,14>(0,0) = Bp;
                lQ.block<3,14>(lmIndex * 3 + 14,0) = Bpl;
                lQ.block<14,3>(0,lmIndex * 3 + 14) = Blp;
                lQ.block<3,3>(lmIndex * 3 + 14,lmIndex * 3 + 14) = Bl;

                // update covariance matrix
                CM_ -= lQ;
            }
        }

        return true;
	}
	

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


            // Get displacement data
            geometry_msgs::TransformStamped uDist;
            uDist = msg->dist;

            tf::Quaternion qDist;
            qDist.setX(uDist.transform.rotation.x);
            qDist.setY(uDist.transform.rotation.y);
            qDist.setZ(uDist.transform.rotation.z);
            qDist.setW(uDist.transform.rotation.w);
            tf::Vector3 pDist;
            pDist.setX(uDist.transform.translation.x);
            pDist.setY(uDist.transform.translation.y);
            pDist.setZ(uDist.transform.translation.z);
            tf::Transform distTemp;
            distTemp.setRotation(qDist);
            distTemp.setOrigin(pDist);

		    // ekf predict step
		    predict(odomTemp, distTemp);

            // get observations - left and right images
            generateLandmarks(msg->imgL, msg->imgR);

            // ekf update step
            update();

            // send corrected pose to viodom node
            tf::Transform updatedOdomC;

            tf::Quaternion qUpdated;
            qUpdated.setW(SV_[3]);
            qUpdated.setX(SV_[4]);
            qUpdated.setY(SV_[5]);
            qUpdated.setZ(SV_[6]);
            tf::Vector3 pUpdated;
            pUpdated.setX(SV_[0]);
            pUpdated.setY(SV_[1]);
            pUpdated.setZ(SV_[2]);

            updatedOdomC.setRotation(qUpdated);
            updatedOdomC.setOrigin(pUpdated);

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


    std::vector<cv::KeyPoint> kptsLeftC_, kptsRightC_;              /**< Stored current (C) keypoints for left and right images*/
    cv::Mat descLeftC_, descRightC_;                                /**< Stored current (C) descriptors for left and right images*/

    cv::Ptr<cv::FastFeatureDetector> fDetector_;                    /**< Feature detector (FAST)*/
    cv::Ptr<cv::xfeatures2d::BriefDescriptorExtractor> fExtractor_; /**< Feature decriptor extractor (BRIEF)*/

    RobustMatcher matcher_;                                         /**< Matcher*/

    std::vector<cv::DMatch> stereoMatchesC_;                        /**< Matching results for current(C) stereo pairs*/
    std::vector<cv::Point3f> pclC_;                                 /**< Stored current(C) 3D point cloud*/
	
    std::vector<double> SV_;                                        /* State vector for pose & map of 100 landmarks */                
    Eigen::MatrixXd CM_;                                            /**< Covariance matrix*/
    Eigen::MatrixXd Q_;                                             /**< Motion Bias Covariance matrix*/
    Eigen::MatrixXd R_;                                             /**< Observation Bias Covariance matrix*/

    bool init_;                                                     /**< Flag indicating if EKF has been initialized*/
    bool pred_;                                                     /**< Flag indicating if prediction step has been done*/
    bool observedL_;                                                /**< Flag indicating if current landmark was observed before*/

    ros::NodeHandle nh_;                                            /**< ROS node handler*/
    ros::Subscriber kptSub_;                                        /**< VIO keyframe subscriber*/

    ros::Publisher posePub_;                                        /**< Publisher for predicted pose*/
    ros::Publisher pcPub_;                                          /**< Publisher for point cloud*/

    ros::Time timeP_;                                               /**< previous time*/

    std::vector<Landmark> lmrksC_;                                  /**< current landmarks*/
    std::vector<Landmark> map;                                      /**< map of landmarks*/

    cv::Mat KL_, PL_, RL_;                                          /**< Stereo camera parameters for left (L) camera*/
    cv::Mat KR_, PR_, RR_;                                          /**< Stereo camera parameters for right (R) camera*/

    int downsampling_;                                              /**< Downsampling factor for the input images*/
    int maxFeatures_;                                               /**< Maximum number of features to find in each bucket*/

    cv::Mat mapL1_, mapL2_, mapR1_, mapR2_;                         /**< Stereo rectification mappings*/
    
    int numLandmarks_;                                               /**< number of landmarks in the map*/                    

    double deltaT_;                                                 /**< time elapsed since last EKF iteration*/
    double simTh_;                                                  /**< similarity threshold for landmark global locations*/
};

#endif








