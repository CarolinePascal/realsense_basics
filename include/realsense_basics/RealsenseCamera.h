/**
 * \file RealsenseCamera.h
 * \brief Header file of the RealsenseCamera class
 *
 * Header file of the RealsenseCamera class - Defines the attributes and methods used to configure and publish a Realsense camera depth measurements
 *
 */

#pragma once

#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include "realsense_basics/FloatParameters.h"
#include "realsense_basics/StringParameter.h"

 /*! \class RealsenseCamera
  * \brief Class sed to configure and publish a Realsense camera depth measurements
  */
class RealsenseCamera
{
    public:

    /*!
     *  \brief Constructor
     */
    RealsenseCamera();

    /*!
     *  \brief Destructor
     */
    ~RealsenseCamera(){};

    /*!
     *  \brief Publishes a point cloud
     */
    void publishPointcloud();

    /*!
     *  \brief Dynamically changes the disparity shift parameter of the camera
     */
    bool disparityShiftService(realsense_basics::FloatParameters::Request& req, realsense_basics::FloatParameters::Response& res);

    /*!
     *  \brief Dynamically changes the thresholds filter parameters of the camera
     */
    bool thresholdFilterService(realsense_basics::FloatParameters::Request& req, realsense_basics::FloatParameters::Response& res);

    private:

    //ROS attributes
    ros::NodeHandle m_nodeHandle;   /*! ROS node handle*/
    ros::Publisher m_pointCloudPublisher;  /*! ROS point cloud publisher*/

    //Realsense camera attributes
    rs2::pipeline m_pipe;
    rs2::frameset m_frames;
    rs2::pointcloud m_pc; 
    rs2::points m_points;
    rs2::frame m_color;
    rs2::frame m_depth;

    rs2::device m_device;

    //Realsense -> PCL conversion attribute
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pclPointCloud;

    //Preset
    std::string m_presetPath;

    //Disparity shift
    int m_disparityShift;
    ros::ServiceServer m_disparityShiftService;

    //Depth units
    double m_depthUnits;

    //Filters
    std::vector<std::string> m_filtersNames;
    std::vector<rs2::filter> m_filters;

    //Temporal filter 

    //Threshold filter
    std::vector<double> m_thresholdFilterParameters;
    ros::ServiceServer m_thresholdFilterService;

    bool m_mutex;
};
