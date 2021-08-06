#ifndef TOOL_CALIBRATION_SERVER_H
#define TOOL_CALIBRATION_SERVER_H

#include <ros/ros.h>

#include <Eigen/Dense>
#include <tf_conversions/tf_eigen.h>

#include <tool_calibration/tool_point_calibration.h>

#include <sss_msgs/GetToolCalibrationSample.h>
#include <tool_calibration/CalibrationUrdfRetrieve.h>
#include <tool_calibration/CalibrationUrdfUpdate.h>

namespace tool_calibration
{
class ToolCalibrationServer
{
public:
  ToolCalibrationServer();

private:
  ros::NodeHandle nh_;

  // Variables
  std::map<std::string, std::string> robot_base_frames_{ { "kr8_r1420_rcb", "base_link" } };
  // TODO: maybe load these from somewhere. Could also have different values for different tools
  uint min_number_of_samples_ = 4;  
  uint required_number_of_orientation_samples_ = 3;  

  // map of the robot tool surface being calibrated to the calibration samples for that tool.
  // stored in memory as the server will be called per point to accommodate running via tasks
  std::map<std::string, tool_point_calibration::Isometry3dVector> robot_tool_calibration_samples_;
  // map of the robot tool surface being calibrated to the calibration result.
  // stored in memory as the server will be called per point to accommodate running via tasks
  std::map<std::string, tool_point_calibration::TcpCalibrationResult> robot_tool_calibration_results_;
  // flags to indicate result is avaliable
  std::map<std::string, bool> touch_point_result_avaliable_;

  // map of the robot tool surface being calibrated to the calibration samples for that tools orientaion.
  // stored in memory as the server will be called per point to accommodate running via tasks
  std::map<std::string, std::vector<Eigen::Isometry3d>> robot_tool_orientation_calibration_samples_;
  // map of the robot tool surface being calibrated to the orientation calibration result.
  // stored in memory as the server will be called per point to accommodate running via tasks
  std::map<std::string, Eigen::Vector3d> robot_tool_orientation_calibration_results_;

  // map of the robot tool surface being calibrated to the tools current calibration values
  // in urdf values
  std::map<std::string, tool_calibration::CalibrationUrdfRetrieve::Response> robot_tool_urdf_formated_calibration_;

  // ros service nodes
  ros::ServiceServer tool_calibration_srv_;

  // ros service clients
  ros::ServiceClient calibration_urdf_retrieve_client_;
  ros::ServiceClient calibration_urdf_update_client_;

  // callback functions
  bool sampleToolCalibrationSampleCallback(const sss_msgs::GetToolCalibrationSample::Request& req,
                                           sss_msgs::GetToolCalibrationSample::Response& res);

  // helper functions
  bool calculateUrdfFormatedToolCalibration(const std::string tool_surface_frame);
  bool calculateRotationFromSamples(const std::string tool_surface_frame);
};

}  // namespace tool_calibration

#endif  // TOOL_CALIBRATION_SERVER_H
