#include <tool_calibration/tool_calibration_server.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2_eigen/tf2_eigen.h>

// Service node to perform calibration of a tool for the KR8 robotic arm

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tool_calibration_server");

  tool_calibration::ToolCalibrationServer tcs;

  ROS_INFO("Tool Calibration server started");

  ros::spin();
}

namespace tool_calibration
{
ToolCalibrationServer::ToolCalibrationServer() : nh_("~")
{
  // Service server
  tool_calibration_srv_ = nh_.advertiseService<sss_msgs::GetToolCalibration::Request, sss_msgs::GetToolCalibration::Response>("tool_calibration_service", boost::bind(&ToolCalibrationServer::toolCalibrationCallback, this, _1, _2));

  // Wait for retrieval server
  calibration_urdf_retrieve_client_ = nh_.serviceClient<tool_calibration::CalibrationUrdfRetrieve>("/calibration_urdf_server/calibration_urdf_retrieve");
  if (!calibration_urdf_retrieve_client_.waitForExistence(ros::Duration(2.0)) && ros::ok())
  {
    ROS_INFO("Waiting for calibration_urdf_server/calibration_urdf_retrieve");
  }

  // Wait for update server
  calibration_urdf_update_client_ = nh_.serviceClient<tool_calibration::CalibrationUrdfUpdate>("/calibration_urdf_server/calibration_urdf_update");
  if (!calibration_urdf_update_client_.waitForExistence(ros::Duration(2.0)) && ros::ok())
  {
    ROS_INFO("Waiting for calibration_urdf_server/calibration_urdf_update");
  }
}

bool ToolCalibrationServer::toolCalibrationCallback(const sss_msgs::GetToolCalibration::Request& req, sss_msgs::GetToolCalibration::Response& res)
{
  /*
  Main callback for the tool calibration server.
  Aims to allow user assisted tool calibration run through the system with multiple tasks to assist the process

  Command type is recieved from the req.cmd.
  Below is a summary of what each command does
    - TOOL_POINT_SAMPLE: collect sample of current joint configuration for tool point calibration
    - TOOL_ORIENTATION_SAMPLE_P1: collects first sample of current joint configuration for orientation calibration
    - TOOL_ORIENTATION_SAMPLE_P2: collects second sample of current joint configuration for orientation calibration
    - TOOL_ORIENTATION_SAMPLE_P3: collects third sample of current joint configuration for orientation calibration
    - TOOL_POINT_SAMPLES_RESET: clears the previous tool point calibration result
    - TOOL_ORIENTATION_SAMPLES_RESET: clears the previous tool orientation calibration result
    - CALCULATE_TOOL_POINT_CALIBRATION: calculates tool point calibration. Should be performed after a sufficent (>4) number of samples have been taken
    - CALCULATE_TOOL_ORIENTATION_CALIBRATION: calculates tool orientation calibration. Should be performed after the correct (3) number of samples have been taken and tool point calibration has been performed
    - CALCULATE_ORIENTATION_CALIBRATION_FROM_REFERENCE_OBJECT: calculates tool orientation calibration from a reference object 
    - UPDATE_TOOL_URDF: updates the tool calibration URDF file with the latest calculated calibration values
  */

  // Sanity check
  if (robot_base_frames_.count(req.robot_id) == 0)
  {
    ROS_ERROR_STREAM(req.robot_id << " is not a currently supported robot");
    res.success = false;
    return true;
  }

  // Setup general variables
  std::string base_frame, tool_surface_frame;

  // Setup frames
  base_frame = req.robot_id + '/' + robot_base_frames_[req.robot_id];
  tool_surface_frame = req.robot_id + '/' + req.tool_surface_name;

  if (req.service_call_cmd == sss_msgs::GetToolCalibrationRequest::TOOL_POINT_SAMPLES_RESET)
  {
    // Clear all tool point samples for the given tool surface
    robot_tool_calibration_samples_.erase(tool_surface_frame);
    robot_tool_calibration_results_.erase(tool_surface_frame);
    touch_point_result_avaliable_.erase(tool_surface_frame);
    ROS_INFO_STREAM("Successfully erased tool point calibration samples");
    res.success = true;
    return true;
  }
  else if (req.service_call_cmd == sss_msgs::GetToolCalibrationRequest::TOOL_ORIENTATION_SAMPLES_RESET)
  {
    // Clear all tool orientation samples for the given tool surface
    robot_tool_orientation_calibration_samples_.erase(tool_surface_frame);
    robot_tool_orientation_calibration_results_.erase(tool_surface_frame);
    ROS_INFO_STREAM("Successfully erased orientation calibration samples");
    res.success = true;
    return true;
  }
  else if (req.service_call_cmd == sss_msgs::GetToolCalibrationRequest::CALCULATE_TOOL_POINT_CALIBRATION)
  {
    // Calculate tool point calibration result and calculate URDF formatted calibration values
    if (robot_tool_calibration_samples_[tool_surface_frame].size() < min_number_of_samples_)
    {
      ROS_ERROR_STREAM("Must have atleast " << min_number_of_samples_ << " to calculate the point calibration result for "  << tool_surface_frame);
      res.success = false;
      return true;
    }

    if (robot_tool_calibration_results_.count(tool_surface_frame) == 0)
    {
      // If no key for the frame exists, add it
      robot_tool_calibration_results_[tool_surface_frame] = {};
    }

    // Calculate tool point calibration
    robot_tool_calibration_results_[tool_surface_frame] = tool_point_calibration::calibrateTcp(robot_tool_calibration_samples_[tool_surface_frame]);

    touch_point_result_avaliable_[tool_surface_frame] = true;

    // Update to the URDF
    if (!calculateUrdfFormatedToolCalibration(tool_surface_frame))
    {
      res.success = false;
      return false;
    }
    else
    {
      // Notify of successful result
      ROS_INFO_STREAM("Successfully Calculated the tool calibration for " << tool_surface_frame);
      ROS_INFO_STREAM("Calibrated tip from expected (meters in xyz): [" << robot_tool_calibration_results_[tool_surface_frame].tcp_offset.transpose() << "]");
      ROS_INFO_STREAM("Touch point determined as: " << robot_tool_calibration_results_[tool_surface_frame].touch_point.transpose());

      res.success = true;
      return true;
    }
  }
  else if (req.service_call_cmd == sss_msgs::GetToolCalibrationRequest::CALCULATE_TOOL_ORIENTATION_CALIBRATION)
  {
    // Calculate tool point calibration result and calculate URDF formated calibration values
    if (robot_tool_orientation_calibration_samples_[tool_surface_frame].size() != required_number_of_orientation_samples_)
    {
      ROS_ERROR_STREAM("Must have " << required_number_of_orientation_samples_ << " samples to calculate the orientation calibration result for " << tool_surface_frame);
      res.success = false;
      return true;
    }

    if (robot_tool_orientation_calibration_samples_.count(tool_surface_frame) == 0)
    {
      // If no key for the frame exists, add it
      robot_tool_orientation_calibration_samples_[tool_surface_frame] = {};
    }

    // Calculate tool point orientation
    calculateRotationFromSamples(tool_surface_frame);

    // Update to the URDF
    if (!calculateUrdfFormatedToolCalibration(tool_surface_frame))
    {
      res.success = false;
      return false;
    }
    else
    {
      // Notify of successful result
      ROS_INFO_STREAM("Successfully calculated the tool orientation calibration for " << tool_surface_frame);
      res.success = true;
      return true;
    }
  }
  else if (req.service_call_cmd == sss_msgs::GetToolCalibrationRequest::CALCULATE_ORIENTATION_CALIBRATION_FROM_REFERENCE_OBJECT)
  {
    // Check if it is a supported tool surface
    if (!(std::count(reference_orientation_calibration_supported_tools_.begin(), reference_orientation_calibration_supported_tools_.end(), tool_surface_frame)))
      {
        ROS_ERROR_STREAM("Currently only tools in reference_orientation_calibration_supported_tools_ are supported");
        res.success = false;
        return false;
      }

    ROS_WARN("Calculating calibration from the robots current pose relative to the given reference object. Not using any saved robot_tool_orientation_calibration_samples_ for this calculation");

    // Need listener
    tf::TransformListener listener;
    std::string error_msg;
    std::string world_frame = "world";

    if (!listener.waitForTransform(world_frame, tool_surface_frame, ros::Time(0), ros::Duration(1.0), ros::Duration(0.01), &error_msg))
    {
      bool world_found = listener.frameExists(world_frame);
      bool tool_found = listener.frameExists(tool_surface_frame);

      if (!world_found && !tool_found)
      {
        ROS_WARN("Check to make sure that a robot state publisher or other node is publishing"
                 " tf frames for your robot. Also check that your base/tool frames names are"
                 " correct and not missing a prefix, for example.");
      }
      else if (!world_found)
      {
        ROS_WARN("Check to make sure that world frame '%s' actually exists.", base_frame.c_str());
      }
      else if (!tool_found)
      {
        ROS_WARN("Check to make sure that tool_surface_frame '%s' actually exists.", tool_surface_frame.c_str());
      }
      ROS_ERROR_STREAM("");
      res.success = false;
      return true;
    }

    Eigen::Isometry3d tool_eigen_pose, ref_frame_eigen_pose;
    tf::StampedTransform world_to_tip_transform;
    try
    {
      listener.lookupTransform(world_frame, tool_surface_frame, ros::Time(0), world_to_tip_transform);
      tf::poseTFToEigen(world_to_tip_transform, tool_eigen_pose);
      ROS_INFO_STREAM("Pose captured transform:\n" << tool_eigen_pose.matrix());
    }
    catch (const tf::TransformException& ex)
    {
      ROS_ERROR("Unable to lookup transform");
      res.success = false;
      return true;
    }

    // Now use the pose of the reference object and find the difference from the world_to_tip_transform
    tf2::fromMsg(req.reference_object_pose, ref_frame_eigen_pose);

    Eigen::Isometry3d tool_to_ref_eigen = ref_frame_eigen_pose.inverse() * tool_eigen_pose;

    ROS_INFO_STREAM("req.reference_object_pose is: \n" << req.reference_object_pose);
    ROS_INFO_STREAM("Rotation ref_frame_eigen_pose (y, p, r) is: \n" << ref_frame_eigen_pose.rotation().eulerAngles(2, 1, 0));
    ROS_INFO_STREAM("Rotation tool_eigen_pose (y, p, r) is: \n" << tool_eigen_pose.rotation().eulerAngles(2, 1, 0));
    ROS_INFO_STREAM("Rotation tool_to_ref_eigen (y, p, r) is: \n" << tool_to_ref_eigen.rotation().eulerAngles(2, 1, 0));
    ROS_INFO_STREAM("Translation is: \n" << tool_to_ref_eigen.translation());

    // At this point tool_to_ref_eigen.rotation().eulerAngles(2, 1, 0) = the yaw, pitch, roll that the welder frame is off from reference. Need to adjust tool frame to match reference

    // Calculate tool orientation calibration result and call service calculate URDF formated values
    if (robot_tool_orientation_calibration_results_.count(tool_surface_frame) == 0)
    {
      // If no key for the frame exists, add it
      robot_tool_orientation_calibration_results_[tool_surface_frame] = Eigen::Vector3d();
    }

    robot_tool_orientation_calibration_results_[tool_surface_frame] = tool_to_ref_eigen.rotation().eulerAngles(2, 1, 0);

    // Caluclate URDF calibration values
    if (!calculateUrdfFormatedToolCalibration(tool_surface_frame))
    {
      res.success = false;
      return false;
    }

    res.success = true;
    return true;
  }
  else if (req.service_call_cmd == sss_msgs::GetToolCalibrationRequest::UPDATE_TOOL_URDF)
  // Called to update the UDRf which defines the KR8 welder arm
  {
    if (!touch_point_result_avaliable_[tool_surface_frame])
    {
      ROS_ERROR_STREAM("No position calibration result avaliable. Not updating tool URDF");
      res.success = false;
      return false;
    }

    if (robot_tool_urdf_formated_calibration_.count(tool_surface_frame) == 0)
    {
      // No value avaliable for the tool surface
      ROS_ERROR_STREAM("No robot_tool_urdf_formated_calibration_ for tool surface: " << tool_surface_frame);
      res.success = false;
      return false;
    }

    // Update the values from the tools' URDF
    tool_calibration::CalibrationUrdfUpdate calibration_urdf_update_srv;
    
    // Fill request
    calibration_urdf_update_srv.request.robot_tool_surface = tool_surface_frame;
    calibration_urdf_update_srv.request.calibration_x = robot_tool_urdf_formated_calibration_[tool_surface_frame].calibration_x;
    calibration_urdf_update_srv.request.calibration_y = robot_tool_urdf_formated_calibration_[tool_surface_frame].calibration_y;
    calibration_urdf_update_srv.request.calibration_z = robot_tool_urdf_formated_calibration_[tool_surface_frame].calibration_z;

    if (robot_tool_orientation_calibration_results_.count(tool_surface_frame) == 0)
    {
      ROS_INFO("Not updating the tools orientation");
      calibration_urdf_update_srv.request.orientation_calibrated = false;
      calibration_urdf_update_srv.request.calibration_roll = 0.0;
      calibration_urdf_update_srv.request.calibration_pitch = 0.0;
      calibration_urdf_update_srv.request.calibration_yaw = 0.0;
    }
    else
    {
      calibration_urdf_update_srv.request.orientation_calibrated = true;
      calibration_urdf_update_srv.request.calibration_roll = robot_tool_urdf_formated_calibration_[tool_surface_frame].calibration_roll;
      calibration_urdf_update_srv.request.calibration_pitch = robot_tool_urdf_formated_calibration_[tool_surface_frame].calibration_pitch;
      calibration_urdf_update_srv.request.calibration_yaw = robot_tool_urdf_formated_calibration_[tool_surface_frame].calibration_yaw;
    }

    if (calibration_urdf_update_client_.call(calibration_urdf_update_srv))
    {
      if (!calibration_urdf_update_srv.response.success)
      {
        ROS_ERROR_STREAM("Failed to update calibration values from URDF for " << tool_surface_frame);
        res.success = false;
        return false;
      }
    }
    else
    {
      ROS_ERROR("Failed to call calibration_urdf_update service node");
      res.success = false;
      return false;
    }

    res.success = true;
    return true;
  }

  // Handle point sampling commands separately below:
  

  // Create a transform listener to query tool frames. Check if they actually exist
  tf::TransformListener listener;
  std::string error_msg;
  if (!listener.waitForTransform(base_frame, tool_surface_frame, ros::Time(0), ros::Duration(1.0), ros::Duration(0.01), &error_msg))
  {
    ROS_WARN_STREAM("Unable to lookup transform between base frame: '" << base_frame << "' and tool frame: '" << tool_surface_frame << "'. TF reported error: " << error_msg);
    bool base_found = listener.frameExists(base_frame);
    bool tool_found = listener.frameExists(tool_surface_frame);
    if (!base_found && !tool_found)
    {
      ROS_WARN("Check to make sure that a robot state publisher or other node is publishing tf frames for your robot. Also check that your base/tool frames names are correct and not missing a prefix, for example.");
    }
    else if (!base_found)
    {
      ROS_WARN("Check to make sure that base frame '%s' actually exists.", base_frame.c_str());
    }
    else if (!tool_found)
    {
      ROS_WARN("Check to make sure that tool_surface_frame '%s' actually exists.", tool_surface_frame.c_str());
    }
    res.success = false;
    return true;
  }

  // Sample the current point
  Eigen::Isometry3d eigen_pose;
  tf::StampedTransform transform;
  try
  {
    listener.lookupTransform(base_frame, tool_surface_frame, ros::Time(0), transform);
    ROS_INFO("Transformed captured for %s to %s", base_frame.c_str(), tool_surface_frame.c_str());
    tf::poseTFToEigen(transform, eigen_pose);
    ROS_INFO_STREAM("Pose captured transform:\n" << eigen_pose.matrix());
  }
  catch (const tf::TransformException& ex)
  {
    ROS_ERROR("Unable to lookup transform");
    res.success = false;
    return true;
  }

  // Save the current point
  if (req.service_call_cmd == sss_msgs::GetToolCalibrationRequest::TOOL_POINT_SAMPLE)
  {
    // Collect tool point sample

    if (robot_tool_calibration_samples_.count(tool_surface_frame) == 0)
    {
      // If no key for the frame exists, add it
      robot_tool_calibration_samples_[tool_surface_frame] = {};
    }

    if (robot_tool_calibration_samples_[tool_surface_frame].size() < 1)
    {
      // Notify for first point collected
      ROS_INFO("Starting tool calibration samples with base frame: '%s' and tool surface frame: '%s'.", base_frame.c_str(), tool_surface_frame.c_str());
    }

    // Save the sampled point
    robot_tool_calibration_samples_[tool_surface_frame].push_back(eigen_pose);
    double number_of_samples = robot_tool_calibration_samples_[tool_surface_frame].size();
    ROS_INFO_STREAM(number_of_samples << " tool point sample(s) collected for " << tool_surface_frame);
  }

  else if (req.service_call_cmd == sss_msgs::GetToolCalibrationRequest::TOOL_ORIENTATION_SAMPLE_P1 || req.service_call_cmd == sss_msgs::GetToolCalibrationRequest::TOOL_ORIENTATION_SAMPLE_P2 || req.service_call_cmd == sss_msgs::GetToolCalibrationRequest::TOOL_ORIENTATION_SAMPLE_P3)
  {
    // Collect tool orientation sample
    if (robot_tool_orientation_calibration_samples_.count(tool_surface_frame) == 0)
    {
      // If no key for the frame exists, add it
      robot_tool_orientation_calibration_samples_[tool_surface_frame] = {};
      robot_tool_orientation_calibration_samples_[tool_surface_frame].resize(3);
      
      // Notify for first orientation collected
      ROS_INFO("Starting tool orientation calibration samples with base frame: '%s' and tool surface frame: '%s'.",
               base_frame.c_str(), tool_surface_frame.c_str());
    }

    int save_index;
    if (req.service_call_cmd == sss_msgs::GetToolCalibrationRequest::TOOL_ORIENTATION_SAMPLE_P1)
    {
      save_index = 0;
    }
    else if (req.service_call_cmd == sss_msgs::GetToolCalibrationRequest::TOOL_ORIENTATION_SAMPLE_P2)
    {
      save_index = 1;
    }
    else if (req.service_call_cmd == sss_msgs::GetToolCalibrationRequest::TOOL_ORIENTATION_SAMPLE_P3)
    {
      save_index = 2;
    }

    // Save the current orientation
    robot_tool_orientation_calibration_samples_[tool_surface_frame][save_index] = eigen_pose;

    ROS_INFO_STREAM("Point " << (save_index + 1) << " tool orientation sample collected for " << tool_surface_frame);
    ROS_INFO_STREAM("Got sample" << "\n" << robot_tool_orientation_calibration_samples_[tool_surface_frame][save_index].matrix());
    res.success = true;
    return true;
  }
  else
  {
    ROS_ERROR_STREAM("Unsupported service_call_cmd");
    res.success = false;
    return true;
  }

  res.success = true;
  return true;
}

bool ToolCalibrationServer::calculateUrdfFormatedToolCalibration(const std::string tool_surface_frame)
{
  // Calculates the tool calibration in CalibrationUrdfRetrieveResponse format

  // Retrieve the latest calibration values from the tools URDF
  tool_calibration::CalibrationUrdfRetrieve calibration_urdf_retrieve_srv;
  calibration_urdf_retrieve_srv.request.robot_tool_surface = tool_surface_frame;
  if (calibration_urdf_retrieve_client_.call(calibration_urdf_retrieve_srv))
  {
    if (!calibration_urdf_retrieve_srv.response.success)
    {
      ROS_ERROR_STREAM("Failed to retieve calibration values from URDF for " << tool_surface_frame);
      return false;
    }
  }
  else
  {
    ROS_ERROR("Failed to call calibration_urdf_retrieve service node");
    return false;
  }

  // Print out returned calibration values
  ROS_INFO("Returned the following XYZ calibration values: %1.4f, %1.4f, %1.4f", calibration_urdf_retrieve_srv.response.calibration_x, calibration_urdf_retrieve_srv.response.calibration_y, calibration_urdf_retrieve_srv.response.calibration_z);
  ROS_INFO("Returned the following RPY calibration values: %1.4f, %1.4f, %1.4f", calibration_urdf_retrieve_srv.response.calibration_roll, calibration_urdf_retrieve_srv.response.calibration_pitch, calibration_urdf_retrieve_srv.response.calibration_yaw);

  tool_calibration::CalibrationUrdfRetrieveResponse tool_calibration;
  tool_calibration.success = false;
  if (robot_tool_urdf_formated_calibration_.count(tool_surface_frame) == 0)
  {
    // If no key for the frame exists, add it
    robot_tool_urdf_formated_calibration_[tool_surface_frame] = tool_calibration;
  }

  // Update point result
  tool_calibration.calibration_x = robot_tool_calibration_results_[tool_surface_frame].tcp_offset.transpose()[0] + calibration_urdf_retrieve_srv.response.calibration_x;
  tool_calibration.calibration_y = robot_tool_calibration_results_[tool_surface_frame].tcp_offset.transpose()[1] + calibration_urdf_retrieve_srv.response.calibration_y;
  tool_calibration.calibration_z = robot_tool_calibration_results_[tool_surface_frame].tcp_offset.transpose()[2] + calibration_urdf_retrieve_srv.response.calibration_z;

  // Update orientation result
  tool_calibration.calibration_roll = calibration_urdf_retrieve_srv.response.calibration_roll - robot_tool_orientation_calibration_results_[tool_surface_frame](2);
  tool_calibration.calibration_pitch = calibration_urdf_retrieve_srv.response.calibration_pitch - robot_tool_orientation_calibration_results_[tool_surface_frame](1);
  tool_calibration.calibration_yaw = calibration_urdf_retrieve_srv.response.calibration_yaw - robot_tool_orientation_calibration_results_[tool_surface_frame](0);

  // Print updated values
  ROS_INFO("Updating XYZ UDRF values as: %1.4f, %1.4f, %1.4f", tool_calibration.calibration_x, tool_calibration.calibration_y, tool_calibration.calibration_z);
  ROS_INFO("Updating RPY UDRF values as: %1.4f, %1.4f, %1.4f", tool_calibration.calibration_roll, tool_calibration.calibration_pitch, tool_calibration.calibration_yaw);

  tool_calibration.success = true;

  robot_tool_urdf_formated_calibration_[tool_surface_frame] = tool_calibration;

  return true;
}

bool ToolCalibrationServer::calculateRotationFromSamples(const std::string tool_surface_frame)
{
  // NOTE: requires that the tool point calibration has already occured
  // Intermediate variable to hold calibrated tool point result
  Eigen::Vector4d r0ns_n;
  r0ns_n << robot_tool_calibration_results_[tool_surface_frame].tcp_offset[0], robot_tool_calibration_results_[tool_surface_frame].tcp_offset[1], robot_tool_calibration_results_[tool_surface_frame].tcp_offset[2], 1;
  
  // Set vector V1
  // NOTE: movement for this calculation should be in +ive X axis
  Eigen::Vector4d V1 = robot_tool_orientation_calibration_samples_[tool_surface_frame][1].inverse()*robot_tool_orientation_calibration_samples_[tool_surface_frame][0]*r0ns_n;
  double term1 = (V1(0) - r0ns_n[0]);
  double term2 = (V1(1) - r0ns_n[1]);
  double term3 = (V1(2) - r0ns_n[2]);
  double delta1 = -sqrt(term1 * term1 + term2 * term2 + term3 * term3);
  double c11 = term1 / delta1;
  double c21 = term2 / delta1;
  double c31 = term3 / delta1;

  // Double check c11^2 + c21^2 + c31^2 = 1
  ROS_INFO("Sanity checking calculations...");
  double check_res = c11*c11 + c21*c21 + c31*c31;
  if(round(check_res) != 1) {
    ROS_WARN("Matrix calculations do not equal 1!");
  }
  ROS_INFO_STREAM(" delta1: " << delta1);
  ROS_INFO_STREAM("c11: " << c11 << ", c21: " << c21 << ", c31: " << c31);

  // Set vector V2
  // NOTE: movement for this calculation should be in +ive Y axis
  Eigen::Vector4d V2 = robot_tool_orientation_calibration_samples_[tool_surface_frame][2].inverse()*robot_tool_orientation_calibration_samples_[tool_surface_frame][0]*r0ns_n;
  term1 = (V2(0) - r0ns_n[0]);
  term2 = (V2(1) - r0ns_n[1]);
  term3 = (V2(2) - r0ns_n[2]);
  double delta2 = -sqrt(term1 * term1 + term2 * term2 + term3 * term3);
  double c12 = term1 / delta2;
  double c22 = term2 / delta2;
  double c32 = term3 / delta2;

  // Double check c12^2 + c22^2 + c32^2 = 1
  ROS_INFO("Sanity checking calculations...");
  check_res = c12*c12 + c22*c22 + c32*c32;
  if(round(check_res) != 1) {
    ROS_WARN("Matrix element sum does not equal 1!");
  }
  ROS_INFO_STREAM(" delta2: " << delta2);
  ROS_INFO_STREAM("c12: " << c12 << " , c22: " << c22 << ", c32: " << c32);

  // Calculate the remaining rotation matrix values
  double k1 = (c21 * c32 - c22 * c31) / (c22 * c11 - c21 * c12);
  double k2 = (c11 * c32 - c12 * c31) / (c12 * c21 - c11 * c22);
  double c33 = sqrt(1 / (1 + k1 * k1 + k2 * k2));
  double c13 = k1 * c33;
  double c23 = k2 * c33;

  // Double check c13^2 + c23^2 + c33^2 = 1
  ROS_INFO("Sanity checking calculations...");
  check_res = c13*c13 + c23*c23 + c33*c33;
  if(round(check_res) != 1) {
    ROS_WARN("Matrix element sum does not equal 1!");
  }
  ROS_INFO_STREAM("c13: " << c13 << ", c23: " << c23 << ", c33: " << c33);

  // Build the rotation matrix
  Eigen::Matrix3d Rot;
  Rot << c11, c12, c13, c21, c22, c23, c31, c32, c33;
  
  // Return the transformation matrix
  Eigen::Isometry3d Anns;
  Anns.matrix() << c11, c12, c13, r0ns_n(0), c21, c22, c23, r0ns_n(1), c31, c32, c33, r0ns_n(2), 0, 0, 0, 1;
  ROS_INFO_STREAM("Final output matrix is:\n" << Anns.matrix());

  // Save the final result 
  robot_tool_orientation_calibration_results_[tool_surface_frame] = Rot.eulerAngles(2, 1, 0);
  ROS_INFO_STREAM("Calibration RPY values calculated as: " << robot_tool_orientation_calibration_results_[tool_surface_frame].matrix().transpose().rowwise().reverse());

  return true;
}

}  // namespace tool_calibration
