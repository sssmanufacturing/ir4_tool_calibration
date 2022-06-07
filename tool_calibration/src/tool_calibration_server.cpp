#include <tool_calibration/tool_calibration_server.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2_eigen/tf2_eigen.h>

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
  // service server
  tool_calibration_srv_ =
      nh_.advertiseService<sss_msgs::GetToolCalibration::Request, sss_msgs::GetToolCalibration::Response>(
          "tool_calibration_service", boost::bind(&ToolCalibrationServer::toolCalibrationCallback, this, _1, _2));

  calibration_urdf_retrieve_client_ = nh_.serviceClient<tool_calibration::CalibrationUrdfRetrieve>("/calibration_"
                                                                                                   "urdf_server/"
                                                                                                   "calibration_"
                                                                                                   "urdf_"
                                                                                                   "retrieve");
  if (!calibration_urdf_retrieve_client_.waitForExistence(ros::Duration(2.0)) && ros::ok())
  {
    ROS_INFO("Waiting for calibration_urdf_server/calibration_urdf_retrieve");
  }
  calibration_urdf_update_client_ = nh_.serviceClient<tool_calibration::CalibrationUrdfUpdate>("/calibration_"
                                                                                               "urdf_server/"
                                                                                               "calibration_urdf_"
                                                                                               "update");
  if (!calibration_urdf_update_client_.waitForExistence(ros::Duration(2.0)) && ros::ok())
  {
    ROS_INFO("Waiting for calibration_urdf_server/calibration_urdf_update");
  }
}

bool ToolCalibrationServer::toolCalibrationCallback(const sss_msgs::GetToolCalibration::Request& req,
                                                    sss_msgs::GetToolCalibration::Response& res)
{
  /*
  Main callback for the tool calibration server.
  Aims to allow user assisted tool calibration run through the system with multiple tasks to assist the process

  Command type is recieved from the req.cmd.
  Summary of what each command is trying to do listed below:
    - TOOL_POINT_SAMPLE: collect sample of current joint configuration for tool point calibration
    - TOOL_ORIENTATION_SAMPLE_P1: collect sample of current joint configuration for orientation calibration
    - TOOL_ORIENTATION_SAMPLE_P2: collect sample of current joint configuration for orientation calibration
    - TOOL_ORIENTATION_SAMPLE_P3: collect sample of current joint configuration for orientation calibration
    - TOOL_POINT_SAMPLES_RESET: resets tool point calibration
    - TOOL_ORIENTATION_SAMPLES_RESET: resets tool orientation calibration
    - CALCULATE_TOOL_POINT_CALIBRATION: calculates tool point calibration
    - CALCULATE_TOOL_ORIENTATION_CALIBRATION: calculates tool orientation calibration (TODO: Currently calculation not working. This cmd currently does not update the in memory calibration
  values)
    - CALCULATE_ORIENTATION_CALIBRATION_FROM_REFERENCE_OBJECT: calculates tool orientation calibration from a reference
      object 
    - UPDATE_TOOL_URDF: updated the tool calibration urdf file with latest calculated calibration values
  */

  // sanity check
  if (robot_base_frames_.count(req.robot_id) == 0)
  {
    ROS_ERROR_STREAM(req.robot_id << " is not a currently supported robot");
    res.success = false;
    return true;
  }

  // setup general variables
  std::string base_frame, tool_surface_frame, tool_surface_parent_frame;

  // setup frames
  base_frame = req.robot_id + '/' + robot_base_frames_[req.robot_id];
  tool_surface_frame = req.robot_id + '/' + req.tool_surface_name;
  tool_surface_parent_frame = req.robot_id + '/' + req.tool_surface_parent_name;

  if (req.service_call_cmd == sss_msgs::GetToolCalibrationRequest::TOOL_POINT_SAMPLES_RESET)
  {
    // clear all tool point samples for the given tool surface
    robot_tool_calibration_samples_.erase(tool_surface_frame);
    robot_tool_calibration_results_.erase(tool_surface_frame);
    touch_point_result_avaliable_.erase(tool_surface_frame);
    ROS_INFO_STREAM("Successfully erased  tool point calibration samples");
    res.success = true;
    return true;
  }
  else if (req.service_call_cmd == sss_msgs::GetToolCalibrationRequest::TOOL_ORIENTATION_SAMPLES_RESET)
  {
    // clear all tool orientation samples for the given tool surface
    robot_tool_orientation_calibration_samples_.erase(tool_surface_frame);
    robot_tool_orientation_calibration_results_.erase(tool_surface_frame);
    ROS_INFO_STREAM("Successfully erased orientation calibration samples");
    res.success = true;
    return true;
  }
  else if (req.service_call_cmd == sss_msgs::GetToolCalibrationRequest::CALCULATE_TOOL_POINT_CALIBRATION)
  {
    // calculate tool point calibration result and calculate urdf formated calibration values
    if (robot_tool_calibration_samples_[tool_surface_frame].size() < min_number_of_samples_)
    {
      ROS_ERROR_STREAM("Must have at least " << min_number_of_samples_ << " to calculate the calibration result for "
                                            << tool_surface_frame);
      res.success = false;
      return true;
    }

    if (robot_tool_calibration_results_.count(tool_surface_frame) == 0)
    {
      // no key for the frame exists. Add
      robot_tool_calibration_results_[tool_surface_frame] = {};
    }

    // calculate result
    robot_tool_calibration_results_[tool_surface_frame] =
        tool_point_calibration::calibrateTcp(robot_tool_calibration_samples_[tool_surface_frame]);

    // TODO: sanity check that the new calibration makes sense (i.e. within tolerance)

    touch_point_result_avaliable_[tool_surface_frame] = true;

    // caluclate urdf calibration values
    if (!calculateUrdfFormatedToolCalibration(tool_surface_frame))
    {
      res.success = false;
      return false;
    }
    else
    {
      ROS_INFO_STREAM("Successfully calculated the tool calibration for " << tool_surface_frame);
      ROS_INFO_STREAM("Calibrated tip from tool_surface_parent_frame (meters in xyz): ["
                      << robot_tool_calibration_results_[tool_surface_frame].tcp_offset.transpose() << "]");
      res.success = true;
      return true;
    }
  }
  else if (req.service_call_cmd == sss_msgs::GetToolCalibrationRequest::CALCULATE_TOOL_ORIENTATION_CALIBRATION)
  {
    // calculate tool point calibration result and calculate URDF formated calibration values
    if (robot_tool_orientation_calibration_samples_[tool_surface_frame].size() !=
        required_number_of_orientation_samples_)
    {
      ROS_ERROR_STREAM("Must have " << required_number_of_orientation_samples_
                                    << " samples to calculate the orientation calibration result for "
                                    << tool_surface_frame);
      res.success = false;
      return true;
    }

    if (robot_tool_orientation_calibration_samples_.count(tool_surface_frame) == 0)
    {
      // no key for the frame exists. Add
      robot_tool_orientation_calibration_samples_[tool_surface_frame] = {};
    }

    // calculate result
    // TODO: currently not calculating as intended and not actually returning anything
    calculateRotationFromSamples(tool_surface_frame);

    // TODO: sanity check that the new calibration makes sense (i.e. within tolerance)

    // caluclate urdf calibration values
    if (!calculateUrdfFormatedToolCalibration(tool_surface_frame))
    {
      res.success = false;
      return false;
    }
    else
    {
      ROS_INFO_STREAM("Successfully calculated the tool orientation calibration for " << tool_surface_frame);
      // TODO: print out of result
      res.success = true;
      return true;
    }
  }
  else if (req.service_call_cmd ==
           sss_msgs::GetToolCalibrationRequest::CALCULATE_ORIENTATION_CALIBRATION_FROM_REFERENCE_OBJECT)
  {
    //TODO: this needs to be checked and updated

    // check if it is a supported tool surface
    if (!(std::count(reference_orientation_calibration_supported_tools_.begin(),
                   reference_orientation_calibration_supported_tools_.end(), tool_surface_frame)))
      {
        ROS_ERROR_STREAM("Currently only tools in reference_orientation_calibration_supported_tools_ are supported");
        res.success = false;
        return false;
      }

    ROS_WARN("Calculating calibration from the robots current pose relative to the given reference object. Not using "
             "any saved robot_tool_orientation_calibration_samples_ for this calculation");

    // need listener
    tf::TransformListener listener;
    std::string error_msg;
    std::string world_frame = "world";

    if (!listener.waitForTransform(world_frame, tool_surface_frame, ros::Time(0), ros::Duration(1.0),
                                   ros::Duration(0.01), &error_msg))
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
    ROS_INFO_STREAM("Rotation ref_frame_eigen_pose (y, p, r) is: \n"
                    << ref_frame_eigen_pose.rotation().eulerAngles(2, 1, 0));
    ROS_INFO_STREAM("Rotation tool_eigen_pose (y, p, r) is: \n" << tool_eigen_pose.rotation().eulerAngles(2, 1, 0));
    ROS_INFO_STREAM("Rotation tool_to_ref_eigen (y, p, r) is: \n" << tool_to_ref_eigen.rotation().eulerAngles(2, 1, 0));
    ROS_INFO_STREAM("Translation is: \n" << tool_to_ref_eigen.translation());

    // At this point tool_to_ref_eigen.rotation().eulerAngles(2, 1, 0) = the yaw, pitch, roll that the welder frame is
    // off from reference. Need to adjust tool frame to match reference

    // Calculate tool orientation calibration result and call service calculate urdf formated values
    if (robot_tool_orientation_calibration_results_.count(tool_surface_frame) == 0)
    {
      // no key for the frame exists. Add
      robot_tool_orientation_calibration_results_[tool_surface_frame] = Eigen::Vector3d();
    }

    robot_tool_orientation_calibration_results_[tool_surface_frame] = tool_to_ref_eigen.rotation().eulerAngles(2, 1, 0);

    // caluclate urdf calibration values
    if (!calculateUrdfFormatedToolCalibration(tool_surface_frame))
    {
      res.success = false;
      return false;
    }

    res.success = true;
    return true;
  }
  else if (req.service_call_cmd == sss_msgs::GetToolCalibrationRequest::UPDATE_TOOL_URDF)
  {
    if (!touch_point_result_avaliable_[tool_surface_frame])
    {
      ROS_ERROR_STREAM("No position calibration result avaliable. Not updating tool urdf");
      res.success = false;
      return false;
    }

    if (robot_tool_urdf_formated_calibration_.count(tool_surface_frame) == 0)
    {
      // no value avaliable for the tool surface
      ROS_ERROR_STREAM("No robot_tool_urdf_formated_calibration_ for tool surface: " << tool_surface_frame);
      res.success = false;
      return false;
    }

    // update the values from the tools urdf
    tool_calibration::CalibrationUrdfUpdate calibration_urdf_update_srv;
    // fill request
    calibration_urdf_update_srv.request.robot_tool_surface = tool_surface_frame;
    calibration_urdf_update_srv.request.calibration_x =
        robot_tool_urdf_formated_calibration_[tool_surface_frame].calibration_x;
    calibration_urdf_update_srv.request.calibration_y =
        robot_tool_urdf_formated_calibration_[tool_surface_frame].calibration_y;
    calibration_urdf_update_srv.request.calibration_z =
        robot_tool_urdf_formated_calibration_[tool_surface_frame].calibration_z;

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
      calibration_urdf_update_srv.request.calibration_roll =
          robot_tool_urdf_formated_calibration_[tool_surface_frame].calibration_roll;
      calibration_urdf_update_srv.request.calibration_pitch =
          robot_tool_urdf_formated_calibration_[tool_surface_frame].calibration_pitch;
      calibration_urdf_update_srv.request.calibration_yaw =
          robot_tool_urdf_formated_calibration_[tool_surface_frame].calibration_yaw;
    }

    if (calibration_urdf_update_client_.call(calibration_urdf_update_srv))
    {
      if (!calibration_urdf_update_srv.response.success)
      {
        ROS_ERROR_STREAM("Failed to update calibration values from urdf for " << tool_surface_frame);
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

  // handle point sampling commands separately below:
  

  // Create a transform listener to query tool frames. Check if they actually exist
  tf::TransformListener listener;
  std::string error_msg;
  if (!listener.waitForTransform(base_frame, tool_surface_parent_frame, ros::Time(0), ros::Duration(1.0), ros::Duration(0.01),
                                 &error_msg))
  {
    ROS_WARN_STREAM("Unable to lookup transform between base frame: '" << base_frame << "' and tool surface parent frame: '"
                                                                       << tool_surface_parent_frame
                                                                       << "'. TF reported error: " << error_msg);
    bool base_found = listener.frameExists(base_frame);
    bool tool_parent_found = listener.frameExists(tool_surface_parent_frame);
    if (!base_found && !tool_parent_found)
    {
      ROS_WARN("Check to make sure that a robot state publisher or other node is publishing"
               " tf frames for your robot. Also check that your base/tool frames names are"
               " correct and not missing a prefix, for example.");
    }
    else if (!base_found)
    {
      ROS_WARN("Check to make sure that base frame '%s' actually exists.", base_frame.c_str());
    }
    else if (!tool_parent_found)
    {
      ROS_WARN("Check to make sure that tool_surface_parent_frame '%s' actually exists.", tool_surface_parent_frame.c_str());
    }
    res.success = false;
    return true;
  }

  // sample point
  Eigen::Isometry3d eigen_pose;
  tf::StampedTransform transform;
  try
  {
    listener.lookupTransform(base_frame, tool_surface_parent_frame, ros::Time(0), transform);
    tf::poseTFToEigen(transform, eigen_pose);
    ROS_INFO_STREAM("Pose captured transform:\n" << eigen_pose.matrix());
  }
  catch (const tf::TransformException& ex)
  {
    ROS_ERROR("Unable to lookup transform");
    res.success = false;
    return true;
  }

  // save sample point
  if (req.service_call_cmd == sss_msgs::GetToolCalibrationRequest::TOOL_POINT_SAMPLE)
  {
    // collect tool point sample

    if (robot_tool_calibration_samples_.count(tool_surface_frame) == 0)
    {
      // no key for the frame exists. Add
      robot_tool_calibration_samples_[tool_surface_frame] = {};
    }

    if (robot_tool_calibration_samples_[tool_surface_frame].size() < 1)
    {
      // first point collected
      ROS_INFO("Starting tool calibration samples with base frame: '%s' and tool surface parent frame: '%s'.",
               base_frame.c_str(), tool_surface_parent_frame.c_str());
    }

    robot_tool_calibration_samples_[tool_surface_frame].push_back(eigen_pose);
    double number_of_samples = robot_tool_calibration_samples_[tool_surface_frame].size();
    ROS_INFO_STREAM(number_of_samples << " tool point sample(s) collected for " << tool_surface_frame);
  }
  else if (req.service_call_cmd == sss_msgs::GetToolCalibrationRequest::TOOL_ORIENTATION_SAMPLE_P1 ||
           req.service_call_cmd == sss_msgs::GetToolCalibrationRequest::TOOL_ORIENTATION_SAMPLE_P2 ||
           req.service_call_cmd == sss_msgs::GetToolCalibrationRequest::TOOL_ORIENTATION_SAMPLE_P3)
  {
    // collect tool orientation sample

    if (robot_tool_orientation_calibration_samples_.count(tool_surface_frame) == 0)
    {
      // no key for the frame exists. Add
      robot_tool_orientation_calibration_samples_[tool_surface_frame] = {};
      robot_tool_orientation_calibration_samples_[tool_surface_frame].resize(3);
      // first point collected
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

    // save sample
    robot_tool_orientation_calibration_samples_[tool_surface_frame][save_index] = eigen_pose;

    ROS_INFO_STREAM("Point " << (save_index + 1) << " tool orientation sample collected for " << tool_surface_frame);
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
  // calculates the tool calibration in CalibrationUrdfRetrieveResponse format

  // retrieve the latest calibration values from the tools urdf
  tool_calibration::CalibrationUrdfRetrieve calibration_urdf_retrieve_srv;
  calibration_urdf_retrieve_srv.request.robot_tool_surface = tool_surface_frame;
  if (calibration_urdf_retrieve_client_.call(calibration_urdf_retrieve_srv))
  {
    if (!calibration_urdf_retrieve_srv.response.success)
    {
      ROS_ERROR_STREAM("Failed to retieve calibration values from urdf for " << tool_surface_frame);
      return false;
    }
  }
  else
  {
    ROS_ERROR("Failed to call calibration_urdf_retrieve service node");
    return false;
  }

  tool_calibration::CalibrationUrdfRetrieveResponse tool_calibration;
  tool_calibration.success = false;
  if (robot_tool_urdf_formated_calibration_.count(tool_surface_frame) == 0)
  {
    // no key for the frame exists. Add
    robot_tool_urdf_formated_calibration_[tool_surface_frame] = tool_calibration;
  }

  tool_calibration.calibration_x = robot_tool_calibration_results_[tool_surface_frame].tcp_offset.transpose()[0];
  tool_calibration.calibration_y = robot_tool_calibration_results_[tool_surface_frame].tcp_offset.transpose()[1];
  tool_calibration.calibration_z = robot_tool_calibration_results_[tool_surface_frame].tcp_offset.transpose()[2];

  // If orientation result avaliable update
  if (robot_tool_orientation_calibration_results_.count(tool_surface_frame) != 0)
  {
    tool_calibration.calibration_roll = robot_tool_orientation_calibration_results_[tool_surface_frame](2);
    tool_calibration.calibration_pitch = robot_tool_orientation_calibration_results_[tool_surface_frame](1);
    tool_calibration.calibration_yaw = robot_tool_orientation_calibration_results_[tool_surface_frame](0);
  }
  else
  {
    // just set as is
    tool_calibration.calibration_roll = calibration_urdf_retrieve_srv.response.calibration_roll;
    tool_calibration.calibration_pitch = calibration_urdf_retrieve_srv.response.calibration_pitch;
    tool_calibration.calibration_yaw = calibration_urdf_retrieve_srv.response.calibration_yaw;
  }

  tool_calibration.success = true;

  robot_tool_urdf_formated_calibration_[tool_surface_frame] = tool_calibration;

  return true;
}

bool ToolCalibrationServer::calculateRotationFromSamples(const std::string tool_surface_frame)
{
  // Code used from https://github.com/mahmoud-a-ali/tool_point_calibration/tree/orientation

  // 0 position vector to match the tool surface frame
  Eigen::Vector4d r0ns_n = { 0.0, 0.0, 0.0, 1 };
  // point 1
  Eigen::Vector4d rd_0 = robot_tool_orientation_calibration_samples_[tool_surface_frame][0] * r0ns_n;
  Eigen::Vector4d r0ns_0 = rd_0;
  //  point 2: calculate delat1 and then c11, c21, c31
  Eigen::Vector4d V1;  //{ vx, vy, vz, 1};
  V1 = robot_tool_orientation_calibration_samples_[tool_surface_frame][1].inverse() *
       robot_tool_orientation_calibration_samples_[tool_surface_frame][0] *
       r0ns_n;  // r0ns_0 has been modified to r0ns_n
  double term1 = (V1(0) - r0ns_n(0));
  double term2 = (V1(1) - r0ns_n(1));
  double term3 = (V1(2) - r0ns_n(2));
  double delta1 = -1 * sqrt(term1 * term1 + term2 * term2 + term3 * term3);
  double c11 = term1 / delta1;
  double c21 = term2 / delta1;
  double c31 = term3 / delta1;
  ROS_INFO_STREAM(" delat1: " << delta1);
  ROS_INFO_STREAM("c11: " << c11 << ", c21: " << c21 << ", c31: " << c31);
  //  point 3: calculate delta2 and then c12, c22, c32
  Eigen::Vector4d V2;  //={ vx, vy, vz, 1};
  V2 = robot_tool_orientation_calibration_samples_[tool_surface_frame][2].inverse() *
       robot_tool_orientation_calibration_samples_[tool_surface_frame][0] *
       r0ns_n;  // r0ns_0 has been modified to r0ns_n
  term1 = (V2(0) - r0ns_n(0));
  term2 = (V2(1) - r0ns_n(1));
  term3 = (V2(2) - r0ns_n(2));
  double delta2 = 1 * sqrt(term1 * term1 + term2 * term2 + term3 * term3);
  double c12 = term1 / delta2;
  double c22 = term2 / delta2;
  double c32 = term3 / delta2;
  ROS_INFO_STREAM(" delat2: " << delta2);
  ROS_INFO_STREAM("c21: " << c21 << " ,c22: " << c22 << ", c32: " << c32);
  // calculate c31, c32, c33;  c13=k1.c33  && c23=k2.c33
  double k1 = (c21 * c32 - c22 * c31) / (c11 * c22 - c21 * c12);
  double k2 = (c12 * c31 - c11 * c31) / (c11 * c22 - c21 * c12);
  double c33 = sqrt(1 / (1 + k1 * k1 + k2 * k2));
  double c13 = k1 * c33;
  double c23 = k2 * c33;
  ROS_INFO_STREAM("c31: " << c31 << ", c32: " << c32 << ", c33: " << c33);
  // store rotation matrix and translation
  Eigen::Matrix3d Rot;
  Rot << c11, c12, c13, c21, c22, c23, c31, c32, c33;
  Eigen::Vector3d transl = { r0ns_n(0), r0ns_n(1), r0ns_n(2) };
  // std::cout<<"Rot_matrix:\n "<< Rot <<std::endl;
  ROS_INFO_STREAM("Rot_matrix:\n " << Rot);
  ROS_INFO_STREAM("translation:\n " << transl);
  // build final transformation
  // form rotation matrix using values of cs
  Eigen::Isometry3d Anns;
  Anns.matrix() << c11, c12, c13, r0ns_n(0), c21, c22, c23, r0ns_n(1), c31, c32, c33, r0ns_n(2), 0, 0, 0, 1;

  Eigen::Isometry3d measured_to_actual =
      robot_tool_orientation_calibration_samples_[tool_surface_frame][0].inverse() * Anns;

  ROS_INFO_STREAM("Rotation measured_to_actual (y, p, r) is: \n" << measured_to_actual.rotation().eulerAngles(2, 1, 0));

  ROS_INFO(" Done !");

  return true;
}

}  // namespace tool_calibration
