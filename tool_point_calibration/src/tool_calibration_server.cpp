#include <tool_point_calibration/tool_calibration_server.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>

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
      nh_.advertiseService<sss_msgs::GetToolCalibrationSample::Request, sss_msgs::GetToolCalibrationSample::Response>(
          "get_tool_calibration_sample_point",
          boost::bind(&ToolCalibrationServer::sampleToolCalibrationSampleCallback, this, _1, _2));

  calibration_urdf_retrieve_client_ = nh_.serviceClient<tool_point_calibration::CalibrationUrdfRetrieve>("/calibration_"
                                                                                                         "urdf_server/"
                                                                                                         "calibration_"
                                                                                                         "urdf_"
                                                                                                         "retrieve");
  if (!calibration_urdf_retrieve_client_.waitForExistence(ros::Duration(2.0)) && ros::ok())
  {
    ROS_INFO("Waiting for calibration_urdf_server/calibration_urdf_retrieve");
  }
  calibration_urdf_update_client_ = nh_.serviceClient<tool_point_calibration::CalibrationUrdfUpdate>("/calibration_"
                                                                                                     "urdf_server/"
                                                                                                     "calibration_urdf_"
                                                                                                     "update");
  if (!calibration_urdf_update_client_.waitForExistence(ros::Duration(2.0)) && ros::ok())
  {
    ROS_INFO("Waiting for calibration_urdf_server/calibration_urdf_update");
  }
}

bool ToolCalibrationServer::sampleToolCalibrationSampleCallback(const sss_msgs::GetToolCalibrationSample::Request& req,
                                                                sss_msgs::GetToolCalibrationSample::Response& res)
{
  // sanity checks // TODO:
  if (robot_base_frames_.count(req.robot_id) == 0)
  {
    ROS_ERROR_STREAM(req.robot_id << " is not a currently supported robot");
    res.success = false;
    return true;
  }

  // setup general variables
  std::string base_frame, tool_surface_frame;

  // setup frames
  base_frame = req.robot_id + '/' + robot_base_frames_[req.robot_id];
  tool_surface_frame = req.robot_id + '/' + req.tool_surface_name;

  if (req.service_call_cmd == sss_msgs::GetToolCalibrationSampleRequest::TOOL_POINT_SAMPLES_RESET)
  {
    // clear all tool point samples for the given tool surface
    robot_tool_calibration_samples_.erase(tool_surface_frame);
    robot_tool_calibration_results_.erase(tool_surface_frame);
    touch_point_result_avaliable_.erase(tool_surface_frame);
    ROS_INFO_STREAM("Successfully erased  tool point calibration samples");
    res.success = true;
    return true;
  }
  else if (req.service_call_cmd == sss_msgs::GetToolCalibrationSampleRequest::TOOL_ORIENTATION_SAMPLES_RESET)
  {
    // clear all tool orientation samples for the given tool surface
    robot_tool_orientation_calibration_samples_.erase(tool_surface_frame);
    robot_tool_orientation_calibration_results_.erase(tool_surface_frame);
    ROS_INFO_STREAM("Successfully erased orientation calibration samples");
    res.success = true;
    return true;
  }
  else if (req.service_call_cmd == sss_msgs::GetToolCalibrationSampleRequest::CALCULATE_TOOL_POINT_CALIBRATION)
  {
    // calculate tool point calibration result and call service call to update the URDF
    if (robot_tool_calibration_samples_[tool_surface_frame].size() < min_number_of_samples_)
    {
      ROS_ERROR_STREAM("Must have atleast " << min_number_of_samples_ << " to calculate the calibration result for "
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

    // TODO: sanity check that the new calibration makes sense

    touch_point_result_avaliable_[tool_surface_frame] = true;

    // get urdf calibration and update
    if (!calculateUrdfFormatedToolCalibration(tool_surface_frame))
    {
      res.success = false;
      return false;
    }
    else
    {
      ROS_INFO_STREAM("Successfully calculated the tool calibration for " << tool_surface_frame);
      ROS_INFO_STREAM("Calibrated tip from expected (meters in xyz): ["
                      << robot_tool_calibration_results_[tool_surface_frame].tcp_offset.transpose() << "]");
      res.success = true;
      return true;
    }
  }
  else if (req.service_call_cmd == sss_msgs::GetToolCalibrationSampleRequest::CALCULATE_TOOL_ORIENTATION_CALIBRATION)
  {
    // calculate tool orientation calibration result and call service call to update the URDF
    if (robot_tool_orientation_calibration_results_.count(tool_surface_frame) == 0)
    {
      // no key for the frame exists. Add
      robot_tool_orientation_calibration_results_[tool_surface_frame] = {};
    }

    // TODO: Save the orientation result
    // Currently just runs the calculations (Which will print results)
    // Need to figure out how to get into the format we actually want
    if (!calculateToolOrientation(tool_surface_frame))
    {
      ROS_ERROR_STREAM("Failed to calculate tool orientation result");
      res.success = false;
      return false;
    }
    else
    {
      res.success = true;
      return true;
    }
  }
  else if (req.service_call_cmd == sss_msgs::GetToolCalibrationSampleRequest::UPDATE_TOOL_URDF)
  {
    if (!touch_point_result_avaliable_[tool_surface_frame])
    {
      ROS_ERROR_STREAM("No calibration result avaliable. Not updating tool urdf");
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
    tool_point_calibration::CalibrationUrdfUpdate calibration_urdf_update_srv;
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
  else if (req.service_call_cmd == sss_msgs::GetToolCalibrationSampleRequest::REFERENCE_OBJECT_ORIENTATION_CALIBRATION)
  {
    // TODO: for now just hard code a check to make sure we are using the "kr8_r1420_rcb/welder_tool_surface"
    if (tool_surface_frame != "kr8_r1420_rcb/welder_tool_surface")
    {
      ROS_ERROR_STREAM("Currently only the kr8_r1420_rcb/welder_tool_surface is supported");
      res.success = false;
      return false;
    }

    // first do a tolerance check on the point calibration
    // TODO: need to pass through the object all the way from the parsing level 

    // need listener
    tf::TransformListener listener;
    std::string error_msg;
    std::string world_frame = "world";

    if (!listener.waitForTransform(world_frame, tool_surface_frame, ros::Time(0), ros::Duration(1.0), ros::Duration(0.01),
                                  &error_msg))
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
      tf::poseTFToEigen(transform, eigen_pose);
      ROS_INFO_STREAM("Pose captured transform:\n" << eigen_pose.matrix());
    }
    catch (const tf::TransformException& ex)
    {
      ROS_ERROR("Unable to lookup transform");
      res.success = false;
      return true;
    }

    // Now use the pose of the reference object and find the difference from the world_to_tip_transform
    tf::poseMsgToEigen(req.reference_object_pose, tool_eigen_pose);

    Eigen::Isometry3d tool_to_ref_eigen = ref_frame_eigen_pose.inverse() * tool_eigen_pose;

    // TODO: Test prints for testing
    ROS_ERROR_STREAM("Rotation is: " << tool_to_ref_eigen.rotation());
    ROS_ERROR_STREAM("Translation is: " << tool_to_ref_eigen.translation()); 


  }

  // Create a transform listener to query tool frames. Check if they actually exist
  tf::TransformListener listener;
  std::string error_msg;
  if (!listener.waitForTransform(base_frame, tool_surface_frame, ros::Time(0), ros::Duration(1.0), ros::Duration(0.01),
                                 &error_msg))
  {
    ROS_WARN_STREAM("Unable to lookup transform between base frame: '" << base_frame << "' and tool frame: '"
                                                                       << tool_surface_frame
                                                                       << "'. TF reported error: " << error_msg);
    bool base_found = listener.frameExists(base_frame);
    bool tool_found = listener.frameExists(tool_surface_frame);
    if (!base_found && !tool_found)
    {
      ROS_WARN("Check to make sure that a robot state publisher or other node is publishing"
               " tf frames for your robot. Also check that your base/tool frames names are"
               " correct and not missing a prefix, for example.");
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

  // sample point
  Eigen::Isometry3d eigen_pose;
  tf::StampedTransform transform;
  try
  {
    listener.lookupTransform(base_frame, tool_surface_frame, ros::Time(0), transform);
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
  if (req.service_call_cmd == sss_msgs::GetToolCalibrationSampleRequest::TOOL_POINT_SAMPLE)
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
      ROS_INFO("Starting tool calibration samples with base frame: '%s' and tool surface frame: '%s'.",
               base_frame.c_str(), tool_surface_frame.c_str());
    }

    robot_tool_calibration_samples_[tool_surface_frame].push_back(eigen_pose);
    double number_of_samples = robot_tool_calibration_samples_[tool_surface_frame].size();
    ROS_INFO_STREAM(number_of_samples << " tool point sample(s) collected for " << tool_surface_frame);
  }
  else if (req.service_call_cmd == sss_msgs::GetToolCalibrationSampleRequest::TOOL_ORIENTATION_SAMPLE)
  {
    // collect tool orientation sample

    if (robot_tool_orientation_calibration_samples_.count(tool_surface_frame) == 0)
    {
      // no key for the frame exists. Add
      robot_tool_orientation_calibration_samples_[tool_surface_frame] = {};
    }

    if (robot_tool_orientation_calibration_samples_[tool_surface_frame].size() < 1)
    {
      // first point collected
      ROS_INFO("Starting tool orientation calibration samples with base frame: '%s' and tool surface frame: '%s'.",
               base_frame.c_str(), tool_surface_frame.c_str());
    }
    else if (robot_tool_orientation_calibration_samples_[tool_surface_frame].size() >= 3)
    {
      ROS_ERROR_STREAM("There is already 3 samples for the tool orientation. Can only be caluclated with 3 samples");
    }

    robot_tool_orientation_calibration_samples_[tool_surface_frame].push_back(eigen_pose);
    ROS_INFO_STREAM(robot_tool_orientation_calibration_samples_[tool_surface_frame].size()
                    << ' tool orientation sample(s) collected for ' << tool_surface_frame);
  }
  else
  {
    ROS_ERROR_STREAM('Unsupported service_call_cmd');
    res.success = false;
    return true;
  }

  res.success = true;
  return true;
}

bool ToolCalibrationServer::calculateToolOrientation(const std::string& tool_surface_frame)
{
  // sanity checks
  if (!touch_point_result_avaliable_[tool_surface_frame])
  {
    ROS_ERROR_STREAM('No touch point result avaliable for ' << tool_surface_frame
                                                            << ". Must get the point calibration before calculating "
                                                               "the orientation calibration");
  }

  if (robot_tool_orientation_calibration_samples_[tool_surface_frame].size() != 3)
  {
    ROS_ERROR_STREAM('Must have 3 samples to calculate the ');
  }

  // TODO: Verify thes calcilations are correct. Taken from
  // https://github.com/mahmoud-a-ali/tool_point_calibration/tree/orientation vector containing the coordinates of the
  // calibrated tool TCP in the Sn ≡ FLANGE coordinate system [1]
  Eigen::Vector3d tcp_transl;
  tcp_transl = robot_tool_calibration_results_[tool_surface_frame].tcp_offset.transpose();
  Eigen::Vector4d r0ns_n = { tcp_transl[0], tcp_transl[1], tcp_transl[2], 1 };
  // vector of the top of the pointed tip attached to the robot's working space [1]
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
  ROS_INFO(" Done !");

  // TODO: actually output the values as needed for the URDFS. Currently just printing

  return true;
}

bool ToolCalibrationServer::calculateUrdfFormatedToolCalibration(const std::string tool_surface_frame)
{
  // calculates the tool calibration in CalibrationUrdfRetrieveResponse format

  // retrieve the latest calibration values from the tools urdf
  tool_point_calibration::CalibrationUrdfRetrieve calibration_urdf_retrieve_srv;
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

  tool_point_calibration::CalibrationUrdfRetrieveResponse tool_calibration;
  tool_calibration.success = false;
  if (robot_tool_urdf_formated_calibration_.count(tool_surface_frame) == 0)
  {
    // no key for the frame exists. Add
    robot_tool_urdf_formated_calibration_[tool_surface_frame] = tool_calibration;
  }

  tool_calibration.calibration_x = robot_tool_calibration_results_[tool_surface_frame].tcp_offset.transpose()[0] +
                                   calibration_urdf_retrieve_srv.response.calibration_x;
  tool_calibration.calibration_y = robot_tool_calibration_results_[tool_surface_frame].tcp_offset.transpose()[1] +
                                   calibration_urdf_retrieve_srv.response.calibration_y;
  tool_calibration.calibration_z = robot_tool_calibration_results_[tool_surface_frame].tcp_offset.transpose()[2] +
                                   calibration_urdf_retrieve_srv.response.calibration_z;

  // TODO: Temp just always set as URDF value
  tool_calibration.calibration_roll = calibration_urdf_retrieve_srv.response.calibration_roll;
  tool_calibration.calibration_pitch = calibration_urdf_retrieve_srv.response.calibration_pitch;
  tool_calibration.calibration_yaw = calibration_urdf_retrieve_srv.response.calibration_yaw;

  tool_calibration.success = true;

  // TODO: figure out orientation formating
  /*
   */

  robot_tool_urdf_formated_calibration_[tool_surface_frame] = tool_calibration;

  return true;
}

}  // namespace tool_calibration
