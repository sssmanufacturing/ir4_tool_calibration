#include <tool_point_calibration/tool_point_calibration.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include<Eigen/Dense>
using namespace Eigen;
using Eigen::MatrixXd;
/**
this node is used to calibrate the orientation of the tool point.
it is an implementation of a paper entitled:
Methods of Calibrating the Orientation of the Industrial Robot Tool[1].
variables names are same as the ones used in the paper.
-------------------
[1]https://www.researchgate.net/publication/330247404_
Methods_of_Calibrating_the_Orientation_of_the_Industrial_Robot_Tool
**/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tool_orientation_calibration");
  ros::NodeHandle pnh ("~");
  // Load user parmameters
  std::string base_frame, tool0_frame;
  int num_samples;
  pnh.param<std::string>("base_frame", base_frame, "base_link");
  pnh.param<std::string>("tool0_frame", tool0_frame, "tool0");
  pnh.param<int>("num_samples", num_samples, 4);
  ROS_INFO("Starting tool calibration with base frame: '%s' and tool0 frame: '%s'.",
           base_frame.c_str(), tool0_frame.c_str());
  ROS_INFO("\n ======= step 1: translation calibration ========== ");
  ROS_INFO("Move the robot to '%d' different poses, each of which should touch"
           " the tool to the same position in space.\n", num_samples);
  // Create a transform listener to query tool frames
  tf::TransformListener listener;
  std::string error_msg;
  if (!listener.waitForTransform(base_frame, tool0_frame, ros::Time(0),
                                 ros::Duration(1.0), ros::Duration(0.01),
                                 &error_msg))
  {
    ROS_WARN_STREAM("Unable to lookup transform between base frame: '" << base_frame
                    << "' and tool frame: '" << tool0_frame << "'. TF reported error: "
                    << error_msg);
    bool base_found = listener.frameExists(base_frame);
    bool tool_found = listener.frameExists(tool0_frame);
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
      ROS_WARN("Check to make sure that tool0 frame '%s' actually exists.", tool0_frame.c_str());
    }
    return 1;
  }
  // Create storage for user observations
  tool_point_calibration::Isometry3dVector observations;
  observations.reserve(num_samples);
  std::string line;
  int count = 0;
  Eigen::Isometry3d eigen_pose;
  // While ros is ok and there are more counts to be done...
  while (ros::ok() && count < num_samples)
  {
    ROS_INFO("Pose %d: Jog robot to a new location touching the shared position and"
             " press enter.", count);
    std::getline(std::cin, line); // Blocks program until enter is pressed
    tf::StampedTransform transform;
    try
    {
      listener.lookupTransform(base_frame, tool0_frame,
                               ros::Time(0), transform);
      tf::poseTFToEigen(transform, eigen_pose);
      observations.push_back(eigen_pose);
      ROS_INFO_STREAM("Pose " << count << ": captured transform:\n" << eigen_pose.matrix());
      count++;
    }
    catch (const tf::TransformException& ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
  }
  ROS_INFO("Calibration captured %d tool poses (out of %d requested). Computing calibration...",
           count, num_samples);
  tool_point_calibration::TcpCalibrationResult result =
      tool_point_calibration::calibrateTcp(observations);
  ROS_INFO_STREAM("Calibrated tcp (meters in xyz): [" << result.tcp_offset.transpose() << "] from " << tool0_frame);
  ROS_INFO_STREAM("Touch point (meters in xyz): [" << result.touch_point.transpose() << "] in frame " << base_frame);
  ROS_INFO_STREAM("Average residual: " << result.average_residual);
  ROS_INFO_STREAM("Converged: " << result.converged);
  // store the translation vector for the next step:
  Vector3d tcp_transl;
  tcp_transl = result.tcp_offset.transpose();
  if (count < 4)
  {
    ROS_WARN("Computing a tool calibration w/ fewer than 4 points may produce an answer with good"
             " convergence and residual error, but without a meaningful result. You are encouraged"
             " to try with more points");
  }
  ROS_INFO("=======\n step 2: TCP Orientation calibration using three point method ========== ");
  std::vector<Isometry3d> A0n; // to store the transformation matrix for the points
  A0n.resize( 3);
  A0n[0] = eigen_pose;
  ROS_INFO("the current pose is considered as the First point ... ");
  for (int i=1; i<3; i++)
  {
    if(i ==1)
        ROS_INFO("Second point: move the TCP to any location along the X axis of the tool and press enter");
    else
        ROS_INFO("Third Point: move TCP to any location along the Y axis of the tool and press enter.");
    std::getline(std::cin, line); // Blocks program until enter is pressed
    tf::StampedTransform transform;
    try
    {
      listener.lookupTransform(base_frame, tool0_frame, ros::Time(0), transform);
      Eigen::Isometry3d eigen_pose;
      tf::poseTFToEigen(transform, eigen_pose); 
      A0n[i] = eigen_pose;
      ROS_INFO_STREAM("Pose " << i << ": captured transform:\n" << eigen_pose.matrix());
    }
    catch (const tf::TransformException& ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
  }
  // vector containing the coordinates of the calibrated tool TCP in the Sn ≡ FLANGE coordinate system [1]
  Vector4d r0ns_n = {tcp_transl[0], tcp_transl[1], tcp_transl[2], 1};
  // vector of the top of the pointed tip attached to the robot's working space [1]
  Vector4d rd_0 = A0n[0]*r0ns_n;
  Vector4d r0ns_0 = rd_0; 
  //  point 2: calculate delat1 and then c11, c21, c31
  Vector4d V1;//{ vx, vy, vz, 1};
  V1 = A0n[1].inverse() * A0n[0] * r0ns_n; // r0ns_0 has been modified to r0ns_n 
  double term1 = ( V1(0) - r0ns_n(0) );
  double term2 = ( V1(1) - r0ns_n(1) );
  double term3 = ( V1(2) - r0ns_n(2) );
  double delta1 = -1* sqrt( term1*term1 + term2*term2 + term3*term3);
  double c11 = term1 / delta1;
  double c21 = term2 / delta1;
  double c31 = term3 / delta1;
  ROS_INFO_STREAM(" delat1: "<< delta1);
  ROS_INFO_STREAM("c11: "<< c11 << ", c21: "<< c21 << ", c31: "<<c31);
  //  point 3: calculate delta2 and then c12, c22, c32
  Vector4d V2; //={ vx, vy, vz, 1};
  V2 = A0n[2].inverse() * A0n[0] * r0ns_n; // r0ns_0 has been modified to r0ns_n
  term1 = ( V2(0) - r0ns_n(0) );
  term2 = ( V2(1) - r0ns_n(1) );
  term3 = ( V2(2) - r0ns_n(2) );
  double delta2 = 1* sqrt(term1*term1 + term2*term2 + term3*term3);
  double c12 = term1 / delta2;
  double c22 = term2 / delta2;
  double c32 = term3 / delta2;
  ROS_INFO_STREAM(" delat2: "<< delta2);
  ROS_INFO_STREAM("c21: "<< c21 << " ,c22: "<< c22 << ", c32: " << c32);
  //calculate c31, c32, c33;  c13=k1.c33  && c23=k2.c33
  double k1= (c21*c32 - c22*c31)/(c11*c22 - c21*c12);
  double k2= (c12*c31 - c11*c31)/(c11*c22 - c21*c12);
  double c33 = sqrt( 1 /(1+k1*k1+k2*k2) );
  double c13 = k1* c33;
  double c23 = k2* c33;
  ROS_INFO_STREAM("c31: "<< c31 << ", c32: "<< c32 << ", c33: " << c33);
  // store rotation matrix and translation
  Matrix3d Rot;
           Rot << c11, c12, c13,
                  c21, c22, c23,
                  c31, c32, c33;
  Vector3d transl= {r0ns_n(0), r0ns_n(1), r0ns_n(2)};
  // std::cout<<"Rot_matrix:\n "<< Rot <<std::endl;
  ROS_INFO_STREAM("Rot_matrix:\n "<< Rot);
  ROS_INFO_STREAM("translation:\n "<< transl);
  // build final transformation
  // form rotation matrix using values of cs
  Isometry3d Anns;
  Anns.matrix() << c11, c12, c13, r0ns_n(0),
                   c21, c22, c23, r0ns_n(1),
                   c31, c32, c33, r0ns_n(2),
                     0,   0,   0,        1;
  ROS_INFO(" Done !");
  return 0;
}
