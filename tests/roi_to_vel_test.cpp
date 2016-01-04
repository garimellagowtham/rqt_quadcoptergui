#include "rqt_quadcoptergui/roi_to_vel.h"
#include <gtest/gtest.h>

TEST(RoiToVelTests, testLevelStraightAhead)
{
  sensor_msgs::RegionOfInterest roi;
  roi.x_offset = 0;
  roi.y_offset = 0;
  roi.height = 0;
  roi.width = 0;

  sensor_msgs::CameraInfo cam_info;
  cam_info.K[0] = 1;
  cam_info.K[2] = 0;
  cam_info.K[4] = 1;
  cam_info.K[5] = 0;

  geometry_msgs::Vector3 body_rpy;
  body_rpy.x = 0;
  body_rpy.y = 0;
  body_rpy.z = 0;

  tf::Transform cam_transform(tf::createQuaternionFromRPY(-M_PI/2,0,-M_PI/2));
  double vel_mag = 1;
  double yaw_gain = 1;

  geometry_msgs::Vector3 body_vel;
  double yaw_vel;
  roiToVel(roi, body_rpy, cam_info, cam_transform, vel_mag, yaw_gain, body_vel, yaw_vel);

  ASSERT_NEAR(body_vel.x,1, 1e-10);
  ASSERT_NEAR(body_vel.y,0, 1e-10);
  ASSERT_NEAR(body_vel.z,0, 1e-10);
  ASSERT_NEAR(yaw_vel,0,1e-10);
}

TEST(RoiToVelTests, testPitchedStraightAhead)
{
  sensor_msgs::RegionOfInterest roi;
  roi.x_offset = 0;
  roi.y_offset = 0;
  roi.height = 0;
  roi.width = 0;

  sensor_msgs::CameraInfo cam_info;
  cam_info.K[0] = 1;
  cam_info.K[2] = 0;
  cam_info.K[4] = 1;
  cam_info.K[5] = 0;

  geometry_msgs::Vector3 body_rpy;
  body_rpy.x = 0;
  body_rpy.y = M_PI/8;
  body_rpy.z = 0;

  tf::Transform cam_transform(tf::createQuaternionFromRPY(-M_PI/2,0,-M_PI/2));
  double vel_mag = 1;
  double yaw_gain = 1;

  geometry_msgs::Vector3 body_vel;
  double yaw_vel;
  roiToVel(roi, body_rpy, cam_info, cam_transform, vel_mag, yaw_gain, body_vel, yaw_vel);

  ASSERT_NEAR(body_vel.x, vel_mag*cos(body_rpy.y), 1e-10);
  ASSERT_NEAR(body_vel.y,0, 1e-10);
  ASSERT_NEAR(body_vel.z, -vel_mag*sin(body_rpy.y), 1e-10);
  ASSERT_NEAR(yaw_vel,0,1e-10);
}

TEST(RoiToVelTests, testRolledPitchedStraightAhead)
{
  sensor_msgs::RegionOfInterest roi;
  roi.x_offset = 0;
  roi.y_offset = 0;
  roi.height = 0;
  roi.width = 0;

  sensor_msgs::CameraInfo cam_info;
  cam_info.K[0] = 1;
  cam_info.K[2] = 0;
  cam_info.K[4] = 1;
  cam_info.K[5] = 0;

  geometry_msgs::Vector3 body_rpy;
  body_rpy.x = M_PI/3;
  body_rpy.y = M_PI/8;
  body_rpy.z = 0;

  tf::Transform cam_transform(tf::createQuaternionFromRPY(-M_PI/2,0,-M_PI/2));
  double vel_mag = 1;
  double yaw_gain = 1;

  geometry_msgs::Vector3 body_vel;
  double yaw_vel;
  roiToVel(roi, body_rpy, cam_info, cam_transform, vel_mag, yaw_gain, body_vel, yaw_vel);

  ASSERT_NEAR(body_vel.x, vel_mag*cos(body_rpy.y), 1e-10);
  ASSERT_NEAR(body_vel.y,0, 1e-10);
  ASSERT_NEAR(body_vel.z, -vel_mag*sin(body_rpy.y), 1e-10);
  ASSERT_NEAR(yaw_vel,0,1e-10);
}

TEST(RoiToVelTests, testLevelRoiLeft)
{
  sensor_msgs::RegionOfInterest roi;
  roi.x_offset = 200;
  roi.width = 100;
  roi.y_offset = 250;
  roi.height = 100;

  sensor_msgs::CameraInfo cam_info;
  cam_info.K[0] = 200;
  cam_info.K[2] = 300;
  cam_info.K[4] = 200;
  cam_info.K[5] = 300;

  geometry_msgs::Vector3 body_rpy;
  body_rpy.x = 0;
  body_rpy.y = 0;
  body_rpy.z = 0;

  tf::Transform cam_transform(tf::createQuaternionFromRPY(-M_PI/2,0,-M_PI/2));
  cam_transform.setOrigin(tf::Vector3(3,3,3));
  double vel_mag = 1;
  double yaw_gain = 1;

  geometry_msgs::Vector3 body_vel;
  double yaw_vel;
  roiToVel(roi, body_rpy, cam_info, cam_transform, vel_mag, yaw_gain, body_vel, yaw_vel);

  ASSERT_NEAR(body_vel.x, .97, 1e-3);
  ASSERT_NEAR(body_vel.y,0, 1e-10);
  ASSERT_NEAR(body_vel.z, 0, 1e-10);
  ASSERT_NEAR(yaw_vel,.245, 1e-3);
}

TEST(RoiToVelTests, testLevelRoiRight)
{
  sensor_msgs::RegionOfInterest roi;
  roi.x_offset = 300;
  roi.width = 100;
  roi.y_offset = 250;
  roi.height = 100;

  sensor_msgs::CameraInfo cam_info;
  cam_info.K[0] = 200;
  cam_info.K[2] = 300;
  cam_info.K[4] = 200;
  cam_info.K[5] = 300;

  geometry_msgs::Vector3 body_rpy;
  body_rpy.x = 0;
  body_rpy.y = 0;
  body_rpy.z = 0;

  tf::Transform cam_transform(tf::createQuaternionFromRPY(-M_PI/2,0,-M_PI/2));
  double vel_mag = 1;
  double yaw_gain = 1;

  geometry_msgs::Vector3 body_vel;
  double yaw_vel;
  roiToVel(roi, body_rpy, cam_info, cam_transform, vel_mag, yaw_gain, body_vel, yaw_vel);

  ASSERT_NEAR(body_vel.x, .97, 1e-3);
  ASSERT_NEAR(body_vel.y,0, 1e-10);
  ASSERT_NEAR(body_vel.z, 0, 1e-10);
  ASSERT_NEAR(yaw_vel,-.245, 1e-3);
}

TEST(RoiToVelTests, testLevelRoiDown)
{
  sensor_msgs::RegionOfInterest roi;
  roi.x_offset = 250;
  roi.width = 100;
  roi.y_offset = 300;
  roi.height = 100;

  sensor_msgs::CameraInfo cam_info;
  cam_info.K[0] = 200;
  cam_info.K[2] = 300;
  cam_info.K[4] = 200;
  cam_info.K[5] = 300;

  geometry_msgs::Vector3 body_rpy;
  body_rpy.x = 0;
  body_rpy.y = 0;
  body_rpy.z = -M_PI/3;

  tf::Transform cam_transform(tf::createQuaternionFromRPY(-M_PI/2,0,-M_PI/2));
  double vel_mag = 1;
  double yaw_gain = 1;

  geometry_msgs::Vector3 body_vel;
  double yaw_vel;
  roiToVel(roi, body_rpy, cam_info, cam_transform, vel_mag, yaw_gain, body_vel, yaw_vel);

  ASSERT_NEAR(body_vel.x, .97, 1e-3);
  ASSERT_NEAR(body_vel.y,0, 1e-10);
  ASSERT_NEAR(body_vel.z, -.242, 1e-3);
  ASSERT_NEAR(yaw_vel, 0, 1e-10);
}

TEST(RoiToVelTests, testLevelRoiUp)
{
  sensor_msgs::RegionOfInterest roi;
  roi.x_offset = 250;
  roi.width = 100;
  roi.y_offset = 200;
  roi.height = 100;

  sensor_msgs::CameraInfo cam_info;
  cam_info.K[0] = 200;
  cam_info.K[2] = 300;
  cam_info.K[4] = 200;
  cam_info.K[5] = 300;

  geometry_msgs::Vector3 body_rpy;
  body_rpy.x = 0;
  body_rpy.y = 0;
  body_rpy.z = M_PI/2;

  tf::Transform cam_transform(tf::createQuaternionFromRPY(-M_PI/2,0,-M_PI/2));
  double vel_mag = 1;
  double yaw_gain = 1;

  geometry_msgs::Vector3 body_vel;
  double yaw_vel;
  roiToVel(roi, body_rpy, cam_info, cam_transform, vel_mag, yaw_gain, body_vel, yaw_vel);

  ASSERT_NEAR(body_vel.x, .97, 1e-3);
  ASSERT_NEAR(body_vel.y,0, 1e-10);
  ASSERT_NEAR(body_vel.z, .242, 1e-3);
  ASSERT_NEAR(yaw_vel, 0, 1e-10);
}

TEST(RoiToVelTests, testPitchedRoiLeft)
{
  sensor_msgs::RegionOfInterest roi;
  roi.x_offset = 200;
  roi.width = 100;
  roi.y_offset = 250;
  roi.height = 100;

  sensor_msgs::CameraInfo cam_info;
  cam_info.K[0] = 200;
  cam_info.K[2] = 300;
  cam_info.K[4] = 200;
  cam_info.K[5] = 300;

  geometry_msgs::Vector3 body_rpy;
  body_rpy.x = 0;
  body_rpy.y = M_PI/8;
  body_rpy.z = 0;

  tf::Transform cam_transform(tf::createQuaternionFromRPY(-M_PI/2,0,-M_PI/2));
  double vel_mag = 1;
  double yaw_gain = 1;

  geometry_msgs::Vector3 body_vel;
  double yaw_vel;
  roiToVel(roi, body_rpy, cam_info, cam_transform, vel_mag, yaw_gain, body_vel, yaw_vel);

  ASSERT_NEAR(body_vel.x, .97*cos(body_rpy.y), 1e-3);
  ASSERT_NEAR(body_vel.y, 0, 1e-10);
  ASSERT_NEAR(body_vel.z, -.97*sin(body_rpy.y), 1e-3);
  ASSERT_NEAR(yaw_vel,.264, 1e-3);
}

TEST(RoiToVelTests, testInitVeld)
{
  sensor_msgs::RegionOfInterest roi;
  roi.x_offset = 0;
  roi.y_offset = 0;
  roi.height = 0;
  roi.width = 0;

  sensor_msgs::CameraInfo cam_info;
  cam_info.K[0] = 1;
  cam_info.K[2] = 0;
  cam_info.K[4] = 1;
  cam_info.K[5] = 0;

  geometry_msgs::Vector3 body_rpy;
  body_rpy.x = 0;
  body_rpy.y = 0;
  body_rpy.z = 0;

  tf::Transform cam_transform(tf::createQuaternionFromRPY(-M_PI/2,0,-M_PI/2));
  double vel_mag = 1;
  double yaw_gain = 1;

  tf::Vector3 des_obj_dir;
  initializeVd(roi, body_rpy, cam_info, cam_transform, des_obj_dir);
  printf("des_obj_dir: %f,%f,%f",des_obj_dir[0], des_obj_dir[1], des_obj_dir[2]);
  //roiToVel(roi, body_rpy, cam_info, cam_transform, vel_mag, yaw_gain, body_vel, yaw_vel);

  //ASSERT_NEAR(body_vel.x,1, 1e-10);
  //ASSERT_NEAR(body_vel.y,0, 1e-10);
  //ASSERT_NEAR(body_vel.z,0, 1e-10);
  //ASSERT_NEAR(yaw_vel,0,1e-10);
}

TEST(RoiToVelTests, testroiVel2)
{
  sensor_msgs::RegionOfInterest roi;
  roi.x_offset = 0;
  roi.y_offset = 0;
  roi.height = 0;
  roi.width = 0;

  sensor_msgs::CameraInfo cam_info;
  cam_info.K[0] = 1;
  cam_info.K[2] = 0;
  cam_info.K[4] = 1;
  cam_info.K[5] = 0;

  geometry_msgs::Vector3 body_rpy;
  body_rpy.x = 0;
  body_rpy.y = 0;
  body_rpy.z = 0;

  tf::Transform cam_transform(tf::createQuaternionFromRPY(-M_PI/2,0,-M_PI/2));
  double vel_mag = 1;
  double yaw_gain = 1;
  double radial_gain = 1;
  double tangential_gain = 1;

  tf::Vector3 des_obj_dir(1/sqrt(3),1/sqrt(3),1/sqrt(3));
  double dist_obj = 2.0;
  double des_obj_dist = 1.0;
  geometry_msgs::Vector3 body_vel;
  double yaw_vel;
  roiToVel2(roi, body_rpy, cam_info, cam_transform, dist_obj, body_vel, yaw_vel, yaw_gain, radial_gain, tangential_gain, des_obj_dir, des_obj_dist);
  printf("des_body_vel: %f,%f,%f; des_yaw_vel: %f",body_vel.x, body_vel.y, body_vel.z, yaw_vel);

  //ASSERT_NEAR(body_vel.x,1, 1e-10);
  //ASSERT_NEAR(body_vel.y,0, 1e-10);
  //ASSERT_NEAR(body_vel.z,0, 1e-10);
  //ASSERT_NEAR(yaw_vel,0,1e-10);
}

int main(int argc, char** argv)
{

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
