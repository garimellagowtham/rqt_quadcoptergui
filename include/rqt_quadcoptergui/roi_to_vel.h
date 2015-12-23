#ifndef _ROITOVEL_H_
#define _ROITOVEL_H_

#define TF_EULER_DEFAULT_ZYX

#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>


/*  @param roi region of interest
 *  @param body_rpy the roll, pitch, and yaw of the body frame relative to the world frame
 *  @param cam_info the camera information
 *  @param cam_transform the transformation of the camera frame relative to the body frame
 *  @param vel_mag the magnitude of the velocity to be output
 *  @param yaw_gain the gain for the yaw velocity control
 *  @param body_vel returns the xyz body velocity needed to move toward the roi.
 *  @param yaw_vel returns the yaw velocity needed to center the roi. It is given in rad/s.
 */
void roiToVel(const sensor_msgs::RegionOfInterest& roi, 
   const geometry_msgs::Vector3& body_rpy, const  sensor_msgs::CameraInfo& cam_info, 
   const tf::Transform& cam_transform, const double& vel_mag, const double& yaw_gain,
   geometry_msgs::Vector3& body_vel, double& yaw_vel) 
{
  double roi_x_cen = roi.x_offset + roi.width/2;
  double roi_y_cen = roi.y_offset + roi.height/2;
  double fx = cam_info.K[0];
  double fy = cam_info.K[4];
  double cx = cam_info.K[2];  
  double cy = cam_info.K[5];  

  double roi3d_x = (roi_x_cen-cx)/fx;
  double roi3d_y = (roi_y_cen-cy)/fy;
  
  tf::Vector3 roi3d(roi3d_x, roi3d_y, 1);
  roi3d.normalize();
  tf::Transform body_rpy_tf(tf::createQuaternionFromRPY(body_rpy.x ,body_rpy.y, 0));
  tf::Vector3 roi_dir = body_rpy_tf*(cam_transform.getBasis()*roi3d);

  body_vel.x = vel_mag*roi_dir.getX();
  body_vel.y = 0;
  body_vel.z = vel_mag*roi_dir.getZ();
  yaw_vel = yaw_gain*atan2(roi_dir.getY(), roi_dir.getX());
}

#endif
