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
/** This function is a controller to a specified pose on a sphere based on known orientation of quadcopter
* and given distance of object from quadcopter and roi of the object
*/
void roiToVel2(const sensor_msgs::RegionOfInterest& roi, 
   const geometry_msgs::Vector3& body_rpy, const  sensor_msgs::CameraInfo& cam_info, 
   const tf::Transform& cam_transform, const double dist_obj,
   geometry_msgs::Vector3& body_vel, double& yaw_vel,
   const double yaw_gain, const double radial_gain, const double tangential_gain, 
   const tf::Vector3& des_obj_dir, const double des_obj_dist
   ) 
{
  double roi_x_cen = roi.x_offset + roi.width/2;
  double roi_y_cen = roi.y_offset + roi.height/2;
  double fx = cam_info.K[0];//Use Opencv function to convert pixel to m using Camera Intrinsics
  double fy = cam_info.K[4];
  double cx = cam_info.K[2];  
  double cy = cam_info.K[5];  

  double roi3d_x = (roi_x_cen-cx)/fx;
  double roi3d_y = (roi_y_cen-cy)/fy;
  
  tf::Vector3 roi3d(roi3d_x, roi3d_y, 1);
  roi3d.normalize();
  tf::Transform body_rpy_tf(tf::createQuaternionFromRPY(body_rpy.x ,body_rpy.y, body_rpy.z));
  tf::Vector3 roi_dir = body_rpy_tf*(cam_transform.getBasis()*roi3d);//In Global frame = Object Frame.
  //The desired vel in global frame:
  tf::Vector3 obj_diff = (des_obj_dist*des_obj_dir - dist_obj*roi_dir);
  tf::Vector3 obj_diff_radial = des_obj_dir*(obj_diff.dot(des_obj_dir));
  tf::Vector3 obj_diff_tangential = obj_diff - obj_diff_radial;
  tf::Vector3 global_vel_tf = obj_diff_radial*radial_gain + obj_diff_tangential*tangential_gain;
  //Convert to Body Frame:
  tf::Vector3 body_vel_tf = tf::Matrix3x3(tf::createQuaternionFromRPY(0,0,-body_rpy.z))*global_vel_tf;
  tf::vector3TFToMsg(body_vel_tf, body_vel);
  //Find Yaw diff between des_obj_dir and current roi_dir
  tf::Vector3 proj_roi_dir = tf::Vector3(roi_dir[0], roi_dir[1], 0);
  tf::Vector3 proj_des_obj_dir = tf::Vector3(des_obj_dir[0], des_obj_dir[1], 0);
  yaw_vel = yaw_gain*proj_des_obj_dir.angle(proj_roi_dir);
}
/**
*/
void initializeVd(const sensor_msgs::RegionOfInterest& roi, 
   const geometry_msgs::Vector3& body_rpy, const  sensor_msgs::CameraInfo& cam_info, 
   const tf::Transform& cam_transform,
   tf::Vector3& des_obj_dir)
{
  double roi_x_cen = roi.x_offset + roi.width/2;
  double roi_y_cen = roi.y_offset + roi.height/2;
  double fx = cam_info.K[0];//Distortion is not accounted for TODO
  double fy = cam_info.K[4];
  double cx = cam_info.K[2];  
  double cy = cam_info.K[5];  

  double roi3d_x = (roi_x_cen-cx)/fx;
  double roi3d_y = (roi_y_cen-cy)/fy;

  tf::Vector3 roi3d(roi3d_x, roi3d_y, 1);
  roi3d.normalize();
  tf::Transform body_rpy_tf(tf::createQuaternionFromRPY(body_rpy.x ,body_rpy.y, body_rpy.z));
  des_obj_dir = (body_rpy_tf*(cam_transform.getBasis()*roi3d)).normalize();//roi direction in global frame
  //des_body_vel = roi_dir
  //des_yaw = yaw from roi_dir
}
#endif
