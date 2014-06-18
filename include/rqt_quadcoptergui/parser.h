/* This program is for parsing various quadcopter data and storing them as internal variables. This is a parent library and 
provides virtual functions for the subclasses to implement. 
	The data that needs to be stored (copied from ardrone_autonomy should be similar or more can be added based on generality). 
		* batteryPercent: The remaining charge of the drone's battery (%)
		* state: The Drone's current state as a string form
		* rotX: Left/right tilt in radians (rotation about the X axis)
		* rotY: Forward/backward tilt in radians (rotation about the Y axis)
		* rotZ: Orientation in radians (rotation about the Z axis)
		* magX, magY, magZ: Magnetometer readings (Unit undecided)
		* pressure: Pressure sensed by Drone's barometer (Unit undecided)
		* temp : Temperature sensed by Drone's sensor(Unit undecided)
		* wind_speed: Estimated wind speed (Unit undecided)
		* wind_angle: Estimated wind angle (Unit undecided)
		* altd: Estimated altitude (m)
		* motor1..4: Motor PWM values (Raw units provided by driver) 
		* vx, vy, vz: Linear velocity (m/s)
		* ax, ay, az: Linear acceleration (g)
		* tm: Timestamp of the data returned by the Drone returned as number of micro-seconds passed since Drone's boot-up.

		The parser needs to convert all the data into NWU format for the user interface and controllers to make sense of it. It should also publish the Imu data in the sensor_msgs/Imu format if it is not already done by the driver. This will be useful to maintain standard format for controllers to use.
		The rpyt commands needs to be given in the form of a quaternion with x,y,z representing roll pitch yaw and w representing thrust. the max and min values for rpy are -Pi to Pi. The max and min values for thrust are set in the parser. This will be later set as parameters or inputs from UI
		Possible States for quadstate which can be used in the UI:
			* 0 -> Unknown
			* 1 -> Init
			* 2 -> Landed
			* 3 -> Flying
			* 4 -> Taking off
//Controller should ensure that the commanded angles are between -pi to pi 
*/
#ifndef PARSER_H
#define PARSER_H
#include <fstream>
#include <iostream>
#include "ros/ros.h"
#include "ros/time.h"
#include <geometry_msgs/TransformStamped.h>
//#include <tf/transform_broadcaster.h>//Looks like problems with boost signals need to additional work to figure this out 
#include <geometry_msgs/Vector3.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <sensor_msgs/Imu.h>

#include <rqt_quadcoptergui/common.h>
#include <boost/thread/mutex.hpp>

//#include "controllers/ctrl_command.h"//for roll pitch, yaw and thrust
//#include "controllers/PIDGains.h"
//#include <DspFilters/Butterworth.h>
using namespace std;
namespace rqt_quadcoptergui{
class Parser
{
	public: //Virtual Functions for commanding the drone. These should be implemented by the subclasses based on the quadcopter we are working with.
		virtual bool takeoff()=0;//Take off the quadcopter
		virtual bool land()=0;//Land the quadcopter
		virtual bool disarm()=0;//Disarm the quadcopter
		virtual bool reset()=0;//Dont know what to do here specifically
		virtual bool calibrateimubias()=0;//Calibrate imu of the quadcopter based on sample data
		virtual bool cmdrpythrust(geometry_msgs::Quaternion &rpytmsg, bool sendyaw = false)=0;//Command the euler angles of the quad and the thrust. the msg format is (x,y,z,w) -> (roll, pitch, yaw, thrust). The thrust value will be adjusted based on whethere there is thrust bias or not.
		virtual void setarmpwm(double *armpwm)=0;//Command Arm pwm
		virtual void foldarm()=0;//Command Arm pwm
		//virtual void setarmpwm(std::vector<float> &armpwm)=0;//Command Arm pwm
		//virtual void setarmangles(std::vector<float> &armangles)=0;//Command Arm angles
		virtual void setarmangles(double *armangles)=0;//Command Arm angles This is a bad way of doing it TODO change marin code and my code to make this consistent
		virtual void estimatethrustbias()=0;
/*		void setthrustbias(float thrustbias_)
		{
			data.thrustbias = thrustbias_;
		}
		*/

		//PluginLib initialization function
		virtual void initialize(ros::NodeHandle &nh_)=0;
		virtual void getquaddata(common::quaddata &d1)=0;
		virtual ~Parser()
		{
		}
		virtual void setaltitude(double altitude_)=0;//Set the altitude value in the data
		virtual void setlogdir(string logdir)=0;//Set whether to log data or not
		virtual void controllog(bool logswitch)=0;
		//const ros::NodeHandle &nh;
	protected: 
		Parser(){};
	private:
			common::quaddata data;
};
};
#endif //Parser_H

