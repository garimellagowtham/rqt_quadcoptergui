/* This file provides common mapping functions and any small math functions needed for the UI and parser. This is done to keep the libraries independent of small libraries that need to be used */
#ifndef RQT_QUADCOPTER_PARSERS_COMMON_H
#define RQT_QUADCOPTER_PARSERS_COMMON_H
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <cmath>
#include <iostream>
#include <geometry_msgs/Vector3.h>
#include <iostream>
namespace rqt_quadcoptergui {
	namespace common{

		inline std::string addtimestring( std::string inmsg)
		{
# define TIME_SIZE 80
			const struct tm *tm;
			size_t len;
			time_t now;
			char *s;

			now = time ( NULL );
			tm = localtime ( &now );

			s = (char *)malloc ( TIME_SIZE * sizeof ( char ) );

			len = strftime ( s, TIME_SIZE, "%d_%B_%Y_%I_%M_%S_%p", tm );
			//char *filename = new char[strlen(inmsg)+1];
			//strcpy(filename,inmsg);
			//			memcpy(filename,inmsg,strlen(inmsg)+1);
			//char *scomb = new char[strlen(inmsg) + strlen(s)+1];
			//strcpy(scomb,filename);
			//strcat(scomb,s);
			
			return (inmsg+std::string(s));
# undef TIME_SIZE
		}

		inline double map(double inp, double inpmin, double inpmax,  double resmin, double resmax)
		{
			if(inp > inpmax)
				return resmax;
			else if(inp < inpmin)
				return resmin;
			//In all other cases
			return (resmin + ((inp-inpmin)*(resmax-resmin))/(inpmax-inpmin));
		}
		inline float map_angle(float inpangle)//Convert the angle from 0 to 2*M_PI to -M_PI to M_PI the input should be in radians
		{
			assert(inpangle <= 2*M_PI);
			assert(inpangle >= -2*M_PI);

			if(inpangle > M_PI)
				inpangle = inpangle - 2*M_PI;
			else if(inpangle < -M_PI)
				inpangle = inpangle + 2*M_PI;
			return inpangle;
		}
		//Quadcopter Data
		struct quaddata{
			double batterypercent;//Can be also battery volts in V
			std::string quadstate;
			geometry_msgs::Vector3 rpydata;//Roll pitch yaw data
			geometry_msgs::Vector3 magdata;//Magnetometer data;
			double pressure, temperature, wind_speed, wind_angle; //Pressure from barometer; temperature from temp sensor; wind speed and wind angle
			double altitude; //estimated altitude
			double *motorpwm;//motorpwm values
			geometry_msgs::Vector3 linvel;//Linear velocity of quadcopter
			geometry_msgs::Vector3 linacc;//Linear acceleration of quadcopter
			double timestamp;//timestamp from drone
			double mass, thrustbias;//Mass of the Quadcopter, the bias in thrust needed if the quadcopter does not compensate for gravity by itself.
			double thrustmin, thrustmax;//Max and min values for thrust Not the output values
			double rpbound; //Max and min values for roll and pitch
			bool armed;//Whether the quadcopter is ready to fly or not
			quaddata(): batterypercent(0)
									,quadstate("NONE")
												,pressure(0), temperature(0), wind_speed(0), wind_angle(0)
													 ,altitude(0),motorpwm(new double[4] {0,0,0,0}), timestamp(0)
														 ,mass(0),thrustbias(0)
														 ,thrustmin(0),thrustmax(0)
														 ,armed(false)
														 {
															 rpydata.x = 0; rpydata.y = 0; rpydata.z = 0;
															 magdata.x = 0; magdata.y = 0; magdata.z = 0;
															 linvel.x = 0; linvel.y = 0; linvel.z = 0;
															 linacc.x = 0; linacc.y = 0; linacc.z = 0;
														 } 
		};
	};
};
#endif//RQT_QUADCOPTER_PARSERS_COMMON_H
