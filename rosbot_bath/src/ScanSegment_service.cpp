/** LiDAR Segementation Service (C++)
*
*	LiDAR Segment Generator for ROSbot Competition
*
*	Written by: Tanmay Burde
**/

#include "ros/ros.h"
#include "rosbot_bath/ScanSegment.h"
#include <vector>

#define tol 0.2
#define min_obs 1.5


class Segmenter
{
	private:
	static int start_ang;// = 0;
	static int end_angle;// = 0;
	static float start_distance;// = 12;
	static float end_distance;// = 12;
	static float min_distance;// = 12;
	static bool start;// = true;
	static float last_distance;	
	static unsigned int i;// = 0;
	static bool seg_cmp;

	void reset(void) //reset internal variables
	{
		start_ang = 0;
		end_angle = 0;
		start_distance = 12;
		end_distance = 12;
		min_distance = 12;
		last_distance = 12;
		start = true;
		seg_cmp = false;
		i = 0;
	}


	public:
	bool segmentScan(rosbot_bath::ScanSegment::Request  &req,
					rosbot_bath::ScanSegment::Response &res) 
	{
	//ROS_INFO("Request Received");
	res.scan_fl.resize(5);
	res.scan_fr.resize(5);
	res.scan_l.resize(5);
	res.scan_r.resize(5);
	reset();
	  while ( i < 90)
	  {
		//ROS_INFO("Dist %f,Last %f, Pos %f", float(req.scan[i]), last_distance, float(i));
		if( (fabs(float(req.scan[i])-last_distance) < float(tol)  && !seg_cmp )|| start)
		{
			
			if(start && (float(req.scan[i]) < min_obs))
			{
				res.scan_fl[2] = float(i)/2.0;
				res.scan_fl[0] = float(req.scan[i]);
				start = false;

			}
			else if (start)
			{
		
			}
			else
			{
				res.scan_fl[1] = float(req.scan[i]);
				res.scan_fl[3] = float(i)/2.0;
			}
			
			if(float(req.scan[i]) < min_distance)
			{
				min_distance = float(req.scan[i]);
				res.scan_fl[4] = float(req.scan[i]);
			}

			last_distance = float(req.scan[i]);

		}
		else if((fabs(float(req.scan[i+1])-last_distance) < float(tol))||(fabs(float(req.scan[i-1])-last_distance) < float(tol)) && !seg_cmp )
		{
			res.scan_fl[1] = last_distance;
			res.scan_fl[3] = float(i)/2.0;
			
			if(float(req.scan[i]) < min_distance)
			{
				min_distance = float(req.scan[i]);
				res.scan_fl[4] = float(req.scan[i]);
			}
		}
		else if (fabs(float(req.scan[i])-last_distance) > float(tol) )
		{
			seg_cmp = true;
		}
		else if(!start && seg_cmp)
		{
			break;
		}

			i++;

	  }


	  reset();
	 
 	while ( i < 90)
	  {
		//ROS_INFO("Dist %f,Last %f, Pos %f", float(req.scan[i]), last_distance, float(i));
		if( (fabs(float(req.scan[720-i-1])-last_distance) < float(tol)  && !seg_cmp )|| start)
		{
			
			if(start && (float(req.scan[720-i-1]) < min_obs))
			{
				res.scan_fr[2] = float(720-i)/2.0;
				res.scan_fr[0] = float(req.scan[720-i-1]);
				start = false;

			}
			else if (start)
			{
		
			}
			else
			{
				res.scan_fr[1] = float(req.scan[720-i-1]);
				res.scan_fr[3] = float(720-i)/2.0;
			}
			
			if(float(req.scan[720-i-1]) < min_distance)
			{
				min_distance = float(req.scan[720-i-1]);
				res.scan_fr[4] = float(req.scan[720-i-1]);
			}

			last_distance = float(req.scan[720-i-1]);

		}
		else if((fabs(float(req.scan[720-(i+1)-1])-last_distance) < float(tol))||(fabs(float(req.scan[720-(i-1)-1])-last_distance) < float(tol)) && !seg_cmp )
		{
			res.scan_fr[1] = last_distance;
			res.scan_fr[3] = float(720-i)/2.0;
			
			if(float(req.scan[720-i-1]) < min_distance)
			{
				min_distance = float(req.scan[720-i-1]);
				res.scan_fr[4] = float(req.scan[720-i-1]);
			}
		}
		else if (fabs(float(req.scan[720-i-1])-last_distance) > float(tol) )
		{
			seg_cmp = true;
		}
		else if(!start && seg_cmp)
		{
			break;
		}

			i++;

	  }


	  reset();

	  while ( i < 180)
	  {
		//ROS_INFO("Dist %f,Last %f, Pos %f", float(req.scan[i]), last_distance, float(i));
		if( (fabs(float(req.scan[i+90])-last_distance) < float(tol)  && !seg_cmp )|| start)
		{
			
			if(start && (float(req.scan[i+90]) < min_obs))
			{
				res.scan_l[2] = float(i+90)/2.0;
				res.scan_l[0] = float(req.scan[i+90]);
				start = false;

			}
			else if (start)
			{
		
			}
			else
			{
				res.scan_l[1] = float(req.scan[i+90]);
				res.scan_l[3] = float(i+90)/2.0;
			}
			
			if(float(req.scan[i+90]) < min_distance)
			{
				min_distance = float(req.scan[i+90]);
				res.scan_l[4] = float(req.scan[i+90]);
			}

			last_distance = float(req.scan[i+90]);

		}
		else if((fabs(float(req.scan[i+1+90])-last_distance) < float(tol))||(fabs(float(req.scan[i-1+90])-last_distance) < float(tol)) && !seg_cmp )
		{
			res.scan_l[1] = last_distance;
			res.scan_l[3] = float(i+90)/2.0;
			
			if(float(req.scan[i+90]) < min_distance)
			{
				min_distance = float(req.scan[i+90]);
				res.scan_l[4] = float(req.scan[i+90]);
			}
		}
		else if (fabs(float(req.scan[i+90])-last_distance) > float(tol) )
		{
			seg_cmp = true;
		}
		else if(!start && seg_cmp)
		{
			break;
		}

			i++;

	  }


	  reset();
	 
 	while ( i < 180)
	  {
		//ROS_INFO("Dist %f,Last %f, Pos %f", float(req.scan[i]), last_distance, float(i));
		if( (fabs(float(req.scan[630-i-1])-last_distance) < float(tol)  && !seg_cmp )|| start)
		{
			
			if(start && (float(req.scan[630-i-1]) < min_obs))
			{
				res.scan_r[2] = float(630-i)/2.0;
				res.scan_r[0] = float(req.scan[630-i-1]);
				start = false;

			}
			else if (start)
			{
		
			}
			else
			{
				res.scan_r[1] = float(req.scan[630-i-1]);
				res.scan_r[3] = float(630-i)/2.0;
			}
			
			if(float(req.scan[630-i-1]) < min_distance)
			{
				min_distance = float(req.scan[630-i-1]);
				res.scan_r[4] = float(req.scan[630-i-1]);
			}

			last_distance = float(req.scan[630-i-1]);

		}
		else if((fabs(float(req.scan[630-(i+1)-1])-last_distance) < float(tol))||(fabs(float(req.scan[630-(i-1)-1])-last_distance) < float(tol)) && !seg_cmp )
		{
			res.scan_r[1] = last_distance;
			res.scan_r[3] = float(630-i)/2.0;
			
			if(float(req.scan[630-i-1]) < min_distance)
			{
				min_distance = float(req.scan[630-i-1]);
				res.scan_r[4] = float(req.scan[630-i-1]);
			}
		}
		else if (fabs(float(req.scan[630-i-1])-last_distance) > float(tol) )
		{
			seg_cmp = true;
		}
		else if(!start && seg_cmp)
		{
			break;
		}

			i++;

	  }

	  reset();
	  
	  //ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
	  //ROS_INFO("sending back response: [%ld]", (long int)res.sum);
	  //ROS_INFO("Sending Response");
	  	return true;
	}
};

//Initialise class variables
unsigned int Segmenter::i = 0;
int Segmenter::start_ang = 0;
int Segmenter::end_angle = 0;
float Segmenter::start_distance = 12;
float Segmenter::end_distance = 12;
float Segmenter::min_distance = 12;
float Segmenter::last_distance = 12;
bool Segmenter::start = true;
bool Segmenter::seg_cmp = false;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ScanSegment_server");
  ros::NodeHandle n;
  Segmenter lidar;
  ros::ServiceServer service = n.advertiseService("lidar_segment", &Segmenter::segmentScan, &lidar);
  ROS_INFO("Ready to segment lidar scan");
  ros::spin();

  return 0;
}
