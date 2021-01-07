#include "ros/ros.h"
#include "rosbot_bath/ScanSegment.h"
#include "sensor_msgs/LaserScan.h"
#include <cstdlib>
#include <vector>



ros::ServiceClient *clientPtr;

void CallbackScan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	
	std::vector<float> scan_data (720);
	rosbot_bath::ScanSegment srv;
	ros::ServiceClient client = (ros::ServiceClient)*clientPtr;

	if(!msg->ranges.empty())
	{
		for(int i = 0; i<720; i++) //(sizeof(msg->ranges))
		{	
			if(msg->ranges[i] == std::numeric_limits<double>::infinity())
			{
				scan_data[i] = 12.0;
			}
			else
			{
				scan_data[i] = (msg->ranges[i]);
			}
			
		}
		
	}



		srv.request.scan = scan_data;

		
		if (client.call(srv))
  		{

    			ROS_INFO("OutputFL: %f, %f, %f, %f, %f", float(srv.response.scan_fl[0]), float(srv.response.scan_fl[1]), float(srv.response.scan_fl[2]), (float)srv.response.scan_fl[3], (float)srv.response.scan_fl[4]);
			ROS_INFO("OutputFR: %f, %f, %f, %f, %f", float(srv.response.scan_fr[0]), float(srv.response.scan_fr[1]), float(srv.response.scan_fr[2]), (float)srv.response.scan_fr[3], (float)srv.response.scan_fr[4]);
    			ROS_INFO("OutputL: %f, %f, %f, %f, %f", float(srv.response.scan_l[0]), float(srv.response.scan_l[1]), float(srv.response.scan_l[2]), (float)srv.response.scan_l[3], (float)srv.response.scan_l[4]);
			ROS_INFO("OutputR: %f, %f, %f, %f, %f", float(srv.response.scan_r[0]), float(srv.response.scan_r[1]), float(srv.response.scan_r[2]), (float)srv.response.scan_r[3], (float)srv.response.scan_r[4]);
			
			
  		}
		else
  		{
    			ROS_ERROR("Failed to call scan segmentation service");
    			
  		}	


}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "ScanSegment_client");

  	ros::NodeHandle n;

  	ros::ServiceClient client = n.serviceClient<rosbot_bath::ScanSegment>("lidar_segment");
  	clientPtr = &client;
  	ros::Subscriber sub = n.subscribe("/scan", 10, CallbackScan);

//	ros::service::waitForService("lidar_segment", 2);

	ros::spin();

  return 0;
}
