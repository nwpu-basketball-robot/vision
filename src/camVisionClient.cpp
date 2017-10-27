#include "ros/ros.h"
#include "basketball_msgs/cylinderElem.h"
#include "basketball_msgs/camVisionDate.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_vision_client");

    ros::NodeHandle    node;
    ros::ServiceClient client = node.serviceClient< basketball_msgs::camVisionDate >("camVisionDate");

    basketball_msgs::camVisionDate srv;

    ros::Rate rate(10);
    while(  ros::ok() )
    {
        if( client.call(srv) )
        {
            double timeLoad = static_cast<double>(srv.response.timeLoad);
            printf("\n\nload time %lf\n", timeLoad);
            printf("time now %lf\n", ros::Time::now().toSec());

            std::vector< basketball_msgs::cylinderElem >     cylinders   = srv.response.cylinders;
            
            printf("Cylinders\n");
            for(int i = 0; i < cylinders.size(); i++)
            {
                printf("[Cylinder] theta %lf\n", static_cast<double>(cylinders[i].theta));
                printf("[Cylinder] distance %lf\n", static_cast<double>(cylinders[i].distance));
            }
        }
        else
        {
            printf("ServiceClient call back false\n\n");
        }

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}