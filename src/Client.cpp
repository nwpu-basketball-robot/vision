#include "ros/ros.h"
#include "basketball_msgs/ballElem.h"
#include "basketball_msgs/cylinderElem.h"
#include "basketball_msgs/visionDate.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_client");

    ros::NodeHandle    node;
    ros::ServiceClient client = node.serviceClient< basketball_msgs::visionDate>("visionDate");

    basketball_msgs::visionDate srv;
    srv.request.model = 1;

    ros::Rate rate(10);
    while(  ros::ok() )
    {
        if( client.call(srv) )
        {
            double timeLoad = static_cast<double>(srv.response.timeLoad);
            printf("\n\nload time %lf\n", timeLoad);
            printf("time now %lf\n", ros::Time::now().toSec());

            std::vector< basketball_msgs::ballElem >         balls       = srv.response.balls;
            std::vector< basketball_msgs::cylinderElem >     cylinders   = srv.response.cylinders;

            printf("Balls\n");
            for(int i = 0; i < balls.size(); i++)
            {
                printf("[Ball] type %d\n", static_cast<int>(balls[i].type));
                printf("[Ball] theta %lf\n", static_cast<double>(balls[i].theta));
                printf("[Ball] distance %lf\n", static_cast<double>(balls[i].distance));
            }

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