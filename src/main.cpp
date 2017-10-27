#include "Judge.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vision");
    ros::NodeHandle node;

    Judge judge(node);
    judge.run();

    return 0;
}