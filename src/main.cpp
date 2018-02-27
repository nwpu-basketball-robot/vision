#include "findobject.h"
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
using namespace boost ;

int main(int argc , char **argv)
{
    ros::init(argc,argv,"ObjectFind") ;
        ros::NodeHandle node ;
        SimpleOpenNIViewer object_find(node) ;
        return 0 ;
}
