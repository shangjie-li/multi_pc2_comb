#include "multi_pc2_comb_core.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "multi_pc2_comb");

    ros::NodeHandle nh("~");

    PclTestCore core(nh);
    return 0;
}

