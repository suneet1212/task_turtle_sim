#include "ros/ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "target_bicycle");
    ros::NodeHandle n;
    int done;
    ros::param::get("/bicycle/done",done); // to check whether the launch file is still running
    int target_achieved;
    ros::param::get("/target_achieved",target_achieved);// to check whether the current path is completed or not 
    while ((ros::ok()) && (done == 0))
    {
        while ((target_achieved == 0) && (ros::ok()))
        {
            // updating values and waiting for the target to be achieved
            ros::param::get("/target_achieved",target_achieved);
            ros::param::get("/bicycle/done",done);
        }
        if(done == 0)
        {
            std::cout << "Do you want to enter new target? Enter \"y\" or \"n\": ";
            char input;
            std::cin >> input;
            if(input == 'n')
            {
                ros::param::set("/bicycle/done",1);
                ros::shutdown();
            }
            else if(input == 'y')
            {
                std::cout << "Enter 3 target values corresponding to x y and theta: ";
                double x,y,theta;
                std::cin >> x >> y >> theta;
                if((x > 10) || (x < 0) || (y > 10) || (y < 0) || (theta > 3.14) || (theta < -3.14)) // check if input is in vakid region
                {
                    std::cout << "ERROR: Values out of range.\n";
                    continue;
                }
                ros::param::set("/bicycle/target_x",x);
                ros::param::set("/bicycle/target_y",y);
                ros::param::set("/bicycle/target_theta",theta);
                ros::param::set("/target_achieved",0);
            }
        }
    }
    ros::shutdown();
    // ros::spin();
    return 0;
}