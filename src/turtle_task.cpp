#include <cppad/ipopt/solve.hpp>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/String.h"
#include <sstream>
#include "turtlesim/Pose.h"
#include "include/mpc_unicycle.h"

float vel_curr;
float ang_vel_curr;
float dist = 20;

// Other constants of motion
int marker = 0;

void obs_position_callback(const turtlesim::Pose& msg)
{
    // called by subscriber of turtle1's pose.
    // to update the obstacle position
    ros::param::set("/unicycle/obs_x", msg.x);
    ros::param::set("/unicycle/obs_y", msg.y);
}
void curr_position_callback(const turtlesim::Pose& msg)
{
    // called by subscriber of turtle2's pose.
    // to update the current position and velocity.
    ros::param::set("/unicycle/x_curr", msg.x);
    ros::param::set("/unicycle/y_curr", msg.y);
    ros::param::set("/unicycle/theta_curr", msg.theta);
    vel_curr = msg.linear_velocity;
    ang_vel_curr = msg.angular_velocity;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_task");
    ros::NodeHandle n;
    ros::Subscriber turt1_pose_sub = n.subscribe("/turtle1/pose", 10, obs_position_callback);
    ros::Subscriber turt2_pose_sub = n.subscribe("/turtle2/pose", 10, curr_position_callback);
    ros::Publisher turt2_vel_pub = n.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel",10);

    int N;
    double T,x_curr,y_curr,theta_curr,obs_x,obs_y,target_x,target_y,target_theta;
    ros::param::get("/unicycle/N", N);
    ros::param::get("/unicycle/T", T);
    ros::param::get("/unicycle/obs_x", obs_x);
    ros::param::get("/unicycle/obs_y", obs_y);
    ros::param::get("/unicycle/x_curr", x_curr);
    ros::param::get("/unicycle/y_curr", y_curr);
    ros::param::get("/unicycle/theta_curr", theta_curr);
    ros::param::get("/unicycle/target_x", target_x);
    ros::param::get("/unicycle/target_y", target_y);
    ros::param::get("/unicycle/target_theta", target_theta);

    typedef CPPAD_TESTVECTOR( double ) Dvector;
    size_t nx = 2*N + 10;
    Dvector X_initial(nx);
    Dvector X_lower(nx);
    Dvector X_upper(nx);

    size_t ng = 2*N+14;
    Dvector gl(ng), gu(ng);

    geometry_msgs::Twist velocity;
    // object is controlled only by giving a linear velocity along the direction it faces 
    // and the angular velocity along the perpendicular to the plane.
    velocity.angular.x = 0;
    velocity.angular.y = 0;
    velocity.linear.y = 0;
    velocity.linear.z = 0;
    

    // object that computes objective and constraints
    FG_eval fg_eval;
    
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // Disables printing IPOPT creator banner
    options += "String  sb          yes\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    // options += "Sparse  true        reverse\n";

    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          0.5\n";

    // ros::Rate loop_rate(T);
    int count = 0;
    // place to return solution
    // ROS_INFO("dist: %ld", dist);
    int done;
    ros::param::get("/unicycle/done", done);
    CppAD::ipopt::solve_result<Dvector> solution;

    // run till the distance between the target and the turtlesim is less than 0.01. Also to stop running if the rosnode is shutdown
    while ((ros::ok()) && (done == 0))
    {
        for (int i = 0; i < N; i++)
        {
            X_lower[2*i] = -1.5;
            X_lower[2*i+1] = -1.0;

            X_upper[2*i] = 1.5;
            X_upper[2*i+1] = 1.0;
        }

        // intialising
        X_initial[2*N] = T;
        X_initial[2*N+1] = target_x;
        X_initial[2*N+2] = target_y;
        X_initial[2*N+3] = target_theta;
        X_initial[2*N+4] = obs_x;
        X_initial[2*N+5] = obs_y;
        X_initial[2*N+6] = x_curr;
        X_initial[2*N+7] = y_curr;
        X_initial[2*N+8] = theta_curr;
        X_initial[2*N+9] = N;

        // upper bounds
        X_upper[2*N] = T;
        X_upper[2*N+1] = target_x;
        X_upper[2*N+2] = target_y;
        X_upper[2*N+3] = target_theta;
        X_upper[2*N+4] = obs_x;
        X_upper[2*N+5] = obs_y;
        X_upper[2*N+6] = x_curr;
        X_upper[2*N+7] = y_curr;
        X_upper[2*N+8] = theta_curr;
        X_upper[2*N+9] = N;

        // lower bounds
        X_lower[2*N] = T;
        X_lower[2*N+1] = target_x;
        X_lower[2*N+2] = target_y;
        X_lower[2*N+3] = target_theta;
        X_lower[2*N+4] = obs_x;
        X_lower[2*N+5] = obs_y;
        X_lower[2*N+6] = x_curr;
        X_lower[2*N+7] = y_curr;
        X_lower[2*N+8] = theta_curr;
        X_lower[2*N+9] = N;
        

        gl[0] = 4.0;
        gu[0] = 1e19;
        for (int i = 1; i < 2*N+1; i++)
        {
            gl[i] = 0;
            gu[i] = 0.5;
        }
        gl[2*N+1] = 0; gu[2*N+1] = 10;
        gl[2*N+2] = 0; gu[2*N+2] = 10;
        gl[2*N+3] = -3.14; gu[2*N+3] = 3.14;

        gl[2*N+4] = T;
        gl[2*N+5] = target_x;
        gl[2*N+6] = target_y;
        gl[2*N+7] = target_theta;
        gl[2*N+8] = obs_x;
        gl[2*N+9] = obs_y;
        gl[2*N+10] = x_curr;
        gl[2*N+11] = y_curr;
        gl[2*N+12] = theta_curr;
        gl[2*N+13] = N;

        gu[2*N+4] = T;
        gu[2*N+5] = target_x;
        gu[2*N+6] = target_y;
        gu[2*N+7] = target_theta;
        gu[2*N+8] = obs_x;
        gu[2*N+9] = obs_y;
        gu[2*N+10] = x_curr;
        gu[2*N+11] = y_curr;
        gu[2*N+12] = theta_curr;
        gu[2*N+13] = N;
        
        if(marker == 0)
        {
            for (int i = 0; i < N; i++)
            {
                X_initial[2*i] = vel_curr;
                X_initial[2*i + 1] = ang_vel_curr;
            }
            marker = 1;
        }
        else
        {
            for (int i = 0; i < N; i++)
            {
                X_initial[2*i] = solution.x[2*i];
                X_initial[2*i + 1] = solution.x[2*i + 1];
            }
        }
        // initial values have been declared.

        CppAD::ipopt::solve<Dvector, FG_eval>(
            options, X_initial, X_lower, X_upper, gl, gu, fg_eval, solution
        );
        // publish
        // ROS_INFO("obj_val: %ld", solution.obj_value);
        // std::cout << "obj_val: " << solution.obj_value << "\n";
        std::cout << "dist " << dist << "\n";
        velocity.linear.x = solution.x[0];
        velocity.angular.z = solution.x[1];
        // std::cout << velocity << "\n";

        // to ensure that the same values of velocity and anglular velocity are published for the 
        // sampling time period after which the values will be calculated again
        double begin = ros::Time::now().toSec();
        while (ros::Time::now().toSec() < begin+T)
        {
            turt2_vel_pub.publish(velocity);
            // std::cout << count << "\n";
        }
        ros::spinOnce();
        // count += 1;
        // loop_rate.sleep();

        // ros::Duration(T).sleep();
        ros::param::get("/unicycle/obs_x", obs_x);
        ros::param::get("/unicycle/obs_y", obs_y);

        ros::param::get("/unicycle/x_curr", x_curr);
        ros::param::get("/unicycle/y_curr", y_curr);
        ros::param::get("/unicycle/theta_curr", theta_curr);
        
        ros::param::get("/unicycle/target_x", target_x);
        ros::param::get("/unicycle/target_y", target_y);
        ros::param::get("/unicycle/target_theta", target_theta);

        ros::param::get("/unicycle/done", done);
        
        // updating the distance between the obstacle and the turtlesim
        dist = pow(pow(x_curr-target_x,2) + pow(y_curr-target_y,2) + pow(theta_curr-target_theta,2),0.5);
        if(dist < 0.4)
        {
            ros::param::set("/target_achieved", 1);
        }
    }
    // ros::spin();
    ros::param::set("/unicycle/done", 1);
    ros::shutdown();
  return 0;
}