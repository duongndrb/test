#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <turtlesim/Spawn.h>
#include <iostream>
#include <algorithm>
#include <sstream>
#include <ctime>
using namespace std;
const float tolerance = 1e-2;
float rate = 5;

geometry_msgs::Twist getMessage(double linear_x, double angular_z)
{
    geometry_msgs::Twist msg;
    msg.linear.x = linear_x;
    msg.angular.z = angular_z;
    return msg;
}

class PoseCallback {
public:
    int turtle_idx;
    ros::Publisher pub;
    ros::Subscriber sub;
    turtlesim::Pose current_pose;

    void callback(const turtlesim::Pose::ConstPtr& msg)
    {
        cout << "turtle " << turtle_idx+1 << " " << msg->x << " " << msg->y << endl;
        current_pose = *msg;
    }
};

double distanceLinear(turtlesim::Pose pose, double goal_X, double goal_Y)
{
    double distance = sqrt (pow(goal_X-pose.x, 2)+pow(goal_Y-pose.y, 2));
    if(distance < tolerance)
    {
        distance = 0;
    }
    return distance;
}
double distanceAngular(turtlesim::Pose pose, double goal_X, double goal_Y)
{
    double angular;
    if(distanceLinear(pose,goal_X,goal_Y)<tolerance)
    {
        angular = 0;
    }
    else
    {
        angular = asin ((cos(pose.theta)*(goal_Y- pose.y)-sin(pose.theta)*(goal_X-pose.x)) / distanceLinear(pose, goal_X, goal_Y));
    }
    return angular;
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "myturtle_control");
    ros::NodeHandle d;
    int n_turtle = atoi(argv[1]);
    PoseCallback array[n_turtle]; 
    cout << "n_turtle = " << n_turtle << endl;

    for (int i = 0; i < n_turtle; i++) 
    { 
        if(i!=0)
        {
        ros::service::waitForService("spawn");    
        ros::ServiceClient spawnTurtle = d.serviceClient<turtlesim::Spawn>("spawn"); 
        turtlesim::Spawn turtle;
        turtle.request.x = rand()%12;
        turtle.request.y = rand()%12;
        spawnTurtle.call(turtle);
        }

        stringstream s;
        s << "turtle" << i+1;
        string name = s.str();

        array[i].pub = d.advertise<geometry_msgs::Twist>(name + "/cmd_vel", 1000);
        array[i].turtle_idx = i;
        array[i].sub = d.subscribe(name+"/pose", 1000, &PoseCallback::callback, &array[i]);
        cout << "subcribe turtle " << i << " to " << name << "/pose" << endl;
    }
    ros::Rate loopRate(rate);

    for (int i = 2; i <= argc-1; i+=2) 
    {
        double x0 = atof(argv[i]), y0 = atof(argv[i+1]);
          while (ros::ok()) 
            {   
        for (int idx = 0; idx < n_turtle; idx++)
        {
            if(distanceLinear(array[idx].current_pose, x0, y0) < tolerance)
            {
		array[idx].pub.publish(getMessage(0, 0));
		break;
		}
            geometry_msgs::Twist msg = getMessage(
                   min(1*distanceLinear(array[idx].current_pose,x0,y0), 4.0),
                    4*distanceAngular(array[idx].current_pose,x0,y0)
                );

            array[idx].pub.publish(msg);
		    
            }
            loopRate.sleep();
            ros::spinOnce();
        }
         
    }
    return 0;
}
