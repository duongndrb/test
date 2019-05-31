
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
float rate = 100;

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
double Factor(turtlesim::Pose pose, double goal_X, double goal_Y)
{
	double factor;
	double x = goal_X - 5.54444;
            double y = goal_Y - 5.54444;
            
            if( x>0 && y>0 )
            {
                factor = 1;
            }
            if( x>0 && y<0 )
            {
                factor = 1;
            }
            if( x<0 && y<0 )
            {
                factor = -1;
            }
            if( x<0 && y>0)
            {
                factor = -1;
            }
	return factor;
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

    double goal[n_turtle][2];
    for(int idx = 0; idx < n_turtle; idx++)
    {
        goal[idx][0] = array[idx].current_pose.x;
        goal[idx][1] = array[idx].current_pose.y;
    }
    int factor = 2;

    ros::Rate loopRate(rate);
    while(ros::ok())
    {
        for(int idx = 0; idx < n_turtle ; idx++)
        {
            if(distanceLinear(array[idx].current_pose, goal[idx][0], goal[idx][1]) < tolerance)
            {
                array[idx].pub.publish(getMessage(0, 0));
                if(factor < argc - 1)
                {
                    goal[idx][0] = atof(argv[factor]);
                    goal[idx][1] = atof(argv[factor + 1]);
                    factor = factor + 2;
                    cout << "turtle" << idx + 1 << ": " << goal[idx][0] << " " << goal[idx][1] << endl;
                }
                
            }

            geometry_msgs::Twist msg = getMessage(Factor(array[idx].current_pose, goal[idx][0], goal[idx][1])*
                min(4*distanceLinear(array[idx].current_pose, goal[idx][0], goal[idx][1]),  16.0),
                Factor(array[idx].current_pose, goal[idx][0], goal[idx][1])*16*distanceAngular(array[idx].current_pose, goal[idx][0], goal[idx][1])
            );
            array[idx].pub.publish(msg);
        }
        
        loopRate.sleep();
        ros::spinOnce();
    }
    return 0;
}


