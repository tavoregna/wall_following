#include "node_wallFollowing.h"
#include <math.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <thread>
#include <atomic>

#define PI 3.141592
#define LOCALIZATION_MESSAGE "amcl_pose" //"poseupdate" "amcl_pose"

NodeWallFollowing::NodeWallFollowing(ros::Publisher pub, double wallDist, double maxSp, int dir, double pr, double di, double an,ros::NodeHandle* node)
{
    wallDistance = wallDist;
    maxSpeed = maxSp;
    direction = dir;
    P = pr;
    D = di;
    angleCoef = an;
    e = 0;
    angleMin = 0;  //angle, at which was measured the shortest distance
    pubMessage = pub;  
    n=node;
    
    n->getParam("wallFollowing/direction",direction);
    n->getParam("wallFollowing/linear_velocity",maxSpeed);
    n->getParam("wallFollowing/distance",wallDistance);
    n->getParam("wallFollowing/endx",endx);
    n->getParam("wallFollowing/endy",endy);
    n->getParam("wallFollowing/avanti",avanti);
    n->getParam("wallFollowing/forwardwall",forwardWall);
}

NodeWallFollowing::~NodeWallFollowing()
{
}

//Publisher
void NodeWallFollowing::publishMessage()
{     
    //preparing message
    geometry_msgs::Twist msg;

    int val=0;
    n->getParam("wallFollowing/avanti",val);
    if(val==0 || (forwardWall>0 && distFront<=forwardWall))
    {
        msg.linear.x=0.0;
        msg.angular.z=0.0;
        pubMessage.publish(msg);
        n->setParam("wallFollowing/avanti",0);
        return;
    }
        
        
    msg.angular.z = direction*(P*e + D*diffE) + angleCoef * (angleMin - PI*direction/2);    //PD controller

    if (distFront < wallDistance){
    msg.linear.x = 0;
    }
    else if (distFront < wallDistance * 2){
    msg.linear.x = 0.5*maxSpeed; //0.5
    }
    else if (fabs(angleMin)>1.75){
    msg.linear.x = 0.4*maxSpeed; //0.4
    }
    else {
    msg.linear.x = maxSpeed;
    }

    //publishing message
    pubMessage.publish(msg);
}

//Subscriber
void NodeWallFollowing::messageCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    int attivo=0;
    n->getParam("programma/following",attivo);
    if(attivo==0)
        return;
    
    n->getParam("wallFollowing/direction",direction);
    n->getParam("wallFollowing/linear_velocity",maxSpeed);
    n->getParam("wallFollowing/distance",wallDistance);
    n->getParam("wallFollowing/forwardwall",forwardWall);
    
    int size = msg->ranges.size();

    //Variables whith index of highest and lowest value in array.
    int minIndex,maxIndex;
    
    if(direction==-1) //direzione destra
    {
        minIndex=size/4;
        maxIndex=size/2;
    }
    else  //direzione sinistra
    {
        minIndex=size/2;
        maxIndex=size*3/4;
    }

    //This cycle goes through array and finds minimum
    for(int i = minIndex; i < maxIndex; i++)
    {
    if(msg->ranges[i]<0.18)
        continue;
    if (msg->ranges[i] < msg->ranges[minIndex] && msg->ranges[i] > 0.0){
      minIndex = i;
    }
    }

    //Calculation of angles from indexes and storing data to class variables.
    angleMin = (minIndex-size/2)*msg->angle_increment;
    double distMin;
    distMin = msg->ranges[minIndex];
    distFront = msg->ranges[size/2];
    diffE = (distMin - wallDistance) - e;
    e = distMin - wallDistance;

    //Invoking method for publishing message
    publishMessage();
}
