#include <morus_control/example_tf_listener.h>
using namespace std;
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/Twist.h>


class VelocityTf
{
public:
    VelocityTf(ros::NodeHandle* nodehandle){
        nh_ = *nodehandle;
        demoTfListener = DemoTfListener(nodehandle);
        my_subscriber = nh_.subscribe("/gazebo/link_states", 1, &VelocityTf::myCallbackVel, this);
        my_publisher = nh_.advertise<geometry_msgs::Twist>("velocity_in_base_link", 1);
    }
     
    void myCallbackVel(const gazebo_msgs::LinkStates& link_state_msgs) {
        velocity_world = link_state_msgs.twist[1];
        demoTfListener.tfListener_->lookupTransform("world", "base", ros::Time(0), stfWorldToBaseLink);//base
        tfWorldToBaseLink = demoTfListener.get_tf_from_stamped_tf(stfWorldToBaseLink);
        
        tf::Vector3 tfVec;
        tf::Matrix3x3 tfR;
        tf::Quaternion quat;
        tfVec = tfWorldToBaseLink.getOrigin();
        tfR = tfWorldToBaseLink.getBasis();
        
        Eigen::MatrixXd Rot_= Eigen::MatrixXd::Zero(3,3);
        Rot_(0,0) = tfR[0][0];
        Rot_(0,1) = tfR[0][1];
        Rot_(0,2) = tfR[0][2];
        Rot_(1,0) = tfR[1][0];
        Rot_(1,1) = tfR[1][1];
        Rot_(1,2) = tfR[1][2];
        Rot_(2,0) = tfR[2][0];
        Rot_(2,1) = tfR[2][1];
        Rot_(2,2) = tfR[2][2];
        
        Eigen::MatrixXd skew_p= Eigen::MatrixXd::Zero(3,3);
        skew_p(0,0) = 0.0;
        skew_p(0,1) = -1*tfVec.getZ();
        skew_p(0,2) = tfVec.getY();
        skew_p(1,0) = tfVec.getZ();
        skew_p(1,1) = 0.0;
        skew_p(1,2) = -1*tfVec.getX();
        skew_p(2,0) = -1*tfVec.getY();
        skew_p(2,1) = tfVec.getX();
        skew_p(2,2) = 0.0;
        
        Eigen::MatrixXd omega= Eigen::MatrixXd::Zero(3,1);
        omega(0,0) = velocity_world.angular.x;
        omega(1,0) = velocity_world.angular.y;
        omega(2,0) = velocity_world.angular.z;
        
        Eigen::MatrixXd vel= Eigen::MatrixXd::Zero(3,1);
        vel(0,0) = velocity_world.linear.x;
        vel(1,0) = velocity_world.linear.y;
        vel(2,0) = velocity_world.linear.z;
        
        Eigen::MatrixXd vel_base_link= Eigen::MatrixXd::Zero(3,1);
        
        vel_base_link= Rot_.transpose()*vel ;//+ skew_p*Rot_*omega;
        velocity_world.linear.x = vel_base_link(0,0);
        velocity_world.linear.y= vel_base_link(1,0);
        velocity_world.linear.z= vel_base_link(2,0);
        
        my_publisher.publish(velocity_world);
        cout << sqrt(vel_base_link(0,0)*vel_base_link(0,0) + vel_base_link(1,0)*vel_base_link(1,0) +vel_base_link(2,0)*vel_base_link(2,0) ) << endl;
        cout << " " << endl;
        
    }

  
private:
    ros::NodeHandle nh_;
    DemoTfListener demoTfListener;
    ros::Subscriber my_subscriber;
    ros::Publisher my_publisher;
    geometry_msgs::Twist velocity_world;
    tf::StampedTransform stfWorldToBaseLink;
    tf::Transform tfWorldToBaseLink;
             
}; 



int main(int argc, char** argv) {

    ros::init(argc, argv, "velocity_transform"); 
    ros::NodeHandle nh; 
    VelocityTf velocityTf(&nh);
    
    ros::spin();

    return 0;
}
