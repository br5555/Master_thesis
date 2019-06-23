//example_tf_listener.cpp:
//wsn, March 2016
//illustrative node to show use of tf listener, with reference to the simple mobile-robot model
// specifically, frames: odom, base_frame, link1 and link2

// this header incorporates all the necessary #include files and defines the class "DemoTfListener"
#include <morus_control/example_tf_listener.h>
using namespace std;

//main pgm to illustrate transform operations

int main(int argc, char** argv) {
    // ROS set-ups:
    ros::init(argc, argv, "demoTfListener"); //node name
    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    ROS_INFO("main: instantiating an object of type DemoTfListener");
    DemoTfListener demoTfListener(&nh); //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use

    tf::StampedTransform stfBaseToLink2, stfBaseToLink1, stfLink1ToLink2;
    tf::StampedTransform testStfBaseToLink2;

    tf::Transform tfBaseToLink1, tfLink1ToLink2, tfBaseToLink2, altTfBaseToLink2;

    demoTfListener.tfListener_->lookupTransform("world", "base", ros::Time(0), stfBaseToLink1);
    cout << endl << "world to base: " << endl;
    demoTfListener.printStampedTf(stfBaseToLink1);
    tfBaseToLink1 = demoTfListener.get_tf_from_stamped_tf(stfBaseToLink1);
    
    tf::Vector3 tfVec;
    tf::Matrix3x3 tfR;
    tf::Quaternion quat;
    tfVec = tfBaseToLink1.getOrigin();
    cout<<"vector from reference frame to to child frame: "<<tfVec.getX()<<","<<tfVec.getY()<<","<<tfVec.getZ()<<endl;
    tfR = tfBaseToLink1.getBasis();

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
        
    Eigen::MatrixXd skew_p = Eigen::MatrixXd::Zero(3,3);
    skew_p(0,0) = 0.0;
    skew_p(0,1) = -1*tfVec.getZ();
    skew_p(0,2) = tfVec.getY();
    skew_p(1,0) = tfVec.getZ();
    skew_p(1,1) = 0.0;
    skew_p(1,2) = -1*tfVec.getX();
    skew_p(2,0) = -1*tfVec.getY();
    skew_p(2,1) = tfVec.getX();
    skew_p(2,2) = 0.0;
    
    cout << skew_p << endl;
    /*Eigen::MatrixXd omega=Eigen::MatrixXd::Zero(3,1);
    omega(0,0) = velocity_world.angular[0];
    omega(1,0) = velocity_world.angular[1];
    omega(2,0) = velocity_world.angular[2];
        
    MatrixXd vel=MatrixXd::Zero(3,1);*/
    //vel(0,0) = velocity_world.linear[0];
    //vel(1,0) = velocity_world.linear[1];
    //vel(2,0) = velocity_world.linear[2];
        
    //MatrixXd vel_base_link(3,1);
        
    //vel_base_link= Rot_*omega + skew_p*Rot_*vel;
    //velocity_world.linear[0] = vel_base_link(0,0);
    //velocity_world.linear[1]= vel_base_link(1,0);
    //velocity_world.linear[2]= vel_base_link(2,0);
    
    return 0;
}
