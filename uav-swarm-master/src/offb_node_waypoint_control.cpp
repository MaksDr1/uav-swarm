#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <vector>
#include <string>

class Waypoint{
    public:
    float x;
    float y;
    float z;
/*
    Waypoint(float x, float y, float z, bool h){
        this -> x = x;
        this -> y = y;
        this -> z = z;
        hover = h;
    }*/

    Waypoint(float xCor, float yCor, float zCor): x(xCor), y(yCor), z(zCor) {}
};

// Print message to console and log with ROS
void notify(std::string message){
    std::cout << message << std::endl;
    //ROS_INFO(message);
}

std::vector<Waypoint> waypointList;

// Add waypoint to waypointList vector: Waypoint(x, y, z, hover)
void add_waypoint(float x, float y, float z){
    waypointList.push_back(Waypoint(x, y, z));
}


class OffboardController{
    public:
        OffboardController(ros::NodeHandle* nodehandle);
        void stateCallback(const mavros_msgs::State::ConstPtr& msg); 
        void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg); 
        void optiTrackCallback(const geometry_msgs::PoseStamped::ConstPtr& msg); 
        void initPoses();
        void check_status();
        std::vector<double> getWaypointError();
        bool reachedWaypoint(std::vector<double> errorInput);
        std::vector<double> calculateWaypointControlSignal(std::vector<double> errorInput);
        void updateWaypoint(bool waypointCheck);
        void take_off();
        void land_drone();
        int start();
        
        
    private:
        ros::NodeHandle nh_;
        ros::Subscriber state_sub_ = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, &OffboardController::stateCallback, this);
        ros::Subscriber pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &OffboardController::poseCallback, this);
        mavros_msgs::SetMode offb_set_mode;
        mavros_msgs::SetMode land_set_mode;
        mavros_msgs::CommandBool arm_cmd;
        ros::Time last_request;

 
    //publishing to setpoint raw
	ros::Publisher local_velocity_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
    ros::ServiceClient arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
        
    ros::Rate rate = 20.0;
    int n_drones;
    int n_drone;
	mavros_msgs::State current_state_;
	geometry_msgs::PoseStamped current_pose_;
	mavros_msgs::PositionTarget velocity_pos_;
	
    bool takeoff_ = true;
    bool hover_ = false;
    bool land_ = false;
    int waypointCounter = 0;
    float accuracy = 0.05;
    float Kp = 1.0; //Note: Document this
    std::vector<double> controlSignal = {};
    std::vector<double> error = {};
   
};


OffboardController::OffboardController(ros::NodeHandle* nodehandle):nh_(*nodehandle){
    initPoses();
    nh_.getParam("offb_node_0/n_drone", n_drone);
    nh_.getParam("n_drones", n_drones);

    ROS_INFO("Drone number: %i", n_drone);
    ROS_INFO("Total number of drones %i", n_drones);

}


void OffboardController::initPoses(){
    velocity_pos_.type_mask = 3011; // ignore 2048,512,256,128,64,2,1
    velocity_pos_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

    velocity_pos_.velocity.x = 0.0;
    velocity_pos_.velocity.y = 0.0;
    velocity_pos_.position.z = 0.0;

}


void OffboardController::stateCallback(const mavros_msgs::State::ConstPtr& msg){
    current_state_ = *msg;
}


void OffboardController::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose_ = *msg;
}

void OffboardController::check_status(){
    offb_set_mode.request.custom_mode = "OFFBOARD";
    land_set_mode.request.custom_mode = "AUTO.LAND";
    arm_cmd.request.value = true;


    if( current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
        if(set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent){
            ROS_INFO("Offboard enabled");
        }

        last_request = ros::Time::now();

        } else {
            if( current_state_.mode == "OFFBOARD" && !current_state_.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
                if(arming_client_.call(arm_cmd) && arm_cmd.response.success){
                    ROS_INFO("Vehicle- armed");
                }
                last_request = ros::Time::now();
            }
        }
    //notify("Mode: " + current_state_.mode);
}

std::vector<double> OffboardController::getWaypointError(){
        error = {
        waypointList[waypointCounter].x - current_pose_.pose.position.x,
        waypointList[waypointCounter].y - current_pose_.pose.position.y,
        waypointList[waypointCounter].z - current_pose_.pose.position.z};
        return(error);
}

bool OffboardController::reachedWaypoint(std::vector<double>errorInput){
    if(errorInput[0] < accuracy && errorInput[0] > -accuracy && errorInput[1] > -accuracy && errorInput[1] < accuracy &&  current_pose_.pose.position.z >= (waypointList[waypointCounter].z-0.1 )){
        return(true);
    }
    return(false);
}

std::vector<double> OffboardController::calculateWaypointControlSignal(std::vector<double> errorInput){
        if(current_pose_.pose.position.z > 0.02){
            controlSignal = {errorInput[0]*Kp,
                            errorInput[1]*Kp};

            velocity_pos_.velocity.x = controlSignal[0];
            velocity_pos_.velocity.y = controlSignal[1];
            if (hover_){
            velocity_pos_.velocity.z = 0.1;
            velocity_pos_.position.z = waypointList[waypointCounter].z;
            }
        }
        return(controlSignal);
}

void OffboardController::updateWaypoint(bool waypointCheck){
    if (waypointCounter >= (waypointList.size())){
        hover_ = false;
        land_= true;
        ROS_INFO("final waypoint reached");
        waypointCounter--;
    }
    else{
        if(waypointCheck){
            waypointCounter++;
            ROS_INFO("waypoint reached");
        }
    }  
}

void OffboardController::take_off(){
    float goalVertical= waypointList[waypointCounter].z;
    if (current_pose_.pose.position.z <= (goalVertical - 0.1)){
        velocity_pos_.position.z = goalVertical;
        velocity_pos_.velocity.z = 0.5;
    }
    else{
       takeoff_ = false;
       hover_ = true; 
       ROS_INFO("takeoff completed");
    }
}

void OffboardController::land_drone(){
    if (current_pose_.pose.position.z <= 0.02)
    {   
        velocity_pos_.velocity.x = 0;
        velocity_pos_.velocity.y = 0;
        velocity_pos_.velocity.z = -2;
        land_= false;
        ROS_INFO("landing completed");
    }
    else{
        velocity_pos_.velocity.z = -0.05;
    }
}


int OffboardController::start(){
    while(ros::ok() && !current_state_.connected){
	    ROS_INFO("NO FCU CONNECTION!");
        ros::spinOnce();
        rate.sleep();
    }
	
    //send a few setpoints before starting
    for(int i = 20; ros::ok() && i > 0; --i){
	    local_velocity_pub_.publish(velocity_pos_);
        std::cout << "Publishing setpoint: " << i << std::endl;

        ros::spinOnce();
        rate.sleep();
    }

    add_waypoint(1, 1, 1);
    add_waypoint(1, -1, 1);
    add_waypoint(-1, -1, 1);
    add_waypoint(-1, 1, 1);
    
    std::vector<double> currentError = {};
    std::vector<double> currentControlSignal = {};

    ros::Time last_request = ros::Time::now();
	
    while(ros::ok()){
        check_status();
        if (current_state_.mode == "OFFBOARD"){
            currentError = getWaypointError();
            if (takeoff_ && !land_ && !hover_){
                take_off();
                local_velocity_pub_.publish(velocity_pos_);
            }
            else if (!takeoff_ && !land_ && hover_){
                updateWaypoint (reachedWaypoint(currentError));
                currentControlSignal = calculateWaypointControlSignal(currentError);
                local_velocity_pub_.publish(velocity_pos_);
            }
            else if (!takeoff_ && land_ && !hover_){
                land_drone();
                currentControlSignal = calculateWaypointControlSignal(currentError);
                local_velocity_pub_.publish(velocity_pos_);
            }
            else{ // code stop
                ROS_INFO("nothing to do!");
                return 0;
            }
        }
        else{ 
            ROS_INFO("Waiting for Mission");
            local_velocity_pub_.publish(velocity_pos_);
        }
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char **argv){   
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    OffboardController OffboardController(&nh);
    OffboardController.start();
    return 0;
}
