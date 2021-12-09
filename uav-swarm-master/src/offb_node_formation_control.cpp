#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <sensor_msgs/NavSatFix.h>
#include <vector>
#include <string>
#include <cmath>

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
        void poseCallbackDrone0(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void poseCallbackDrone1(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void poseCallbackDrone2(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void stateCallbackDrone0(const mavros_msgs::State::ConstPtr& msg);
        void stateCallbackDrone1(const mavros_msgs::State::ConstPtr& msg);
        void stateCallbackDrone2(const mavros_msgs::State::ConstPtr& msg);
        void optiTrackCallback(const geometry_msgs::PoseStamped::ConstPtr& msg); 
        void initPoses();
        void check_status();
        void getWaypointError();
        bool reachedWaypointCheck();
        void calculateWaypointControlSignal();
        void updateWaypoint(bool waypointCheck);
        void take_off();
        void land_drone();
        void sendDronePos();
        void getSwarmPos();
        void calculateSwarmCentroid();
        double matrixNorm(std::vector<double>vectorInput);
        void calculateEdgesAndDistances();
        void calculateFormationControlSignal();
        std::vector<double> limitControlSignal(std::vector<double>controlSignal,std::vector<double>limit);
        std::vector<double> limitVelocity(double maxVelocity);
        double cappedGain(double gain,bool polarity);
        int start();
        
        
    private:
        ros::NodeHandle nh_;
        ros::Subscriber state_sub_ = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, &OffboardController::stateCallback, this);
        ros::Subscriber pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &OffboardController::poseCallback, this);
        ros::Subscriber pose_sub_0 = nh_.subscribe<geometry_msgs::PoseStamped>("uav0/mavros/local_position/pose", 10, &OffboardController::poseCallbackDrone0, this);
        ros::Subscriber pose_sub_1 = nh_.subscribe<geometry_msgs::PoseStamped>("uav1/mavros/local_position/pose", 10, &OffboardController::poseCallbackDrone1, this);
        ros::Subscriber pose_sub_2 = nh_.subscribe<geometry_msgs::PoseStamped>("uav2/mavros/local_position/pose", 10, &OffboardController::poseCallbackDrone2, this);
        ros::Subscriber state_sub_0 = nh_.subscribe<mavros_msgs::State>("uav0/mavros/state", 10, &OffboardController::stateCallbackDrone0, this);
        ros::Subscriber state_sub_1 = nh_.subscribe<mavros_msgs::State>("uav1/mavros/state", 10, &OffboardController::stateCallbackDrone1, this);
        ros::Subscriber state_sub_2 = nh_.subscribe<mavros_msgs::State>("uav2/mavros/state", 10, &OffboardController::stateCallbackDrone2, this);
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
	mavros_msgs::State current_state_drone0;
    mavros_msgs::State current_state_drone1;
    mavros_msgs::State current_state_drone2;
	geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::PoseStamped drone_0;
    geometry_msgs::PoseStamped drone_1;
    geometry_msgs::PoseStamped drone_2;
	mavros_msgs::PositionTarget velocity_pos_;
	
    bool takeoff_ = true;
    bool hover_ = false;
    bool land_ = false;
    bool simulation = false;
    bool swarmReachedWaypoint = false;
    bool waypointReached = false;
    bool waypointReached1 = false;
    bool waypointReached2 = false;
    bool waypointReached3 = false;
    bool startTimer= true;
    int waypointCounter = 0;
    float accuracy = 0.05;
    float kp = 0; //Note: Document this <-- velocity control p
    float kf = 0; // formation control p 
    float kd = 0; // formation control d 
    float dt = 0;  // timestep
    float previousTime =0;
    float currentTime =0;
    std::vector<double> uGoal = {0,0};
    std::vector<double> waypointError = {};
    std::vector<double> DronePos0 ={0,0,0};
    std::vector<double> DronePos1 ={0,0,0};
    std::vector<double> DronePos2 ={0,0,0};
    std::vector<std::vector<double>> dronesPos ={};
    std::vector<double> swarmCentroid ={};
    std::vector<std::vector<double>> desiredConfig ={{0,0},{-1.74,1},{-1.74,-1}};
    std::vector<std::vector<double>> edgesAndDistances = {};
    std::vector<double> zk = {0,0};
    std::vector<double> formationSignal ={0,0};
    std::vector<double> previousFormationSignal={0,0};
    std::vector<double> uFormation ={0,0};
    std::vector<std::vector<double>> limits = {{0,0},{0,0}};    
    std::vector<double> cappedControlSignal = {0,0};
};


OffboardController::OffboardController(ros::NodeHandle* nodehandle):nh_(*nodehandle){
    initPoses();
    nh_.getParam( ros::this_node::getName()+"/n_drone", n_drone);
    nh_.getParam("n_drones", n_drones);
    ROS_INFO("Drone number: %i", n_drone);
    ROS_INFO("Total number of drones %i", n_drones);
}


void OffboardController::initPoses(){
    velocity_pos_.type_mask = 3011; // ignore 2048,512,256,128,64,2,1 flags
    velocity_pos_.coordinate_frame = 1; // 1 local coordinate system & 8 is global coordinate system

    velocity_pos_.velocity.x = 0.0;
    velocity_pos_.velocity.y = 0.0;
    velocity_pos_.velocity.z = 0.0;
    velocity_pos_.position.z = 0.0;

}


void OffboardController::stateCallback(const mavros_msgs::State::ConstPtr& msg){
    current_state_ = *msg;
}

void OffboardController::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose_ = *msg;
}
void OffboardController::poseCallbackDrone0(const geometry_msgs::PoseStamped::ConstPtr& msg){
    drone_0 = *msg;
}
void OffboardController::poseCallbackDrone1(const geometry_msgs::PoseStamped::ConstPtr& msg){
    drone_1 = *msg;
}
void OffboardController::poseCallbackDrone2(const geometry_msgs::PoseStamped::ConstPtr& msg){
    drone_2 = *msg;
}

void OffboardController::stateCallbackDrone0(const mavros_msgs::State::ConstPtr& msg){
    current_state_drone0 = *msg;
}

void OffboardController::stateCallbackDrone1(const mavros_msgs::State::ConstPtr& msg){
    current_state_drone1 = *msg;
}

void OffboardController::stateCallbackDrone2(const mavros_msgs::State::ConstPtr& msg){
    current_state_drone2 = *msg;
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

void OffboardController::getWaypointError(){
       waypointError = {
        waypointList[waypointCounter].x - swarmCentroid[0],
        waypointList[waypointCounter].y - swarmCentroid[1],
        waypointList[waypointCounter].z - swarmCentroid[2]};
        //std::cout<< "error: " << waypointError[0]<< " , " <<waypointError[1]<< std::endl;
}

bool OffboardController::reachedWaypointCheck(){
    //std::cout<< n_drone<<": " <<ros::Time::now() - last_request<< std::endl;
    nh_.getParam( "offb_node_0/waypoint_reached", waypointReached1);
    nh_.getParam( "offb_node_1/waypoint_reached", waypointReached2);
    nh_.getParam( "offb_node_2/waypoint_reached", waypointReached3);
    if(!swarmReachedWaypoint && waypointError[0] < accuracy && waypointError[0] > -accuracy && waypointError[1] > -accuracy && waypointError[1] < accuracy &&  current_pose_.pose.position.z >= (waypointList[waypointCounter].z-0.1) && (ros::Time::now() - last_request > ros::Duration(30.0))){
        last_request = ros::Time::now();
        waypointReached = true;
        nh_.setParam( ros::this_node::getName()+"/waypoint_reached", waypointReached);
    }

    if(!swarmReachedWaypoint && waypointReached1 && waypointReached2  && waypointReached3){
        swarmReachedWaypoint = true;
      
    }

    if(swarmReachedWaypoint && !waypointReached1 && !waypointReached2 && !waypointReached3){
        swarmReachedWaypoint = false;
    }

    if(waypointReached  && swarmReachedWaypoint && (ros::Time::now() - last_request > ros::Duration(5.0))){
        last_request = ros::Time::now();
        waypointReached = false;
        nh_.setParam( ros::this_node::getName()+"/waypoint_reached", waypointReached);
        return(true);
    }
    return(false);
}

void OffboardController::calculateWaypointControlSignal(){
    uGoal={0,0};
    nh_.getParam("kp", kp);
    kp = cappedGain(kp,true);
    std::cout<< n_drone << " kp:" << kp <<std::endl;
    if(current_pose_.pose.position.z > 0.02){
        uGoal = {waypointError[0]*kp,
                 waypointError[1]*kp};

        
        if (hover_){
        velocity_pos_.velocity.z = 0.5;
        velocity_pos_.position.z = waypointList[waypointCounter].z;
        }
    }
    //std::cout<< "waypoint control:" <<uGoal[0]<< " , "<< uGoal[1] <<std::endl;
}

void OffboardController::updateWaypoint(bool waypointCheck){
    if (waypointCounter >= (waypointList.size())){
        hover_ = false;
        land_= true;
        ROS_INFO("final waypoint reached");
        waypointCounter = waypointList.size();
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
        velocity_pos_.position.z = 0;
    }
}

void OffboardController::getSwarmPos(){
    // the mavros/local_position/pose has its own coordinate frame at its starting position
    // this means that we need to take into account an offset when using a simulation
    if (simulation = true){
    DronePos0 = {drone_0.pose.position.x,drone_0.pose.position.y,drone_0.pose.position.z}; 
    DronePos1 = {(drone_1.pose.position.x + 1),drone_1.pose.position.y,drone_1.pose.position.z};
    DronePos2 = {drone_2.pose.position.x,(drone_2.pose.position.y + 1 ),drone_2.pose.position.z};
    }
    else{
    DronePos0 = {drone_0.pose.position.x,drone_0.pose.position.y,drone_0.pose.position.z}; 
    DronePos1 = {drone_1.pose.position.x,drone_1.pose.position.y,drone_1.pose.position.z};
    DronePos2 = {drone_2.pose.position.x,drone_2.pose.position.y,drone_2.pose.position.z};
    }
    dronesPos = {DronePos0,DronePos1,DronePos2};
    //std::cout << "drone 0 x:" << DronePos0[0] << " y:" << DronePos0[1] << " z:" << DronePos0[2] << std::endl;
    //std::cout << "drone 1 x:" << DronePos1[0] << " y:" << DronePos1[1] << " z:" << DronePos1[2] << std::endl;
    //std::cout << "drone 2 x:" << DronePos2[0] << " y:" << DronePos2[1] << " z:" << DronePos2[2] << std::endl;
    
}

void OffboardController::calculateSwarmCentroid(){
    swarmCentroid = {(DronePos0[0]+DronePos1[0]+DronePos2[0])/3,
                      (DronePos0[1]+DronePos1[1]+DronePos2[1])/3,
                      (DronePos0[2]+DronePos1[2]+DronePos2[2])/3};
    //std::cout << "swarm centroid x:" << swarmCentroid[0] << " swarm centroid y:" << swarmCentroid[1] << " swarm centroid z:" << swarmCentroid[2] << std::endl;
}

double OffboardController::matrixNorm(std::vector<double>vectorInput){
    // distance = sqrt((x0-x1)^2+(y0-y1)^2)
    return(std::sqrt(std::pow((vectorInput[0]),2) + std::pow((vectorInput[1]),2)));
}


void OffboardController::calculateEdgesAndDistances(){
    // distance = sqrt((x0-x1)^2+(y0-y1)^2)
    double distance1 = matrixNorm({desiredConfig[0][0]- desiredConfig[1][0],desiredConfig[0][1]-desiredConfig[1][1]});
    double distance2 = matrixNorm({desiredConfig[1][0]- desiredConfig[2][0],desiredConfig[1][1]-desiredConfig[2][1]});
    double distance3 = matrixNorm({desiredConfig[0][0]- desiredConfig[2][0],desiredConfig[0][1]-desiredConfig[2][1]});
    edgesAndDistances = {{0,1,distance1}, {1,2,distance2},{0,2,distance3}};
    //std::cout<<"distance: " << distance1 << " , " << distance2 << " , " << distance3 << std::endl;
}

void OffboardController::calculateFormationControlSignal(){
    previousFormationSignal = {0,0};
    previousFormationSignal = formationSignal;
    formationSignal={0,0};
    uFormation = {0,0};
    previousTime = currentTime;
    nh_.getParam("kf", kf);
    nh_.getParam("kd", kd);
    kf = cappedGain(kf,false);
    kd = cappedGain(kd,false);
    std::cout<< n_drone << " kf:" << kf << " kd: " << kd <<std::endl;
    currentTime = ros::Time::now().toSec();
    for(int i=0;i < n_drones;i++){
        if (i != n_drone){
            zk = {dronesPos[n_drone][0]-dronesPos[i][0],dronesPos[n_drone][1]-dronesPos[i][1]};
            //std::cout<< n_drone << " to drone " << i << " zk: " << matrixNorm(zk) <<std::endl;
            formationSignal[0] = formationSignal[0] + zk[0]/matrixNorm(zk) * (matrixNorm(zk)-edgesAndDistances[i][2]);
            formationSignal[1] = formationSignal[1] + zk[1]/matrixNorm(zk) * (matrixNorm(zk)-edgesAndDistances[i][2]);
        }
    }
    dt = currentTime - previousTime;

    // uFormation = kp * error + kd * diffence in error)
    uFormation[0] = formationSignal[0] * kf + ((formationSignal[0]-previousFormationSignal[0])/dt) * kd;
    uFormation[1] = formationSignal[1] * kf + ((formationSignal[1]-previousFormationSignal[1])/dt) * kd;  
    //std::cout<< "formation control:" <<uFormation[0]<< " , "<< uFormation[1] <<std::endl;
}

std::vector<double> OffboardController::limitControlSignal(std::vector<double>controlSignal,std::vector<double>limit){
    if (controlSignal[0] >= limit[0]){
        controlSignal[0] = limit[0];
    }
    else if(controlSignal[0] <= -limit[0]){
        controlSignal[0] = -limit[0];
    }
    if (controlSignal[1] >= limit[1]){
        controlSignal[1] = limit[1];
    }
    else if(controlSignal[1] <= -limit[1]){
        controlSignal[1] = -limit[1];
    }
    return controlSignal;
}


std::vector<double> OffboardController::limitVelocity(double maxVelocity){ //if max vel = 1 m/s & the formation has a vel of 0,5 the other 0,5 can be used for the goal velocity
    for(int i=0;i < 2;i++){
        if (uFormation[i] >= maxVelocity || uFormation[i] <= -maxVelocity){
            limits[0][i]= maxVelocity;
            limits[1][i]=0;

            uFormation = limitControlSignal(uFormation,limits[0]);
        }
        else{
            if (uFormation[i] >= 0){
               limits[1][i]=maxVelocity - uFormation[i];  
            }
            else{
               limits[1][i]=maxVelocity + uFormation[i];
            }
        }
        //std::cout << "limit: " << limits[1][i] << std::endl;
    }
    uGoal = limitControlSignal(uGoal,limits[1]);
    cappedControlSignal[0] = uFormation[0] + uGoal[0];
    cappedControlSignal[1] = uFormation[1] + uGoal[1];
    return cappedControlSignal;
}
double OffboardController::cappedGain(double gain,bool polarity){
    if(polarity){
        if(gain < 0){
        gain = 0.5;
        }
    } 
    if(!polarity){
        if(gain > 0){
        gain = -0.5;
        }
    } 
    return (gain);
}


int OffboardController::start(){
    while(ros::ok() && !current_state_.connected){
	    ROS_INFO("NO FCU CONNECTION!");
        ros::spinOnce();
        rate.sleep();
    }
	
    //send a few setpoints before starting
    for(int i = 30; ros::ok() && i > 0; --i){
	    local_velocity_pub_.publish(velocity_pos_);
        std::cout << "Publishing setpoint: " << i << std::endl;

        ros::spinOnce();
        rate.sleep();
    }
    
    std::vector<double> currentControlSignal = {0,0};

    ros::Time last_request = ros::Time::now();

    calculateEdgesAndDistances();

    //add_waypoint(0.5, 0, 1);
    add_waypoint(0, 0, 1.5);
    add_waypoint(0, 0, 1.5);
    //add_waypoint(-0.5, 0, 1);
    

    nh_.getParam("simulation", simulation);
	
    while(ros::ok()){
        check_status();
        if (current_state_.mode == "OFFBOARD"){
            getSwarmPos();
            calculateSwarmCentroid();
            getWaypointError();
            calculateWaypointControlSignal();
            calculateFormationControlSignal(); 
            currentControlSignal = limitVelocity(1);           
            //std::cout<< currentControlSignal[0]<< " , "<< currentControlSignal[1] <<std::endl;
            if ((takeoff_ && !land_ && !hover_) && (current_state_drone0.mode == "OFFBOARD") && (current_state_drone1.mode == "OFFBOARD") && (current_state_drone2.mode == "OFFBOARD" ) ){
                take_off();
                local_velocity_pub_.publish(velocity_pos_); 
            }
            else if (!takeoff_ && !land_ && hover_){
                updateWaypoint (reachedWaypointCheck());
                velocity_pos_.velocity.x = currentControlSignal[0];
                velocity_pos_.velocity.y = currentControlSignal[1];
                local_velocity_pub_.publish(velocity_pos_);
            }
            else if (!takeoff_ && land_ && !hover_){
                velocity_pos_.velocity.x = currentControlSignal[0];
                velocity_pos_.velocity.y = currentControlSignal[1];
                land_drone();
                local_velocity_pub_.publish(velocity_pos_);
            }
            else if (!takeoff_ && !land_ && !hover_){ // code stops
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

