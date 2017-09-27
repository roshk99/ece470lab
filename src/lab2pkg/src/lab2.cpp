#include "ros/ros.h"
#include "ur_msgs/IOStates.h"
#include "ur_msgs/Analog.h"

#include "lab2pkg/lab2.h" 
#define PI 3.14159265359
#define SPIN_RATE 20  /* Hz */

//arrays defining Waypoints
double home[]={156.8*PI/180,-80*PI/180,55.02*PI/180,-60.11*PI/180,-92.14*PI/180,0};

double arr11[]={146*PI/180,-49.2*PI/180,104.1*PI/180,-142.7*PI/180,-93.3*PI/180,0};
double arr12[]={144*PI/180,-56.1*PI/180,107.5*PI/180,-141.41*PI/180,-88.54*PI/180,0};
double arr13[]={144*PI/180,-61.91*PI/180,106.17*PI/180,-135.25*PI/180,-88.53*PI/180,0};
double arr14[]={143.5*PI/180,-67.4*PI/180,106.3*PI/180,-131.8*PI/180,-87.6*PI/180,0};

double arr21[]={154.5*PI/180,-50*PI/180,107.4*PI/180,-145.2*PI/180,-91*PI/180,0};
double arr22[]={154.5*PI/180,-57.02*PI/180,108.6*PI/180,-140.98*PI/180,-90.36*PI/180,0};
double arr23[]={154.5*PI/180,-63.32*PI/180,108.43*PI/180,-136.7*PI/180,-90.19*PI/180,0};
double arr24[]={153*PI/180,-67.5*PI/180,104.2*PI/180,-126.4*PI/180,-87.3*PI/180,0};

double arr31[]={165.76*PI/180,-49.89*PI/180,107.52*PI/180,-147.03*PI/180,-90.23*PI/180,0};
double arr32[]={165.78*PI/180,-56.52*PI/180,110.39*PI/180,-147.29*PI/180,-90.18*PI/180,0};
double arr33[]={165.78*PI/180,-62.3*PI/180,106.64*PI/180,-135.36*PI/180,-90.18*PI/180,0};
double arr34[]={164.9*PI/180,-67*PI/180,102.2*PI/180,-123.9*PI/180,-89.2*PI/180,0};

// array to define final velocity of point to point moves.  For now slow down to zero once 
// each point is reached
double arrv[]={0,0,0,0,0,0};

//vectors to be used to publish commands to UR3 ROS Driver (ece470_ur3_driver)
std::vector<double> QH (home,home+sizeof(home) / sizeof(home[0]));

std::vector<double> Q11 (arr11,arr11+sizeof(arr11) / sizeof(arr11[0]));
std::vector<double> Q12 (arr12,arr12+sizeof(arr12) / sizeof(arr12[0]));
std::vector<double> Q13 (arr13,arr13+sizeof(arr13) / sizeof(arr13[0]));
std::vector<double> Q14 (arr14,arr14+sizeof(arr14) / sizeof(arr14[0]));
std::vector<double> Q21 (arr21,arr21+sizeof(arr21) / sizeof(arr21[0]));
std::vector<double> Q22 (arr22,arr22+sizeof(arr22) / sizeof(arr22[0]));
std::vector<double> Q23 (arr23,arr23+sizeof(arr23) / sizeof(arr23[0]));
std::vector<double> Q24 (arr24,arr24+sizeof(arr24) / sizeof(arr24[0]));
std::vector<double> Q31 (arr31,arr31+sizeof(arr31) / sizeof(arr31[0]));
std::vector<double> Q32 (arr32,arr32+sizeof(arr32) / sizeof(arr32[0]));
std::vector<double> Q33 (arr33,arr33+sizeof(arr33) / sizeof(arr33[0]));
std::vector<double> Q34 (arr34,arr34+sizeof(arr34) / sizeof(arr34[0]));

std::vector<double> v (arrv,arrv+sizeof(arrv) / sizeof(arrv[0]));

// creating an array of these vectors allows us to iterate through them
// and programatically choose where to go.
std::vector<double> Q [3][4] = {
    {Q11, Q12, Q13, Q14},
    {Q21, Q22, Q23, Q24},
    {Q31, Q32, Q33, Q34}
};


ur_msgs::SetIO srv;

ece470_ur3_driver::command driver_msg;


// Global bool variables that are assigned in the callback associated when subscribed 
// to the "ur3/position" topic
bool isReady=1;
bool pending=0;
bool hasBlock=0;

void block_callback(const ur_msgs::IOStates msg)
{
	if (msg.analog_in_states[0].state < 1.5) {
		hasBlock = 0;
	} else {
		hasBlock = 1;
	}
}

// Whenever ur3/position publishes info this callback function is run.
void position_callback(const ece470_ur3_driver::positions::ConstPtr& msg)
{
	isReady=msg->isReady; // When isReady is True the robot arm has made it to its desired position
						  // and is ready to be told to go to another point if desired.
	pending=msg->pending; // pending is the opposite of isReady, pending is true until a new position is reached
//	ROS_INFO("Debug isRdy = %d, pending = %d",(int)isReady,(int)pending);
}


int move_arm(	ros::Publisher pub_command , ros::Rate loop_rate, std::vector<double> dest, float duration)
{
    int error = 0;
    driver_msg.destination=dest;
    driver_msg.duration = duration;  
	pub_command.publish(driver_msg);  // publish command, but note that is possible that
											  // the subscriber will not receive this message.
	int spincount = 0;
	while (isReady) { // Waiting for isReady to be false meaning that the driver has the new command
		ros::spinOnce();  // Allow other ROS functionallity to run
		loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
		if (spincount > SPIN_RATE) {  // if isReady does not get set within 1 second re-publish
			pub_command.publish(driver_msg);
			ROS_INFO("Just Published again driver_msg");
			spincount = 0;
		}
		spincount++;  // keep track of loop count
	}

	ROS_INFO("waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.
	while(!isReady)
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
    
    return error;
}

int move_block(ros::Publisher pub_command ,
                ros::Rate loop_rate,
                ros::ServiceClient srv_SetIO,
                ur_msgs::SetIO srv,
                int start_loc,
                int start_height,
                int end_loc,
                int end_height)
{
    int error = 0;
    move_arm(pub_command, loop_rate, Q[start_loc-1][3], 0.0);
    move_arm(pub_command, loop_rate, Q[start_loc-1][start_height-1], 0.0);
    
    srv.request.fun = 1;
	srv.request.pin = 0;  //Digital Output 0
	srv.request.state = 1.0; //Set DO0 on
	if (srv_SetIO.call(srv)) {
		ROS_INFO("True: Switched Suction ON");
	} else {
		ROS_INFO("False");
	}
	move_arm(pub_command, loop_rate, Q[start_loc-1][3], 0.0);

	//Check if have block
	if (!hasBlock) {
		error = 1;
		srv.request.state = 0.0; //Set DO0 off
		if (srv_SetIO.call(srv)) {
			ROS_INFO("True: Switched Suction OFF");
		} else {
			ROS_INFO("False");
		}
		return error;
	}
    move_arm(pub_command, loop_rate, Q[end_loc-1][3], 0.0);
    move_arm(pub_command, loop_rate, Q[end_loc-1][end_height-1], 0.0);
    
    //srv.request.fun = 1;
	//srv.request.pin = 0; // Digital Output 0
	srv.request.state = 0.0; //Set DO0 off
	if (srv_SetIO.call(srv)) {
		ROS_INFO("True: Switched Suction OFF");
	} else {
		ROS_INFO("False");
	}
    move_arm(pub_command, loop_rate, Q[end_loc-1][3], 0.0);
    return error;
}

int main(int argc, char **argv)
{
	int inputdone = 0;
	int Loopcnt = 0;
	int error;
//initialization & variable definition
	ros::init(argc, argv, "lab2node");	//initialzation of ros required for each Node.
	ros::NodeHandle nh;				//handler for this node.
	
	//initialized publisher ur3/command, buffer size of 10.
	ros::Publisher pub_command=nh.advertise<ece470_ur3_driver::command>("ur3/command",10);
	// initialize subscriber to ur3/position and call function position_callback each time data is published
	ros::Subscriber sub_position=nh.subscribe("ur3/position",1,position_callback);
	ros::Subscriber block=nh.subscribe("ur_driver/io_states",1000, block_callback);
	ros::ServiceClient srv_SetIO = nh.serviceClient<ur_msgs::SetIO>("ur_driver/set_io");

	std::string inputString;
	while (!inputdone) {
		std::cout << "Enter Number of Loops <Either 1 2 or 3>";
		std::getline(std::cin, inputString);
		std::cout << "You entered " << inputString << "\n";
		if (inputString == "1") {
			inputdone = 1;
			Loopcnt = 1;
		} else if (inputString == "2") {
			inputdone = 1;
			Loopcnt = 2;
		} else if (inputString == "3") {
			inputdone = 1;
			Loopcnt = 3;
		} else {
			std:cout << "Please just enter the character 1 2 or 3\n\n";
		}
	}

	while(!ros::ok()){};	//check if ros is ready for operation
		
	ROS_INFO("sending Goals");

	ros::Rate loop_rate(SPIN_RATE); // Initialize the rate to publish to ur3/command
	int spincount = 0;
 
	while(Loopcnt > 0) {
		move_arm(pub_command, loop_rate, QH, 0.0);
		//move_arm(pub_command, loop_rate, Q[2][0], 0.0);
		//return 0;
		int start_pos;
		int end_pos;

		cout<<"Enter start position"<<endl;
		cin>>start_pos;
		cout<<"Enter end position"<<endl;
		cin>>end_pos;

		int mid_pos;
		if ((start_pos % 3) + 1 == end_pos) {
			mid_pos = (end_pos % 3) + 1;
		} else {
			mid_pos = (start_pos % 3) + 1;
		}
		error = move_block(pub_command, loop_rate, srv_SetIO, srv, start_pos, 3, end_pos, 1);
		if (error==1) {
			move_arm(pub_command, loop_rate, QH, 0.0);
			ROS_INFO("No Block Found!");
			return 0;
		}
		error = move_block(pub_command, loop_rate, srv_SetIO, srv, start_pos, 2, mid_pos, 1);
		if (error==1) {
			move_arm(pub_command, loop_rate, QH, 0.0);
			ROS_INFO("No Block Found!");
			return 0;
		}
		error = move_block(pub_command, loop_rate, srv_SetIO, srv, end_pos, 1, mid_pos, 2);
		if (error==1) {
			move_arm(pub_command, loop_rate, QH, 0.0);
			ROS_INFO("No Block Found!");
			return 0;
		}
		error = move_block(pub_command, loop_rate, srv_SetIO, srv, start_pos, 1, end_pos, 1);
		if (error==1) {
			move_arm(pub_command, loop_rate, QH, 0.0);
			ROS_INFO("No Block Found!");
			return 0;
		}
		error = move_block(pub_command, loop_rate, srv_SetIO, srv, mid_pos, 2, start_pos, 1);
		if (error==1) {
			move_arm(pub_command, loop_rate, QH, 0.0);
			ROS_INFO("No Block Found!");
			return 0;
		}
		error = move_block(pub_command, loop_rate, srv_SetIO, srv, mid_pos, 1, end_pos, 2);
		if (error==1) {
			move_arm(pub_command, loop_rate, QH, 0.0);
			ROS_INFO("No Block Found!");
			return 0; 
		}
		error = move_block(pub_command, loop_rate, srv_SetIO, srv, start_pos, 1, end_pos, 3);
		if (error==1) {
			move_arm(pub_command, loop_rate, QH, 0.0);
			ROS_INFO("No Block Found!");
			return 0;
		}

		move_arm(pub_command, loop_rate, QH, 0.0);
		Loopcnt--;
	}

	return 0;
}
