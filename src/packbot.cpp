
/**
 * @description: 
 *		this file is to communicate with libpackbot and publish
 *		the odometry, inclinometer, internal compass, flipper,
 * 		brake and arm readings of the packbot as a rosmsg.
 *
 * @author: Aravind
 */
 


#include <fstream>
#include <sstream>
#include <string>

#include <yaml-cpp/yaml.h>

#include <packbot/PackbotDef.hh>
#include <packbot/Packbot.hh>

#include "ros/ros.h"

#include "packbot_ros/PackbotState.h"
#include "packbot_ros/ArmPose.h"
#include "packbot_ros/Misc.h"



using namespace std;



/// the object to communicate with libpackbot defined as global
Packbot::Messenger *packlib;



/// this function reads the yaml file and gets the serial number of the packbot
unsigned int read_yaml( char *yaml_file )
{
	ifstream fin( yaml_file ); 
	if ( fin.fail() ) 
	{
	  ROS_ERROR( "could not open %s which contains the serial number.\n", yaml_file );
	  exit(-1);
	}
	YAML::Parser parser( fin );   
	YAML::Node doc;
	parser.GetNextDocument( doc );
	unsigned int sno;
	try { doc["serial"] >> sno; } 
	catch (YAML::InvalidScalar) 
	{ 
	  printf( "the yaml file, %s does not contain the serial number for packbot.", yaml_file );
	  exit(-1);
	}
	return sno;
}



/// this function returns the current state of the arm
packbot_ros::Arm get_packbot_arm_state()
{
	packbot_ros::Arm a;	
	// turret
	a.turret.joint 			= "turret";
	a.turret.joint_id 	= 0;
	a.turret.position 	= packlib->getArmJointPos( Packbot::JointID::turret );
	a.turret.velocity 	= packlib->getArmJointVel( Packbot::JointID::turret );
	// shoulder
	a.shoulder.joint 		= "shoulder";
	a.shoulder.joint_id = 1;
	a.shoulder.position = packlib->getArmJointPos( Packbot::JointID::shoulder );
	a.shoulder.velocity = packlib->getArmJointVel( Packbot::JointID::shoulder );	
	// elbow1
	a.elbow1.joint 			= "elbow1";
	a.elbow1.joint_id 	= 2;
	a.elbow1.position 	= packlib->getArmJointPos( Packbot::JointID::elbow1 );
	a.elbow1.velocity		= packlib->getArmJointVel( Packbot::JointID::elbow1 );	
	// elbow2
	a.elbow2.joint 			= "elbow2";
	a.elbow2.joint_id 	= 3;
	a.elbow2.position 	= packlib->getArmJointPos( Packbot::JointID::elbow2 );
	a.elbow2.velocity 	= packlib->getArmJointVel( Packbot::JointID::elbow2 );	
	// tilt
	a.tilt.joint 				= "tilt";
	a.tilt.joint_id 		= 4;
	a.tilt.position 		= packlib->getArmJointPos( Packbot::JointID::tilt );
	a.tilt.velocity 		= packlib->getArmJointVel( Packbot::JointID::tilt );	
	// pan
	a.pan.joint 				= "pan";
	a.pan.joint_id 			= 5;
	a.pan.position 			= packlib->getArmJointPos( Packbot::JointID::pan );
	a.pan.velocity 			= packlib->getArmJointVel( Packbot::JointID::pan );
	// wrist
	a.wrist.joint 			= "wrist";
	a.wrist.joint_id 		= 6;
	a.wrist.position 		= packlib->getArmJointPos( Packbot::JointID::wrist );
	a.wrist.velocity 		= packlib->getArmJointVel( Packbot::JointID::wrist );	
	// grip
	a.grip.joint 				= "grip";
	a.grip.joint_id 		= 7;
	a.grip.position 		= packlib->getArmJointPos( Packbot::JointID::grip );
	a.grip.velocity 		= packlib->getArmJointVel( Packbot::JointID::grip );	
	return a;
}



/// topic: packbot_set_velocity
void callback_velocity( const packbot_ros::Velocity::ConstPtr& msg )
{
	float v_lin = (float)msg->linear;
	float v_ang = (float)msg->angular;
	packlib->setVelocity( v_lin, v_ang );
	ROS_INFO( "set the velocity of packbot to lin = %f, ang = %f", v_lin, v_ang );
}



/// topic: packbot_set_brakes
void callback_brakes( const packbot_ros::Brakes::ConstPtr& msg )
{
	bool b_main = (bool)msg->main;
	bool b_arm  = (bool)msg->arm;
	packlib->setMainBrake( b_main );
	packlib->setArmBrake( b_arm );
	ROS_INFO( "set the brakes of packbot to main = %d, arm = %d", b_main, b_arm );
}



/// topic: packbot_set_flipper
void callback_flipper( const packbot_ros::Flipper::ConstPtr& msg )
{
	float vel = (float)msg->velocity;
	packlib->setFlipperVel( vel );
	ROS_INFO( "set the flipper velocity to vel = %f", vel );
}



/// topic: packbot_set_light
void callback_light( const packbot_ros::Misc::ConstPtr& msg )
{
	bool on = msg->light;
	packlib->setAttackLight( on, false );
	ROS_INFO( "set the packbot action light to %d", on );
}



/// topic: packbot_set_camera_tilt
void callback_camera_tilt( const packbot_ros::Misc::ConstPtr& msg )
{
	int tilt = msg->camera_tilt;
	packlib->setDriveCameraTiltDegrees( tilt, false );
	ROS_INFO( "set the drive camera tilt to tilt = %d", tilt );
}



/// topic: packbot_set_armjoint_vel
void callback_armjoint_vel( const packbot_ros::ArmJoint::ConstPtr& msg )
{
	int joint_id = msg->joint_id;
	float vel    = (float)msg->velocity;
	packlib->setArmJointVel( (Packbot::JointID::JointId)joint_id, vel );
	ROS_INFO( "set joint %s to speed %f", (msg->joint).c_str(), vel );
}



/// topic: packbot_set_arm_pose
void callback_arm_pose( const packbot_ros::ArmPose::ConstPtr& msg )
{
	int pose = msg->arm_pose;
	packlib->setArmPose( (Packbot::ArmPose::ArmPose)pose, false );
	ROS_INFO( "set packbot arm to position %d", pose );
}



/// main
int main( int argc, char **argv )
{

	ros::init( argc, argv, "packbot" );
	
	if( argc < 2 )
	{
		printf( "\n\tUSAGE:\n\t\t%s <path to the yaml file with the serial number>\n", argv[0] );
		exit( -1 );
	}
	
	unsigned int serial = read_yaml( argv[1] );
	
	packlib = new Packbot::Messenger( serial );
	
	if( !packlib->initialize() )
	{
		ROS_ERROR( "unable to initialize libpackbot..." );
		ROS_INFO( "more details need to be reported later if possible..." );
		ros::shutdown();
	}
	
	ros::NodeHandle n;
	
	/// call all the subscribers to the different topics
	ros::Subscriber vel_sub = n.subscribe( "packbot_set_velocity", 			10, callback_velocity );
	ros::Subscriber bks_sub = n.subscribe( "packbot_set_brakes",   			10, callback_brakes );
	ros::Subscriber flp_sub = n.subscribe( "packbot_set_flipper",  			10, callback_flipper );
	ros::Subscriber lgt_sub = n.subscribe( "packbot_set_light",    			10, callback_light );
	ros::Subscriber cmt_sub = n.subscribe( "packbot_set_camera_tilt", 	10, callback_camera_tilt );
	ros::Subscriber ajv_sub = n.subscribe( "packbot_set_armjoint_vel", 	10, callback_armjoint_vel );
	ros::Subscriber aps_sub = n.subscribe( "packbot_set_arm_pose", 			10, callback_arm_pose );
	
	/// setup the publisher
	ros::Publisher pbs_pub = n.advertise<packbot_ros::PackbotState>( "packbot_state", 10 );
	
	ros::Rate loop_rate( 20 );
	
	packbot_ros::PackbotState pbs;
	
	float v_lin = 0.0, v_ang = 0.0;
	float roll = 0.0, pitch = 0.0, yaw = 0.0;
	
	unsigned int count = 0;
	
	while( ros::ok() )
	{
	
		packlib->getVelocity( v_lin, v_ang );
		packlib->getPose( yaw, roll, pitch );
		
		/// assign the values to the PackbotState msg datatype
		pbs.velocity.linear				= v_lin;
		pbs.velocity.angular			= v_ang;
		pbs.orientation.roll			= roll;
		pbs.orientation.pitch			= pitch;
		pbs.orientation.yaw				= yaw;
		pbs.brakes.main						= packlib->isMainBrakeEngaged();
		pbs.brakes.arm						= packlib->isArmBrakeEngaged();
		pbs.flipper.position			= packlib->getFlipperPos();
		pbs.flipper.velocity			= packlib->getFlipperVel();
		
		/// get the state of the packbot's arm
		pbs.arm										= get_packbot_arm_state();
		
		/// create the header for the msg
		pbs.header.seq 						= count;
		pbs.header.stamp 					= ros::Time::now();
		pbs.header.frame_id 			= "base_link";
		
		/// publish the msg
		pbs_pub.publish( pbs );
		
		count++;
		
		ros::spin();
		loop_rate.sleep();
	
	}
	
	return 0;
	
}



