#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include <sstream>
#include <tf2_ros/transform_broadcaster.h>
#include <boost/circular_buffer.hpp>
#include <iostream>
#include <fstream>
#include <math.h>
#include "pf2_6d.h"
#include "gnuplot.h"
#include "text/to_string.h"
#include "nlink_parser/LinktrackNodeframe2.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>


ros::Publisher  pub_uav1_pose;
ros::Publisher  pub_human_pose;

//publish the relative poses based on human
ros::Publisher  pub_human_uav1_pose;

//publish the relative poses based on uav1
ros::Publisher  pub_uav1_human_pose;


typedef pf2::ParticleFilter6D            ParticleFilter_UWB;

/** Information about the pose of a human */
class AgentPoseRepresenation
{
public:
	AgentPoseRepresenation()
{
		accu_dis=0;
		accu_angle=0;
		pre_accu_dis=0;
		pre_accu_angle=0;
}
	ParticleFilter_UWB * particle_filter; //particle filter for estimation
	carmen_6d_point_t position;//current estimated pose
	double variance_xx;
	double variance_yy;
	double variance_theta;
	std::vector< carmen_6d_point_t > trajectories;
	std::vector< double > v_timestamps;
	std::vector< carmen_6d_point_t > trajectories_low_frequency;
	std::vector< double > timestamps_low_frequency;
	std::vector< carmen_6d_point_t > raw_odom;
	std::vector< double > distances;
	double accu_dis;
	double accu_angle;
	double pre_accu_dis;
	double pre_accu_angle;

	double timestamp;
};


class RangingMeasurement
{
public: RangingMeasurement(){}
double timestamp;//ROS system
long int system_time;//UWB system time, synchronized already
std::string tag_id;//on the drone
std::string anchor_id;//on the human
//if range is smaller than 0, this range is not valid
double range;
double variance;
//signal strength
int rssi;
};


//Define Noop UWB packet
class NoopLoopUWBPacket
{
public: NoopLoopUWBPacket(){}
double timestamp;//timestamp in seconds
long int local_time;//local time of a UWB device
long int system_time;//system of a UWB network
std::string tag_id;//tag id
std::map<std::string, double> v_ranging;//rangings to the anchors
std::map<std::string, double> v_rss_difference;//Absolute RSS difference be rxRSS and fpRSS
};

class TempUWB
{
public: TempUWB(){}
std::string tag_id;
std::string anchor_id;
double range;//mean range values
double variance;
std::vector< double > v_range;
};

class TempAgent
{
public: TempAgent(){}
std::string agent_name;
std::string peer_agent_name;
};


boost::circular_buffer<NoopLoopUWBPacket> v_nl_uwb_packets(600);//a circular buffer for the UWB measurements

typedef std::map< std::string, AgentPoseRepresenation > AgentMappingInfo;

AgentMappingInfo m_Agent;

int m_num_samples = 200;//number of particles used for localization, more particles will consume more time
double m_init_range_uncertainty=0.5;
double m_init_angle_uncertainty=0.2;

//minimum and maximum particle weights
double m_minLikelihood=0.001;
double m_maxLikelihood=0.999;

double m_dBinningMinX=-2;
double m_dBinningMaxX=8;
double m_dBinningMinY=-8;
double m_dBinningMaxY=4;
double m_dBinningMinZ=0;
double m_dBinningMaxZ=6;

double m_sigma=1.0;//uwb ranging noise

double m_odom_motion_noise=0.5;
double m_odom_cross_motion_noise=0.0;
double m_odom_orientation_noise=0.05;
double m_odom_theta_d_noise=0.00;

double m_human_odom_motion_noise=0.5;
double m_human_odom_cross_motion_noise=0.0;
double m_human_odom_orientation_noise=0.05;
double m_human_odom_theta_d_noise=0.00;

double m_noise_z=0.2;
double m_initial_z_noise=0.2;

int m_enable_z_correction=-1;
int m_force_human_2d=-1;

std::string m_frame_id="odom";

std::string m_uav1_frame_id="odom";
std::string m_human_frame_id="odom";


std::string m_uav1_child_frame_id="uav1";
std::string m_human_child_frame_id="human";


double m_fast_motion_ratio=0.000;

double m_static_update_ratio=0.003;

double m_human_odom_downsample_ratio=0.5;

double m_uav_odom_downsample_ratio=0.5;

double m_uwb_ranging_filtering=0.5;

int m_use_min_ranging=-1;


//only apply the update if the robot moves over a distance or rotate over an angle
double m_update_min_d=0.2;
double m_update_min_a=0.02;


double m_enable_visualization=-1;
double m_time_constraints=1.0;



GnuplotInterface * m_plot_samples= new GnuplotInterface();
GnuplotInterface * m_plot_track= new GnuplotInterface();
GnuplotInterface * m_plot_odom= new GnuplotInterface();
GnuplotInterface * m_plot_3d_samples= new GnuplotInterface();

GnuplotInterface * m_plot_3d_track_xy= new GnuplotInterface();
GnuplotInterface * m_plot_3d_track_xz= new GnuplotInterface();
GnuplotInterface * m_plot_3d_track_yz= new GnuplotInterface();
GnuplotInterface * m_plot_3d_track= new GnuplotInterface();


class ObjectConfig
{
public: ObjectConfig(){}
//tags and the relative locations
std::map <std::string, carmen_6d_point_t > tag_information;
//starting height of a drone or human
double height;
};

//for the drone uwb configuration
std::map< std::string, ObjectConfig > m_drone_config;

//for the drone uwb configuration
std::map< std::string, ObjectConfig > m_agent_config;//agent name (drone1 2 3, human 1) to configure

//for the human uwb configuration
std::map< std::string, ObjectConfig > m_human_config;

//tag id to the drone name
std::map< std::string, std::string > m_tag_drone;//map from tag to drones

//tag id to the human name
std::map< std::string, std::string > m_tag_human;//map from tag to humans

//tag id to the human name or drone name
std::map< std::string, std::string > m_tag_objects;//map from tag to objects, including drone and human

//count of human odom have received so far
int num_human_odom=0;

//previous human odometry
carmen_6d_point_t previous_human_odom;

//previous drone1 odometry
carmen_6d_point_t previous_uav1_odom;


carmen_point_t m_initial_uav1;
carmen_point_t m_initial_human;

//denote if the human odometry is available
int human_odom_available=-1;

//denote if the drone1 odometry is available
int uav1_odom_available=-1;

//previous human odom time
ros::Time human_odom_previous_time;

//previous human odom time for visualization
ros::Time human_visualization_previous_time;

//previous drone1 odom time
ros::Time uav1_odom_previous_time;

//invalid uav1 uwb odometry
int invalid_uwb_uav1=0;

//invalid human uwb odometry
int invalid_uwb_human=0;

int total_uwb_uav1=0;
int total_uwb_human=0;

double total_uwb_delay_uav1=0;

double total_uwb_delay_human=0;


int invalid_odom_uav1=0;

int total_odom_uav1=0;


double total_odom_delay_uav1=0;

std::string m_UAV1_uwb="/drone1_uwb";
std::string m_human_uwb="/human_uwb";

std::string m_UAV1_odom="/camera_1/odom/sample";
std::string m_human_odom="/imu_odometry";

std::string m_UAV1_pose="/UAV1Pose";
std::string m_Human_pose="/HumanPose";

std::string m_human_UAV1_pose="/HumanUAV1RelativePose";

std::string m_UAV1_human_pose="/UAV1HumanRelativePose";

double odom_time=-1;
double first_human_odom_time=-1;

// --------------------------------------------------------------------------
inline double
SIGN_SQRT( double d )
// --------------------------------------------------------------------------
{
	bool sign = (d < 0 );
	if ( sign )
		return -sqrt( -d );
	else
		return sqrt( d );
}

// --------------------------------------------------------------------------
void sendPose(ros::Publisher pub, AgentMappingInfo agent_estimation, std::string str_from, std::string str_to, std::string str_frame_id)
// --------------------------------------------------------------------------
{
	AgentMappingInfo::iterator it_agent_from;
	AgentMappingInfo::iterator it_agent_to;
	it_agent_from=agent_estimation.find(str_from);
	it_agent_to=agent_estimation.find(str_to);
	if(it_agent_from!=agent_estimation.end() && it_agent_to!=agent_estimation.end())
	{

		tf::Pose from_pose;
		from_pose.setOrigin( tf::Vector3(it_agent_from->second.position.x,it_agent_from->second.position.y,it_agent_from->second.position.z) );
		tf::Quaternion from_q;
		from_q.setRPY(it_agent_from->second.position.theta_x, it_agent_from->second.position.theta_y, it_agent_from->second.position.theta);
		from_pose.setRotation( from_q );

		tf::Pose to_pose;
		to_pose.setOrigin( tf::Vector3(it_agent_to->second.position.x,it_agent_to->second.position.y,it_agent_to->second.position.z) );
		tf::Quaternion to_q;
		to_q.setRPY(it_agent_to->second.position.theta_x, it_agent_to->second.position.theta_y, it_agent_to->second.position.theta);
		to_pose.setRotation( from_q );

		tf::Pose relative_pose=from_pose.inverse()*to_pose;

		tf::Quaternion relative_q=relative_pose.getRotation();
		tf::Vector3 relative_pose_v=relative_pose.getOrigin();	


		geometry_msgs::PoseWithCovarianceStamped pose_with_timestamp;
		//position in x,y, and z
		pose_with_timestamp.pose.pose.position.x = relative_pose_v.x();
		pose_with_timestamp.pose.pose.position.y = relative_pose_v.y();
		pose_with_timestamp.pose.pose.position.z = relative_pose_v.z();

    	        //rotation
		pose_with_timestamp.pose.pose.orientation.x = relative_q.x();
		pose_with_timestamp.pose.pose.orientation.y = relative_q.y();
		pose_with_timestamp.pose.pose.orientation.z = relative_q.z();
		pose_with_timestamp.pose.pose.orientation.w = relative_q.w();

		pose_with_timestamp.header.stamp=ros::Time(it_agent_from->second.timestamp);

		pose_with_timestamp.header.frame_id=str_frame_id;
		pub.publish(pose_with_timestamp);
	}

}


// --------------------------------------------------------------------------
void plot_all_samples()
//plot particle in 2D
// --------------------------------------------------------------------------
{
	std::string cmd;//( "set size ratio 1\n");

	cmd+="set grid\n";
	cmd+="set xlabel 'x(m)'\n";
	cmd+="set ylabel 'y(m)'\n";
	cmd+="set title 'Particle states'\n";
	cmd+="set size ratio -1\n";

	AgentMappingInfo::iterator it;

	cmd+="set key bottom right height 0.5 width -2\n";


	cmd+="unset arrow\n";


	for(it = m_Agent.begin(); it != m_Agent.end(); ++it)
	{

		ParticleFilter_UWB * pf=it->second.particle_filter;
		for(int i=0;i<pf->size();i=i+1)
		{
			pf2::Particle6D *particle_sample = pf->at(i);
			carmen_point_t from;

			carmen_point_t to;

			from.x=particle_sample->x;
			from.y=particle_sample->y;

			to.x=particle_sample->x+0.7*cos(particle_sample->theta_z);
			to.y=particle_sample->y+0.7*sin(particle_sample->theta_z);

			cmd += "set arrow from "+toString(from.x)+ ','+toString(from.y)+','+toString(0)+" to " + toString(to.x)+ ','+toString(to.y)+','+toString(0) + " lc -1 lw 1\n";
		}

	}


	cmd+="plot ["+toString(m_dBinningMinX)+':'+toString(m_dBinningMaxX)+"]["+toString(m_dBinningMinY)+':'+toString(m_dBinningMaxY)+"] ";

	int line_style=1;

	//set the line colors
	for(it = m_Agent.begin(); it != m_Agent.end(); ++it)
	{
		cmd+="'-' u 1:2 w p pt 6 ps 1 lc "+toString(line_style)+" ti '"+it->first+"',";
		line_style=line_style+1;
	}
	cmd += "\n";

	//fill in the particle locations
	for(it = m_Agent.begin(); it != m_Agent.end(); ++it)
	{
		//output the measurements
		ParticleFilter_UWB * pf=it->second.particle_filter;
		for(int i=0;i<pf->size();i=i+1)
		{
			pf2::Particle6D *particle_sample = pf->at(i);
			cmd += toString( particle_sample->x ) + ' ' + toString( particle_sample->y ) + ' ' + toString( 0.5 ) + '\n';
		}
		cmd += "e\n";
	}
	m_plot_samples->commandStr( cmd );

}


// --------------------------------------------------------------------------
void plot_raw_odom()
//plot the raw odometry
// --------------------------------------------------------------------------
{
	std::string cmd;//( "set size ratio 1\n");

	cmd+="set grid\n";
	cmd+="set xlabel 'x(m)'\n";
	cmd+="set ylabel 'y(m)'\n";
	cmd+="set title 'Raw Odom'\n";
	cmd+="set size ratio -1\n";
	cmd+="unset arrow\n";

	AgentMappingInfo::iterator it;
	for(it = m_Agent.begin(); it != m_Agent.end(); ++it)
	{
		//output the measurements
		std::vector< carmen_6d_point_t > trajectories=it->second.raw_odom;
		std::cout<<it->first<<" "<<trajectories.size()<<std::endl;
		int plot_start=trajectories.size()-100;
		if(plot_start<=0)
			plot_start=0;

		for(int i=0;i<trajectories.size();i=i+1)
		{

			carmen_point_t from;

			carmen_point_t to;

			from.x=trajectories[i].x;
			from.y=trajectories[i].y;

			to.x=from.x+0.3*cos(trajectories[i].theta);
			to.y=from.y+0.3*sin(trajectories[i].theta);

			//cmd += "set arrow from "+toString(from.x)+ ','+toString(from.y)+','+toString(0)+" to " + toString(to.x)+ ','+toString(to.y)+','+toString(0) + " lc -1 lw 1\n";
		}
	}



	cmd+="set key bottom right height 0.5 width -2\n";


	cmd+="plot ["+toString(m_dBinningMinX)+':'+toString(m_dBinningMaxX)+"]["+toString(m_dBinningMinY)+':'+toString(m_dBinningMaxY)+"] ";

	int line_style=1;

	//set the line colors
	for(it = m_Agent.begin(); it != m_Agent.end(); ++it)
	{
		cmd+="'-' u 1:2 w lp lw 1 pt 6 ps 1 lc "+toString(line_style)+" ti '"+it->first+"',";
		line_style=line_style+1;
	}
	cmd += "\n";

	for(it = m_Agent.begin(); it != m_Agent.end(); ++it)
	{
		//output the measurements
		std::vector< carmen_6d_point_t > trajectories=it->second.raw_odom;
		int plot_start=trajectories.size()-100;
		if(plot_start<=0)
			plot_start=0;
		for(int i=0;i<trajectories.size();i=i+1)
		{
			cmd += toString( trajectories[i].x ) + ' ' + toString( trajectories[i].y ) + ' ' + toString( 0.5 ) + '\n';
		}
		cmd += "e\n";
	}
	m_plot_odom->commandStr( cmd );

}


// --------------------------------------------------------------------------
void publish_poses(std::vector< std::string > agent_list)
//publish the updated pose
// --------------------------------------------------------------------------
{

	for(int i=0;i<agent_list.size();i++)
	{

		AgentMappingInfo::iterator it_agent;
		it_agent=m_Agent.find(agent_list[i]);
		if(it_agent!=m_Agent.end())
		{
			//we find this object and now we need to publish this message
			geometry_msgs::PoseWithCovarianceStamped pose_with_timestamp;
			pose_with_timestamp.header.stamp=ros::Time(it_agent->second.timestamp);
			pose_with_timestamp.header.frame_id=m_frame_id;

			tf2::Quaternion pose_quaternion;
			pose_quaternion.setRPY( it_agent->second.position.theta_x, it_agent->second.position.theta_y, it_agent->second.position.theta);


			//robot's position in x,y, and z
			pose_with_timestamp.pose.pose.position.x = it_agent->second.position.x;
			pose_with_timestamp.pose.pose.position.y = it_agent->second.position.y;
			pose_with_timestamp.pose.pose.position.z = it_agent->second.position.z;


			pose_with_timestamp.pose.pose.orientation.x = pose_quaternion.x();
			pose_with_timestamp.pose.pose.orientation.y = pose_quaternion.y();
			pose_with_timestamp.pose.pose.orientation.z = pose_quaternion.z();
			pose_with_timestamp.pose.pose.orientation.w = pose_quaternion.w();

			pose_with_timestamp.pose.covariance[0*6+0] = it_agent->second.variance_xx;
			pose_with_timestamp.pose.covariance[1*6+1] = it_agent->second.variance_yy;
			pose_with_timestamp.pose.covariance[5*6+5] = it_agent->second.variance_theta;

			carmen_6d_point_t published_pose;

			published_pose.x=it_agent->second.position.x;
			published_pose.y=it_agent->second.position.y;
			published_pose.z=it_agent->second.position.z;
			published_pose.theta=it_agent->second.position.theta;
			published_pose.theta_x=it_agent->second.position.theta_x;
			published_pose.theta_y=it_agent->second.position.theta_y;
			m_Agent[it_agent->first].trajectories_low_frequency.push_back(published_pose);

			m_Agent[it_agent->first].timestamps_low_frequency.push_back(it_agent->second.timestamp);

			geometry_msgs::TransformStamped pose_tf_stamped;			

			//robot's position in x,y, and z
			pose_tf_stamped.transform.translation.x = it_agent->second.position.x;
			pose_tf_stamped.transform.translation.y = it_agent->second.position.y;
			pose_tf_stamped.transform.translation.z = it_agent->second.position.z;
			//robot's heading in quaternion
			pose_tf_stamped.transform.rotation.x = pose_quaternion.x();
			pose_tf_stamped.transform.rotation.y = pose_quaternion.y();
			pose_tf_stamped.transform.rotation.z = pose_quaternion.z();
			pose_tf_stamped.transform.rotation.w = pose_quaternion.w();
			pose_tf_stamped.header.stamp = ros::Time(it_agent->second.timestamp);


			if(it_agent->first=="drone1")
			{
				pub_uav1_pose.publish(pose_with_timestamp);	

			}
			else if(it_agent->first=="human1")
			{
				pub_human_pose.publish(pose_with_timestamp);
			}

		}

	}
}
// --------------------------------------------------------------------------
void perturbLiuAndWest(ParticleFilter_UWB *particleFilter,
		double discountFactor,
		const Pose& mean,
		const double covariance[36] )
// --------------------------------------------------------------------------
{	
	// Mixture ratio (called 'a' in Joho et al., 2009)
	double a = ( 3*discountFactor - 1 ) / ( 2 * discountFactor );
	double h_square = 1.0 - a*a;
	double rot_xx      = h_square * covariance[ 0*6 + 0 ];
	double rot_yy      = h_square * covariance[ 1*6 + 1 ];
	double rot_xy      = h_square * covariance[ 0*6 + 1 ];
	double rot_yx      = h_square * covariance[ 1*6 + 0 ];
	double rot_zz      = h_square * covariance[ 2*6 + 2 ];
	double var_theta_z = h_square * covariance[ 5*6 + 5 ];

	double x, y, z, theta_z, weight;
	double _theta_zx, _theta_zy; // temporary variables
	double random_x, random_y, random_z; // temporary variables
	double noise_x,  noise_y,  noise_z;
	const double equal_weight = 1.0 / particleFilter->size();
	for ( uint i = 0; i < particleFilter->size(); ++i )
	{
		x=particleFilter->at(i)->x;
		y=particleFilter->at(i)->y;
		theta_z=particleFilter->at(i)->theta_z;
		weight=particleFilter->at(i)->weight;

		// Compute deterministic mixture based on particle
		x = a * x + (1-a) * mean.x;
		y = a * y + (1-a) * mean.y;
		z = a * z + (1-a) * mean.z;
		_theta_zx = a * cos( theta_z ) + (1-a) * cos( mean.theta_z );
		_theta_zy = a * sin( theta_z ) + (1-a) * sin( mean.theta_z );
		theta_z = atan2( _theta_zy, _theta_zx );

		// Perturb particle position
		double random_xx = carmen_gaussian_random( 0.0, 1.0 );
		double random_yy = carmen_gaussian_random( 0.0, 1.0 );
		double random_xy = carmen_gaussian_random( 0.0, 1.0 );
		double random_yx = carmen_gaussian_random( 0.0, 1.0 );
		double random_z = carmen_gaussian_random( 0.0, 1.0 );
		noise_x = rot_xx * random_xx + rot_xy * random_xy;
		noise_y = rot_yx * random_yx + rot_yy * random_yy;

		noise_z = rot_zz * random_z; // covariance with X and Y not considered
		x       += SIGN_SQRT( noise_x );
		y       += SIGN_SQRT( noise_y );
		z       += SIGN_SQRT( noise_z );
		theta_z += SIGN_SQRT( carmen_gaussian_random( 0.0, 1.0 ) * var_theta_z );
		// Update weight
		(*particleFilter)[i].x = x;
		(*particleFilter)[i].y = y;
		(*particleFilter)[i].z = z;
		(*particleFilter)[i].theta_z = theta_z;
		(*particleFilter)[i].weight = weight;
	}
}


// --------------------------------------------------------------------------
void updateParticleWeights(std::map< std::string, std::vector< RangingMeasurement > > agent_measurements, double timestamp, int prediction)
//update the particle weights
// --------------------------------------------------------------------------
{
	Pose mean;
	double covariance[36];


	//we use augmented pf to determine the pose
	std::map< std::string, std::vector< RangingMeasurement > >::iterator it_agent_measurements;
	for(it_agent_measurements = agent_measurements.begin(); it_agent_measurements != agent_measurements.end(); ++it_agent_measurements)
	{
		std::string agent_name=it_agent_measurements->first;
		std::vector< RangingMeasurement > measurements=it_agent_measurements->second;
		//std::cout<<agent_name<<" "<<measurements.size()<<std::endl;

		AgentMappingInfo::iterator it_agent_drone;
		it_agent_drone=m_Agent.find(agent_name);
		if(it_agent_drone!=m_Agent.end())
		{
			//update the weights of the pf
			ParticleFilter_UWB * drone_PF=m_Agent[it_agent_drone->first].particle_filter;

			if(prediction==1)
			{
				if(measurements.size()<1)
					continue;
				
				for(int u1=0;u1<drone_PF->size();u1++)
				{
					pf2::Particle6D *particle_u1 = drone_PF->at(u1);
					particle_u1->x=particle_u1->x+carmen_gaussian_random(0,0.1);
					particle_u1->y=particle_u1->y+carmen_gaussian_random(0,0.1);
					if(m_enable_z_correction)
					{
						particle_u1->z=particle_u1->z+carmen_gaussian_random(0,0.1);
					}					
					particle_u1->theta_z=particle_u1->theta_z+carmen_gaussian_random(0,0.005);
					particle_u1->theta_z=carmen_normalize_theta(particle_u1->theta_z);

				}

			}
			for(int u1=0;u1<drone_PF->size();u1++)
			{
				pf2::Particle6D *particle_u1 = drone_PF->at(u1);

				double weight=1.0;				
				double sum_weight=0.0;				
				int count=0;
				for(int k=0;k<measurements.size();k++)
				{

					std::string tag_id=measurements[k].tag_id;
					std::string anchor_id=measurements[k].anchor_id;
					double range=measurements[k].range;
					std::map <std::string, std::string > ::iterator it_peer_drone;
					it_peer_drone=m_tag_objects.find(anchor_id);

					std::map <std::string, std::string > ::iterator it_drone;
					it_drone=m_tag_objects.find(tag_id);

					if(it_peer_drone!=m_tag_objects.end()&&it_drone!=m_tag_objects.end())
					{
						std::string peer_agent_name=it_peer_drone->second;

						//we find the peer agent 
						AgentMappingInfo::iterator it_agent_peer_drone;

						it_agent_peer_drone=m_Agent.find(peer_agent_name);
						if(it_agent_peer_drone!=m_Agent.end())
						{						

							double cov_xx=it_agent_peer_drone->second.variance_xx;
							double cov_yy=it_agent_peer_drone->second.variance_yy;
							double denominator=1.0/(2.0*m_sigma*m_sigma);
							double pre_factor=1.0/(m_sigma*sqrt(2*M_PI));

							//now we get the tag configure
							carmen_6d_point_t tag_relative_drone=m_agent_config[agent_name].tag_information[tag_id];
							carmen_6d_point_t tag_relative_peer_drone=m_agent_config[peer_agent_name].tag_information[anchor_id];
							carmen_6d_point_t peer_position=m_Agent[peer_agent_name].position;


							double weight_to_mean=0;	

							double p_theta_drone=particle_u1->theta_z;							
							double p_x_drone=particle_u1->x+tag_relative_drone.x*cos(p_theta_drone)-tag_relative_drone.y*sin(p_theta_drone);;
							double p_y_drone=particle_u1->y+tag_relative_drone.x*sin(p_theta_drone)+tag_relative_drone.y*cos(p_theta_drone);;
							double p_z_drone=particle_u1->z+tag_relative_drone.z;


							double x_tag_peer_drone=peer_position.x+tag_relative_peer_drone.x*cos(peer_position.theta)-tag_relative_peer_drone.y*sin(peer_position.theta);;
							double y_tag_peer_drone=peer_position.y+tag_relative_peer_drone.x*sin(peer_position.theta)+tag_relative_peer_drone.y*cos(peer_position.theta);;
							double z_tag_peer_drone=peer_position.z+tag_relative_peer_drone.z;

							double distance_to_mean=sqrt((p_x_drone-x_tag_peer_drone)*(p_x_drone-x_tag_peer_drone) + 
									(p_y_drone-y_tag_peer_drone)*(p_y_drone-y_tag_peer_drone)+
									(p_z_drone-z_tag_peer_drone)*(p_z_drone-z_tag_peer_drone));

							//std::cout<<"p:"<<u1<<" "<<k<<" "<<range<<", xyz: "<<agent_name<<" "<<p_x_drone<<" "<<p_y_drone<<" "<<p_z_drone<<", peer xyz: "<<peer_agent_name<<" "<<x_tag_peer_drone<<" "<<y_tag_peer_drone <<" "<<z_tag_peer_drone<<std::endl;
							double dis_diff=distance_to_mean-range;
							weight_to_mean=exp( -dis_diff*dis_diff*denominator );


							if(weight_to_mean<m_minLikelihood)
							{
								weight_to_mean=m_minLikelihood;
							}
							else if(weight_to_mean>m_maxLikelihood)
							{
								weight_to_mean=m_maxLikelihood;
							}


							weight=weight*weight_to_mean;
							sum_weight=sum_weight+weight_to_mean;
							count=count+1;

						}
					}



				}

				//double final_weight=pow(weight,1.0/((double)(count)));
				double final_weight=sum_weight/((double)(count));
				//double final_weight=weight;
				//std::cout<<agent_name<<" "<<u1<<" "<<particle_u1->x<<" "<<particle_u1->y<<" "<<particle_u1->z<<" "<<final_weight<<" "<<weight<<" "<<count<<std::endl;
				if(final_weight<m_minLikelihood)
				{
					final_weight=m_minLikelihood;
				}
				else if(final_weight>m_maxLikelihood)
				{
					final_weight=m_maxLikelihood;
				}

				particle_u1->weight *= final_weight;

			}

			//normalize
			drone_PF->normalize();

			//resample if needed
			if (drone_PF->getEffectiveSampleSize() < drone_PF->size()/2 )
			{
				//	std::cout<<"resample"<<std::endl;
				drone_PF->resample();	
			}	


			drone_PF->computeUnweightedMeanAndCovariance( mean, covariance);

			if(prediction==1)
			{				
				//perturbLiuAndWest(drone_PF,0.98, mean, covariance);
			}

			carmen_6d_point_t estimate_drone;
			estimate_drone.x=mean.x;
			estimate_drone.y=mean.y;
			estimate_drone.z=mean.z;
			estimate_drone.theta=mean.theta_z;
			estimate_drone.theta_x=mean.theta_x;
			estimate_drone.theta_y=mean.theta_y;


			m_Agent[agent_name].trajectories.push_back(estimate_drone);
			m_Agent[agent_name].v_timestamps.push_back(timestamp);
			m_Agent[agent_name].position.x=estimate_drone.x;
			m_Agent[agent_name].position.y=estimate_drone.y;
			m_Agent[agent_name].position.z=estimate_drone.z;
			m_Agent[agent_name].position.theta=estimate_drone.theta;
			m_Agent[agent_name].position.theta_x=estimate_drone.theta_x;
			m_Agent[agent_name].position.theta_y=estimate_drone.theta_y;

			m_Agent[agent_name].variance_xx=covariance[0*6+0];
			m_Agent[agent_name].variance_yy=covariance[1*6+1];
			m_Agent[agent_name].variance_theta=covariance[5*6+5];


			m_Agent[agent_name].timestamp=timestamp;
		}

	}




}

// --------------------------------------------------------------------------
void processUWBMeasurements(double timestamp, long int uwb_system_timestamp)
// --------------------------------------------------------------------------
{	

	ros::Time start_time=ros::Time::now();
	double time_constrain=0.25;
	//1 check the moving distance of the robot moved
	std::vector< std::string > agentsList;

	std::vector< TempUWB > tempUWBMeasures;

	AgentMappingInfo::iterator it_agent;
	for(it_agent = m_Agent.begin(); it_agent != m_Agent.end(); ++it_agent)
	{
		//we check the accumulated odom
		if(it_agent->second.accu_dis-it_agent->second.pre_accu_dis>m_update_min_d
				||it_agent->second.accu_angle-it_agent->second.pre_accu_angle>m_update_min_a)
		{
			agentsList.push_back(it_agent->first);
		}
	}

	if(agentsList.size()==0)
		return;
	//2 get the peer list to be updated
	for(int i=0;i<agentsList.size();i++)
	{
		for(it_agent = m_Agent.begin(); it_agent != m_Agent.end(); ++it_agent)
		{
			if(it_agent->first!=agentsList[i])
			{
				//agent(agentsList[i])--> peer agent (it_agent->first)
				std::string agent_name=agentsList[i];
				std::string peer_agent_name=it_agent->first;

				std::map< std::string, ObjectConfig >::iterator it_agent_config;
				std::map< std::string, ObjectConfig >::iterator it_peer_agent_config;

				it_agent_config=m_agent_config.find(agent_name);
				it_peer_agent_config=m_agent_config.find(peer_agent_name);
				if(it_agent_config!=m_agent_config.end() && it_peer_agent_config!=m_agent_config.end())
				{
					//we get the tag list to peer anchor list 
					std::map <std::string, carmen_6d_point_t >::iterator it_tag_list;
					std::map <std::string, carmen_6d_point_t >::iterator it_anchor_list;
					for(it_tag_list = it_agent_config->second.tag_information.begin(); it_tag_list != it_agent_config->second.tag_information.end(); ++it_tag_list)
					{
						for(it_anchor_list = it_peer_agent_config->second.tag_information.begin(); it_anchor_list != it_peer_agent_config->second.tag_information.end(); ++it_anchor_list)
						{
							TempUWB temp_uwb;
							temp_uwb.tag_id=it_tag_list->first;
							temp_uwb.anchor_id=it_anchor_list->first;
							tempUWBMeasures.push_back(temp_uwb);
						}


					}


				}

			}
		}
	}

	//3 compose the distance of UWB from the historical uwb measurements, we treat 1->2 and 2->3 the same and take the average		
	for(int i=0;i<v_nl_uwb_packets.size();i++)
	{
		NoopLoopUWBPacket uwb_packet=v_nl_uwb_packets[i];

		if(fabs(v_nl_uwb_packets[i].system_time-uwb_system_timestamp)<time_constrain*1000.0)
		{
			std::map<std::string, double> uwb_rangings=uwb_packet.v_ranging;
			std::map<std::string, double> uwb_rss_difference=uwb_packet.v_rss_difference;//Absolute RSS difference be rxRSS and fpRSS

			std::map<std::string, double>::iterator it_ranging;
			for(it_ranging = uwb_rangings.begin(); it_ranging != uwb_rangings.end(); ++it_ranging)
			{
				//we push back the measures to the temp uwb
				for(int j=0;j<tempUWBMeasures.size();j++)
				{
					TempUWB temp_m=tempUWBMeasures[j];		
					if((temp_m.tag_id==uwb_packet.tag_id&&temp_m.anchor_id==it_ranging->first)
							||(temp_m.tag_id==it_ranging->first&&temp_m.anchor_id==uwb_packet.tag_id))				
					{				
						tempUWBMeasures[j].v_range.push_back(it_ranging->second);	
						//	std::cout<<fabs(v_nl_uwb_packets[i].system_time-uwb_system_timestamp)<<" "<<uwb_packet.tag_id<<" "<< it_ranging->first <<" "<<it_ranging->second<<" "<<uwb_rss_difference[it_ranging->first]<<std::endl;
					}

				}
			}		


		}

	}

	//4 we compute the avarage distance
	for(int j=0;j<tempUWBMeasures.size();j++)
	{
		TempUWB temp_m=tempUWBMeasures[j];
		std::vector<double> v_ranging_values=temp_m.v_range;
		double sum=0;
		double mean=0;

		double sum_variance=0;
		double variance=0;
		for(int k=0;k<v_ranging_values.size();k++)
		{
			sum=sum+v_ranging_values[k];
		}
		mean=sum/((double)(v_ranging_values.size()));


		for(int k=0;k<v_ranging_values.size();k++)
		{
			sum_variance=sum_variance+(v_ranging_values[k]-mean)*(v_ranging_values[k]-mean);
		}
		variance=sqrt(sum_variance/((double)(v_ranging_values.size())));
		if(v_ranging_values.size()>0)
		{
			tempUWBMeasures[j].range=mean;
			tempUWBMeasures[j].variance=variance;
		}

	}


	std::map< std::string, int > tag_list;

	//7 we update the accumulated odom
	for(int i=0;i<agentsList.size();i++)
	{
		//std::cout<<"list:"<<agentsList[i]<<std::endl;	
	}

	std::vector< TempAgent > updated_agents_list;

	std::map< std::string, int > agentsUpdateList;

	std::map< std::string, std::vector< RangingMeasurement > > raw_agent_measurements;


	std::map< std::string, std::vector< RangingMeasurement > >::iterator it_agent_measurements;


	std::map< std::string, std::vector< RangingMeasurement > > agent_measurements;


	for(int j=0;j<tempUWBMeasures.size();j++)
	{
		TempUWB temp_m=tempUWBMeasures[j];
		if(temp_m.v_range.size()>0)
		{

			RangingMeasurement measurement;
			measurement.tag_id=temp_m.tag_id;
			measurement.anchor_id=temp_m.anchor_id;
			measurement.timestamp=timestamp;
			measurement.range=temp_m.range;		
			std::string agent_name=m_tag_objects[measurement.tag_id];
			std::string peer_agent_name=m_tag_objects[measurement.anchor_id];

			if(m_uwb_ranging_filtering>=0)
			{
				//we need to check the variance to see if it is a line of sight or non-line-of-sight
				if(temp_m.variance<m_uwb_ranging_filtering)
				{					
					raw_agent_measurements[agent_name].push_back(measurement);
					std::cout<<j<<", moving: "<<agent_name<<" "<<peer_agent_name<<" "<<measurement.tag_id<<" "<<measurement.anchor_id<<" "<<temp_m.range<<" "<<temp_m.variance<<" "<<temp_m.v_range.size()<<std::endl;

				}
				else
				{
					//hmmm, bad luck it is it is a NLOS measurement
					ROS_WARN("NLOS (moving): %d %s %s %s %s %f %f %d",j, agent_name.c_str(), peer_agent_name.c_str(), measurement.tag_id.c_str(), measurement.anchor_id.c_str(), temp_m.range, temp_m.variance, temp_m.v_range.size());				

				}
			}
			else
			{
				raw_agent_measurements[agent_name].push_back(measurement);
				std::cout<<j<<" "<<agent_name<<" "<<peer_agent_name<<" "<<measurement.tag_id<<" "<<measurement.anchor_id<<" "<<temp_m.range<<" "<<temp_m.variance<<" "<<temp_m.v_range.size()<<std::endl;
			}

		}


	}





	if(m_use_min_ranging>0)
	{
		//use the min ranging
		//we need to take the min distance, and ignore the large distance values
		for(it_agent_measurements = raw_agent_measurements.begin(); it_agent_measurements != raw_agent_measurements.end(); ++it_agent_measurements)
		{
			std::string agent_name=it_agent_measurements->first;
			std::vector< RangingMeasurement > v_ranging=it_agent_measurements->second;

			std::map< std::string, std::vector< RangingMeasurement > > peer_drone_measurements;

			for(int j=0;j<v_ranging.size();j++)
			{
				RangingMeasurement measurement;
				measurement.tag_id=v_ranging[j].tag_id;
				measurement.anchor_id=v_ranging[j].anchor_id;
				measurement.timestamp=v_ranging[j].timestamp;
				measurement.range=v_ranging[j].range;		
				std::string agent_name=m_tag_objects[v_ranging[j].tag_id];
				std::string peer_agent_name=m_tag_objects[v_ranging[j].anchor_id];

				peer_drone_measurements[peer_agent_name].push_back(measurement);


			}

			std::map< std::string, std::vector< RangingMeasurement > >::iterator it_peer_agent_measurements;

			for(it_peer_agent_measurements = peer_drone_measurements.begin(); it_peer_agent_measurements != peer_drone_measurements.end(); ++it_peer_agent_measurements)
			{
				std::vector< RangingMeasurement > v_peer_ranging=it_peer_agent_measurements->second;


				double min_ranging=INFINITY;
				int min_ranging_index=-1;
				for(int j=0;j<v_peer_ranging.size();j++)
				{
					if(v_peer_ranging[j].range<min_ranging)
					{
						min_ranging=v_peer_ranging[j].range;
						min_ranging_index=j;
					}
				}

				if(min_ranging_index>=0)
				{
					//we find the best index here, now we need to push it for sensor fusion
					RangingMeasurement measurement;
					measurement.tag_id=v_peer_ranging[min_ranging_index].tag_id;
					measurement.anchor_id=v_peer_ranging[min_ranging_index].anchor_id;
					measurement.timestamp=v_peer_ranging[min_ranging_index].timestamp;
					measurement.range=v_peer_ranging[min_ranging_index].range;		
					std::string agent_name=m_tag_objects[v_peer_ranging[min_ranging_index].tag_id];
					std::string peer_agent_name=m_tag_objects[v_peer_ranging[min_ranging_index].anchor_id];

					agent_measurements[agent_name].push_back(measurement);

					agentsUpdateList[agent_name]=agentsUpdateList[agent_name]+1;
					agentsUpdateList[peer_agent_name]=agentsUpdateList[peer_agent_name]+1;			

					std::cout<<"minimum ranging:"<<min_ranging_index<<" "<<agent_name<<" "<<peer_agent_name<<" "<<measurement.tag_id<<" "<<measurement.anchor_id<<" "<<measurement.range<<std::endl;

				}

			}


		}
	}
	else
	{
		//use all rangings
		for(it_agent_measurements = raw_agent_measurements.begin(); it_agent_measurements != raw_agent_measurements.end(); ++it_agent_measurements)
		{
			std::vector< RangingMeasurement > v_ranging=it_agent_measurements->second;

			for(int j=0;j<v_ranging.size();j++)
			{
				RangingMeasurement measurement;
				measurement.tag_id=v_ranging[j].tag_id;
				measurement.anchor_id=v_ranging[j].anchor_id;
				measurement.timestamp=v_ranging[j].timestamp;
				measurement.range=v_ranging[j].range;		
				std::string agent_name=m_tag_objects[v_ranging[j].tag_id];
				std::string peer_agent_name=m_tag_objects[v_ranging[j].anchor_id];
				agent_measurements[it_agent_measurements->first].push_back(measurement);

				agentsUpdateList[agent_name]=agentsUpdateList[agent_name]+1;
				agentsUpdateList[peer_agent_name]=agentsUpdateList[peer_agent_name]+1;		

			}
		}


	}



	//5 we do update of the weights of the particles
	updateParticleWeights(agent_measurements, timestamp, 0);	

	//6 we publish the robot pose as well
	publish_poses(agentsList);
	//	if(agentsList.size()>0)
	//	std::cout<<std::endl;	

	//7 we update the accumulated odom
	for(int i=0;i<agentsList.size();i++)
	{
		it_agent=m_Agent.find(agentsList[i]);
		if(it_agent!=m_Agent.end())
		{
			m_Agent[it_agent->first].pre_accu_dis=m_Agent[it_agent->first].accu_dis;
			m_Agent[it_agent->first].pre_accu_angle=m_Agent[it_agent->first].accu_angle;
			/*
			std::map< std::string, int >::iterator it_agentsUpdateList;

			it_agentsUpdateList=agentsUpdateList.find(it_agent->first);
			if(it_agentsUpdateList!=agentsUpdateList.end())
			{
				m_Agent[it_agent->first].pre_accu_dis=m_Agent[it_agent->first].accu_dis;
				m_Agent[it_agent->first].pre_accu_angle=m_Agent[it_agent->first].accu_angle;
			}
			 */

		}

	}

	ros::Time end_time=ros::Time::now();
	ROS_WARN("Time for particle weighting: %f", end_time.toSec()-start_time.toSec());
}


// --------------------------------------------------------------------------
void staticUWBMeasurementsUpdate(double timestamp, long int uwb_system_timestamp)
// --------------------------------------------------------------------------
{
	ros::Time start_time=ros::Time::now();

	double time_constrain=0.25;
	//1 check the moving distance of the robot moved
	std::vector< std::string > agentsList;

	std::vector< TempUWB > tempUWBMeasures;

	AgentMappingInfo::iterator it_agent;
	for(it_agent = m_Agent.begin(); it_agent != m_Agent.end(); ++it_agent)
	{
		//we check the drone odom
		if(it_agent->second.accu_dis-it_agent->second.pre_accu_dis<m_update_min_d
				&&it_agent->second.accu_angle-it_agent->second.pre_accu_angle<m_update_min_a)
		{			
			agentsList.push_back(it_agent->first);
		}	
	}

	//2 get the peer list to be updated
	for(int i=0;i<agentsList.size();i++)
	{
		for(it_agent = m_Agent.begin(); it_agent != m_Agent.end(); ++it_agent)
		{
			if(it_agent->first!=agentsList[i])
			{
				//agent(agentsList[i])--> peer agent (it_agent->first)
				std::string agent_name=agentsList[i];
				std::string peer_agent_name=it_agent->first;

				std::map< std::string, ObjectConfig >::iterator it_agent_config;
				std::map< std::string, ObjectConfig >::iterator it_peer_agent_config;

				it_agent_config=m_agent_config.find(agent_name);
				it_peer_agent_config=m_agent_config.find(peer_agent_name);
				if(it_agent_config!=m_agent_config.end() && it_peer_agent_config!=m_agent_config.end())
				{
					//we get the tag list to peer anchor list 
					std::map <std::string, carmen_6d_point_t >::iterator it_tag_list;
					std::map <std::string, carmen_6d_point_t >::iterator it_anchor_list;
					for(it_tag_list = it_agent_config->second.tag_information.begin(); it_tag_list != it_agent_config->second.tag_information.end(); ++it_tag_list)
					{
						for(it_anchor_list = it_peer_agent_config->second.tag_information.begin(); it_anchor_list != it_peer_agent_config->second.tag_information.end(); ++it_anchor_list)
						{
							TempUWB temp_uwb;
							temp_uwb.tag_id=it_tag_list->first;
							temp_uwb.anchor_id=it_anchor_list->first;
							tempUWBMeasures.push_back(temp_uwb);
						}


					}


				}

			}
		}
	}

	//3 compose the distance of UWB from the historical uwb measurements, we treat 1->2 and 2->3 the same and take the average		
	for(int i=0;i<v_nl_uwb_packets.size();i++)
	{
		NoopLoopUWBPacket uwb_packet=v_nl_uwb_packets[i];

		if(fabs(v_nl_uwb_packets[i].system_time-uwb_system_timestamp)<time_constrain*1000.0)
		{
			std::map<std::string, double> uwb_rangings=uwb_packet.v_ranging;
			std::map<std::string, double> uwb_rss_difference=uwb_packet.v_rss_difference;//Absolute RSS difference be rxRSS and fpRSS
			std::map<std::string, double>::iterator it_ranging;
			for(it_ranging = uwb_rangings.begin(); it_ranging != uwb_rangings.end(); ++it_ranging)
			{
				//we push back the measures to the temp uwb
				for(int j=0;j<tempUWBMeasures.size();j++)
				{
					TempUWB temp_m=tempUWBMeasures[j];		
					if((temp_m.tag_id==uwb_packet.tag_id&&temp_m.anchor_id==it_ranging->first)
							||(temp_m.tag_id==it_ranging->first&&temp_m.anchor_id==uwb_packet.tag_id))				
					{				
						tempUWBMeasures[j].v_range.push_back(it_ranging->second);	
						//	std::cout<<fabs(v_nl_uwb_packets[i].system_time-uwb_system_timestamp)<<" "<<uwb_packet.tag_id<<" "<< it_ranging->first <<" "<<it_ranging->second<<" "<<uwb_rss_difference[it_ranging->first]<<std::endl;
					}

				}
			}		


		}

	}


	//4 we compute the avarage distance
	for(int j=0;j<tempUWBMeasures.size();j++)
	{
		TempUWB temp_m=tempUWBMeasures[j];
		std::vector<double> v_ranging_values=temp_m.v_range;
		double sum=0;
		double mean=0;

		double sum_variance=0;
		double variance=0;
		for(int k=0;k<v_ranging_values.size();k++)
		{
			sum=sum+v_ranging_values[k];
		}
		mean=sum/((double)(v_ranging_values.size()));


		for(int k=0;k<v_ranging_values.size();k++)
		{
			sum_variance=sum_variance+(v_ranging_values[k]-mean)*(v_ranging_values[k]-mean);
		}
		variance=sqrt(sum_variance/((double)(v_ranging_values.size())));
		if(v_ranging_values.size()>0)
		{
			tempUWBMeasures[j].range=mean;
			tempUWBMeasures[j].variance=variance;
		}

	}


	std::map< std::string, int > tag_list;

	//7 we update the accumulated odom
	for(int i=0;i<agentsList.size();i++)
	{
		//std::cout<<"list:"<<agentsList[i]<<std::endl;	
	}

	std::vector< TempAgent > updated_agents_list;

	std::map< std::string, int > agentsUpdateList;

	std::map< std::string, std::vector< RangingMeasurement > > raw_agent_measurements;

	std::map< std::string, std::vector< RangingMeasurement > > agent_measurements;
	std::map< std::string, std::vector< RangingMeasurement > >::iterator it_agent_measurements;

	for(int j=0;j<tempUWBMeasures.size();j++)
	{
		TempUWB temp_m=tempUWBMeasures[j];
		if(temp_m.v_range.size()>0)
		{

			RangingMeasurement measurement;
			measurement.tag_id=temp_m.tag_id;
			measurement.anchor_id=temp_m.anchor_id;
			measurement.timestamp=timestamp;
			measurement.range=temp_m.range;		
			std::string agent_name=m_tag_objects[measurement.tag_id];
			std::string peer_agent_name=m_tag_objects[measurement.anchor_id];

			if(m_uwb_ranging_filtering>=0)
			{
				if(temp_m.variance<m_uwb_ranging_filtering)
				{
					raw_agent_measurements[agent_name].push_back(measurement);			


					std::cout<<j<<", static: "<<agent_name<<" "<<peer_agent_name<<" "<<measurement.tag_id<<" "<<measurement.anchor_id<<" "<<temp_m.range<<" "<<temp_m.variance<<" "<<temp_m.v_range.size()<<std::endl;

				}
				else
				{						
					ROS_WARN("NLOS (static): %d %s %s %s %s %f %f %d",j, agent_name.c_str(), peer_agent_name.c_str(), measurement.tag_id.c_str(), measurement.anchor_id.c_str(), temp_m.range, temp_m.variance, temp_m.v_range.size());				
				}
			}
			else
			{
				raw_agent_measurements[agent_name].push_back(measurement);
				std::cout<<j<<", static: "<<agent_name<<" "<<peer_agent_name<<" "<<measurement.tag_id<<" "<<measurement.anchor_id<<" "<<temp_m.range<<" "<<temp_m.variance<<" "<<temp_m.v_range.size()<<std::endl;
			}

		}


	}

	if(m_use_min_ranging>0)
	{
		//we need to take the min distance, and ignore the large distance values
		for(it_agent_measurements = raw_agent_measurements.begin(); it_agent_measurements != raw_agent_measurements.end(); ++it_agent_measurements)
		{
			std::string agent_name=it_agent_measurements->first;
			std::vector< RangingMeasurement > v_ranging=it_agent_measurements->second;

			std::map< std::string, std::vector< RangingMeasurement > > peer_drone_measurements;

			for(int j=0;j<v_ranging.size();j++)
			{
				RangingMeasurement measurement;
				measurement.tag_id=v_ranging[j].tag_id;
				measurement.anchor_id=v_ranging[j].anchor_id;
				measurement.timestamp=v_ranging[j].timestamp;
				measurement.range=v_ranging[j].range;		
				std::string agent_name=m_tag_objects[v_ranging[j].tag_id];
				std::string peer_agent_name=m_tag_objects[v_ranging[j].anchor_id];

				peer_drone_measurements[peer_agent_name].push_back(measurement);


			}

			std::map< std::string, std::vector< RangingMeasurement > >::iterator it_peer_agent_measurements;

			for(it_peer_agent_measurements = peer_drone_measurements.begin(); it_peer_agent_measurements != peer_drone_measurements.end(); ++it_peer_agent_measurements)
			{
				std::vector< RangingMeasurement > v_peer_ranging=it_peer_agent_measurements->second;


				double min_ranging=INFINITY;
				int min_ranging_index=-1;
				for(int j=0;j<v_peer_ranging.size();j++)
				{
					if(v_peer_ranging[j].range<min_ranging)
					{
						min_ranging=v_peer_ranging[j].range;
						min_ranging_index=j;
					}
				}

				if(min_ranging_index>=0)
				{
					//we find the best index here, now we need to push it for sensor fusion
					RangingMeasurement measurement;
					measurement.tag_id=v_peer_ranging[min_ranging_index].tag_id;
					measurement.anchor_id=v_peer_ranging[min_ranging_index].anchor_id;
					measurement.timestamp=v_peer_ranging[min_ranging_index].timestamp;
					measurement.range=v_peer_ranging[min_ranging_index].range;		
					std::string agent_name=m_tag_objects[v_peer_ranging[min_ranging_index].tag_id];
					std::string peer_agent_name=m_tag_objects[v_peer_ranging[min_ranging_index].anchor_id];

					agent_measurements[agent_name].push_back(measurement);

					agentsUpdateList[agent_name]=agentsUpdateList[agent_name]+1;
					agentsUpdateList[peer_agent_name]=agentsUpdateList[peer_agent_name]+1;			

					std::cout<<"minimum ranging:"<<min_ranging_index<<" "<<agent_name<<" "<<peer_agent_name<<" "<<measurement.tag_id<<" "<<measurement.anchor_id<<" "<<measurement.range<<std::endl;



				}

			}


		}

	}
	else
	{
		//use all rangings
		for(it_agent_measurements = raw_agent_measurements.begin(); it_agent_measurements != raw_agent_measurements.end(); ++it_agent_measurements)
		{
			std::vector< RangingMeasurement > v_ranging=it_agent_measurements->second;



			for(int j=0;j<v_ranging.size();j++)
			{
				RangingMeasurement measurement;
				measurement.tag_id=v_ranging[j].tag_id;
				measurement.anchor_id=v_ranging[j].anchor_id;
				measurement.timestamp=v_ranging[j].timestamp;
				measurement.range=v_ranging[j].range;		
				std::string agent_name=m_tag_objects[v_ranging[j].tag_id];
				std::string peer_agent_name=m_tag_objects[v_ranging[j].anchor_id];
				agent_measurements[it_agent_measurements->first].push_back(measurement);


				agentsUpdateList[agent_name]=agentsUpdateList[agent_name]+1;
				agentsUpdateList[peer_agent_name]=agentsUpdateList[peer_agent_name]+1;		
			}
		}

	}



	//5 we do update of the weights of the particles, with random prediction
	updateParticleWeights(agent_measurements, timestamp, 1);	

	//6 we publish the robot pose as well
	publish_poses(agentsList);
	if(agentsList.size()>0)
		std::cout<<std::endl;


	ros::Time end_time=ros::Time::now();


	//std::cout<<"time consumed:"<<end_time.toSec()-start_time.toSec()<<std::endl;


}

// --------------------------------------------------------------------------
void particleInitializationBasedOnUWB(std::string agent_name, double uwb_time, carmen_point_t initial_position)
// --------------------------------------------------------------------------
{	
	Pose mean;
	double covariance[36];

	//we check if the PF is initialized
	std::map< std::string, ObjectConfig >::iterator it_drone_config;
	for(it_drone_config = m_drone_config.begin(); it_drone_config != m_drone_config.end(); ++it_drone_config)
	{
		std::string stateName="drone"+it_drone_config->first;
		double height=it_drone_config->second.height;
		if(stateName==agent_name)
		{
			AgentMappingInfo::iterator it_agent;
			it_agent=m_Agent.find(stateName);
			if(it_agent==m_Agent.end())
			{
				//initialize the particle filter
				m_Agent[stateName].particle_filter=new ParticleFilter_UWB();

				ParticleFilter_UWB * pf=m_Agent[stateName].particle_filter;
				m_Agent[stateName].timestamp=0;
				pf2::Particle6D p;

				//initialize the particle filter, the particle filter is initialized uniformly in the environment
				for ( int i = 0; i < m_num_samples; ++i )
				{

					p.x=initial_position.x+carmen_uniform_random(-m_init_range_uncertainty*0.5,m_init_range_uncertainty*0.5);
					p.y =initial_position.y+ carmen_uniform_random(-m_init_range_uncertainty*0.5,m_init_range_uncertainty*0.5);

					if(m_enable_z_correction>0)
					{
						p.z = height+carmen_uniform_random(-m_initial_z_noise*0.5,m_initial_z_noise*0.5);	
					}
					else
					{
						p.z = height;
					}

					p.theta_z = initial_position.theta+carmen_uniform_random(-m_init_angle_uncertainty*0.5,m_init_angle_uncertainty*0.5);
					p.theta_x = 0;
					p.theta_y = 0;

					p.weight = 1.0/(double)m_num_samples;
					p.theta_z=carmen_normalize_theta(p.theta_z);
					pf->addParticle(p);
				}
				pf->normalize();
				pf->computeMeanAndCovariance( mean, covariance );

				carmen_6d_point_t estimate_uav;
				estimate_uav.x=mean.x;
				estimate_uav.y=mean.y;
				estimate_uav.z=mean.z;
				estimate_uav.theta=mean.theta_z;
				estimate_uav.theta_x=mean.theta_x;
				estimate_uav.theta_y=mean.theta_y;
			
				

				m_Agent[stateName].trajectories.push_back(estimate_uav);
				m_Agent[stateName].v_timestamps.push_back(uwb_time);

				m_Agent[stateName].position.x=estimate_uav.x;
				m_Agent[stateName].position.y=estimate_uav.y;
				m_Agent[stateName].position.z=estimate_uav.z;
				m_Agent[stateName].position.theta=estimate_uav.theta;
				m_Agent[stateName].position.theta_x=estimate_uav.theta_x;
				m_Agent[stateName].position.theta_y=estimate_uav.theta_y;

				m_Agent[stateName].variance_xx=covariance[0*6+0];
				m_Agent[stateName].variance_yy=covariance[1*6+1];
				m_Agent[stateName].variance_theta=covariance[5*6+5];

				m_Agent[stateName].timestamp=uwb_time;


			}

		}


	}
}


// --------------------------------------------------------------------------
//UAV1 UWB call back
void uav1UWBCallback(const nlink_parser::LinktrackNodeframe2::ConstPtr& msg)
// --------------------------------------------------------------------------
{ 		
	double uwb_time=msg->header.stamp.toSec();
	long int latest_uwb_system_timestamp=-1;

	if(first_human_odom_time<0)
	{
		return;
	}


	if(odom_time>0){

		total_uwb_uav1=total_uwb_uav1+1;

		double delay_time=fabs(odom_time-uwb_time);
		total_uwb_delay_uav1=total_uwb_delay_uav1+delay_time;

		if(delay_time>m_time_constraints)
		{
			invalid_uwb_uav1=invalid_uwb_uav1+1;			
		}

	}
	else
	{
		return;
	}

	particleInitializationBasedOnUWB("drone1",uwb_time, m_initial_uav1);

	std::string node_id=std::to_string(msg->id);
	long int system_time=msg->systemTime;
	long int local_time=msg->localTime;

	//fill in the UWB measurements
	NoopLoopUWBPacket uwb_packet;
	uwb_packet.timestamp=uwb_time;
	uwb_packet.local_time=local_time;
	uwb_packet.system_time=system_time;
	uwb_packet.tag_id=node_id;

	if(system_time>=0)
	{
		latest_uwb_system_timestamp=system_time;	
	}


	for (size_t i = 0; i < msg->node.size(); i++)
	{
		std::string anchor_node_id=std::to_string(msg->node[i].id);
		double distance=msg->node[i].dis;
		double rxRSS=msg->node[i].rxRssi;
		double fpRSS=msg->node[i].fpRssi;

		double rss_diff=fabs(fpRSS-rxRSS);

		if(distance>=0&&distance<=100)
		{
			uwb_packet.v_ranging[anchor_node_id]=distance;
			uwb_packet.v_rss_difference[anchor_node_id]=rss_diff;	
		}	

	}

	v_nl_uwb_packets.push_back(uwb_packet);

	if(latest_uwb_system_timestamp>=0)
	{
		processUWBMeasurements(uwb_time,latest_uwb_system_timestamp);
		double selective_ratio = carmen_uniform_random(0,1);
		if(selective_ratio<=m_static_update_ratio)
		{
			staticUWBMeasurementsUpdate(uwb_time,latest_uwb_system_timestamp);
		}
	}

}


// --------------------------------------------------------------------------
//Human UWB call back
void humanUWBCallback(const nlink_parser::LinktrackNodeframe2::ConstPtr& msg)
// --------------------------------------------------------------------------
{
	double uwb_time=msg->header.stamp.toSec();
	long int latest_uwb_system_timestamp=-1;

	if(first_human_odom_time<0)
	{
		return;
	}

	if(odom_time>0){

		total_uwb_human=total_uwb_human+1;

		double delay_time=fabs(odom_time-uwb_time);
		total_uwb_delay_human=total_uwb_delay_human+delay_time;

		if(delay_time>m_time_constraints)
		{
			invalid_uwb_human=invalid_uwb_human+1;			
		}

	}
	else
	{
		return;
	}


	std::string node_id=std::to_string(msg->id);
	long int system_time=msg->systemTime;
	long int local_time=msg->localTime;

	//fill in the UWB measurements
	NoopLoopUWBPacket uwb_packet;
	uwb_packet.timestamp=uwb_time;
	uwb_packet.local_time=local_time;
	uwb_packet.system_time=system_time;
	uwb_packet.tag_id=node_id;

	if(system_time>=0)
	{
		latest_uwb_system_timestamp=system_time;	
	}


	for (size_t i = 0; i < msg->node.size(); i++)
	{
		std::string anchor_node_id=std::to_string(msg->node[i].id);
		double distance=msg->node[i].dis;
		double rxRSS=msg->node[i].rxRssi;
		double fpRSS=msg->node[i].fpRssi;

		double rss_diff=fabs(fpRSS-rxRSS);

		if(distance>=0&&distance<=100)
		{
			uwb_packet.v_ranging[anchor_node_id]=distance;
			uwb_packet.v_rss_difference[anchor_node_id]=rss_diff;	
		}	

	}

	v_nl_uwb_packets.push_back(uwb_packet);

	if(latest_uwb_system_timestamp>=0)
	{
		processUWBMeasurements(uwb_time,latest_uwb_system_timestamp);	

		double selective_ratio = carmen_uniform_random(0,1);
		if(selective_ratio<=m_static_update_ratio)
		{
			staticUWBMeasurementsUpdate(uwb_time,latest_uwb_system_timestamp);
		}
	}

}



// --------------------------------------------------------------------------
//human odom call back
void humanOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
// --------------------------------------------------------------------------
{

	double ratio = carmen_uniform_random(0,1);
	if(ratio>m_human_odom_downsample_ratio)
		return;

	odom_time=msg->header.stamp.toSec();

	if(first_human_odom_time<0)
	{
		first_human_odom_time=odom_time;
	}

	if(first_human_odom_time<0)
	{
		return;
	}	

	Pose mean;
	double covariance[36];

	num_human_odom=num_human_odom+1;

	double x_odom=msg->pose.pose.position.x;
	double y_odom=msg->pose.pose.position.y;
	double z_odom=msg->pose.pose.position.z;

	//we get the yaw angle of the estimation
	tf::Quaternion q_odom(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	double roll_odom, pitch_odom, yaw_odom;
	tf::Matrix3x3(q_odom).getRPY(roll_odom, pitch_odom, yaw_odom);

	if(m_force_human_2d>0)
	{
		roll_odom=0;
		pitch_odom=0;	
		z_odom=0;
	}

	//we need to check if human particles are initialized
	std::map< std::string, ObjectConfig >::iterator it_human_config;

	for(it_human_config = m_human_config.begin(); it_human_config != m_human_config.end(); ++it_human_config)
	{
		std::string stateName="human"+it_human_config->first;
		AgentMappingInfo::iterator it_agent_human;
		it_agent_human=m_Agent.find(stateName);
		if(it_agent_human==m_Agent.end())
		{
			//initialize the particle filter
			//std::cout<<"particle filter initialize:"<<" "<<m.timestamp<<" "<<m.id<<std::endl;
			m_Agent[stateName].particle_filter=new ParticleFilter_UWB();

			ParticleFilter_UWB * pf=m_Agent[stateName].particle_filter;
			m_Agent[stateName].timestamp=0;
			pf2::Particle6D p;

			//initialize the particle filter, the particle filter is initialized uniformly in the environment
			for ( int i = 0; i < m_num_samples; ++i )
			{

				p.x=m_initial_human.x+carmen_uniform_random(-m_init_range_uncertainty*0.5,m_init_range_uncertainty*0.5);
				p.y =m_initial_human.y+carmen_uniform_random(-m_init_range_uncertainty*0.5,m_init_range_uncertainty*0.5);

				//check if we have to do the z estimation
				if(m_enable_z_correction>0)
				{
					p.z = it_human_config->second.height + carmen_uniform_random(-m_initial_z_noise*0.5,m_initial_z_noise*0.5);	
				}
				else
				{
					p.z = it_human_config->second.height;
				}


				p.theta_z = m_initial_human.theta+carmen_uniform_random(-m_init_angle_uncertainty*0.5,m_init_angle_uncertainty*0.5);

				p.theta_x = roll_odom;
				p.theta_y = pitch_odom;			


				p.weight = 1.0/(double)m_num_samples;
				p.theta_z=carmen_normalize_theta(p.theta_z);
				pf->addParticle(p);
			}
			pf->normalize();
			pf->computeMeanAndCovariance( mean, covariance );

		}

	}

	//std::cout<<msg->pose.pose.orientation.x<<" "<<msg->pose.pose.orientation.y<<" "<<msg->pose.pose.orientation.z<<" "<<msg->pose.pose.orientation.w<<" "<<roll_odom<<" "<<pitch_odom<<" "<<yaw_odom<<" "<<sqrt(msg->pose.pose.orientation.x*msg->pose.pose.orientation.x+msg->pose.pose.orientation.y*msg->pose.pose.orientation.y+msg->pose.pose.orientation.z*msg->pose.pose.orientation.z+msg->pose.pose.orientation.w*msg->pose.pose.orientation.w)<<std::endl;

	if(human_odom_available<0)
	{
		//we are not doing prediction of first odom measure
		human_odom_available=1;
		human_odom_previous_time=msg->header.stamp;
		human_visualization_previous_time=msg->header.stamp;


		previous_human_odom.x=x_odom;
		previous_human_odom.y=y_odom;
		previous_human_odom.z=z_odom;
		previous_human_odom.theta_z=yaw_odom;
		previous_human_odom.theta_x=roll_odom;
		previous_human_odom.theta_y=pitch_odom;

	}
	else
	{

		tf::Pose pose_diff;

		tf::Pose previous_human_pose;
		previous_human_pose.setOrigin( tf::Vector3(previous_human_odom.x, previous_human_odom.y, previous_human_odom.z) );
		tf::Quaternion pervious_human_q;
		pervious_human_q.setRPY( previous_human_odom.theta_x, previous_human_odom.theta_y, previous_human_odom.theta_z);
		previous_human_pose.setRotation( pervious_human_q );


		tf::Pose current_human_pose;
		current_human_pose.setOrigin( tf::Vector3(x_odom, y_odom, z_odom) );
		tf::Quaternion current_human_q;
		current_human_q.setRPY( roll_odom, pitch_odom, yaw_odom);
		current_human_pose.setRotation( current_human_q );

		pose_diff=previous_human_pose.inverse()*current_human_pose;
		tf::Quaternion q=pose_diff.getRotation();

		double relative_roll, relative_pitch, relative_yaw;
		tf::Matrix3x3(q).getRPY(relative_roll, relative_pitch, relative_yaw);
		tf::Vector3 pose_diff_d=pose_diff.getOrigin();	

		AgentMappingInfo::iterator it_agent_human;

		for(it_agent_human = m_Agent.begin(); it_agent_human != m_Agent.end(); ++it_agent_human)
		{
			std::string subStr=it_agent_human->first.substr(0,5);
			double human_height=0;
			//we find the configure as well
			std::map< std::string, ObjectConfig >::iterator it_agent_config;


			it_agent_config=m_agent_config.find(it_agent_human->first);
			if(it_agent_config!=m_agent_config.end())
			{
				human_height=it_agent_config->second.height;
			}



			if(subStr=="human")
			{
				//we do prediction here

				ParticleFilter_UWB * human_PF=m_Agent[it_agent_human->first].particle_filter;

				double actual_delta_z=pose_diff_d.z();
				double actual_delta_d=sqrt(pose_diff_d.x()*pose_diff_d.x()+pose_diff_d.y()*pose_diff_d.y()+pose_diff_d.z()*pose_diff_d.z());
				double actual_delta_xy=sqrt(pose_diff_d.x()*pose_diff_d.x()+pose_diff_d.y()*pose_diff_d.y());

				double actual_delta_theta=relative_yaw;
				actual_delta_theta=carmen_normalize_theta(actual_delta_theta);

				m_Agent[it_agent_human->first].accu_dis=m_Agent[it_agent_human->first].accu_dis+actual_delta_d;
				m_Agent[it_agent_human->first].accu_angle=m_Agent[it_agent_human->first].accu_angle+fabs(actual_delta_theta);

				//std::cout<<"(debug) human odom: "<<roll_odom<<" "<<pitch_odom<<" "<<yaw_odom<<" "<<pose_diff_d.x()<<" "<<pose_diff_d.y()<<" "<<pose_diff_d.z()<<std::endl;

				//we predict human movement
				for(int u=0;u<human_PF->size();u++)
				{
					pf2::Particle6D *current_particle = human_PF->at(u);

					tf::Pose current_particle_pose;
					current_particle_pose.setOrigin( tf::Vector3(current_particle->x, current_particle->y, current_particle->z) );
					tf::Quaternion particle_q;
					particle_q.setRPY(current_particle->theta_x, current_particle->theta_y, current_particle->theta_z);					
					current_particle_pose.setRotation( particle_q );
					

					tf::Pose particle_pose_motion;					
					particle_pose_motion=current_particle_pose*pose_diff;

					tf::Vector3 v_transformed=particle_pose_motion.getOrigin();
					tf::Quaternion q_transformed=particle_pose_motion.getRotation();

					double roll_transformed, pitch_transformed, yaw_transformed;
					tf::Matrix3x3(q_transformed).getRPY(roll_transformed, pitch_transformed, yaw_transformed);

					double delta_x=pose_diff_d.x();
					double delta_y=pose_diff_d.y();
					double delat_xy=sqrt(delta_x*delta_x+delta_y*delta_y);

					double fast_ratio = carmen_uniform_random(0,1);
					if(ratio<=m_fast_motion_ratio)
					{
						//predict with fast motion, 5 times of normal noise just in movement in x and y
						double xx_noise=delta_x*carmen_gaussian_random(0,5.0*m_human_odom_motion_noise);
						double yy_noise=delta_y*carmen_gaussian_random(0,5.0*m_human_odom_motion_noise);

						double xy_cross_noise=delta_y*carmen_gaussian_random(0,m_human_odom_cross_motion_noise);
						double yx_cross_noise=delta_x*carmen_gaussian_random(0,m_human_odom_cross_motion_noise);

						double theta_to_theta_noise=fabs(actual_delta_theta)*carmen_gaussian_random(0,m_human_odom_orientation_noise);

						double theta_to_d_noise=delat_xy*carmen_gaussian_random(0,m_human_odom_theta_d_noise);

						current_particle->x=v_transformed.x()+xx_noise+xy_cross_noise;
						current_particle->y=v_transformed.y()+yy_noise+yx_cross_noise;

						//check if we do the z prediction as well
						if(m_enable_z_correction>0)
						{
							current_particle->z=v_transformed.z()+actual_delta_z*carmen_gaussian_random(0,5.0*m_noise_z);
						}
						else
						{
							current_particle->z=v_transformed.z();
						}

						current_particle->theta_x=roll_odom;
						current_particle->theta_y=pitch_odom;

						current_particle->theta_z=current_particle->theta_z+actual_delta_theta+theta_to_theta_noise+theta_to_d_noise;										

						current_particle->theta_z=carmen_normalize_theta(current_particle->theta_z);

					}
					else
					{
						//predict with normal motion
						double xx_noise=delta_x*carmen_gaussian_random(0,m_human_odom_motion_noise);
						double yy_noise=delta_y*carmen_gaussian_random(0,m_human_odom_motion_noise);

						double xy_cross_noise=delta_y*carmen_gaussian_random(0,m_human_odom_cross_motion_noise);
						double yx_cross_noise=delta_x*carmen_gaussian_random(0,m_human_odom_cross_motion_noise);

						double theta_to_theta_noise=fabs(actual_delta_theta)*carmen_gaussian_random(0,m_human_odom_orientation_noise);


						double theta_to_d_noise=delat_xy*carmen_gaussian_random(0,m_human_odom_theta_d_noise);


						current_particle->x=v_transformed.x()+xx_noise+xy_cross_noise;
						current_particle->y=v_transformed.y()+yy_noise+yx_cross_noise;

						//check if we do the z prediction as well
						if(m_enable_z_correction>0)
						{
							current_particle->z=v_transformed.z()+actual_delta_z*carmen_gaussian_random(0,m_noise_z);
						}
						else
						{
							current_particle->z=v_transformed.z();
						}

						current_particle->theta_x=roll_odom;
						current_particle->theta_y=pitch_odom;

						current_particle->theta_z=current_particle->theta_z+actual_delta_theta+theta_to_theta_noise+theta_to_d_noise;					
						current_particle->theta_z=carmen_normalize_theta(current_particle->theta_z);
					}


				}

				//get the mean and covariance
				human_PF->computeUnweightedMeanAndCovariance( mean, covariance);				

				carmen_6d_point_t estimate_human;
				estimate_human.x=mean.x;
				estimate_human.y=mean.y;
				estimate_human.z=mean.z;
				estimate_human.theta=mean.theta_z;
				estimate_human.theta_x=mean.theta_x;
				estimate_human.theta_y=mean.theta_y;


				carmen_6d_point_t human_odom;
				human_odom.x=x_odom;
				human_odom.y=y_odom;
				human_odom.z=z_odom;
				human_odom.theta=yaw_odom;

				m_Agent[it_agent_human->first].raw_odom.push_back(human_odom);
				m_Agent[it_agent_human->first].trajectories.push_back(estimate_human);
				m_Agent[it_agent_human->first].v_timestamps.push_back(odom_time);

				m_Agent[it_agent_human->first].position.x=estimate_human.x;
				m_Agent[it_agent_human->first].position.y=estimate_human.y;
				m_Agent[it_agent_human->first].position.z=estimate_human.z;
				m_Agent[it_agent_human->first].position.theta=estimate_human.theta;
				m_Agent[it_agent_human->first].position.theta_x=estimate_human.theta_x;
				m_Agent[it_agent_human->first].position.theta_y=estimate_human.theta_y;

				m_Agent[it_agent_human->first].variance_xx=covariance[0*6+0];
				m_Agent[it_agent_human->first].variance_yy=covariance[1*6+1];		
				m_Agent[it_agent_human->first].variance_theta=covariance[5*6+5];	

				m_Agent[it_agent_human->first].timestamp=odom_time;


				//publish human odom
				geometry_msgs::PoseWithCovarianceStamped pose_with_timestamp;
				pose_with_timestamp.header.stamp=ros::Time(odom_time);
				pose_with_timestamp.header.frame_id=m_frame_id;
				tf2::Quaternion pose_quaternion;
				pose_quaternion.setRPY( mean.theta_x, mean.theta_y, mean.theta_z);
				//robot's position in x,y, and z
				pose_with_timestamp.pose.pose.position.x = mean.x;
				pose_with_timestamp.pose.pose.position.y = mean.y;
				pose_with_timestamp.pose.pose.position.z = mean.z;
				//set the quaternion to be published
				pose_with_timestamp.pose.pose.orientation.x = pose_quaternion.x();
				pose_with_timestamp.pose.pose.orientation.y = pose_quaternion.y();
				pose_with_timestamp.pose.pose.orientation.z = pose_quaternion.z();
				pose_with_timestamp.pose.pose.orientation.w = pose_quaternion.w();


				pose_with_timestamp.pose.covariance[0*6+0] = covariance[0*6+0];
				pose_with_timestamp.pose.covariance[1*6+1] = covariance[1*6+1];
				pose_with_timestamp.pose.covariance[5*6+5] = covariance[5*6+5];


				pub_human_pose.publish(pose_with_timestamp);

				
				double time_diff = (msg->header.stamp - human_odom_previous_time).toSec ();

				double time_diff_visulization = (msg->header.stamp - human_visualization_previous_time).toSec ();


				//we dowsample the odom for visualization
				if(time_diff>0.5)
				{
					//std::cout<<it_agent_human->first<<", human: roll:"<<roll_odom<<",pitch:"<<pitch_odom<<",yaw:"<<yaw_odom<<std::endl;
					m_Agent[it_agent_human->first].trajectories_low_frequency.push_back(estimate_human);
					m_Agent[it_agent_human->first].timestamps_low_frequency.push_back(odom_time);
					human_odom_previous_time=msg->header.stamp;
				}

				//we dowsample the odom for visualization
				if(time_diff_visulization>1.0)
				{	
					if(m_enable_visualization>0)
					{

						//plot_all_samples();
						//plot_raw_odom();


						//plot_3D_tracks();							
					}

					human_visualization_previous_time=msg->header.stamp;

					printf("UWB (UAV1, Human), invalid: %d %d, total: %d %d, average delay (s): %f %f\n",invalid_uwb_uav1, invalid_uwb_human,total_uwb_uav1,total_uwb_human, total_uwb_delay_uav1/((double)(total_uwb_uav1)), total_uwb_delay_human/((double)(total_uwb_human)));
				}


			}

		}


		previous_human_odom.x=x_odom;
		previous_human_odom.y=y_odom;
		previous_human_odom.z=z_odom;
		previous_human_odom.theta_z=yaw_odom;
		previous_human_odom.theta_x=roll_odom;
		previous_human_odom.theta_y=pitch_odom;
	}

}



// --------------------------------------------------------------------------
//uav1 odom
void uav1OdomCallBack(const nav_msgs::Odometry::ConstPtr& msg)
// --------------------------------------------------------------------------
{

	double uav_odom_time=msg->header.stamp.toSec();	

	if(first_human_odom_time<0)
	{
		return;
	}


	if(odom_time>0){

		total_odom_uav1=total_odom_uav1+1;

		double delay_time=fabs(odom_time-uav_odom_time);
		total_odom_delay_uav1=total_odom_delay_uav1+delay_time;

		if(delay_time>m_time_constraints)
		{
			invalid_odom_uav1=invalid_odom_uav1+1;			
		}

	}
	else
	{
		return;
	}

	double ratio = carmen_uniform_random(0,1);
	if(ratio>m_uav_odom_downsample_ratio)
		return;

	Pose mean;
	double covariance[36];


	tf::Quaternion q_odom(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	double roll_odom, pitch_odom, yaw_odom;//theta_x, theta_y, theta_z
	tf::Matrix3x3(q_odom).getRPY(roll_odom, pitch_odom, yaw_odom);

	//we check if the PF is initialized
	std::map< std::string, ObjectConfig >::iterator it_drone_config;
	for(it_drone_config = m_drone_config.begin(); it_drone_config != m_drone_config.end(); ++it_drone_config)
	{
		std::string stateName="drone"+it_drone_config->first;
		if(stateName=="drone1")
		{
			AgentMappingInfo::iterator it_agent;
			it_agent=m_Agent.find(stateName);
			if(it_agent==m_Agent.end())
			{
				//initialize the particle filter
				m_Agent[stateName].particle_filter=new ParticleFilter_UWB();

				ParticleFilter_UWB * pf=m_Agent[stateName].particle_filter;
				m_Agent[stateName].timestamp=0;
				pf2::Particle6D p;

				//initialize the particle filter, the particle filter is initialized uniformly in the environment
				for ( int i = 0; i < m_num_samples; ++i )
				{

					p.x=m_initial_uav1.x+carmen_uniform_random(-m_init_range_uncertainty*0.5,m_init_range_uncertainty*0.5);
					p.y =m_initial_uav1.y + carmen_uniform_random(-m_init_range_uncertainty*0.5,m_init_range_uncertainty*0.5);

					if(m_enable_z_correction>0)
					{
						p.z = it_drone_config->second.height+carmen_uniform_random(-m_initial_z_noise*0.5,m_initial_z_noise*0.5);	
					}
					else
					{
						p.z = it_drone_config->second.height;
					}

					p.theta_z = m_initial_uav1.theta+carmen_uniform_random(-m_init_angle_uncertainty*0.5,m_init_angle_uncertainty*0.5);
					p.theta_x = 0;
					p.theta_y = 0;

					p.weight = 1.0/(double)m_num_samples;
					p.theta_z=carmen_normalize_theta(p.theta_z);
					pf->addParticle(p);
				}
				pf->normalize();
				pf->computeMeanAndCovariance( mean, covariance );

			}

		}


	}



	if(uav1_odom_available<0)
	{
		//we are not doing prediction of first odom measure
		uav1_odom_available=1;

		previous_uav1_odom.x=msg->pose.pose.position.x;
		previous_uav1_odom.y=msg->pose.pose.position.y;
		previous_uav1_odom.z=msg->pose.pose.position.z;
		previous_uav1_odom.theta=yaw_odom;
		previous_uav1_odom.theta_x=roll_odom;
		previous_uav1_odom.theta_y=pitch_odom;
		uav1_odom_previous_time=msg->header.stamp;

	}
	else
	{

		AgentMappingInfo::iterator it_agent_uav;

		tf::Pose pose_diff;

		tf::Pose previous_uav_pose;
		previous_uav_pose.setOrigin( tf::Vector3(previous_uav1_odom.x, previous_uav1_odom.y, previous_uav1_odom.z) );
		tf::Quaternion pervious_uav_q;
		pervious_uav_q.setRPY( previous_uav1_odom.theta_x, previous_uav1_odom.theta_y, previous_uav1_odom.theta_z);
		previous_uav_pose.setRotation( pervious_uav_q );


		tf::Pose current_uav_pose;
		current_uav_pose.setOrigin( tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z) );
		tf::Quaternion current_uav_q;
		current_uav_q.setRPY(roll_odom, pitch_odom, yaw_odom);
		current_uav_pose.setRotation( current_uav_q );

		pose_diff=previous_uav_pose.inverse()*current_uav_pose;
		tf::Quaternion q=pose_diff.getRotation();
		double relative_roll, relative_pitch, relative_yaw;
		tf::Matrix3x3(q).getRPY(relative_roll, relative_pitch, relative_yaw);
		tf::Vector3 pose_diff_d=pose_diff.getOrigin();	


		for(it_agent_uav = m_Agent.begin(); it_agent_uav != m_Agent.end(); ++it_agent_uav)
		{			
			std::string subStr=it_agent_uav->first.substr(0,6);
			if(subStr=="drone1")
			{
				//we do prediction here

				ParticleFilter_UWB * uav_pf=m_Agent[it_agent_uav->first].particle_filter;

				double actual_delta_z=pose_diff_d.z();
				double actual_delta_d=sqrt(pose_diff_d.x()*pose_diff_d.x()+pose_diff_d.y()*pose_diff_d.y()+pose_diff_d.z()*pose_diff_d.z());
				double actual_delta_xy=sqrt(pose_diff_d.x()*pose_diff_d.x()+pose_diff_d.y()*pose_diff_d.y());
				double actual_delta_theta=relative_yaw;
				actual_delta_theta=carmen_normalize_theta(actual_delta_theta);

				m_Agent[it_agent_uav->first].accu_dis=m_Agent[it_agent_uav->first].accu_dis+actual_delta_d;
				m_Agent[it_agent_uav->first].accu_angle=m_Agent[it_agent_uav->first].accu_angle+fabs(actual_delta_theta);

				//we predict drone movement
				for(int u=0;u<uav_pf->size();u++)
				{
					pf2::Particle6D *current_particle = uav_pf->at(u);

					tf::Pose particle_pose;
					particle_pose.setOrigin( tf::Vector3(current_particle->x, current_particle->y, current_particle->z) );
					tf::Quaternion particle_q;
					particle_q.setRPY( current_particle->theta_x, current_particle->theta_y, current_particle->theta_z);
					particle_pose.setRotation( particle_q );					

					tf::Pose new_particle_pose;
					new_particle_pose=particle_pose*pose_diff;					

					tf::Vector3 new_particles_d=new_particle_pose.getOrigin();	


					double delta_x=pose_diff_d.x();
					double delta_y=pose_diff_d.y();
					double delat_xy=sqrt(delta_x*delta_x+delta_y*delta_y);

					double xx_noise=delta_x*carmen_gaussian_random(0,m_odom_motion_noise);
					double yy_noise=delta_y*carmen_gaussian_random(0,m_odom_motion_noise);

					double xy_cross_noise=delta_y*carmen_gaussian_random(0,m_odom_cross_motion_noise);
					double yx_cross_noise=delta_y*carmen_gaussian_random(0,m_odom_cross_motion_noise);

					double theta_to_theta_noise=fabs(actual_delta_theta)*carmen_gaussian_random(0,m_odom_orientation_noise);

					double theta_to_d_noise=delat_xy*carmen_gaussian_random(0,m_odom_theta_d_noise);


					current_particle->x=new_particles_d.x()+xx_noise+xy_cross_noise;
					current_particle->y=new_particles_d.y()+yy_noise+yx_cross_noise;

					if(m_enable_z_correction>0)
					{
						current_particle->z=new_particles_d.z()+actual_delta_z*carmen_gaussian_random(0,m_noise_z);						
					}
					else
					{
						current_particle->z=new_particles_d.z();
					}

					current_particle->theta_x=roll_odom;
					current_particle->theta_y=pitch_odom;
					current_particle->theta_z=current_particle->theta_z+actual_delta_theta+theta_to_theta_noise+theta_to_d_noise;
					current_particle->theta_z=carmen_normalize_theta(current_particle->theta_z);

					//std::cout<<u<<" "<<delta_d<<",dx:"<<delta_d*cos(current_particle->theta_z)<<" "<<xx_noise<<" "<<xy_cross_noise<<", dy:"<<delta_d*sin(current_particle->theta_z)<<" "<<yy_noise<<" "<<yx_cross_noise<<std::endl;


				}

				//get the mean and covariance
				uav_pf->computeUnweightedMeanAndCovariance( mean, covariance);				

				carmen_6d_point_t estimate_uav;
				estimate_uav.x=mean.x;
				estimate_uav.y=mean.y;
				estimate_uav.z=mean.z;
				estimate_uav.theta=mean.theta_z;
				estimate_uav.theta_x=mean.theta_x;
				estimate_uav.theta_y=mean.theta_y;


				carmen_6d_point_t uav_odom;
				uav_odom.x=msg->pose.pose.position.x;
				uav_odom.y=msg->pose.pose.position.y;
				uav_odom.z=msg->pose.pose.position.z;
				uav_odom.theta=yaw_odom;
				m_Agent[it_agent_uav->first].raw_odom.push_back(uav_odom);

				m_Agent[it_agent_uav->first].trajectories.push_back(estimate_uav);
				m_Agent[it_agent_uav->first].v_timestamps.push_back(uav_odom_time);

				m_Agent[it_agent_uav->first].position.x=estimate_uav.x;
				m_Agent[it_agent_uav->first].position.y=estimate_uav.y;
				m_Agent[it_agent_uav->first].position.z=estimate_uav.z;
				m_Agent[it_agent_uav->first].position.theta=estimate_uav.theta;
				m_Agent[it_agent_uav->first].position.theta_x=estimate_uav.theta_x;
				m_Agent[it_agent_uav->first].position.theta_y=estimate_uav.theta_y;


				m_Agent[it_agent_uav->first].variance_xx=covariance[0*6+0];
				m_Agent[it_agent_uav->first].variance_yy=covariance[1*6+1];
				m_Agent[it_agent_uav->first].variance_theta=covariance[5*6+5];

				m_Agent[it_agent_uav->first].timestamp=uav_odom_time;



				//publish uav1 odom
				geometry_msgs::PoseWithCovarianceStamped pose_with_timestamp;
				pose_with_timestamp.header.stamp=ros::Time(uav_odom_time);
				pose_with_timestamp.header.frame_id=m_frame_id;
				tf2::Quaternion pose_quaternion;
				pose_quaternion.setRPY( mean.theta_x, mean.theta_y, mean.theta_z);
				//robot's position in x,y, and z
				pose_with_timestamp.pose.pose.position.x = mean.x;
				pose_with_timestamp.pose.pose.position.y = mean.y;
				pose_with_timestamp.pose.pose.position.z = mean.z;
				//set the quaternion to be published
				pose_with_timestamp.pose.pose.orientation.x = pose_quaternion.x();
				pose_with_timestamp.pose.pose.orientation.y = pose_quaternion.y();
				pose_with_timestamp.pose.pose.orientation.z = pose_quaternion.z();
				pose_with_timestamp.pose.pose.orientation.w = pose_quaternion.w();

				pose_with_timestamp.pose.covariance[0*6+0] = covariance[0*6+0];
				pose_with_timestamp.pose.covariance[1*6+1] = covariance[1*6+1];
				pose_with_timestamp.pose.covariance[5*6+5] = covariance[5*6+5];

				pub_uav1_pose.publish(pose_with_timestamp);

				double time_diff = (msg->header.stamp - uav1_odom_previous_time).toSec ();

				//we dowsample the odom for visualization
				if(time_diff>1.0)
				{
					//std::cout<<it_agent_human->first<<", human: roll:"<<roll_odom<<",pitch:"<<pitch_odom<<",yaw:"<<yaw_odom<<std::endl;
					m_Agent[it_agent_uav->first].trajectories_low_frequency.push_back(estimate_uav);
					m_Agent[it_agent_uav->first].timestamps_low_frequency.push_back(uav_odom_time);

					uav1_odom_previous_time=msg->header.stamp;
				}


			}

		}


		previous_uav1_odom.x=msg->pose.pose.position.x;
		previous_uav1_odom.y=msg->pose.pose.position.y;
		previous_uav1_odom.z=msg->pose.pose.position.z;
		previous_uav1_odom.theta=yaw_odom;
		previous_uav1_odom.theta_x=roll_odom;
		previous_uav1_odom.theta_y=pitch_odom;
	}
}

// --------------------------------------------------------------------------
void loadConfigureFile(const char * file_name)
// --------------------------------------------------------------------------
{
	std::string modelFileName(file_name);
	std::string         line, key_word;
	std::istringstream  iss_line;

	std::ifstream model_in( modelFileName.c_str(), std::ios::in );

	if ( ! model_in )
	{
		std::cerr << "Configure file " << modelFileName << " not found, please re-check it." << std::endl;
		exit(0);
	}

	std::istream                 * fin     = NULL;
	fin = new std::istream( model_in.rdbuf() );
	if ( ! fin )
	{
		std::cerr << "Error opening regular file:" << modelFileName  << std::endl;
		exit(0);
	}
	// load key parameters
	size_t key_word_separator;
	uint   line_count = 0;
	uint   total_line_count = 0;

	while ( fin->good() )
	{
		iss_line.clear();
		getline( *fin, line );

		total_line_count++;
		// Filter out comment lines
		if ( line.length() == 0 || line[0] == '#' )
			continue;

		// Now, all remaining lines have a key word in their beginning,
		// followed by white spaces and the rest of the data
		key_word_separator = line.find( ' ' );
		if ( key_word_separator == std::string::npos )
			continue;

		iss_line.str( line );
		key_word = line.substr( 0, key_word_separator );
		iss_line >> key_word;


		if(key_word == "drone")
		{
			std::map< std::string, ObjectConfig > config;
			std::string drone_id;
			std::string tag_id;
			carmen_6d_point_t position;

			iss_line>> drone_id;

			iss_line>> tag_id;
			iss_line>> position.x;
			iss_line>> position.y;
			iss_line>> position.z;

			m_drone_config[drone_id].tag_information[tag_id]=position;

			m_tag_drone[tag_id]=drone_id;

			std::string objectName="drone"+drone_id;
			m_tag_objects[tag_id]=objectName;	
			m_agent_config[objectName].tag_information[tag_id]=position;

		}
		else if(key_word == "human")
		{
			std::map< std::string, ObjectConfig > config;
			std::string human_id;
			std::string tag_id;
			carmen_6d_point_t position;

			iss_line>> human_id;

			iss_line>> tag_id;
			iss_line>> position.x;
			iss_line>> position.y;
			iss_line>> position.z;

			m_human_config[human_id].tag_information[tag_id]=position;
			m_tag_human[tag_id]=human_id;

			std::string objectName="human"+human_id;
			m_tag_objects[tag_id]=objectName;
			m_agent_config[objectName].tag_information[tag_id]=position;

		}
		else if(key_word == "height")
		{
			std::string identity;
			iss_line>> identity;

			if(identity=="human")
			{
				std::string human_id;
				double height;
				iss_line>> human_id;
				iss_line>> height;
				m_human_config[human_id].height=height;

				std::string objectName="human"+human_id;
				m_agent_config[objectName].height=height;

			}
			else if(identity=="drone")
			{
				std::string drone_id;
				double height;
				iss_line>> drone_id;
				iss_line>> height;
				m_drone_config[drone_id].height=height;

				std::string objectName="drone"+drone_id;
				m_agent_config[objectName].height=height;
			}
		}
		else if(key_word == "initial_UAV1")
		{
			iss_line>> m_initial_uav1.x;
			iss_line>> m_initial_uav1.y;
			iss_line>> m_initial_uav1.theta;
		}
		else if(key_word == "initial_human")
		{
			iss_line>> m_initial_human.x;
			iss_line>> m_initial_human.y;
			iss_line>> m_initial_human.theta;
		}
		else if(key_word == "enable_visualization")
		{
			iss_line>> m_enable_visualization;
		}	
		else if(key_word == "enable_z_correction")
		{
			iss_line>> m_enable_z_correction;
		}
		else if(key_word == "force_human_in_2d")
		{
			iss_line>> m_force_human_2d;
		}
		else if(key_word == "particles")
		{
			iss_line>> m_num_samples;
		}
		else if(key_word == "update_min_d")
		{
			iss_line>> m_update_min_d;
		}
		else if(key_word == "update_min_a")
		{
			iss_line>> m_update_min_a;
		}
		else if(key_word == "frame_id")
		{
			iss_line>> m_frame_id;			
		}
		else if(key_word == "UAV1_frame_id")
		{
			iss_line>> m_uav1_frame_id;			
		}
		else if(key_word == "Human_frame_id")
		{
			iss_line>> m_human_frame_id;			
		}
		else if(key_word == "UAV1_child_frame_id")
		{
			iss_line>> m_uav1_child_frame_id;			
		}		
		else if(key_word == "Human_child_frame_id")
		{
			iss_line>> m_human_child_frame_id;			
		}
		else if(key_word == "fast_motion_ratio")
		{
			iss_line>> m_fast_motion_ratio;			
		}
		else if(key_word == "static_update_ratio")
		{
			iss_line>> m_static_update_ratio;			
		}
		else if(key_word == "human_odom_downsample_ratio")
		{
			iss_line>> m_human_odom_downsample_ratio;			
		}
		else if(key_word == "uav_odom_downsample_ratio")
		{
			iss_line>> m_uav_odom_downsample_ratio;			
		}
		else if(key_word == "UAV1_uwb")
		{
			iss_line>> m_UAV1_uwb;
		}				
		else if(key_word == "Human_uwb")
		{
			iss_line>> m_human_uwb;			
		}		
		else if(key_word == "UAV1_odom")
		{
			iss_line>> m_UAV1_odom;			
		}
		else if(key_word == "human_odom")
		{
			iss_line>> m_human_odom;			
		}
		else if(key_word == "UAV1_pose")
		{
			iss_line>> m_UAV1_pose;			
		}
		else if(key_word == "Human_pose")
		{
			iss_line>> m_Human_pose;			
		}
		else if(key_word == "relative_human_UAV1_published_topic")
		{
			iss_line>> m_human_UAV1_pose;			
		}
		else if(key_word == "relative_UAV1_human_published_topic")
		{
			iss_line>> m_UAV1_human_pose;			
		}
		else if(key_word == "UWB_ranging_filtering")
		{
			iss_line>> m_uwb_ranging_filtering;			
		}
		else if(key_word == "UWB_ranging_sigma")
		{
			iss_line>> m_sigma;			
		}
		else if(key_word == "use_min_ranging")
		{
			iss_line>> m_use_min_ranging;
		}			
		else 
		{
			std::cout<<"Unknown key word in model file,"<<key_word<<std::endl;
		}
	}

}

// --------------------------------------------------------------------------
void generate_time_based_random_seed()
// --------------------------------------------------------------------------
{
	unsigned int seed;
	struct timeval tv;

	if ( gettimeofday(&tv, NULL) < 0 )
		fprintf( stderr, "error in gettimeofday : %s\n", strerror( errno) );
	seed = tv.tv_sec + tv.tv_usec;
	srand( seed );
}

// --------------------------------------------------------------------------
int main(int argc, char **argv)
// --------------------------------------------------------------------------
{

	ros::init(argc, argv, "droneLocalization");

	ros::NodeHandle node_handler("~");

	std::string configure_file;

	node_handler.getParam("configure", configure_file);


	generate_time_based_random_seed();
	loadConfigureFile(configure_file.c_str());


   
   	//register the global pose to be published
	pub_uav1_pose = node_handler.advertise<geometry_msgs::PoseWithCovarianceStamped>(m_UAV1_pose, 1);
	pub_human_pose = node_handler.advertise<geometry_msgs::PoseWithCovarianceStamped>(m_Human_pose, 1);

	//register the relative pose to be published
	pub_human_uav1_pose = node_handler.advertise<geometry_msgs::PoseWithCovarianceStamped>(m_human_UAV1_pose, 1);
		

        pub_uav1_human_pose = node_handler.advertise<geometry_msgs::PoseWithCovarianceStamped>(m_UAV1_human_pose, 1);
	
	
	//subscriber to the odometry
	ros::Subscriber sub_uav1_odom = node_handler.subscribe(m_UAV1_odom, 1000, uav1OdomCallBack);
	ros::Subscriber sub_human_odom = node_handler.subscribe(m_human_odom, 1000, humanOdomCallback);

	//subscriber to the UWBs
	ros::Subscriber sub_uav1_uwb = node_handler.subscribe(m_UAV1_uwb, 1000, uav1UWBCallback);
	ros::Subscriber sub_human_uwb = node_handler.subscribe(m_human_uwb, 1000, humanUWBCallback);


	std::cout<<"UWB topics: "<<m_UAV1_uwb<<" "<<m_human_uwb<<std::endl;
	std::cout<<"Odom topics: "<<m_UAV1_odom<<" "<<m_human_odom<<std::endl;
	std::cout<<"enable_visualization: "<<m_enable_visualization<<std::endl;
	std::cout<<"enable_z_correction: "<<m_enable_z_correction<<std::endl;
	std::cout<<"force_human_2d: "<<m_force_human_2d<<std::endl;
	std::cout<<"use_min_ranging: "<<m_use_min_ranging<<std::endl;
	std::cout<<"uwb_ranging_sigma: "<<m_sigma<<std::endl;
	std::cout<<"initial uav1: "<<m_initial_uav1.x<<" "<<m_initial_uav1.y<<" "<<m_initial_uav1.theta<<std::endl;
	std::cout<<"initial human: "<<m_initial_human.x<<" "<<m_initial_human.y<<" "<<m_initial_human.theta<<std::endl;

	std::cout<<"m_initial_z_noise: "<<m_initial_z_noise<<std::endl;

	ros::spin();
	return 0;

}







