#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/NavSatFix.h>
#include <rosbag/bag.h>

#include <mrpt_bridge/pose.h>
#include <mrpt_bridge/laser_scan.h>
#include <mrpt_bridge/time.h>
#include <mrpt_bridge/beacon.h>
#include <mrpt_bridge/landmark.h>
#include <mrpt_bridge/imu.h>
#include <mrpt_bridge/image.h>
#include <mrpt_bridge/GPS.h>
#include <mrpt_bridge/stereo_image.h>
#include <mrpt/system/filesystem.h>

#include <mrpt/utils.h>
#include <mrpt/obs.h>
#include <string.h>

#include <map>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::obs;
using namespace mrpt::poses;

struct Parameters
{
	Parameters();
	bool debug;
	std::string rawlog_file;
	CActionRobotMovement2D::TMotionModelOptions motionModelOptions;
};

struct ParametersN : public Parameters
{
	static const int MOTION_MODEL_GAUSSIAN = 0;
	static const int MOTION_MODEL_THRUN = 1;
	ParametersN();

	//void update(const unsigned long& loop_count);
	double rate;
	std::string base_frame;
	std::string odom_frame;
	std::string tf_prefix;
	int parameter_update_skip;
};

Parameters::Parameters()
	: debug(false),
	  rawlog_file("/home/user/s.rawlog")
{
}

ParametersN::ParametersN() : Parameters()
{
	odom_frame = "odom";
	base_frame = "base_link";
	tf_prefix = "/";
}

void read(std::string fname);
bool nextEntry();
void loop(std::string filename);
bool writeSingleObservation(CObservation::Ptr obs);

void writeLaser();
void writeRGBD();
void writeIMU();
void writeGPS();
void writeStereo();
void writeImage();
void writeOdometry();

sensor_msgs::LaserScan msg_laser_;
sensor_msgs::Imu imu;
nav_msgs::Odometry msg_odom_;
sensor_msgs::Image img_l;
sensor_msgs::Image img_r;
sensor_msgs::Image img;
stereo_msgs::DisparityImage img_disp;
sensor_msgs::NavSatFix gps;

std::string odom_frame_;
std::string base_frame_;

size_t entry_ = 0;

mrpt::poses::CPose3DPDFGaussian robotPose;

ParametersN* param_;

rosbag::Bag bag;
CFileGZInputStream rs;
std::map<std::string, int> seqmap;
int missed_imgs = 0;

int main(int argc, char *argv[])
{
	std::string filename;
	std::string bagname;
	if(argc > 1)
		filename = argv[1];
	bagname = argc > 2 ? argv[2] : "rawlogbag.bag";

	//std::string filename = "/home/user/urbrect.rawlog";
	//RawlogPlayNode my_node(0);
	param_ = new ParametersN();
	bag.open(bagname, rosbag::bagmode::Write);
	loop(filename);
	std::cout<<"done"<<std::endl;
	if(missed_imgs > 0)
		std::cout<<missed_imgs<<" images missed"<<std::endl;
}

bool writeSingleObservation(CObservation::Ptr observation)
{
	mrpt::poses::CPose3D pose_sensor;
	observation->getSensorPose(pose_sensor);

	geometry_msgs::Pose msg_pose_sensor;
	tf::Transform transform;

	//declare a geometry msg
	geometry_msgs::TransformStamped msg;
	//declare a std::vector of geometry msgs 
	std::vector<geometry_msgs::TransformStamped> vec_msg;
	//declare the tf:tfmessage, this is what is actually written to the bag file
	tf::tfMessage tfmsg;
	
	double obs_time = mrpt::system::timestampToDouble(observation->getTimeStamp());

	if(IS_CLASS(observation, CObservation2DRangeScan)) // laser scanner <<
	{
		std::cout<<"2d range"<<std::endl;
		auto laser = mrpt::ptr_cast<CObservation2DRangeScan>::from(observation); // cast observation ptr

		mrpt_bridge::convert(*laser, msg_laser_, msg_pose_sensor); // convert mrpt->ros
		
		msg_laser_.header.frame_id = laser->sensorLabel;
		if (msg_laser_.header.frame_id.empty() || msg_laser_.header.frame_id == "") // set frame_id if empty
			msg_laser_.header.frame_id = "laser_link";

		mrpt_bridge::convert(laser->timestamp, msg_laser_.header.stamp); // convert stamp and frame_id

		std::string childframe = tf::resolve(param_->tf_prefix, msg_laser_.header.frame_id); // tf resolve

		mrpt_bridge::convert(pose_sensor, transform); // convert pose -> tf transform

		tf::StampedTransform sttf = tf::StampedTransform( transform, msg_laser_.header.stamp + ros::Duration(0.05),
			base_frame_, childframe);
		
		tf::transformStampedTFToMsg(sttf,msg); // tf -> ros msg

		vec_msg.erase(vec_msg.begin(),vec_msg.end());
		vec_msg.push_back(msg);
		tfmsg.transforms.push_back(vec_msg[0]);
		
		if(seqmap.find(msg_laser_.header.frame_id) == seqmap.end()) // set sequence #
			seqmap[msg_laser_.header.frame_id] = 0;
		
		seqmap[msg_laser_.header.frame_id] += 1;
		msg_laser_.header.seq = seqmap[msg_laser_.header.frame_id];
		
		bag.write("/" + tf::resolve("laser", msg_laser_.header.frame_id), ros::Time(mrpt::system::timestampToDouble(laser->getTimeStamp())), msg_laser_); // write to bag
		bag.write("/tf", ros::Time(mrpt::system::timestampToDouble(laser->getTimeStamp())),tfmsg);
		//std::cout<<std::fixed<<" "<<msg_laser_.angle_min<<" "<<laser->sensorLabel<<msg_laser_.scan_time<<" "<<sizeof(msg_laser_.intensities)<<" "<<mrpt::system::timestampToDouble(laser->getTimeStamp())<<std::endl;
	}
	else if(IS_CLASS(observation, CObservation3DRangeScan)) // rgbd
	{
		std::cout<<"3d range"<<std::endl;
	}
	else if(IS_CLASS(observation, CObservationBeaconRanges)) // copied
	{
		std::cout<<"beacon"<<std::endl;
	}
	else if(IS_CLASS(observation, CObservationBearingRange)) // copied
	{
		std::cout<<"bearing"<<std::endl;
	}
	else if(IS_CLASS(observation, CObservationGPS)) // gps
	{
		std::cout<<"gps"<<std::endl;
		std_msgs::Header gps_hdr;

		auto m_gps = mrpt::ptr_cast<CObservationGPS>::from(observation);	
		bool gs = mrpt_bridge::GPS::mrpt2ros(*m_gps, gps_hdr, gps);

		gps.header.frame_id = m_gps->sensorLabel;
		if (gps.header.frame_id.empty() || gps.header.frame_id == "") // set frame_id if empty
			gps.header.frame_id = "gps";	
		gps_hdr.stamp = ros::Time(obs_time);

		if(seqmap.find(gps.header.frame_id) == seqmap.end()) // set sequence #
			seqmap[gps.header.frame_id] = 0;
		
		seqmap[gps.header.frame_id] += 1;
		gps.header.seq = seqmap[gps.header.frame_id];
		mrpt_bridge::convert(m_gps->timestamp, gps.header.stamp);
		//std::cout<<gps.latitude<<" "<<gps.longitude<<" "<<gps.altitude<<std::endl;

		bag.write("/gps", ros::Time(mrpt::system::timestampToDouble(m_gps->getTimeStamp())), gps);
	}
	else if(IS_CLASS(observation, CObservationImage)) // image
	{
		std::cout<<"image"<<std::endl;
		try{
			std_msgs::Header i_hdr;
			auto m_img = mrpt::ptr_cast<CObservationImage>::from(observation);
			bool s = mrpt_bridge::image::mrpt2ros(*m_img, i_hdr, img);
			//std::cout<<img.width<<" "<<img.height<<" "<<img.width<<" "<<img.height<<" "<<img.step<<" "<<sizeof(img.data)<<std::endl;

			mrpt_bridge::convert(m_img->timestamp, i_hdr.stamp);
  			i_hdr.frame_id = m_img->sensorLabel;
			if (i_hdr.frame_id.empty() || i_hdr.frame_id == "")
				i_hdr.frame_id = "camera";
			img.header = i_hdr;

			if(seqmap.find(img.header.frame_id) == seqmap.end())
				seqmap[img.header.frame_id] = 0;
		
			seqmap[img.header.frame_id] += 1;
			img.header.seq = seqmap[img.header.frame_id];

			std::string childframe = tf::resolve(param_->tf_prefix, img.header.frame_id);

			mrpt_bridge::convert(pose_sensor, transform);
			
			tf::StampedTransform sttf = tf::StampedTransform(transform, img.header.stamp + ros::Duration(0.05),
				base_frame_, childframe);

			tf::transformStampedTFToMsg(sttf,msg);

			vec_msg.erase(vec_msg.begin(),vec_msg.end());
			vec_msg.push_back(msg);
			tfmsg.transforms.push_back(vec_msg[0]);

			bag.write("/camera", ros::Time(mrpt::system::timestampToDouble(m_img->getTimeStamp())), img);
			bag.write("/tf", ros::Time(mrpt::system::timestampToDouble(m_img->getTimeStamp())),tfmsg);
		}
		catch (std::exception &e)
		{std::cout<<"*"<<std::endl; missed_imgs ++; return false;}
	}
	else if(IS_CLASS(observation, CObservationIMU)) // IMU <<
	{
		
		std_msgs::Header imu_hdr;
		imu_hdr.stamp = ros::Time(obs_time);
		auto m_imu = mrpt::ptr_cast<CObservationIMU>::from(observation);

		std::vector<double> measurements = m_imu->rawMeasurements;
		imu.orientation.x = measurements.at(24);
		imu.orientation.y = measurements.at(25);
		imu.orientation.z = measurements.at(26);
		imu.orientation.w = measurements.at(27);

		imu.linear_acceleration.x = measurements.at(0);
		imu.linear_acceleration.y = measurements.at(1);
		imu.linear_acceleration.z = measurements.at(2);

		imu.angular_velocity.x = measurements.at(9);
		imu.angular_velocity.y = measurements.at(10);
		imu.angular_velocity.z = measurements.at(11);
		
		imu.header.frame_id = m_imu->sensorLabel;
		if (imu.header.frame_id.empty() || imu.header.frame_id == "")
			imu.header.frame_id = "imu";

		mrpt_bridge::convert(m_imu->timestamp, imu.header.stamp);

		if(seqmap.find(imu.header.frame_id) == seqmap.end())
			seqmap[imu.header.frame_id] = 0;

		seqmap[imu.header.frame_id] += 1;
		imu.header.seq = seqmap[imu.header.frame_id];

		std::cout<<"imu "<<std::endl;
		//std::cout<<imu.linear_acceleration.x<<" "<<imu.linear_acceleration.y<<" "<<imu.linear_acceleration.z<<" "<<std::endl;

		std::cout<<std::endl;
		bag.write("/imu", ros::Time(mrpt::system::timestampToDouble(m_imu->getTimeStamp())), imu);
	}
	else if(IS_CLASS(observation, CObservationOdometry)) // Odometry
	{
		auto m_odom = mrpt::ptr_cast<CObservationStereoImages>::from(observation);
		std::cout<<"odometry"<<std::endl;
		if(seqmap.find(msg_odom_.header.frame_id) == seqmap.end())
			seqmap[msg_odom_.header.frame_id] = 0;
		CPose3D pose;
		m_odom->getSensorPose(pose);
		seqmap[msg_odom_.header.frame_id] += 1;
		msg_odom_.header.seq = seqmap[msg_odom_.header.frame_id];
		mrpt_bridge::convert(m_odom->timestamp, msg_odom_.header.stamp);
		robotPose = CPose3DPDFGaussian(pose);

		mrpt_bridge::convert(robotPose, msg_odom_.pose);
		mrpt_bridge::convert(robotPose, transform);

		// Write tf to bag
		//***declare a geometry msg
		geometry_msgs::TransformStamped msg;
		//declare a std::vector of geometry msgs 
		std::vector<geometry_msgs::TransformStamped> vec_msg;
		//declare the tf:tfmessage, this is what is actually written to the bag file
		tf::tfMessage tfmsg;
		
		tf::StampedTransform sttf = tf::StampedTransform(
			transform.inverse(), msg_odom_.header.stamp + ros::Duration(0.05),
			odom_frame_, base_frame_);
		
		tf::transformStampedTFToMsg(sttf,msg);

		//erase all value from the std::vector
		vec_msg.erase(vec_msg.begin(),vec_msg.end());
		//push back into the vector the geometry msg
		vec_msg.push_back(msg);
		//Set the tf::tfmessage transforms vector
		tfmsg.transforms.push_back(vec_msg[0]);
		msg.header.seq = seqmap[msg_odom_.header.frame_id];
		bag.write("/odom", msg_odom_.header.stamp,msg_odom_);
		bag.write("/tf", msg_odom_.header.stamp + ros::Duration(0.05),tfmsg);
	}
	else if(IS_CLASS(observation, CObservationStereoImages)) // left+right || left+disparity imgs <<
	{
		try{
			std::cout<<"stereo"<<std::endl;
			std_msgs::Header s_hdr;
			auto stereo = mrpt::ptr_cast<CObservationStereoImages>::from(observation);
			bool s = mrpt_bridge::stereo_image::mrpt2ros(*stereo, s_hdr, img_l, img_r, img_disp);
			//std::cout<<img_l.width<<" "<<img_l.height<<" "<<img_r.width<<" "<<img_r.height<<" "<<img_l.step<<" "<<sizeof(img_r.data)<<std::endl;
			
			mrpt_bridge::convert(stereo->timestamp, s_hdr.stamp);
  			s_hdr.frame_id = stereo->sensorLabel;
			if (s_hdr.frame_id.empty() || s_hdr.frame_id == "")
				s_hdr.frame_id = "stereo_link";
			
			if(seqmap.find(img_l.header.frame_id) == seqmap.end())
			seqmap[img_l.header.frame_id] = 0;
		
			seqmap[img_l.header.frame_id] += 1;
			img_l.header.seq = seqmap[img_l.header.frame_id];
			img_r.header.seq = seqmap[img_l.header.frame_id];

			std::string childframe = tf::resolve(param_->tf_prefix, s_hdr.frame_id);

			mrpt_bridge::convert(pose_sensor, transform);
			
			tf::StampedTransform sttf = tf::StampedTransform(transform, s_hdr.stamp + ros::Duration(0.05),
				base_frame_, childframe);

			tf::transformStampedTFToMsg(sttf,msg);

			vec_msg.erase(vec_msg.begin(),vec_msg.end());
			vec_msg.push_back(msg);
			tfmsg.transforms.push_back(vec_msg[0]);

			bag.write("/camera_left", ros::Time(mrpt::system::timestampToDouble(stereo->getTimeStamp())), img_l);
			bag.write("/camera_right", ros::Time(mrpt::system::timestampToDouble(stereo->getTimeStamp())), img_r);
			bag.write("/tf", ros::Time(mrpt::system::timestampToDouble(stereo->getTimeStamp())),tfmsg);
		}
		catch (std::exception &e)
		{std::cout<<"*"<<std::endl; missed_imgs ++; return false;}
	}
	return true;
}

bool nextEntry( )
{
	CActionCollection::Ptr action;
	CSensoryFrame::Ptr observations;
	CObservation::Ptr obs;

	bool succ = false;

	if (!CRawlog::getActionObservationPairOrObservation(
			rs, action, observations, obs, entry_))
	{
		std::cout<<"end of stream"<<std::endl;
		return true;
	}
	tf::Transform transform;

	// Process single obs, if present:
	if (obs) succ = writeSingleObservation(obs);
	// and process all obs into a CSensoryFrame, if present:
	std::cout<<"for obs"<<std::endl;
	if(observations != NULL)
	{
		for (const auto& o : *observations)
		{
			//std::cout<<"passed for obs"<<std::endl;
			succ = writeSingleObservation(o);
		}
	}
	if(action)
	{
		mrpt::poses::CPose3DPDFGaussian out_pose_increment;
		action->getFirstMovementEstimation(out_pose_increment);	
		robotPose -= out_pose_increment;

		msg_odom_.header.frame_id = "odom";
		msg_odom_.child_frame_id = base_frame_;

		if(obs)
			msg_odom_.header.stamp = ros::Time(mrpt::system::timestampToDouble(obs->getTimeStamp()));
		else if(action)
			msg_odom_.header.stamp = ros::Time(mrpt::system::timestampToDouble((*(observations->begin()))->getTimeStamp()));

		if(seqmap.find(msg_odom_.header.frame_id) == seqmap.end())
			seqmap[msg_odom_.header.frame_id] = 0;
		
		seqmap[msg_odom_.header.frame_id] += 1;
		msg_odom_.header.seq = seqmap[msg_odom_.header.frame_id];

		mrpt_bridge::convert(robotPose, msg_odom_.pose);
		mrpt_bridge::convert(robotPose, transform);

		// Write tf to bag
		//***declare a geometry msg
		geometry_msgs::TransformStamped msg;
		//declare a std::vector of geometry msgs 
		std::vector<geometry_msgs::TransformStamped> vec_msg;
		//declare the tf:tfmessage, this is what is actually written to the bag file
		tf::tfMessage tfmsg;
		
		tf::StampedTransform sttf = tf::StampedTransform(
			transform.inverse(), msg_odom_.header.stamp + ros::Duration(0.05),
			odom_frame_, base_frame_);
		
		tf::transformStampedTFToMsg(sttf,msg);

		//erase all value from the std::vector
		vec_msg.erase(vec_msg.begin(),vec_msg.end());
		//push back into the vector the geometry msg
		vec_msg.push_back(msg);
		//Set the tf::tfmessage transforms vector
		tfmsg.transforms.push_back(vec_msg[0]);
		msg.header.seq = seqmap[msg_odom_.header.frame_id];
		bag.write("/odom", msg_odom_.header.stamp,msg_odom_);
		bag.write("/tf", msg_odom_.header.stamp + ros::Duration(0.05),tfmsg);
	
	}
	return false;
}

void loop(std::string filename)
{
	int loop_count = 0;
	rs.open(filename);

	// initialize odom and base frame
	odom_frame_ = tf::resolve(param_->tf_prefix, param_->odom_frame);
	base_frame_ = tf::resolve(param_->tf_prefix, param_->base_frame);

	robotPose = mrpt::poses::CPose3DPDFGaussian();

	bool end = false;
	for (; !end; loop_count++)
	{
		end = nextEntry();
	}

// roslaunch 1h_config_common.launch path:=/home/user/Documents/rlog/build/test2.bag

}


