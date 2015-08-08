#ifndef CAMERA_ROS_BASE_H_
#define CAMERA_ROS_BASE_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <diagnostic_updater/publisher.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <mavros_extras/CamIMUStamp.h>
#include <vector>
#include <stdint.h>
#include <std_srvs/Trigger.h>
#include <ros/callback_queue.h>
namespace camera_base {

/**
 * @brief getParam Util function for getting ros parameters under nodehandle
 * @param nh Node handle
 * @param name Parameter name
 * @return Parameter value
 */
template <typename T>
T getParam(const ros::NodeHandle& nh, const std::string& name) {
  T value{};
  if (!nh.getParam(name, value)) {
    ROS_ERROR("Cannot find parameter: %s", name.c_str());
  }
  return value;
}

/**
 * @brief The CameraRosBase class
 * This class implements a ros camera
 */
class CameraRosBase {
 public:
  explicit CameraRosBase(const ros::NodeHandle& pnh,
                         const std::string& prefix = std::string())
      : pnh_(pnh),
        cnh_(pnh, prefix),
        it_(cnh_),
        camera_pub_(it_.advertiseCamera("image_raw", 1000)),
        cinfo_mgr_(cnh_, getParam<std::string>(cnh_, "camera_name"),
                   getParam<std::string>(cnh_, "calib_url")),
        image_msg_(new sensor_msgs::Image()),
        cinfo_msg_(new sensor_msgs::CameraInfo(cinfo_mgr_.getCameraInfo())),
        fps_(10),
        diagnostic_updater_(pnh_, cnh_),
        topic_diagnostic_(
            prefix.empty() ? "image_raw" : (prefix +"/image_raw"),
            diagnostic_updater_,
            diagnostic_updater::FrequencyStatusParam(&fps_, &fps_, 0.1, 10),
            diagnostic_updater::TimeStampStatusParam(-0.01, 0.1)) {
	  cnh_.setCallbackQueue(&ros_base_queue_);
    pnh_.param<std::string>("frame_id", frame_id_, "camera");
    cnh_.param<std::string>("identifier", identifier_, "");
    //timestamp_sub_ = cnh_.subscribe("/mavros/cam_imu_sync/cam_imu_stamp", 1000, &CameraRosBase::TriggerCamera,this);
    timestamp_sub_ = cnh_.subscribe("/mavros/cam_imu_sync/cam_imu_stamp", 10, &CameraRosBase::BufferTimestamp,this);
    image_msg_buffer_.reserve(100);
    cinfo_msg_buffer_.reserve(100);
   // timestamp_buffer_.reserve(100);
    timestamp_msg_buffer_.reserve(100);
    frame_seq_id_=0;
    ros::param::get("~starttopic",starttigrertopic_name_);
    client_cam_=cnh_.serviceClient<std_srvs::Trigger>(starttigrertopic_name_);


  }

  CameraRosBase() = delete;
  CameraRosBase(const CameraRosBase&) = delete;
  CameraRosBase& operator=(const CameraRosBase&) = delete;
  virtual ~CameraRosBase() = default;

  const std::string& identifier() const { return identifier_; }
  const std::string& frame_id() const { return frame_id_; }

  double fps() const { return fps_; }
  void set_fps(double fps) { fps_ = fps; }

  /**
   * @brief SetHardwareId Set hardware id for diagnostic updater
   * @param id harware id
   */
  void SetHardwareId(const std::string& id) {
    diagnostic_updater_.setHardwareID(id);
  }

  /**
   * @brief PublishCamera Publish a camera topic with Image and CameraInfo
   * @param time Acquisition time stamp
   */
  void PublishCamera(const ros::Duration& time) {
	      added2triggertime_=time; // update additional time camera needs to update
		  //ros::spinOnce(); // Pumps timestamp of the triggering signal from ROS network into callback function TriggerCamera()

  }

  void UpdateAdded2triggertime(const ros::Duration& time)
  {
	  added2triggertime_=time;
  }


  void TriggerCamera(const mavros_extras::CamIMUStamp& msg){
  ROS_INFO("I heard something %u.%u",msg.frame_stamp.sec,msg.frame_stamp.nsec);
  image_msg_->header.stamp = msg.frame_stamp;

  	  image_msg_->header.frame_id = frame_id_;
  	 // ROS_INFO("seconds: %u nanoseconds: %u",time.sec,time.nsec);
  	  //image_msg_->header.stamp = time;
  	  if (Grab(image_msg_, cinfo_msg_)) {
  		  // Update camera info header
  		  cinfo_msg_->header = image_msg_->header;
  		  camera_pub_.publish(image_msg_, cinfo_msg_);
  		  topic_diagnostic_.tick(image_msg_->header.stamp);
  	  }



  	  diagnostic_updater_.update();

  }

	void BufferTimestamp(const mavros_extras::CamIMUStamp& msg) {
		//  ROS_INFO("Buffered timestamp %u.%u",msg.frame_stamp.sec,msg.frame_stamp.nsec);
		//timestamp_buffer_.push_back(msg.frame_stamp);
		// Add half of exposure time to triggering instance
		mavros_extras::CamIMUStamp tmp=msg;
		ROS_INFO("Time stamp in subscriber buffer:%u", tmp.frame_seq_id);
	//	tmp.frame_stamp=tmp.frame_stamp+added2triggertime_;
//		timestamp_msg_buffer_.push_back(tmp);


// Check whether there is a time stamp message void between the last and the second last time stamp
//		auto buffersize=timestamp_msg_buffer_.size();
//
//		if(buffersize>1) {
//
//			auto n_void=timestamp_msg_buffer_.at(buffersize-1).frame_seq_id-timestamp_msg_buffer_.at(buffersize-2).frame_seq_id;
//			while(n_void>1 && ros::ok()){
//				stamp_interpolated_.frame_seq_id=timestamp_msg_buffer_.at(buffersize-2).frame_seq_id+1;
//				void_difference_=timestamp_msg_buffer_.at(buffersize-1).frame_stamp-timestamp_msg_buffer_.at(buffersize-2).frame_stamp;
//				stamp_interpolated_.frame_stamp=timestamp_msg_buffer_.at(buffersize-2).frame_stamp+ros::Duration(void_difference_.toSec()/n_void);
//				timestamp_msg_buffer_.insert(timestamp_msg_buffer_.end()-1,stamp_interpolated_);
//				buffersize=timestamp_msg_buffer_.size();
//				n_void=timestamp_msg_buffer_.at(buffersize-1).frame_seq_id-timestamp_msg_buffer_.at(buffersize-2).frame_seq_id;
//				ROS_INFO("Detected missing time stamp message. Create time stamp message with interpolation at %u.%u",stamp_interpolated_.frame_stamp.sec,stamp_interpolated_.frame_stamp.nsec);
//
//			}
//
//		}
		 // Check whether buffer exceeds limit size and if so throw away oldest image
		//if(timestamp_msg_buffer_.size()>5){
			//timestamp_msg_buffer_.erase(timestamp_msg_buffer_.begin());
		//}
	//	ROS_INFO("Timestamp buffer: %u",timestamp_msg_buffer_.size());
	}

  void GrabandBufferImage(){
 	  if (Grab(image_msg_, cinfo_msg_)) {
 		 frame_seq_id_=frame_seq_id_+1;
 		  image_msg_->header.seq=frame_seq_id_;
 		  // cache image pointer in buffer
 		  image_msg_buffer_.push_back(image_msg_);
 		 cinfo_msg_buffer_.push_back(cinfo_msg_);

 		 // Check whether buffer exceeds limit size and if so throw away oldest image
//if(image_msg_buffer_.size()>5){
	//image_msg_buffer_.erase(image_msg_buffer_.begin());
//}
 		//ROS_INFO("Timestamp buffer: %u",timestamp_msg_buffer_.size());
 	  }

   }

  unsigned int StampandPublishImage(unsigned int bufferindx) {

	  // Check whether image with corresponding time stamp are buffered
	  int timestamp_indx=CheckandFind(bufferindx);
	  //timestamp_indx=-1;
	//  bufferindx=0;
	 //int  timestamp_indx=0;
	  if (timestamp_indx) {

		  // Copy corresponding images and time stamps
		  sensor_msgs::ImagePtr image_msg_topublish;
		  sensor_msgs::CameraInfoPtr cinfo_msg_topublish;
		  image_msg_topublish=image_msg_buffer_.at(bufferindx);
		  cinfo_msg_topublish=cinfo_msg_buffer_.at(bufferindx);
		  image_msg_topublish->header.stamp=timestamp_msg_buffer_.at(timestamp_indx).frame_stamp;
		  cinfo_msg_topublish->header = image_msg_topublish->header;
		  //ROS_INFO( "Published image with sequence %u",image_msg_topublish->header.seq);

		  // Publish image in ROS
		  	camera_pub_.publish(image_msg_topublish, cinfo_msg_topublish);
		  // Erase published images and used timestamp from buffer
		  image_msg_buffer_.erase(image_msg_buffer_.begin()+bufferindx);
		  cinfo_msg_buffer_.erase(cinfo_msg_buffer_.begin()+bufferindx);
		  timestamp_msg_buffer_.erase(timestamp_msg_buffer_.begin()+timestamp_indx);
		  ROS_INFO("Number of elements in buffer left: %u",(unsigned int)image_msg_buffer_.size());
		  return 0;
		  //timestamp_buffer_.erase(timestamp_buffer_.begin());
	  }
	  else
	  {
		  return 1;
	  }
  }

  void PublishImagebuffer(){


	  if  (image_msg_buffer_.size() && timestamp_msg_buffer_.size()){
		  unsigned int i;
		  for(i=0;i<image_msg_buffer_.size() && timestamp_msg_buffer_.size()>0;)
		  {
			  i=i+StampandPublishImage(i);
		  }

	  }

  }


  unsigned int CheckandFind(unsigned int bufferindx) {
		/* This function checks whether there are corresponding time stamps and and images in buffer
		 * Assumptions:
		 * 1) Trigger signal (electric pulse) by external source
		 * 2) Image was taken by camera. (Placed object in result queue)
		 * 3) Trigger time stamp message did not arrive here.
		 * 4) Last time stamp message was in sync with last image retrieved from camera
		 * */

	  // Check whether there is at least one image in image buffer
	  if(image_msg_buffer_.size()<1){
		  return 0;
	  }


		// Check whether image in image buffer with index "bufferindex" has corresponding element in time stamp buffer
		unsigned int k=0;
		while(k<timestamp_msg_buffer_.size() && ros::ok()) {
			if (image_msg_buffer_.at(bufferindx)->header.seq==(uint)timestamp_msg_buffer_.at(k).frame_seq_id) {
				return k;
			}
			else
			{
				k=k+1;
			}
		}
		return 0;
	}

	void PrintImageBuffer() {
		uint k;
		for(k=0;k<image_msg_buffer_.size();k++) {
			ROS_INFO("Image with sequence %u still in buffer",image_msg_buffer_.at(k)->header.seq);

		}
	}

	bool SendReadyforTrigger(){
		if (client_cam_.call(srv_cam_))
		{
			ROS_INFO_STREAM("Camera "<<getParam<std::string>(cnh_, "camera_name")<<" could successfully call ready-for-trigger signal service.");
			return true;
		}
		else
		{
			ROS_ERROR_STREAM("Camera "<<getParam<std::string>(cnh_, "camera_name")<<" could not call ready-for-trigger signal service.");
			return false;

		}

	}

	void CallCallbackOnce(){
		ros_base_queue_.callOne();
	}

	void CallCallbackBuffer(){
		ros_base_queue_.callAvailable();
	}

  /**
   * @brief Grab Fill image_msg and cinfo_msg from low level camera driver
   * @param image_msg Ros message ImagePtr
   * @param cinfo_msg Ros message CameraInfoPtr
   * @return True if successful
   */
  virtual bool Grab(const sensor_msgs::ImagePtr& image_msg,
                    const sensor_msgs::CameraInfoPtr& cinfo_msg) = 0;

 private:
  ros::NodeHandle pnh_;
  ros::NodeHandle cnh_;
  image_transport::ImageTransport it_;
  image_transport::CameraPublisher camera_pub_;
  camera_info_manager::CameraInfoManager cinfo_mgr_;
  ros::ServiceClient client_cam_;
  std_srvs::Trigger srv_cam_;
  sensor_msgs::ImagePtr image_msg_;
  std::vector<sensor_msgs::ImagePtr> image_msg_buffer_;
  std::vector<sensor_msgs::CameraInfoPtr> cinfo_msg_buffer_;
 // std::vector<ros::Time> timestamp_buffer_;
  std::vector<mavros_extras::CamIMUStamp> timestamp_msg_buffer_;
  sensor_msgs::CameraInfoPtr cinfo_msg_;
  double fps_;
  diagnostic_updater::Updater diagnostic_updater_;
  diagnostic_updater::TopicDiagnostic topic_diagnostic_;
  std::string frame_id_;
  std::string identifier_;
  ros::Subscriber timestamp_sub_;
  ros::Duration added2triggertime_;
  uint32_t frame_seq_id_;
  mavros_extras::CamIMUStamp stamp_interpolated_;
  ros::Duration void_difference_;
  std::string starttigrertopic_name_;
  ros::CallbackQueue ros_base_queue_;
};

}  // namespace camera_base

#endif  // ROS_CAMERA_BASE_H_
