#include <iostream>
#include <vector>
//#include <tuple>
#include <stdexcept>
#include <string>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <darknet_actions/obj_detectionAction.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

struct less_by_xmin
{
    inline bool operator() (const darknet_ros_msgs::BoundingBox& box1, const darknet_ros_msgs::BoundingBox& box2)
    {
        return (box1.xmin < box2.xmin);
    }
};

class ObjectDetection{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<darknet_actions::obj_detectionAction> as_;
    std::string action_name_;
    darknet_actions::obj_detectionFeedback feedback_;
    darknet_actions::obj_detectionResult result_;
public:
    ros::Subscriber cam_sub_;
    std::vector<std::string> obj_detection_buf_;
    darknet_ros_msgs::BoundingBoxes detect_objs_;
    std::string to_detected_obj_;

    ObjectDetection(std::string name):
       as_(nh_, name, false),
       action_name_(name)
    {
        ROS_INFO_STREAM("Object Detection action server");
        as_.registerGoalCallback(boost::bind(&ObjectDetection::goalCB,this));
        as_.registerPreemptCallback(boost::bind(&ObjectDetection::preemptCB,this));
        as_.start();
    }

    ~ObjectDetection(void){

    }


    void goalCB() {
        cam_sub_ = nh_.subscribe<darknet_ros_msgs::BoundingBoxes>("darknet_ros/bounding_boxes", 10, &ObjectDetection::boundingboxesCallback, this);
        to_detected_obj_ = as_.acceptNewGoal()->to_detected_obj;
    }

    void preemptCB() {
        //ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
    }

    void boundingboxesCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& detected_boundingboxes) {
        if (!as_.isActive())
            return;
        detect_objs_ = *detected_boundingboxes;
        ROS_INFO_STREAM("scan recevied");
        std::sort(detect_objs_.bounding_boxes.begin(), detect_objs_.bounding_boxes.end(), less_by_xmin());
		/*
		std::vector<darknet_ros_msgs::BoundingBox>::iterator it = detect_objs_.bounding_boxes.begin();
		std::vector<darknet_ros_msgs::BoundingBox>::iterator itlast = detect_objs_.bounding_boxes.begin();
		while (it != detect_objs_.bounding_boxes.end()) {
			itlast = it;
			ROS_INFO_STREAM(it->Class << " "<<it->xmin);
			if(it->Class == "bottle" || it->Class == "cup" || it->Class =="traffic light"
			|| it-> Class == "stop sign")
			{
				it++;
				if(it != detect_objs_.bounding_boxes.end()  &&  itlast->Class == it->Class){
					ROS_INFO_STREAM("second "<<it->Class<< " deleted");
					it = detect_objs_.bounding_boxes.erase(it);
				}
			}else {
				ROS_INFO_STREAM(it->Class<< " deleted");
				it = detect_objs_.bounding_boxes.erase(it);
			}
        }
		*/
        bool goal_detected = false;
        ROS_INFO_STREAM("goal: "<<to_detected_obj_<<std::endl<<" number of obj: "<<detect_objs_.bounding_boxes.size());
        if (detect_objs_.bounding_boxes.size() >=1 ) {
            int result_pos = 0;
            for (int i = 0; i < detect_objs_.bounding_boxes.size(); i++) {
                if(detect_objs_.bounding_boxes.at(i).Class == to_detected_obj_) {
                    result_pos = i;
                    goal_detected = true;
                }
            }
            detect_objs_.bounding_boxes.clear();
        if (goal_detected){
            ROS_INFO_STREAM("result: "<<to_detected_obj_<<" is at pos " <<result_pos+1);
            result_.obj_pos = result_pos + 1;
            as_.setSucceeded(result_);
        }else{
			ROS_INFO_STREAM(to_detected_obj_<<" is not detected or visible");
            result_.obj_pos = 0;
            as_.setSucceeded(result_);
        }
        }

    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "yolo_obj_detection_position_server");
    ObjectDetection obj_detect("yolo_obj_detection_position_server");
    ros::spin();
    return 0;
}
