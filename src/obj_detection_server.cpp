#include <iostream>
#include <vector>
//#include <tuple>
#include <stdexcept>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <darknet_actions/obj_detectionAction.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

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
    const darknet_ros_msgs::BoundingBoxes::ConstPtr* detected_boundingboxes_;
    ObjectDetection(std::string name):
       as_(nh_, name, false),
       action_name_(name)
    {
        as_.registerGoalCallback(boost::bind(&ObjectDetection::goalCB,this));
        as_.registerPreemptCallback(boost::bind(&ObjectDetection::preemptCB,this));

        as_.start();
    }

    ~ObjectDetection(void){

    }


    void goalCB() {
        ROS_INFO_STREAM("goal recevied");
        cam_sub_ = nh_.subscribe<darknet_ros_msgs::BoundingBoxes>("darknet_ros/bounding_boxes", 1, &ObjectDetection::boundingboxesCallback, this);
        as_.acceptNewGoal();
        ROS_INFO_STREAM(obj_detection_buf_.size());
        if (obj_detection_buf_.size() >=1 ) {
            cam_sub_.shutdown();
            for (int i = 0; i < obj_detection_buf_.size(); i++) {
                ROS_INFO_STREAM(i+1<<". Obj: "<< obj_detection_buf_.at(i));
                //result_.detected_obj[i] = obj_detection_buf_.at(i);
            }
            obj_detection_buf_.clear();
            ROS_INFO_STREAM("goal reached");
            as_.setSucceeded(result_);
        }else{
            //as_.setAborted(result_);
        }
    }

    void preemptCB() {
        //ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
    }

    void boundingboxesCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& detected_boundingboxes) {
        if (!as_.isActive())
            return;
        detected_boundingboxes_ = &detected_boundingboxes;
        ROS_INFO_STREAM("scan recevied");
        //cam_sub_.shutdown();
        //const darknet_ros_msgs::BoundingBoxes::ConstPtr& left, mid, right;
        //std::vector<> pos;
        for (int i = 0; i < detected_boundingboxes->bounding_boxes.size(); i++) {
            obj_detection_buf_.push_back(detected_boundingboxes->bounding_boxes.at(i).Class);

        }

        for (int i = 0; i < detected_boundingboxes->bounding_boxes.size(); i++) {

        }
        //detected_boundingboxes_->bounding_boxes.at(i);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "objectdetection");
    ObjectDetection obj_detect("objectdetection");
    ros::spin();
    return 0;
}
