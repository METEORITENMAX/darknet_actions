#include <iostream>
#include <vector>
//#include <tuple>
#include <stdexcept>

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


    ObjectDetection(std::string name):
       as_(nh_, name, false),
       action_name_(name)
    {
        as_.registerGoalCallback(boost::bind(&ObjectDetection::goalCB,this));
        as_.registerPreemptCallback(boost::bind(&ObjectDetection::preemptCB,this));
        cam_sub_ = nh_.subscribe<darknet_ros_msgs::BoundingBoxes>("darknet_ros/bounding_boxes", 1, &ObjectDetection::boundingboxesCallback, this);
        as_.start();
    }

    ~ObjectDetection(void){

    }


    void goalCB() {

        std::string to_detected_obj = as_.acceptNewGoal()->to_detected_obj;
        std::string result_pos;
        bool goal_detected = false;
        ROS_INFO_STREAM("goal: "<<to_detected_obj);
        if (detect_objs_.bounding_boxes.size() >=1 ) {
            int j = 0;
            for (int i = 0; i < detect_objs_.bounding_boxes.size(); i++) {
                if(detect_objs_.bounding_boxes.at(i).Class == to_detected_obj) {
                    j = i;
                    goal_detected = true;
                }
            }
            detect_objs_.bounding_boxes.clear();
            if (goal_detected){
            ROS_INFO_STREAM("to_detected_obj: "<<to_detected_obj<<" is at pos " <<j+1);
            as_.setSucceeded(result_);
        }else{
            ROS_INFO_STREAM(to_detected_obj<<" is detected or visible");
        }
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
        detect_objs_ = *detected_boundingboxes;
        ROS_INFO_STREAM("scan recevied");
        std::sort(detect_objs_.bounding_boxes.begin(), detect_objs_.bounding_boxes.end(), less_by_xmin());

        for (int i = 0; i < detect_objs_.bounding_boxes.size(); i++) {
            ROS_INFO_STREAM(detect_objs_.bounding_boxes.at(i).Class << " "
                            <<detect_objs_.bounding_boxes.at(i).xmin);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "objectdetection");
    ObjectDetection obj_detect("objectdetection");
    ros::spin();
    return 0;
}
