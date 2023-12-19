#include <ros/ros.h>
#include <std_msgs/String.h>
#include <deque>

class TrafficLightController {
private:
    ros::NodeHandle nh_;
    ros::Subscriber tl_state_sub_;
    ros::Publisher mode_pub_;
    std::deque<std::string> state_buffer_;

    enum State {
        RED_STATE,
        BLUE_STATE
    } current_state_;

public:
    TrafficLightController() : current_state_(RED_STATE) {
        tl_state_sub_ = nh_.subscribe("/tl_state", 10, &TrafficLightController::tlCallback, this);
        mode_pub_ = nh_.advertise<std_msgs::String>("/mode", 10);
    }

    void tlCallback(const std_msgs::String::ConstPtr& msg) {
        if(state_buffer_.size() >= 5) {  // We want to maintain only last 5 states
            state_buffer_.pop_front();
        }
        state_buffer_.push_back(msg->data);

        switch(current_state_) {
            case RED_STATE:
                if(isAllRed()) {
                    current_state_ = BLUE_STATE;
                    state_buffer_.clear();  // Clear the buffer for the next state
                }
                break;
            case BLUE_STATE:
                if(isAllBlue()) {
                    std_msgs::String out_msg;
                    out_msg.data = "run";
                    mode_pub_.publish(out_msg);
                    current_state_ = RED_STATE;
                    state_buffer_.clear();  // Clear the buffer for the next state
                }
                break;
        }
    }

    bool isAllRed() {
        for(const auto& state : state_buffer_) {
            if(state != "red") return false;
        }
        return true;
    }

    bool isAllBlue() {
        for(const auto& state : state_buffer_) {
            if(state != "blue") return false;
        }
        return true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "traffic_light_controller");
    TrafficLightController tlc;

    ros::spin();
    return 0;
}