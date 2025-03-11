#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>

class LivoxBackPointCloudSwitcher {
public:
    LivoxBackPointCloudSwitcher() {
        // Initialize the node
        ros::NodeHandle nh;

        // Subscribers
        fold_state_sub = nh.subscribe("/fold_state", 10, &LivoxBackPointCloudSwitcher::foldStateCallback, this);
        livox_back_empty_sub = nh.subscribe("/livox_back_empty/points", 10, &LivoxBackPointCloudSwitcher::livoxBackEmptyCallback, this);
        livox_back_with_cart_sub = nh.subscribe("/livox_back_with_cart/points", 10, &LivoxBackPointCloudSwitcher::livoxBackWithCartCallback, this);

        // Publisher
        livox_back_pub = nh.advertise<sensor_msgs::PointCloud2>("/livox_back/points", 10);

        // Initialize state variables
        fold_state = "UNKNOWN";
    }

private:
    // Callback for /fold_state
    void foldStateCallback(const std_msgs::String::ConstPtr& msg) {
        fold_state = msg->data;
        publishLivoxBackPoints();
    }

    // Callback for /livox_back_empty/points
    void livoxBackEmptyCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        livox_back_empty_points = *msg;
        publishLivoxBackPoints();
    }

    // Callback for /livox_back_with_cart/points
    void livoxBackWithCartCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        livox_back_with_cart_points = *msg;
        publishLivoxBackPoints();
    }

    // Logic to publish /livox_back/points
    void publishLivoxBackPoints() {
        if (fold_state == "LOADED" && !livox_back_with_cart_points.data.empty()) {
            livox_back_pub.publish(livox_back_with_cart_points);
        } else if (!livox_back_empty_points.data.empty()) {
            livox_back_pub.publish(livox_back_empty_points);
        }
    }

    // Subscribers
    ros::Subscriber fold_state_sub;
    ros::Subscriber livox_back_empty_sub;
    ros::Subscriber livox_back_with_cart_sub;

    // Publisher
    ros::Publisher livox_back_pub;

    // State variables
    std::string fold_state;
    sensor_msgs::PointCloud2 livox_back_empty_points;
    sensor_msgs::PointCloud2 livox_back_with_cart_points;
};

int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "livox_back_switch");

    // Create an instance of the class
    LivoxBackPointCloudSwitcher switcher;

    // Spin to process callbacks
    ros::spin();

    return 0;
}