#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>

class ImageSubscriberNode : public rclcpp::Node
{
public:
    ImageSubscriberNode()
    : Node("image_subscriber_node")
    {
        // Create an image_transport subscriber
        image_sub_ = image_transport::create_subscription(
            this, "/dwe/camera_0", std::bind(&ImageSubscriberNode::imageCallback, this, std::placeholders::_1), "raw");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        try
        {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::imshow("Subscribed Image", cv_ptr->image);
            cv::waitKey(1); // Needed to display the image
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
    }

    image_transport::Subscriber image_sub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageSubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
