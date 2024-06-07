#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>
#include <chrono>

using namespace std;

class ImageSubscriberNode : public rclcpp::Node
{
public:
    ImageSubscriberNode()
    : Node("image_subscriber_node")
    {
        // Create an image_transport subscriber
        image_sub_ = image_transport::create_subscription(
            this, "/dwe/camera", std::bind(&ImageSubscriberNode::imageCallback, this, std::placeholders::_1), "raw");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        try
        {
            if (images_recv == 0) {
                start_time = chrono::high_resolution_clock::now();
            }

            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::imshow("Subscribed Image", cv_ptr->image);
            cv::waitKey(1); // Needed to display the image
            images_recv++;

            auto elapsed = chrono::duration_cast<chrono::seconds>(chrono::high_resolution_clock::now() - start_time);
            if (elapsed.count() > 5) {
                float fps = images_recv / 5.0;
                RCLCPP_INFO(get_logger(), "Topic /dwe/camera publishes with FPS of %f", fps);
                images_recv = 0;
            }
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
    }

    image_transport::Subscriber image_sub_;
    int images_recv = 0;
    chrono::high_resolution_clock::time_point start_time;



};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageSubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
