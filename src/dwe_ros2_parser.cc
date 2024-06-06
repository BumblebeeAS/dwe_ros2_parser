#include "dwe_ros2_parser/dwe_ros2_parser.hh"

// Node is running variable
atomic<bool> running(true);

// CTRL+C
void signal_handler(int signum) {
    running = false;
}
// Constructor
DWE_Ros2_Parser::DWE_Ros2_Parser() : Node("dwe_ros2_parser") {

    // Fetch ROS parameters
    fetch_ros_parameters();

    // Configure publisher
    image_pub_ = create_publisher<sensor_msgs::msg::Image>(image_topic_, 1);

    // Start DWE Loop
    dwe_loop();

}

// Destructor
DWE_Ros2_Parser::~DWE_Ros2_Parser() {

}

// Fetch ROS parameters
void DWE_Ros2_Parser::fetch_ros_parameters() {

    // Get ROS parameter
    declare_parameter("device", 9);
    declare_parameter("image_topic", "/dwe/camera_0");
    declare_parameter("width", 1920);
    declare_parameter("height", 1080);
    declare_parameter("framerate", 30);
    declare_parameter("auto_exposure", false);
    declare_parameter("exposure", 100);
    declare_parameter("show_image", false);

    // Fetch parameters
    device_ = get_parameter("device").as_int();
    image_topic_ = get_parameter("image_topic").as_string();
    width_ = get_parameter("width").as_int();
    height_ = get_parameter("height").as_int();
    framerate_ = get_parameter("framerate").as_int();
    auto_exposure_ = get_parameter("auto_exposure").as_bool();
    exposure_ = get_parameter("exposure").as_int();
    show_image_ = get_parameter("show_image").as_bool();
}

// Image Callback
void DWE_Ros2_Parser::dwe_loop() {

    // Create a video capture object
    cv::VideoCapture dwe_camera = cv::VideoCapture(device_, cv::CAP_V4L2); 
    const int MJPG = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    dwe_camera.set(cv::CAP_PROP_FOURCC, MJPG);
    dwe_camera.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    dwe_camera.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    dwe_camera.set(cv::CAP_PROP_FPS, framerate_);

    // Check if auto exposure should be disabled
    if (!auto_exposure_) {
        dwe_camera.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
        dwe_camera.set(cv::CAP_PROP_EXPOSURE, 90);
    }

    // Loop variables
    cv_bridge::CvImage cv_image;
    cv_image.encoding="bgr8";
    cv::Mat image;
    
    // Looks for interupts
    signal(SIGINT, signal_handler);

    // Loop is run until Node is told to quit 
    while(running) {


        // Retrive image        
        bool success = dwe_camera.read(image);

        // If image is received
        if (success) {

            // Shows image if set to true
            if (show_image_) {
                cv::imshow("DWE Camera", image);
                
                if (cv::waitKey(1) == 'q') {
                    break;
                }
            }

        // Publish image to ROS2
        cv_image.image = image;
        sensor_msgs::msg::Image image_msg = *cv_image.toImageMsg();
        image_msg.header.stamp = this->now();
        image_msg.header.frame_id = "camera_frame";
        image_pub_->publish(image_msg);

        }

    }

    // Release camera object
    dwe_camera.release();
    cv::destroyAllWindows();

    // Exit node
    exit(0);
}
