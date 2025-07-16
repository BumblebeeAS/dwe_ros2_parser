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
    
    // Load camera calibration
    load_camera_calibration();
    
    // Setup camera info template
    setup_camera_info();

    // Configure publishers
    if (publish_raw_) {
        image_pub_ = create_publisher<sensor_msgs::msg::Image>(image_topic_, 1);
    }
    if (publish_compressed_) {
        compressed_image_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(compressed_image_topic_, 1);
    }
    camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic_, 1);

    // Start DWE Loop
    dwe_loop();

}

// Destructor
DWE_Ros2_Parser::~DWE_Ros2_Parser() {

}

// Fetch ROS parameters
void DWE_Ros2_Parser::fetch_ros_parameters() {

    // Get ROS parameter
    declare_parameter("device", "/dev/dwe_camera");
    declare_parameter("image_topic", "/dwe/image_raw");
    declare_parameter("compressed_image_topic", "/dwe/image_raw/compressed");
    declare_parameter("camera_info_topic", "/dwe/camera_info");
    declare_parameter("calib_file", "/path/to/calib.yaml");
    declare_parameter("width", 800);    // Originally 1600
    declare_parameter("height", 600);   // Originally 1200
    declare_parameter("framerate", 60);
    declare_parameter("auto_exposure", false);
    declare_parameter("exposure", 100);
    declare_parameter("show_image", false);
    declare_parameter("use_h264", false);
    declare_parameter("save_images", false);
    declare_parameter("save_folder", "~");
    declare_parameter("image_prefix", "image");
    declare_parameter("publish_compressed", true);
    declare_parameter("jpeg_quality", 80);
    declare_parameter("publish_raw", false);


    // Fetch parameters
    device_ = get_parameter("device").as_string();
    image_topic_ = get_parameter("image_topic").as_string();
    compressed_image_topic_ = get_parameter("compressed_image_topic").as_string();
    width_ = get_parameter("width").as_int();
    height_ = get_parameter("height").as_int();
    framerate_ = get_parameter("framerate").as_int();
    auto_exposure_ = get_parameter("auto_exposure").as_bool();
    exposure_ = get_parameter("exposure").as_int();
    show_image_ = get_parameter("show_image").as_bool();
    use_h264_ = get_parameter("use_h264").as_bool();
    save_images_ = get_parameter("save_images").as_bool();
    save_folder_ = get_parameter("save_folder").as_string();
    image_prefix_ = get_parameter("image_prefix").as_string();
    camera_info_topic_ = get_parameter("camera_info_topic").as_string();
    calib_file_path_ = get_parameter("calib_file").as_string();
    publish_compressed_ = get_parameter("publish_compressed").as_bool();
    jpeg_quality_ = get_parameter("jpeg_quality").as_int();
    publish_raw_ = get_parameter("publish_raw").as_bool();
    
}

// Load camera calibration from YAML file
void DWE_Ros2_Parser::load_camera_calibration() {
    RCLCPP_INFO(get_logger(), "Loading calibration file: %s", calib_file_path_.c_str());
    
    // Check if calibration file exists
    if (!std::filesystem::exists(calib_file_path_)) {
        RCLCPP_WARN(get_logger(), "Calibration file does not exist: %s", calib_file_path_.c_str());
        RCLCPP_WARN(get_logger(), "Creating default camera matrix and distortion coefficients");
        
        // Create default camera matrix
        camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
        camera_matrix_.at<double>(0, 0) = 800.0;  // fx
        camera_matrix_.at<double>(1, 1) = 800.0;  // fy
        camera_matrix_.at<double>(0, 2) = width_ / 2.0;   // cx
        camera_matrix_.at<double>(1, 2) = height_ / 2.0;  // cy
        
        // Create default distortion coefficients (no distortion)
        dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
        return;
    }
    
    // Load calibration data
    cv::FileStorage fs(calib_file_path_, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        RCLCPP_ERROR(get_logger(), "Failed to open calibration file: %s", calib_file_path_.c_str());
        RCLCPP_WARN(get_logger(), "Using default calibration parameters");
        
        // Create default camera matrix
        camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
        camera_matrix_.at<double>(0, 0) = 800.0;  // fx
        camera_matrix_.at<double>(1, 1) = 800.0;  // fy
        camera_matrix_.at<double>(0, 2) = width_ / 2.0;   // cx
        camera_matrix_.at<double>(1, 2) = height_ / 2.0;  // cy
        
        // Create default distortion coefficients (no distortion)
        dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
        return;
    }
    
    fs["camera_matrix"] >> camera_matrix_;
    fs["distortion_coefficients"] >> dist_coeffs_;
    fs.release();
    
    RCLCPP_INFO(get_logger(), "Successfully loaded calibration file");
}

// Setup camera info template once
void DWE_Ros2_Parser::setup_camera_info() {
    camera_info_template_.height = height_;
    camera_info_template_.width = width_;
    
    // Set distortion model
    camera_info_template_.distortion_model = "plumb_bob";
    
    // Copy camera matrix (K) - row-major order
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            camera_info_template_.k[i * 3 + j] = camera_matrix_.at<double>(i, j);
        }
    }
    
    // Copy distortion coefficients
    camera_info_template_.d.resize(dist_coeffs_.total());
    for (size_t i = 0; i < dist_coeffs_.total(); i++) {
        camera_info_template_.d[i] = dist_coeffs_.at<double>(i);
    }
    
    // Set rectification matrix (R) - identity for monocular camera
    std::fill(camera_info_template_.r.begin(), camera_info_template_.r.end(), 0.0);
    camera_info_template_.r[0] = camera_info_template_.r[4] = camera_info_template_.r[8] = 1.0;
    
    // Set projection matrix (P) - copy K to first 3 columns, last column zeros
    std::fill(camera_info_template_.p.begin(), camera_info_template_.p.end(), 0.0);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            camera_info_template_.p[i * 4 + j] = camera_matrix_.at<double>(i, j);
        }
    }
    
    // Set binning and ROI (Region of Interest)
    camera_info_template_.binning_x = 1;
    camera_info_template_.binning_y = 1;
    camera_info_template_.roi.x_offset = 0;
    camera_info_template_.roi.y_offset = 0;
    camera_info_template_.roi.height = 0;  // 0 means full resolution
    camera_info_template_.roi.width = 0;   // 0 means full resolution
    camera_info_template_.roi.do_rectify = false;

    RCLCPP_INFO(get_logger(), "Camera info template created successfully");
}

// Image Callback - simplified camera info publishing
void DWE_Ros2_Parser::dwe_loop() {
    // Create a video capture object
    cv::VideoCapture dwe_camera = cv::VideoCapture(device_, cv::CAP_V4L2); 

    // Choose compression. MJPG is FAST, H264 is more efficient but can bring latency
    int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G'); 
    if (use_h264_) {
        fourcc = cv::VideoWriter::fourcc('H', '2', '6', '4');
    }
    dwe_camera.set(cv::CAP_PROP_FOURCC, fourcc);

    // These don't work properly
    dwe_camera.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    dwe_camera.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    //dwe_camera.set(cv::CAP_PROP_FPS, framerate_);

    // Set buffer to zero to not build up latency
    //dwe_camera.set(cv::CAP_PROP_BUFFERSIZE, 1);

    // Check if auto exposure should be disabled
    if (!auto_exposure_) {
        dwe_camera.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
        dwe_camera.set(cv::CAP_PROP_EXPOSURE, exposure_);
    }

    // Check if save dir exists, and if not create it 
    if (save_images_) {
        std::filesystem::path save_dir(save_folder_);
        if (!std::filesystem::exists(save_dir)) {
            std::filesystem::create_directory(save_dir);
        }
    }

    // Loop variables
    cv::Mat image;
    
    // Look for interrupts
    signal(SIGINT, signal_handler);

    // Loop is run until Node is told to quit 
    while(running) {
        // Retrieve image        
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
            rclcpp::Time timestamp = this->now();
            
            // Publish raw image if enabled
            if (publish_raw_ && image_pub_) {
                cv_bridge::CvImage cv_image;
                cv_image.encoding = "bgr8";
                cv_image.image = image;
                sensor_msgs::msg::Image image_msg = *cv_image.toImageMsg();
                image_msg.header.stamp = timestamp;
                image_msg.header.frame_id = "camera_frame";
                image_pub_->publish(image_msg);
            }

            // Publish compressed image if enabled
            if (publish_compressed_ && compressed_image_pub_) {
                sensor_msgs::msg::CompressedImage compressed_msg;
                compressed_msg.header.stamp = timestamp;
                compressed_msg.header.frame_id = "camera_frame";
                compressed_msg.format = "jpeg";
                
                // Encode image as JPEG
                std::vector<uchar> buffer;
                std::vector<int> compression_params;
                compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
                compression_params.push_back(jpeg_quality_);
                
                cv::imencode(".jpg", image, buffer, compression_params);
                compressed_msg.data = buffer;
                
                compressed_image_pub_->publish(compressed_msg);
            }

            // Publish camera info (use the same timestamp)
            sensor_msgs::msg::CameraInfo camera_info_msg = camera_info_template_;
            camera_info_msg.header.stamp = timestamp;
            camera_info_msg.header.frame_id = "camera_frame";
            camera_info_pub_->publish(camera_info_msg);
            
            // Save images if enabled
            if (save_images_) {
                string filename = save_folder_ + "/" + image_prefix_ + 
                                to_string(timestamp.seconds()) + "." + 
                                to_string(timestamp.nanoseconds()) + ".jpg";
                cv::imwrite(filename, image);
            }
        }
    }

    // Release camera object
    dwe_camera.release();
    cv::destroyAllWindows();
}
