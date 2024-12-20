#include "ebug_base/gridmap_controller.hpp"


namespace ebug
{
    GridmapController::GridmapController(const rclcpp::NodeOptions& options) : Node("GridmapController", options)
    {
        this->declare_parameter<double>("sigma_x", 0.5);
        this->declare_parameter<double>("sigma_y", 0.5);
        this->declare_parameter<double>("amplitude", 0.0001);
        
        this->declare_parameter<double>("scale_down", 1);
        
        rclcpp::Time now = this->get_clock()->now();
        subscription_ = this->create_subscription<ebug_base::msg::RobotPose>(
            "/global_poses", 10, std::bind(&GridmapController::topic_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
            "grid_map", rclcpp::QoS(1).transient_local());
    }

    void GridmapController::topic_callback(const ebug_base::msg::RobotPose::SharedPtr msg)
    {
        auto receive_time = this->get_clock()->now();
        RCLCPP_INFO(this->get_logger(), "Received pose message for %s at time %lf",
                    msg->robot_id.c_str(), receive_time.seconds());
        // Update the map with the new pose
        double cov_x = msg->pose.covariance[0]; 
        double cov_y = msg->pose.covariance[7];
        robot_poses_[msg->robot_id] = std::make_pair(msg->pose.pose, std::make_pair(cov_x, cov_y));

        // Update the grid
        update_grid_map();
    }

    // void GridmapController::update_grid_map(const ebug_base::msg::RobotPose::SharedPtr& msg, const rclcpp::Time& receive_time)
    void GridmapController::update_grid_map()
    {
        // Define the parameters of the Gaussian function
        double sigma_x, sigma_y, amplitude, scale_down;
        this->get_parameter("sigma_x", sigma_x);
        this->get_parameter("sigma_y", sigma_y);
        this->get_parameter("amplitude", amplitude);
        this->get_parameter("scale_down", scale_down);

        if (scale_down == 0.0)
            scale_down = 1.0;

        sigma_x /= scale_down;
        sigma_y /= scale_down;

        auto start_time = this->get_clock()->now();
       
        // Initialise the grid map
        grid_map::GridMap map({"elevation"});
        map.setFrameId("map");
        
        map.setGeometry(grid_map::Length(19.20 / scale_down, 10.80 / scale_down), 0.03);    //  3 cm x  3 cm per cell

        map["elevation"].setConstant(0.0);

        // Scale Factors 
        const double scale_x = 11.1 / scale_down; // cm per RVIZ unit
        const double scale_y = 12.65 / scale_down;  // cm per RVIZ unit
        const double max_uncertainty = 1.5 / scale_down; //To limit the visualisations 


        //Loop through Robot pose and apply grid map visuals
        for (const auto& [id, pose_cov] : robot_poses_) {
            const auto& pose = pose_cov.first;
            const auto& cov = pose_cov.second;

            //Robot Pose conversion from Real Robot Pose to RVIZ 
            double real_x = pose.position.x * scale_x; 
            double real_y = pose.position.y * scale_y;

            double cov_x = cov.first;
            double cov_y = cov.second;
            
            //Applying the min-max filter to the gaussian variances 
            double adjusted_sigma_x = sigma_x * std::sqrt(std::abs(cov_x));
            adjusted_sigma_x = std::max(sigma_x, adjusted_sigma_x);
            adjusted_sigma_x = std::min(adjusted_sigma_x, max_uncertainty);

            double adjusted_sigma_y = sigma_y * std::sqrt(std::abs(cov_y));
            adjusted_sigma_y = std::max(sigma_y, adjusted_sigma_y);
            adjusted_sigma_y = std::min(adjusted_sigma_y, max_uncertainty);

            for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it)
            {
                grid_map::Position position;
                map.getPosition(*it, position);
                double distx = real_x - position.x();  
                double disty = real_y - position.y();

                // Set elevation for the robot's position
                if (map.isInside(position)) {
                    double elevation = amplitude * std::exp(-(distx * distx) / (2 * adjusted_sigma_x * adjusted_sigma_x) -
                                                        (disty * disty) / (2 * adjusted_sigma_y * adjusted_sigma_y)); //Gaussian Formula 
                    map.at("elevation", *it) += elevation; //Additive symbol to account for all robots that are present in the arena. 
                }
            }
        }
        auto message = grid_map::GridMapRosConverter::toMessage(map);
        publisher_->publish(std::move(message));

        auto end_time = this->get_clock()->now();
        RCLCPP_INFO(this->get_logger(), "Processed and published grid map in %lf seconds",
                    (end_time - start_time).seconds());
    }
}


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ebug::GridmapController)