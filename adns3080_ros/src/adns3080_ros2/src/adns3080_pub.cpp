#include "rclcpp/rclcpp.hpp"
#include "adns3080_interfaces/msg/motion_data.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include <iostream>
#include <deque>
#include "adns3080_ros2/camera.h"
#include <bcm2835.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

struct MD {
    int motion;
    signed char dx, dy;
    int squal;
    int shutter;
    uint8_t max_pix;
};



class ADNS3080Publisher : public rclcpp::Node
{
public:
    ADNS3080Publisher() : Node("adns3080_publisher"),
                          global_dx_(0.0),
                          global_dy_(0.0)
    {
        motion_publisher_ = this->create_publisher<adns3080_interfaces::msg::MotionData>("adns3080/motion", 10);
        displacement_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("adns3080/displacement", 10);

        timer_ = this->create_wall_timer(
            0.1ms, std::bind(&ADNS3080Publisher::publish_data, this));

        if (setup()) {
            RCLCPP_FATAL(this->get_logger(), "Failed to init bcm2835!");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "ADNS3080 initialized.");
    }

    ~ADNS3080Publisher()
    {
        bcm2835_spi_end();
        bcm2835_close();
    }

private:
    void publish_data()
    {
        MD motion;
        mousecam_read_motion(&motion);

        // Publish raw motion data
        auto msg = adns3080_interfaces::msg::MotionData();
        msg.motion = motion.motion;
        msg.dx = motion.dx;
        msg.dy = motion.dy;
        msg.squal = motion.squal;
        msg.shutter = motion.shutter;
        msg.max_pix = motion.max_pix;
        motion_publisher_->publish(msg);

        // Add current dx/dy to the buffer
        update_buffer(motion.dx, motion.dy);

        // Sum the buffer (last 5 frames)
        float sum_dx = 0, sum_dy = 0;
        for (const auto &entry : dx_dy_buffer_) {
            sum_dx += static_cast<float>(entry.first);
            sum_dy += static_cast<float>(entry.second);
        }

        // Apply smoothed delta to global position
        global_dx_ += sum_dx/dx_dy_buffer_.size();
        global_dy_ += sum_dy/dx_dy_buffer_.size();

        geometry_msgs::msg::Vector3 disp_msg;
        disp_msg.x = global_dx_*computeConversionFactor();
        disp_msg.y = global_dy_*computeConversionFactor();
        disp_msg.z = 0.0;
        displacement_publisher_->publish(disp_msg);

        RCLCPP_INFO(this->get_logger(),
                    "Smoothed Δ dx=%.2f dy=%.2f | Global x=%.2f y=%.2f",
                    sum_dx, sum_dy, global_dx_, global_dy_);
    }

    void update_buffer(signed char dx, signed char dy)
    {
        dx_dy_buffer_.emplace_back(dx, dy);
        if (dx_dy_buffer_.size() > 4) {
            dx_dy_buffer_.pop_front();
        }
    }

    rclcpp::Publisher<adns3080_interfaces::msg::MotionData>::SharedPtr motion_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr displacement_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    float global_dx_;
    float global_dy_;
    std::deque<std::pair<signed char, signed char>> dx_dy_buffer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ADNS3080Publisher>());
    rclcpp::shutdown();
    return 0;
}
