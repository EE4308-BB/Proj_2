#include "ee4308_drone/controller.hpp"

namespace ee4308::drone
{
    Controller::Controller(const std::string &name = "controller_ee4308") : Node(name)
    {
        initParam(this, "frequency", frequency_, 10.0);
        initParam(this, "use_ground_truth", use_ground_truth_, false); 
        initParam(this, "enable", enable_, true);
        initParam(this, "lookahead_distance", lookahead_distance_, 1.0);
        initParam(this, "max_xy_vel", max_xy_vel_, 1.0);
        initParam(this, "max_z_vel", max_z_vel_, 0.5);
        initParam(this, "yaw_vel", yaw_vel_, -0.3);
        initParam(this, "kp_xy", kp_xy_, 1.0);
        initParam(this, "kp_z", kp_z_, 1.0);

        pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel", rclcpp::ServicesQoS());
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            (use_ground_truth_ ? "odom" : "est_odom"), rclcpp::SensorDataQoS(),
            std::bind(&Controller::cbOdom, this, std::placeholders::_1));
        sub_plan_ = this->create_subscription<nav_msgs::msg::Path>(
            "plan", rclcpp::SensorDataQoS(),
            std::bind(&Controller::cbPlan, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(1s / frequency_, std::bind(&Controller::cbTimer, this));
    }

    void Controller::cbOdom(const nav_msgs::msg::Odometry msg)
    {
        odom_ = msg;
    }

    void Controller::cbPlan(const nav_msgs::msg::Path msg)
    {
        plan_ = msg;
    }

    void Controller::cbTimer()
    {
        if (!enable_)
            return;

        //check if path exists
        if (plan_.poses.empty())
        {
            // RCLCPP_WARN_STREAM(this->get_logger(), "No path published");
            publishCmdVel(0, 0, 0, 0);
            return;
        }

        // Get current pose
        auto current_pose = odom_.pose.pose;
        double px = current_pose.position.x;
        double py = current_pose.position.y;
        double pz = current_pose.position.z;


          // ==== Step 1: Find the closest point on the path ====
        size_t closest = 0; 
        double min_dist = std::numeric_limits<double>::max(); //some big number
        for (size_t i = 0; i < plan_.poses.size(); i++) {
            auto& point = plan_.poses[i].pose.position; //point is just a placeholder
            double dist = std::hypot(point.x - px, point.y - py);
            if (dist < min_dist) {
                min_dist = dist;
                closest = i;
            }
        }

         // ==== Step 2: Find the lookahead point ====
        geometry_msgs::msg::PoseStamped lookahead_point = plan_.poses.back(); // default to last pose
        for (size_t i = closest; i < plan_.poses.size(); ++i) {
            auto& wp = plan_.poses[i].pose.position;
            double dist = std::hypot(wp.x - px, wp.y - py);
            if (dist >= lookahead_distance_) {
                lookahead_point = plan_.poses[i];
                break;

            }
        }



        // ==== Step 3: Compute velocities to reach the lookahead point ====
        double dx = lookahead_point.pose.position.x - px;
        double dy = lookahead_point.pose.position.y - py;
        double dz = lookahead_point.pose.position.z - pz;

        double phi_r = ee4308::getYawFromQuaternion(current_pose.orientation);

        double x_dash = dx * std::cos(phi_r) + dy * std::sin(phi_r);
        double y_dash = dy * std::cos(phi_r) - dx * std::sin(phi_r);

        
        // Calculate the 2D distance
        double distance_xy = std::hypot(dx, dy);

        // Calculate the desired velocities in x, y, and z directions
        double x_vel = kp_xy_ * x_dash / distance_xy;
        double y_vel = kp_xy_ * y_dash / distance_xy;
        double z_vel = kp_z_ * dz;

        // Constrain the velocities to the maximum limits
        x_vel = std::clamp(x_vel, -max_xy_vel_, max_xy_vel_);
        y_vel = std::clamp(y_vel, -max_xy_vel_, max_xy_vel_);
        z_vel = std::clamp(z_vel, -max_z_vel_, max_z_vel_);

        // ==== Step 4: Apply the constant yaw velocity ====
        double yaw_vel = yaw_vel_;  // Set constant yaw velocity

        // ==== Step 5: Publish the velocities ====
        publishCmdVel(x_vel, y_vel, z_vel, yaw_vel);
    


        
        

        // ==== make use of ====
        // plan_.poses
        // odom_
        // ee4308::getYawFromQuaternion()
        // std::hypot()
        // std::clamp()
        // std::cos(), std::sin() 
        // lookahead_distance_
        // kp_xy_
        // kp_z_
        // max_xy_vel_
        // max_z_vel_
        // yaw_vel_
        // publishCmdVel()
        // ==== ====

        // publish
        //publishCmdVel(0, 0, 0, 0);
    }

    // ================================  PUBLISHING ========================================
    void Controller::publishCmdVel(double x_vel, double y_vel, double z_vel, double yaw_vel)
    {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = x_vel;
        cmd_vel.linear.y = y_vel;
        cmd_vel.linear.z = z_vel;
        cmd_vel.angular.z = yaw_vel;
        pub_cmd_vel_->publish(cmd_vel);
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<ee4308::drone::Controller>());
    rclcpp::shutdown();
    return 0;
}
