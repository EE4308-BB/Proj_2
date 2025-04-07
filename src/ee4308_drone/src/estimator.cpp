#include "ee4308_drone/estimator.hpp"

namespace ee4308::drone
{
    // ================================ IMU sub callback / EKF Prediction ========================================
    void Estimator::cbIMU(const sensor_msgs::msg::Imu msg)
    {
        rclcpp::Time tnow = msg.header.stamp;
        double dt = tnow.seconds() - last_predict_time_;
        last_predict_time_ = tnow.seconds();

        if (dt < ee4308::THRES)
            return;

        // !!! NOT ALLOWED TO USE ORIENTATION FROM IMU as ORIENTATION IS DERIVED FROM ANGULAR VELOCTIY !!!

        // !!! Store the states in Xx_, Xy_, Xz_, and Xa_.
        //      Store the covariances in Px_, Py_, Pz_, and Pa_.
        //      Required for terminal printing during demonstration.

        double theta = Xa_(0);


        double u_xk = msg.linear_acceleration.x;
        double u_yk = msg.linear_acceleration.y;

        double U_zk = msg.linear_acceleration.z - GRAVITY;
        double U_ak = msg.angular_velocity.z;

        Eigen::Vector2d U_linear_xy;
        U_linear_xy << u_xk, u_yk;

        Eigen::Matrix2d F_xk;
        F_xk << 1, dt,
                0, 1;

        Eigen::Matrix2d W_xk;
        W_xk << -0.5 * dt * dt * std::cos(theta), 0.5 * dt * dt * std::sin(theta),
                -dt * std::cos(theta), dt * std::sin(theta);

        Eigen::Matrix2d Q_x;
        Q_x << var_imu_x_, 0,
                0       , var_imu_y_;

        Xx_ = F_xk * Xx_ + W_xk * U_linear_xy;
        Px_ = F_xk * Px_ * F_xk.transpose() + W_xk * Q_x * W_xk.transpose();
        //----------------------------------------------------------------

        Eigen::Matrix2d F_yk;
        F_yk << 1, dt,
                0, 1;

        Eigen::Matrix2d W_yk;
        W_yk << -0.5 * dt * dt * std::sin(theta), -0.5 * dt * dt * std::cos(theta),
                -dt * std::sin(theta), -dt * std::cos(theta);
        
        Eigen::Matrix2d Q_y;
        Q_y << var_imu_x_, 0,
                0       , var_imu_y_;
        
        Xy_ = F_yk * Xy_ + W_yk * U_linear_xy;
        Py_ = F_yk * Py_ * F_yk.transpose() + W_yk * Q_y * W_yk.transpose();
        //----------------------------------------------------------------
        
        double Q_z = var_imu_z_;
        
        Eigen::Matrix3d F_zk;
        F_zk << 1, dt, 0,
                0, 1, 0,
                0, 0, 1;

        Eigen::Vector3d W_zk;
        W_zk << 0.5 * dt * dt,
                dt,
                0;
        
        Xz_ = F_zk * Xz_ + W_zk * U_zk;
        Pz_ = F_zk * Pz_ * F_zk.transpose() + W_zk * Q_z * W_zk.transpose();
        //----------------------------------------------------------------

        double Q_a = var_imu_a_;

        Eigen::Matrix2d F_ak;
        F_ak << 1, 0,
                0, 0;

        Eigen::Vector2d W_ak;
        W_ak << dt,
                1;
        
        Xa_ = F_ak * Xa_ + W_ak * U_ak; // Need to chehck U_ak
        Pa_ = F_ak * Pa_ * F_ak.transpose() + W_ak * Q_a * W_ak.transpose();
        
        // ==== make use of ====
        // msg.linear_acceleration
        // msg.angular_velocity
        // GRAVITY
        // var_imu_x_, var_imu_y_, var_imu_z_, var_imu_a_
        // Xx_, Xy_, Xz_, Xa_
        // Px_, Py_, Pz_, Pa_
        // dt
        // std::cos(), std::sin()
        // ====  ====


        // ==== [FOR LAB 2 ONLY] ==== 
        // for proj 2, comment out / delete the following, so the pink covariance bubble does not fill up RViz for lab 2 ====
        //Px_ << 0.1, 0, 0, 0.1;
        //Py_ << 0.1, 0, 0, 0.1;
        // ==== ====
    }

    // ================================ Sonar sub callback / EKF Correction ========================================
    void Estimator::cbSonar(const sensor_msgs::msg::Range msg)
    {
        (void)msg;

        if (msg.range > msg.max_range)
        { // skip erroneous measurements
            return;
        }

        Eigen::Vector3d H;
        H << 1, 0, 0;
        Eigen::MatrixXd H_snr = H.transpose();

        Eigen::Matrix<double, 1, 1> V_snr;
        V_snr(0, 0) = 1.0;
        Eigen::Matrix<double, 1, 1> R_snr;
        R_snr(0, 0) = var_sonar_;
        Eigen::Matrix<double, 1, 1> Y;
        Y(0, 0) = msg.range;
        Ysonar_ = msg.range;

        Eigen::MatrixXd temp_term;
        temp_term = (H_snr * Pz_ * (H_snr.transpose()) + V_snr * R_snr * (V_snr.transpose())).inverse();

        Eigen::MatrixXd K;
        K = Pz_ * (H_snr.transpose()) * temp_term;


        Xz_ = Xz_ + K * (Y - H_snr * Xz_);
        Pz_ = Pz_ - K * H_snr * Pz_;
    }

    // ================================ GPS sub callback / EKF Correction ========================================
    Eigen::Vector3d Estimator::getECEF(
        const double &sin_lat, const double &cos_lat,
        const double &sin_lon, const double &cos_lon,
        const double &alt)
    {
        Eigen::Vector3d ECEF;
        // ==== make use of ====
        // RAD_POLAR, RAD_EQUATOR
        // std::sqrt()
        // all the arguments above.
        // ====  ====

        double e_sq = (RAD_EQUATOR * RAD_EQUATOR - RAD_POLAR * RAD_POLAR) /
              (RAD_EQUATOR * RAD_EQUATOR);

        double N = RAD_EQUATOR / sqrt(1 - e_sq * sin_lat * sin_lat);

        double x = (N + alt) * cos_lat * cos_lon;
        double y = (N + alt) * cos_lat * sin_lon;
        double z = ((1 - e_sq) * N + alt) * sin_lat;

        ECEF << x, y, z;

        return ECEF;
    }


    void Estimator::cbGPS(const sensor_msgs::msg::NavSatFix msg)
    {
        (void)msg;

        constexpr double DEG2RAD = M_PI / 180;
        double lat = -msg.latitude * DEG2RAD;  // !!! Gazebo spherical coordinates have a bug. Need to negate.
        double lon = -msg.longitude * DEG2RAD; // !!! Gazebo spherical coordinates have a bug. Need to negate.
        double alt = msg.altitude;

        double sin_lat = sin(lat); // sin(phi)
        double cos_lat = cos(lat); // cos(phi)
        double sin_lon = sin(lon); // sin(delta)
        double cos_lon = cos(lon); // cos(delta)

        if (initialized_ecef_ == false)
        {
            initial_ECEF_ = getECEF(sin_lat, cos_lat, sin_lon, cos_lon, alt);
            initialized_ecef_ = true;
            return;
        }

        Eigen::Vector3d ECEF = getECEF(sin_lat, cos_lat, sin_lon, cos_lon, alt);


        // ==== make use of ====
        // Ygps_
        // initial_position_
        // initial_ECEF_
        // sin_lat, cost_lat, sin_lon, cos_lon, alt
        // var_gps_x_, var_gps_y_, var_gps_z_
        // Px_, Py_, Pz_
        // Xx_, Xy_, Xz_
        //
        // - other Eigen methods like .transpose().
        // - Possible to divide a VectorXd element-wise by a double by using the divide operator '/'.
        // - Matrix multiplication using the times operator '*'.
        // ====  ====

        Eigen::Matrix3d R_en;
        R_en << -sin_lat * cos_lon, -sin_lon, -cos_lat * cos_lon,
                -sin_lat * sin_lon, cos_lon, -cos_lat * sin_lon,
                cos_lat,                0,              -sin_lat;                


        Eigen::Vector3d xyz_n =  R_en.transpose() * (ECEF - initial_ECEF_);


        Eigen::Matrix3d R_mn;
        R_mn << 0, 1, 0,
                1, 0, 0,
                0, 0, -1;

        Ygps_ = R_mn * xyz_n + initial_position_;

        Eigen::Matrix<double, 1, 1> V_gps;
        V_gps(0, 0) = 1.0;

        Eigen::Vector2d H;
        H << 1, 0;
        Eigen::MatrixXd H_gps = H.transpose();

        Eigen::Matrix<double, 1, 1> R_gps_x;
        Eigen::Matrix<double, 1, 1> R_gps_y;
        Eigen::Matrix<double, 1, 1> R_gps_z;

        R_gps_x(0, 0) = var_gps_x_;
        R_gps_y(0, 0) = var_gps_y_;
        R_gps_z(0, 0) = var_gps_z_;

        Eigen::MatrixXd temp_term;
        Eigen::MatrixXd K;

        // X-axis
        Eigen::Matrix<double, 1, 1> Y_x;
        Y_x(0, 0) = Ygps_[0];
        temp_term = (H_gps * Px_ * (H_gps.transpose()) + V_gps * R_gps_x * (V_gps.transpose())).inverse();
        K = Px_ * (H_gps.transpose()) * temp_term;
        Xx_ = Xx_ + K * (Y_x - H_gps * Xx_);
        Px_ = Px_ - K * H_gps * Px_;


        // Y-axis
        Eigen::Matrix<double, 1, 1> Y_y;
        Y_y(0, 0) = Ygps_[1];
        temp_term = (H_gps * Py_ * (H_gps.transpose()) + V_gps * R_gps_y * (V_gps.transpose())).inverse();
        K = Py_ * (H_gps.transpose()) * temp_term;
        Xy_ = Xy_ + K * (Y_y - H_gps * Xy_);
        Py_ = Py_ - K * H_gps * Py_;

        // Z-axis
        Eigen::Vector3d H_z;
        H_z << 1, 0, 0;
        Eigen::MatrixXd H_gps_z = H_z.transpose();

        Eigen::Matrix<double, 1, 1> Y_z;
        Y_z(0, 0) = Ygps_[2];
        temp_term = (H_gps_z * Pz_ * (H_gps_z.transpose()) + V_gps * R_gps_z * (V_gps.transpose())).inverse();
        K = Pz_ * (H_gps_z.transpose()) * temp_term;
        Xz_ = Xz_ + K * (Y_z - H_gps_z * Xz_);
        Pz_ = Pz_ - K * H_gps_z * Pz_;

        // Need to check Q
    }

    // ================================ Magnetic sub callback / EKF Correction ========================================
    void Estimator::cbMagnetic(const geometry_msgs::msg::Vector3Stamped msg)
    {
        (void)msg;
        // Along the horizontal plane, the magnetic north in Gazebo points towards +x, when it should point to +y. It is a bug.
        // As the drone always starts pointing towards +x, there is no need to offset the calculation with an initial heading.
        
        // magnetic force direction in drone's z-axis can be ignored.

        Eigen::Vector2d H;
        H << 1, 0;
        Eigen::MatrixXd H_mag = H.transpose();

        Eigen::Matrix<double, 1, 1> V_mag;
        V_mag(0, 0) = 1.0;
        Eigen::Matrix<double, 1, 1> R_mag;
        R_mag(0, 0) = var_magnet_;
        Eigen::Matrix<double, 1, 1> Y_mag;

        double mx = msg.vector.x;
        double my = msg.vector.y;

        double yaw = -std::atan2(my, mx);
        yaw = ee4308::limitAngle(yaw);

        Y_mag(0, 0) = yaw;
        Ymagnet_ = yaw;
        

        if (!initialized_magnetic_) {
            Xa_(0) = yaw;
            initialized_magnetic_ = true;
            return; // don't run the update on first measurement
        }
        
        Eigen::MatrixXd temp_term;
        Eigen::MatrixXd K;
        temp_term = (H_mag * Pa_ * (H_mag.transpose()) + V_mag * R_mag * (V_mag.transpose())).inverse();
        K = Pa_ * (H_mag.transpose()) * temp_term;
        Xa_ = Xa_ + K * (Y_mag - H_mag * Xa_);
        Xa_(0) = ee4308::limitAngle(Xa_(0));
        Pa_ = Pa_ - K * H_mag * Pa_;
    }

    // ================================ Baro sub callback / EKF Correction ========================================
    void Estimator::cbBaro(const geometry_msgs::msg::PointStamped msg)
    {
        // if this section is done, Pz_ has to be augmented with the barometer bias to become a 3-state vector.

        (void)msg;

        if (!initialized_baro_) {
            baro_offset_ = msg.point.z;
            initialized_baro_ = true;
            return;
        }        
        
        Ybaro_ = msg.point.z - baro_offset_;

        Eigen::Vector3d H;
        H << 1, 0, 1;
        Eigen::MatrixXd H_bar = H.transpose();

        Eigen::Matrix<double, 1, 1> V_bar;
        V_bar(0, 0) = 1.0;

        Eigen::Matrix<double, 1, 1> R_bar;
        R_bar(0, 0) = var_baro_;

        Eigen::Matrix<double, 1, 1> Y_bar;
        Y_bar(0, 0) = Ybaro_;

        Eigen::MatrixXd temp_term;
        Eigen::MatrixXd K;
        temp_term = (H_bar * Pz_ * (H_bar.transpose()) + V_bar * R_bar * (V_bar.transpose())).inverse();
        K = Pz_ * (H_bar.transpose()) * temp_term;

        Xz_ = Xz_ + K * (Y_bar - H_bar * Xz_);
        Pz_ = Pz_ - K * H_bar * Pz_;
        

        // ==== make use of ====
        // Ybaro_ 
        // msg.point.z
        // var_baro_
        // Pz_
        // Xz_
        // .transpose()
        // ====  ====
    }


    Estimator::Estimator(
        const double &initial_x, const double &initial_y, const double &initial_z,
        const std::string &name = "estimator")
        : Node(name)
    {
        // parameters
        initParam(this, "frequency", frequency_, 10.0);
        initParam(this, "var_imu_x", var_imu_x_, 0.2);
        initParam(this, "var_imu_y", var_imu_y_, 0.2);
        initParam(this, "var_imu_z", var_imu_z_, 0.2);
        initParam(this, "var_imu_a", var_imu_a_, 0.2);
        initParam(this, "var_gps_x", var_gps_x_, 0.2);
        initParam(this, "var_gps_y", var_gps_y_, 0.2);
        initParam(this, "var_gps_z", var_gps_z_, 0.2);
        initParam(this, "var_baro", var_baro_, 0.2);
        initParam(this, "var_sonar", var_sonar_, 0.2);
        initParam(this, "var_magnet", var_magnet_, 0.2);
        initParam(this, "verbose", verbose_, true);

        // topics
        pub_est_odom_ = create_publisher<nav_msgs::msg::Odometry>("est_odom", rclcpp::ServicesQoS());
        auto qos = rclcpp::SensorDataQoS();
        qos.keep_last(1);
        sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
            "odom", qos, std::bind(&Estimator::cbOdom, this, std::placeholders::_1)); // ground truth in sim.
        sub_gps_ = create_subscription<sensor_msgs::msg::NavSatFix>(
            "gps", qos, std::bind(&Estimator::cbGPS, this, std::placeholders::_1));
        sub_sonar_ = create_subscription<sensor_msgs::msg::Range>(
            "sonar", qos, std::bind(&Estimator::cbSonar, this, std::placeholders::_1));
        sub_magnetic_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "magnetic", qos, std::bind(&Estimator::cbMagnetic, this, std::placeholders::_1));
        sub_baro_ = create_subscription<geometry_msgs::msg::PointStamped>(
            "altitude", qos, std::bind(&Estimator::cbBaro, this, std::placeholders::_1));
        sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
            "imu", qos, std::bind(&Estimator::cbIMU, this, std::placeholders::_1));

        // states
        initial_position_ << initial_x, initial_y, initial_z;
        Xx_ << initial_x, 0;
        Xy_ << initial_y, 0;
        Xz_ << initial_z, 0, 0; // , 0 // change hpp as well.
        Xa_ << 0, 0;
        Px_ = Eigen::Matrix2d::Constant(1e3),
        Py_ = Eigen::Matrix2d::Constant(1e3),
        Pz_ = Eigen::Matrix3d::Constant(1e3); // Matrix3d; change hpp as well.
        Pa_ = Eigen::Matrix2d::Constant(1e3);
        initial_ECEF_ << NAN, NAN, NAN;
        Ygps_ << NAN, NAN, NAN;
        Ymagnet_ = NAN;
        Ybaro_ = NAN;
        Ysonar_ = NAN;

        last_predict_time_ = this->now().seconds();
        initialized_ecef_ = false;
        initialized_magnetic_ = false;
        initialized_baro_ = false;

        timer_ = this->create_wall_timer(
            1s / frequency_,
            std::bind(&Estimator::cbTimer, this));
    }

    void Estimator::cbTimer()
    {
        nav_msgs::msg::Odometry est_odom;

        est_odom.header.stamp = this->now();
        est_odom.child_frame_id = "";     //; std::string(this->get_namespace()) + "/base_footprint";
        est_odom.header.frame_id = "map"; //; std::string(this->get_namespace()) + "/odom";

        est_odom.pose.pose.position.x = Xx_[0];
        est_odom.pose.pose.position.y = Xy_[0];
        est_odom.pose.pose.position.z = Xz_[0];
        getQuaternionFromYaw(Xa_[0], est_odom.pose.pose.orientation);
        est_odom.pose.covariance[0] = Px_(0, 0);
        est_odom.pose.covariance[7] = Py_(0, 0);
        est_odom.pose.covariance[14] = Pz_(0, 0);
        est_odom.pose.covariance[35] = Pa_(0, 0);

        est_odom.twist.twist.linear.x = Xx_[1];
        est_odom.twist.twist.linear.y = Xy_[1];
        est_odom.twist.twist.linear.z = Xz_[1];
        est_odom.twist.twist.angular.z = Xa_[1];
        est_odom.twist.covariance[0] = Px_(1, 1);
        est_odom.twist.covariance[7] = Py_(1, 1);
        est_odom.twist.covariance[14] = Pz_(1, 1);
        est_odom.twist.covariance[35] = Pa_(1, 1);

        pub_est_odom_->publish(est_odom);

        if (verbose_)
        {

            RCLCPP_INFO_STREAM(this->get_logger(), "===");
            std::cout << std::fixed;
            std::cout << "    Pose("
                      << std::setw(7) << std::setprecision(3) << Xx_(0) << ","
                      << std::setw(7) << std::setprecision(3) << Xy_(0) << ","
                      << std::setw(7) << std::setprecision(3) << Xz_(0) << ","
                      << std::setw(7) << std::setprecision(3) << limitAngle(Xa_(0)) << ")"
                      << std::endl;
            std::cout << "   Twist("
                      << std::setw(7) << std::setprecision(3) << Xx_(1) << ","
                      << std::setw(7) << std::setprecision(3) << Xy_(1) << ","
                      << std::setw(7) << std::setprecision(3) << Xz_(1) << ","
                      << std::setw(7) << std::setprecision(3) << Xa_(1) << ")"
                      << std::endl;
            std::cout << " ErrPose("
                      << std::setw(7) << std::setprecision(3) << odom_.pose.pose.position.x - Xx_(0) << ","
                      << std::setw(7) << std::setprecision(3) << odom_.pose.pose.position.y - Xy_(0) << ","
                      << std::setw(7) << std::setprecision(3) << odom_.pose.pose.position.z - Xz_(0) << ","
                      << std::setw(7) << std::setprecision(3) << ee4308::limitAngle(ee4308::getYawFromQuaternion(odom_.pose.pose.orientation) - Xa_(0)) << ")"
                      << std::endl;
            std::cout << "ErrTwist("
                      << std::setw(7) << std::setprecision(3) << odom_.twist.twist.linear.x - Xx_(1) << ","
                      << std::setw(7) << std::setprecision(3) << odom_.twist.twist.linear.y - Xy_(1) << ","
                      << std::setw(7) << std::setprecision(3) << odom_.twist.twist.linear.z - Xz_(1) << ","
                      << std::setw(7) << std::setprecision(3) << odom_.twist.twist.angular.z - Xa_(1) << ")"
                      << std::endl;
            std::cout << "     GPS("
                      << std::setw(7) << std::setprecision(3) << Ygps_(0) << ","
                      << std::setw(7) << std::setprecision(3) << Ygps_(1) << ","
                      << std::setw(7) << std::setprecision(3) << Ygps_(2) << ","
                      << std::setw(8) << "---  )"
                      << std::endl;
            std::cout << "    Baro("
                       << std::setw(8) << "---  ,"
                       << std::setw(8) << "---  ,"
                       << std::setw(7) << std::setprecision(3) << Ybaro_ << ","
                       << std::setw(8) << "---  )"
                       << std::endl;
            std::cout << "   BBias("
                       << std::setw(8) << "---  ,"
                       << std::setw(8) << "---  ,"
                       << std::setw(7) << std::setprecision(3) << Xz_(2) << ","
                       << std::setw(8) << "---  )"
                       << std::endl;
            std::cout << "   Sonar("
                      << std::setw(8) << "---  ,"
                      << std::setw(8) << "---  ,"
                      << std::setw(7) << std::setprecision(3) << Ysonar_ << ","
                      << std::setw(8) << "---  )"
                      << std::endl;
            std::cout << "   Magnt("
                      << std::setw(8) << "---  ,"
                      << std::setw(8) << "---  ,"
                      << std::setw(8) << "---  ,"
                      << std::setw(7) << std::setprecision(3) << Ymagnet_ << ")"
                      << std::endl;
        }
    }

    void Estimator::cbOdom(const nav_msgs::msg::Odometry msg)
    {
        odom_ = msg;
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    double initial_x = std::stod(argv[1]);
    double initial_y = std::stod(argv[2]);
    double initial_z = std::stod(argv[3]);

    rclcpp::spin(std::make_shared<ee4308::drone::Estimator>(initial_x, initial_y, initial_z));
    rclcpp::shutdown();
    return 0;
}