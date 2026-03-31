#include <iostream>
#include <string>
#include <chrono>
#include <sstream>
#include <vector>
#include <cmath>

// ROS 2 관련 헤더
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

// 시리얼 통신을 위한 헤더 (Linux 환경 C++ 표준)
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

class EbimuPublisher : public rclcpp::Node
{
public:
    EbimuPublisher() : Node("ebimu_publisher")
    {
        this->declare_parameter("port", "/dev/ttyUSB1");
        this->declare_parameter("baudrate", 115200);

        std::string port = this->get_parameter("port").as_string();
        int baudrate = this->get_parameter("baudrate").as_int();

        // Imu 메시지 타입으로 변경하고, 토픽명을 'imu/data'로 지정
        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);

        // 시리얼 초기화
        initSerialCommand(port, baudrate);

        // 센서 읽기 타이머 설정 (1000us = 1ms)
        timer_ = this->create_wall_timer(
            std::chrono::microseconds(1000),
            std::bind(&EbimuPublisher::timer_callback, this));
    }

    ~EbimuPublisher()
    {
        if (serial_fd_ >= 0) {
            close(serial_fd_);
        }
    }

private:
    int serial_fd_ = -1;
    std::string read_buffer_;

    void initSerialCommand(const std::string& port, int baudrate)
    {
        serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "시리얼 포트를 여는데 실패했습니다: %s. 권한을 확인해주세요.", port.c_str());
            return;
        }

        struct termios options;
        tcgetattr(serial_fd_, &options);
        
        speed_t speed;
        switch(baudrate) {
            case 9600: speed = B9600; break;
            case 19200: speed = B19200; break;
            case 38400: speed = B38400; break;
            case 57600: speed = B57600; break;
            case 115200: speed = B115200; break;
            default: speed = B115200; break;
        }

        cfsetispeed(&options, speed);
        cfsetospeed(&options, speed);

        options.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
        options.c_cflag |= (CLOCAL | CREAD | CS8);
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

        tcsetattr(serial_fd_, TCSANOW, &options);

        RCLCPP_INFO(this->get_logger(), "EBIMU 시리얼 포트(%s, %d) 설정이 완료되었습니다.", port.c_str(), baudrate);
    }

    void timer_callback()
    {
        if (serial_fd_ == -1) return;

        char buffer[256];
        int n = read(serial_fd_, buffer, sizeof(buffer) - 1);

        if (n > 0) {
            buffer[n] = '\0';
            read_buffer_ += buffer;

            size_t pos;
            while ((pos = read_buffer_.find('\n')) != std::string::npos) {
                std::string line = read_buffer_.substr(0, pos);
                read_buffer_.erase(0, pos + 1);

                if (!line.empty() && line.back() == '\r') {
                    line.pop_back();
                }

                // ---------------- 데이터 파싱 및 쿼터니언 변환 ----------------
                // EBIMU 센서는 보통 *12.3,45.6,78.9... (<Roll>,<Pitch>,<Yaw>) 형태로 줍니다.
                if(!line.empty() && line[0] == '*') {
                    line = line.substr(1); // 앞의 '*' 문자 제거
                }

                std::vector<double> parsed_data;
                std::stringstream ss(line);
                std::string token;
                
                // 쉼표(,)를 기준으로 문자열을 쪼개어 숫자로 배열에 담습니다.
                while (std::getline(ss, token, ',')) {
                    try {
                        parsed_data.push_back(std::stod(token));
                    } catch (...) {}
                }

                // 적어도 Roll, Pitch, Yaw 3개의 값이 버퍼에 들어왔다면 변환을 시작합니다.
                if (parsed_data.size() >= 3) {
                    auto imu_msg = sensor_msgs::msg::Imu();
                    imu_msg.header.stamp = this->now();
                    
                    // 주의: URDF에 정의된 imu의 링크 이름과 동일해야 합니다!
                    imu_msg.header.frame_id = "imu_link"; 

                    // 각도를 Radian으로 변환
                    // EBIMU는 Degree로 값을 줍니다. ROS는 통상적으로 Radian 단위를 사용합니다.
                    double roll = parsed_data[0] * M_PI / 180.0;
                    double pitch = parsed_data[1] * M_PI / 180.0;
                    double yaw = parsed_data[2] * M_PI / 180.0;

                    // 오일러 각(Roll, Pitch, Yaw) -> 쿼터니언(Quaternion) 변환 공식 적용
                    double cy = cos(yaw * 0.5);
                    double sy = sin(yaw * 0.5);
                    double cp = cos(pitch * 0.5);
                    double sp = sin(pitch * 0.5);
                    double cr = cos(roll * 0.5);
                    double sr = sin(roll * 0.5);

                    imu_msg.orientation.w = cr * cp * cy + sr * sp * sy;
                    imu_msg.orientation.x = sr * cp * cy - cr * sp * sy;
                    imu_msg.orientation.y = cr * sp * cy + sr * cp * sy;
                    imu_msg.orientation.z = cr * cp * sy - sr * sp * cy;

                    imu_msg.orientation_covariance.fill(0.0);
                    imu_msg.angular_velocity_covariance.fill(0.0);
                    imu_msg.linear_acceleration_covariance.fill(0.0);

                    imu_msg.orientation_covariance[0] = 99999.0;
                    imu_msg.orientation_covariance[4] = 99999.0;
                    imu_msg.orientation_covariance[8] = 0.02;

                    imu_msg.angular_velocity_covariance[0] = -1.0;
                    imu_msg.linear_acceleration_covariance[0] = -1.0;

                    // 선형 가속도, 각속도 데이터도 EBIMU 설정에 따라 넘어온다면 배열 인덱스를 통해 추가 할 수 있습니다.

                    publisher_->publish(imu_msg);
                }
            }
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting ebimu_publisher with ROS2 Parameters..");
    
    auto node = std::make_shared<EbimuPublisher>();
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
