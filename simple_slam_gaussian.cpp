// ROS 및 관련 메시지 타입, 필요한 헤더 파일 포함
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <cmath>
#include <memory>
#include <tf/transform_broadcaster.h>

class LidarProcessor {
public:
    // 생성자: 객체 초기화 시 필요한 매개변수를 받습니다.
    LidarProcessor(double resolution, double robot_x, double robot_y, double robot_theta) 
    : resolution_(resolution), robot_x_(robot_x), robot_y_(robot_y), robot_theta_(robot_theta) {}

    // processScan 함수: LiDAR 스캔 데이터를 처리하여 장애물의 위치를 계산합니다.
    std::vector<std::pair<double, double>> processScan(const sensor_msgs::LaserScan::ConstPtr& scan) {
        std::vector<std::pair<double, double>> obstacles; // 장애물 위치를 저장할 벡터
        // 가우시안 필터를 적용하여 스캔 데이터를 평활화합니다.
        std::vector<float> filtered_ranges = applyGaussianFilter(scan->ranges, 5, 1.0);

        // 스캔된 각 포인트에 대하여 장애물의 위치를 계산합니다.
        for (unsigned int i = 0; i < filtered_ranges.size(); ++i) {
            double angle = scan->angle_min + i * scan->angle_increment; // 각도 계산
            double distance = filtered_ranges[i]; // 거리
            // 거리가 유효 범위 내에 있을 경우에만 처리
            if (distance > scan->range_min && distance < scan->range_max) {
                // 로봇 위치와 각도를 고려하여 장애물의 좌표를 계산합니다.
                double obstacle_x = robot_x_ + distance * cos(robot_theta_ + angle) / resolution_;
                double obstacle_y = robot_y_ + distance * sin(robot_theta_ + angle) / resolution_;
                obstacles.push_back(std::make_pair(obstacle_x, obstacle_y)); // 장애물 위치 저장
            }
        }
        return obstacles; // 계산된 장애물 위치 반환
    }

private:
    double resolution_; // 지도 해상도
    double robot_x_, robot_y_, robot_theta_; // 로봇의 위치와 방향

    // 가우시안 필터를 적용하는 함수
    std::vector<float> applyGaussianFilter(const std::vector<float>& input, int window_size, double sigma) {
        int half_window = window_size / 2; // 필터 윈도우의 절반 크기
        std::vector<float> output(input.size(), 0.0); // 출력 벡터 초기화

        // 가우시안 커널 계산
        std::vector<double> kernel(window_size, 0.0);
        double kernel_sum = 0.0;
        for (int i = -half_window; i <= half_window; ++i) {
            kernel[i + half_window] = exp(-(i * i) / (2 * sigma * sigma)) / (sqrt(2 * M_PI) * sigma);
            kernel_sum += kernel[i + half_window];
        }
        // 커널 정규화
        for (int i = 0; i < window_size; ++i) {
            kernel[i] /= kernel_sum;
        }

        // 가우시안 필터 적용
        for (size_t i = 0; i < input.size(); ++i) {
            for (int j = -half_window; j <= half_window; ++j) {
                int index = std::max(0, std::min(int(input.size() - 1), int(i + j)));
                output[i] += input[index] * kernel[j + half_window];
            }
        }

        return output; // 필터링된 결과 반환
    }
};
class MapManager {
public:
    // 생성자: 지도의 너비, 높이, 해상도를 초기화합니다.
    MapManager(int width, int height, double resolution) : width_(width), height_(height), resolution_(resolution) {
        map_.resize(width_ * height_, -1); // 지도 데이터 초기화
    }

    // updateMap 함수: 계산된 장애물 위치를 기반으로 지도를 업데이트합니다.
    void updateMap(const std::vector<std::pair<double, double>>& obstacles) {
        for (auto& obstacle : obstacles) {
            int x = static_cast<int>(obstacle.first);
            int y = static_cast<int>(obstacle.second);
            if (x >= 0 && x < width_ && y >= 0 && y < height_) {
                map_[y * width_ + x] = 100; // 지도에 장애물 표시 (100은 장애물을 의미)
            }
        }
    }

    // publishMap 함수: ROS를 통해 지도 데이터를 발행합니다.
    void publishMap(ros::Publisher& map_pub) {
        nav_msgs::OccupancyGrid grid; // OccupancyGrid 메시지 초기화
        grid.header.stamp = ros::Time::now(); // 현재 시간 설정
        grid.header.frame_id = "map"; // 프레임 ID 설정
        grid.info.resolution = resolution_; // 지도 해상도 설정
        grid.info.width = width_; // 지도 너비 설정
        grid.info.height = height_; // 지도 높이 설정
        grid.info.origin.position.x = -5; // 지도 원점의 X 좌표 설정
        grid.info.origin.position.y = -5; // 지도 원점의 Y 좌표 설정
        grid.info.origin.orientation.w = 1.0; // 지도 원점의 방향 설정
        grid.data.assign(map_.begin(), map_.end()); // 지도 데이터 설정
        map_pub.publish(grid); // 지도 데이터 발행
    }

private:
    int width_, height_; // 지도의 너비와 높이
    double resolution_; // 지도의 해상도
    std::vector<int8_t> map_; // 지도 데이터를 저장하는 벡터
};
class SimpleSLAM {
public:
    // 생성자: ROS 노드 핸들을 통해 객체를 초기화합니다.
    SimpleSLAM(ros::NodeHandle& nh) : nh_(nh), robot_x_(100.0), robot_y_(100.0), robot_theta_(0.0) {
        // 파라미터 서버에서 맵의 크기와 해상도를 가져옵니다.
        nh_.param("map_width", map_width_, 200);
        nh_.param("map_height", map_height_, 200);
        nh_.param("map_resolution", map_resolution_, 0.05);

        // LidarProcessor와 MapManager 객체를 초기화합니다.
        lidar_processor_ = std::make_unique<LidarProcessor>(map_resolution_, robot_x_, robot_y_, robot_theta_);
        map_manager_ = std::make_unique<MapManager>(map_width_, map_height_, map_resolution_);

        // LiDAR 데이터를 구독하는 서브스크라이버와 맵 데이터를 발행하는 퍼블리셔를 설정합니다.
        lidar_sub_ = nh_.subscribe("/scan", 1000, &SimpleSLAM::lidarCallback, this);
        map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 10);
    }

    // lidarCallback 함수: LiDAR 데이터가 수신될 때 호출됩니다.
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        auto obstacles = lidar_processor_->processScan(scan); // 장애물 위치 계산
        map_manager_->updateMap(obstacles); // 지도 업데이트
        map_manager_->publishMap(map_pub_); // 지도 발행
        publishTransform(); // 로봇의 위치 변환 정보를 발행
    }

    // run 함수: ROS 메시지 루프를 실행합니다.
    void run() {
        ros::spin();
    }

private:
    // publishTransform 함수: 로봇의 위치와 방향에 대한 변환 정보를 tf로 발행합니다.
    void publishTransform() {
        static tf::TransformBroadcaster br;
        tf::Transform transform;

        // 로봇의 현재 위치와 방향을 설정합니다.
        transform.setOrigin(tf::Vector3(robot_x_, robot_y_, 0.0));
        tf::Quaternion q;
        q.setRPY(0, 0, robot_theta_);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));

        // 로봇의 base_link 프레임에서 LiDAR 센서까지의 변환을 설정합니다.
        tf::Transform laserTransform;
        laserTransform.setOrigin(tf::Vector3(0, 0, 0)); // 센서의 위치
        tf::Quaternion laserQ;
        laserQ.setRPY(0, 0, 0); // 센서의 방향
        laserTransform.setRotation(laserQ);
        br.sendTransform(tf::StampedTransform(laserTransform, ros::Time::now(), "base_link", "laser"));
    }

    ros::NodeHandle nh_; // ROS 노드 핸들
    std::unique_ptr<LidarProcessor> lidar_processor_; // LiDAR 데이터 처리 객체
    std::unique_ptr<MapManager> map_manager_; // 지도 관리 객체
    ros::Subscriber lidar_sub_; // LiDAR 데이터 구독자
    ros::Publisher map_pub_; // 지도 데이터 발행자
    int map_width_, map_height_; // 지도의 너비와 높이
    double map_resolution_, robot_x_, robot_y_, robot_theta_; // 지도 해상도 및 로봇 위치, 방향
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "simple_slam"); // ROS 초기화 및 노드 이름 설정
    ros::NodeHandle nh; // ROS 노드 핸들 생성

    SimpleSLAM slam(nh); // SimpleSLAM 객체 생성
    slam.run(); // SLAM 실행

    return 0;
}
