#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <cmath>
#include <memory> // For std::unique_ptr

// LiDAR 데이터 처리를 담당하는 클래스입니다.
// LiDAR 스캔을 받아 장애물의 위치를 계산하고, 이를 기반으로 SLAM에 필요한 정보를 제공합니다.
class LidarProcessor {
public:
    // Constructor: 초기화에 필요한 매개변수를 받습니다.
    // resolution: 맵의 해상도, robot_x/y/theta: 로봇의 초기 위치 및 방향
    LidarProcessor(double resolution, double robot_x, double robot_y, double robot_theta) 
    : resolution_(resolution), robot_x_(robot_x), robot_y_(robot_y), robot_theta_(robot_theta) {}

    // 주어진 LiDAR 스캔 데이터를 처리하여 장애물의 위치를 계산합니다.
    // 각 장애물은 (x, y) 좌표 쌍으로 반환됩니다.
    std::vector<std::pair<double, double>> processScan(const sensor_msgs::LaserScan::ConstPtr& scan) {
        std::vector<std::pair<double, double>> obstacles;
        for (unsigned int i = 0; i < scan->ranges.size(); ++i) {
            double angle = scan->angle_min + i * scan->angle_increment;
            double distance = scan->ranges[i];
            if (distance > scan->range_min && distance < scan->range_max) {
                double obstacle_x = robot_x_ + distance * cos(robot_theta_ + angle) / resolution_;
                double obstacle_y = robot_y_ + distance * sin(robot_theta_ + angle) / resolution_;
                obstacles.push_back(std::make_pair(obstacle_x, obstacle_y));
            }
        }
        return obstacles;
    }

private:
    double resolution_;
    double robot_x_, robot_y_, robot_theta_;
};

// 맵 데이터를 관리하고 업데이트하는 클래스입니다.
// 이 클래스는 맵의 현재 상태를 저장하고, 장애물 정보를 바탕으로 맵을 업데이트합니다.
class MapManager {
public:
    // Constructor: 맵의 너비, 높이, 해상도를 초기화합니다.
    MapManager(int width, int height, double resolution) : width_(width), height_(height), resolution_(resolution) {
        map_.resize(width_ * height_, -1); // 맵을 -1로 초기화합니다. -1은 미탐색 영역을 의미합니다.
    }

    // 장애물 데이터를 받아 맵을 업데이트합니다.
    void updateMap(const std::vector<std::pair<double, double>>& obstacles) {
        for (auto& obstacle : obstacles) {
            int x = static_cast<int>(obstacle.first);
            int y = static_cast<int>(obstacle.second);
            if (x >= 0 && x < width_ && y >= 0 && y < height_) {
                map_[y * width_ + x] = 1; // 장애물 위치를 맵에 표시합니다. 1은 장애물을 의미합니다.
            }
        }
    }

    // 맵 데이터를 ROS 메시지로 변환하여 발행합니다.
    void publishMap(ros::Publisher& map_pub) {
        nav_msgs::OccupancyGrid grid;
        grid.header.stamp = ros::Time::now();
        grid.header.frame_id = "map";
        grid.info.resolution = resolution_;
        grid.info.width = width_;
        grid.info.height = height_;
        grid.info.origin.position.x = 0;
        grid.info.origin.position.y = 0;
        grid.info.origin.orientation.w = 1.0;
        grid.data.assign(map_.begin(), map_.end()); // 맵 데이터를 메시지에 할당합니다.
        map_pub.publish(grid);
    }

private:
    int width_, height_;
    double resolution_;
    std::vector<int> map_; // 맵 데이터를 저장하는 벡터입니다.
};

// SimpleSLAM 클래스: SLAM 시스템의 메인 클래스입니다.
// 이 클래스는 ROS 노드를 초기화하고, LiDAR 데이터를 구독하여 SLAM 작업을 수행합니다.
class SimpleSLAM {
public:
    SimpleSLAM(ros::NodeHandle& nh) : nh_(nh) {
        // ROS 파라미터 서버에서 매개변수를 읽어와 클래스 멤버 변수를 초기화합니다.
        nh_.param("map_width", map_width_, 100);
        nh_.param("map_height", map_height_, 100);
        nh_.param("map_resolution", map_resolution_, 0.05);
        nh_.param("robot_initial_x", robot_x_, 50.0);
        nh_.param("robot_initial_y", robot_y_, 50.0);
        nh_.param("robot_initial_theta", robot_theta_, 0.0);

        // LidarProcessor와 MapManager 객체를 생성합니다.
        lidar_processor_ = std::make_unique<LidarProcessor>(map_resolution_, robot_x_, robot_y_, robot_theta_);
        map_manager_ = std::make_unique<MapManager>(map_width_, map_height_, map_resolution_);
        
        // LiDAR 데이터를 구독하고, 맵 데이터를 발행하기 위한 퍼블리셔를 설정합니다.
        lidar_sub_ = nh_.subscribe("/scan", 1000, &SimpleSLAM::lidarCallback, this);
        map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 10);
    }

    // LiDAR 데이터가 수신될 때 호출되는 콜백 함수입니다.
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        auto obstacles = lidar_processor_->processScan(scan); // LiDAR 데이터를 처리합니다.
        map_manager_->updateMap(obstacles); // 맵을 업데이트합니다.
        map_manager_->publishMap(map_pub_); // 업데이트된 맵을 발행합니다.
    }

    // ROS 이벤트 루프를 실행합니다.
    void run() {
        ros::spin();
    }

private:
    ros::NodeHandle nh_;
    std::unique_ptr<LidarProcessor> lidar_processor_; // LiDAR 데이터 처리 객체
    std::unique_ptr<MapManager> map_manager_; // 맵 관리 객체
    ros::Subscriber lidar_sub_; // LiDAR 데이터 구독자
    ros::Publisher map_pub_; // 맵 데이터 발행자
    int map_width_, map_height_;
    double map_resolution_, robot_x_, robot_y_, robot_theta_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "simple_slam"); // ROS 시스템과 노드를 초기화합니다.
    ros::NodeHandle nh; // ROS 노드 핸들을 생성합니다.

    SimpleSLAM slam(nh); // SimpleSLAM 객체를 생성하고,
    slam.run(); // 메인 루프를 실행합니다.

    return 0;
}



