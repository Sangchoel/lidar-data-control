#include <ros/ros.h> // ROS 헤더 파일
#include <sensor_msgs/LaserScan.h> // 라이다 스캔 메시지 헤더
#include <nav_msgs/OccupancyGrid.h> // 점유 격자 맵 메시지 헤더
#include <vector> // 벡터 사용
#include <cmath> // 수학 함수 사용

// 라이다 데이터를 처리하는 클래스
class LidarProcessor {
public:
    LidarProcessor() {} // 생성자

    // 라이다 스캔 데이터를 처리하여 장애물 위치를 벡터로 반환
    std::vector<std::pair<float, float>> processScan(const sensor_msgs::LaserScan::ConstPtr& scan) {
        std::vector<std::pair<float, float>> obstacles;

        // 모든 측정 지점을 순회하며 장애물의 위치를 계산
        for (unsigned int i = 0; i < scan->ranges.size(); ++i) {
            float angle = scan->angle_min + i * scan->angle_increment; // 측정 지점의 각도
            float distance = scan->ranges[i]; // 측정 지점까지의 거리
            if (distance > scan->range_min && distance < scan->range_max) {
                // 거리와 각도를 사용하여 장애물의 위치 계산
                float x = distance * cos(angle); // x 좌표
                float y = distance * sin(angle); // y 좌표
                obstacles.push_back(std::make_pair(x, y)); // 장애물 좌표를 벡터에 추가
            }
        }
        return obstacles; // 장애물 위치 벡터 반환
    }
};

// 맵을 관리하는 클래스
class MapManager {
public:
    // 생성자: 맵의 너비와 높이를 받아 초기화
    MapManager(int width, int height) : width_(width), height_(height) {
        map_.resize(width_ * height_, -1); // 맵 데이터를 초기화
    }

    // 장애물 위치를 맵에 업데이트
    void updateMap(const std::vector<std::pair<float, float>>& obstacles) {
        for (auto& obstacle : obstacles) {
            int x = static_cast<int>(obstacle.first + width_ / 2); // 맵 중앙을 기준으로 x 위치 조정
            int y = static_cast<int>(obstacle.second + height_ / 2); // 맵 중앙을 기준으로 y 위치 조정
            if (x >= 0 && x < width_ && y >= 0 && y < height_) {
                map_[y * width_ + x] = 100; // 장애물 위치에 값을 설정
            }
        }
    }

    // 맵을 발행
    void publishMap(ros::Publisher& map_pub) {
        nav_msgs::OccupancyGrid grid; // 점유 격자 맵 메시지
        grid.header.stamp = ros::Time::now(); // 현재 시간으로 설정
        grid.header.frame_id = "map"; // 맵의 프레임 ID 설정
        grid.info.width = width_; // 맵의 너비 설정
        grid.info.height = height_; // 맵의 높이 설정
        grid.info.origin.position.x = 0; // 맵의 원점 x 좌표 설정
        grid.info.origin.position.y = 0; // 맵의 원점 y 좌표 설정
        grid.data.assign(map_.begin(), map_.end()); // 맵 데이터를 메시지에 할당
        map_pub.publish(grid); // 맵 메시지 발행
    }

private:
    int width_, height_; // 맵의 너비와 높이
    std::vector<int8_t> map_; // 맵 데이터
};

// SLAM(Simultaneous Localization and Mapping)을 수행하는 클래스
class SimpleSLAM {
public:
    // 생성자: 노드 핸들을 받아와 초기화
    SimpleSLAM(ros::NodeHandle& nh) : nh_(nh) {
        lidar_processor_ = std::make_unique<LidarProcessor>(); // 라이다 프로세서 객체 생성
        map_manager_ = std::make_unique<MapManager>(100, 100); // 맵 매니저 객체 생성 및 초기화
        lidar_sub_ = nh_.subscribe("/scan", 1000, &SimpleSLAM::lidarCallback, this); // 라이다 구독 설정
        map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 10); // 맵 발행 설정
    }

    // 라이다 콜백 함수: 라이다 데이터를 받아와 맵을 업데이트하고 발행
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        auto obstacles = lidar_processor_->processScan(scan); // 라이다 데이터 처리
        map_manager_->updateMap(obstacles); // 맵 업데이트
        map_manager_->publishMap(map_pub_); // 맵 발행
    }

    // 노드 실행
    void run() {
        ros::spin(); // ROS 이벤트 루프 실행
    }

private:
    ros::NodeHandle nh_; // ROS 노드 핸들
    std::unique_ptr<LidarProcessor> lidar_processor_; // 라이다 프로세서 객체 포인터
    std::unique_ptr<MapManager> map_manager_; // 맵 매니저 객체 포인터
    ros::Subscriber lidar_sub_; // 라이다 구독자
    ros::Publisher map_pub_; // 맵 발행자
};

// 메인 함수
int main(int argc, char **argv) {
    ros::init(argc, argv, "simple_slam"); // ROS 초기화
    ros::NodeHandle nh; // 노드 핸들 생성

    SimpleSLAM slam(nh); // SLAM 객체 생성
    slam.run(); // SLAM 실행

    return 0;
}
