#include <ros/ros.h> // ROS 기본 헤더 파일
#include <sensor_msgs/LaserScan.h> // LiDAR 데이터를 위한 메시지 타입
#include <nav_msgs/OccupancyGrid.h> // 맵 데이터를 위한 메시지 타입
#include <vector> // 동적 배열을 위한 표준 라이브러리
#include <cmath> // 수학 계산을 위한 표준 라이브러리

class SimpleSLAM {
public:
    SimpleSLAM(ros::NodeHandle& nh) : nh_(nh) {
        // 파라미터 서버에서 매개변수 읽기 (파라미터 이름, 변수, 기본값)
        nh_.param("map_width", map_width_, 100);
        nh_.param("map_height", map_height_, 100);
        nh_.param("map_resolution", map_resolution_, 0.05);
        nh_.param("robot_initial_x", robot_x_, 50.0);
        nh_.param("robot_initial_y", robot_y_, 50.0);
        nh_.param("robot_initial_theta", robot_theta_, 0.0);

        // 맵 초기화 및 구독자, 발행자 설정
        map_.resize(map_width_ * map_height_, -1); 
        lidar_sub_ = nh_.subscribe("/scan", 1000, &SimpleSLAM::lidarCallback, this);
        map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 10);
    }

    // LiDAR 콜백 함수: LiDAR 데이터를 받아 처리합니다.
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        ROS_INFO("Lidar data received"); // 로그 메시지 출력
        
        // LiDAR 데이터를 순회하며 맵을 업데이트합니다.
        for (unsigned int i = 0; i < scan->ranges.size(); ++i) {
            double angle = scan->angle_min + i * scan->angle_increment; // 현재 측정값의 각도 계산
            double distance = scan->ranges[i]; // 거리 데이터
            if (distance > scan->range_min && distance < scan->range_max) { // 유효한 거리 데이터인지 확인
                // 로봇 기준 세계 좌표계로 장애물의 위치 계산
                double obstacle_x = robot_x_ + distance * cos(robot_theta_ + angle) / map_resolution_;
                double obstacle_y = robot_y_ + distance * sin(robot_theta_ + angle) / map_resolution_;

                // 맵 좌표계로 변환하여 인덱스 계산
                int map_index = static_cast<int>(obstacle_y) * map_width_ + static_cast<int>(obstacle_x);
                if (map_index >= 0 && map_index < map_width_ * map_height_) {
                    map_[map_index] = 1; // 장애물을 맵에 표시 (1은 장애물을 의미)
                }
            }
        }

        // 맵 데이터를 발행합니다.
        publishMap();
    }

    // 맵 데이터를 nav_msgs/OccupancyGrid 메시지 형식으로 변환하여 발행하는 함수
    void publishMap() {
        nav_msgs::OccupancyGrid grid; // OccupancyGrid 메시지 객체 생성
        grid.header.stamp = ros::Time::now(); // 현재 시간으로 타임스탬프 설정
        grid.header.frame_id = "osc_map"; // 프레임 ID 설정
        grid.info.resolution = map_resolution_; // 맵 해상도 설정
        grid.info.width = map_width_; // 맵 너비 설정
        grid.info.height = map_height_; // 맵 높이 설정
        grid.info.origin.position.x = map_width_/2; // 맵의 원점 설정
        grid.info.origin.position.y = map_height_/2;
        grid.info.origin.orientation.w = 1.0;
        grid.data.assign(map_.begin(), map_.end()); // 맵 데이터 할당

        map_pub_.publish(grid); // 맵 데이터 발행
    }

    // 노드의 메인 루프를 실행하는 함수
    void run() {
        ros::spin(); // ROS 이벤트 루프 실행
    }

private:
    ros::NodeHandle nh_; // ROS 노드 핸들
    ros::Subscriber lidar_sub_; // LiDAR 데이터 구독자
    ros::Publisher map_pub_; // 맵 데이터 발행자
    int map_width_, map_height_; // 맵의 너비와 높이
    double map_resolution_; // 맵의 해상도 (미터 단위)
    std::vector<int> map_; // 맵 데이터 저장하는 벡터
    double robot_x_, robot_y_, robot_theta_; // 로봇의 위치와 방향
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "simple_slam"); // ROS 초기화 및 노드 이름 설정
    ros::NodeHandle nh; // 노드 핸들 생성

    SimpleSLAM slam(nh); // SimpleSLAM 객체 생성
    slam.run(); // 메인 루프 실행

    return 0;
}


