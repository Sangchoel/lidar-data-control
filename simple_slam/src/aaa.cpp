#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <cmath>
#include <memory>
#include <tf/transform_broadcaster.h>

class LidarProcessor {
public:
//생성자
    LidarProcessor(double resolution, double robot_x, double robot_y, double robot_theta)
        : resolution_(resolution), robot_x_(robot_x), robot_y_(robot_y), robot_theta_(robot_theta) {}

    std::vector<std::pair<double, double>> processScan(const sensor_msgs::LaserScan::ConstPtr& scan) {
        std::vector<std::pair<double, double>> obstacles;
        std::vector<float> filtered_ranges = applyGaussianFilter(scan->ranges, 5, 1.0); // 가우시안 필터 적용 입력데이터, 커널 사이즈, 필터폭(클스록부드러운 출력넓은 범위에 영향) 

        for (unsigned int i = 0; i < filtered_ranges.size(); ++i) {
            double angle = scan->angle_min + i * scan->angle_increment;
            double distance = filtered_ranges[i];
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

    // 가우시안 필터를 적용하는 함수
    std::vector<float> applyGaussianFilter(const std::vector<float>& input, int window_size, double sigma) {
        // 이 값은 가우시안 필터를 적용할 때 현재 데이터 포인트를 중심으로 좌우 각각 몇 개의 데이터 포인트를 포함할지 결정합니다.
        // 'window_size'의 총 크기에 대해, 'half_window'는 각 데이터 포인트의 좌우 양쪽에서 고려될 포인트 수를 나타냅니다.
        int half_window = window_size / 2; 
        std::vector<float> output(input.size(), 0.0);

        // 가우시안 커널 배열을 생성하고 모든 요소를 0.0으로 초기화합니다.
        std::vector<double> kernel(window_size, 0.0);
        // 커널 가중치의 총합을 저장할 변수를 0.0으로 초기화합니다.
        double kernel_sum = 0.0;

            // 가우시안 커널을 계산합니다.
        for (int i = -half_window; i <= half_window; ++i) {
            // 커널의 각 위치에 대한 가우시안 함수 값을 계산하여 저장합니다.
            // exp(-(i * i) / (2 * sigma * sigma))는 가우시안 공식을 이용하여 각 포인트의 가중치를 계산합니다.
            // sqrt(2 * M_PI) * sigma는 정규화 상수입니다.
            kernel[i + half_window] = exp(-(i * i) / (2 * sigma * sigma)) / (sqrt(2 * M_PI) * sigma);
            // 계산된 가중치를 총합에 더합니다.
            kernel_sum += kernel[i + half_window];
        }

        // 커널 정규화
        // 커널의 모든 가중치 값을 커널 가중치의 총합으로 나누어 정규화합니다.
        for (int i = 0; i < window_size; ++i) {
            kernel[i] /= kernel_sum;
            }

        // 가우시안 필터 적용
        // 입력 데이터에 대해 가우시안 필터를 적용합니다.
        for (size_t i = 0; i < input.size(); ++i) {
            // 현재 데이터 포인트를 중심으로 커널을 적용합니다.
            for (int j = -half_window; j <= half_window; ++j) {
                // 인덱스 범위 확인: 범위를 벗어나는 인덱스는 경계값을 사용합니다.
                int index = std::max(0, std::min(int(input.size() - 1), int(i + j)));
                // 가중치가 적용된 입력값을 현재 포인트의 결과값에 누적합니다.
                output[i] += input[index] * kernel[j + half_window];
                }
        }


        return output;
    }
};

class MapManager {
public:
    MapManager(int width, int height, double resolution) : width_(width), height_(height), resolution_(resolution) {
        map_.resize(width_ * height_, -1);
    }

    void updateMap(const std::vector<std::pair<double, double>>& obstacles) {
        for (auto& obstacle : obstacles) {
            int x = static_cast<int>(obstacle.first);
            int y = static_cast<int>(obstacle.second);
            if (x >= 0 && x < width_ && y >= 0 && y < height_) {
                map_[y * width_ + x] = 100; // 장애물 위치를 맵에 표시합니다. 100은 장애물을 의미합니다.
            }
        }
    }

    void publishMap(ros::Publisher& map_pub) {
        nav_msgs::OccupancyGrid grid;
        grid.header.stamp = ros::Time::now();
        grid.header.frame_id = "map";
        grid.info.resolution = resolution_;
        grid.info.width = width_;
        grid.info.height = height_;
        grid.info.origin.position.x = -5; // 맵 원점 설정
        grid.info.origin.position.y = -5; // 맵 원점 설정
        grid.info.origin.orientation.w = 1.0;
        grid.data.assign(map_.begin(), map_.end());
        map_pub.publish(grid);
    }

private:
    int width_, height_;
    double resolution_;
    std::vector<int8_t> map_;
};

class SimpleSLAM {
    ros::NodeHandle SimpleSLAM_nh;;
    std::unique_ptr<LidarProcessor> lidar_processor_;
    std::unique_ptr<MapManager> map_manager_;
    //변수선언 
    ros::Subscriber lidar_sub_;
    ros::Publisher map_pub_;
    int map_width_, map_height_;
    double map_resolution_, robot_x_, robot_y_, robot_theta_;

public:
    SimpleSLAM(ros::NodeHandle& nh) : SimpleSLAM_nh(nh), robot_x_(100.0), robot_y_(100.0), robot_theta_(0.0) {
        // 파라미터 및 객체 초기화 SimpleSLAM_nh(nh) 메인함수의 nh 참조 NodeHandle 객체로 토픽 구독, 메시지 발행, 서비스 관리 등 이루어짐
        SimpleSLAM_nh.param("map_width", map_width_, 200);
        SimpleSLAM_nh.param("map_height", map_height_, 200);
        SimpleSLAM_nh.param("map_resolution", map_resolution_, 0.05);

        lidar_processor_ = std::make_unique<LidarProcessor>(map_resolution_, robot_x_, robot_y_, robot_theta_);
        map_manager_ = std::make_unique<MapManager>(map_width_, map_height_, map_resolution_);
        //변수 초기화(생성자)- 생성자가 class 를 만든다
        lidar_sub_ = SimpleSLAM_nh.subscribe("/scan", 1000, &SimpleSLAM::lidarCallback, this); // 토픽이름, 큐사이즈,콜백함수, 콜백함수객체 포인터
        map_pub_ = SimpleSLAM_nh.advertise<nav_msgs::OccupancyGrid>("/map", 10); //토픽이름,큐사이즈
    }

    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        auto obstacles = lidar_processor_->processScan(scan);
        map_manager_->updateMap(obstacles);
        map_manager_->publishMap(map_pub_);
        publishTransform();
    }

    void run() {
        ros::spin(); //ros 동작
    }

private:
    void publishTransform() {
        static tf::TransformBroadcaster br;
        //tf::Transform transform; -로봇의 기본 변환으로 로봇 전체의 위치와 방향(엔코더와 조정 필요)
        //tf::Transform laserTransform; - 센서가 로봇의 어디에 어느방향으로 있는지 변환


        // 변환 정보를 전송할 TransformBroadcaster 객체 생성
        tf::Transform transform;
        // 변환 객체 생성 및 2D pose 정보를 3D pose에 복사

        // base_link에서 map으로의 변환
        transform.setOrigin(tf::Vector3(robot_x_, robot_y_, 0.0));
        tf::Quaternion q;
        q.setRPY(0, 0, robot_theta_);
        transform.setRotation(q);
        // transform 객체에 rotation 설정
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
        // TransformBroadcaster로 변환정보 전달
        // publishing 하는 변환정보에 타임스태프 설정(ros::Time::now())
        // parent frmae의 이름 전달("map")
        // child frame의 이름 전달("base_link")

        // base_link에서 laser로의 정적 변환 (예제 값, 실제 센서 위치에 맞게 조정 필요)
        tf::Transform laserTransform;
        laserTransform.setOrigin(tf::Vector3(0, 0, 0)); // 센서의 base_link 대비 위치
        tf::Quaternion laserQ;
        laserQ.setRPY(0, 0, 0); // 센서의 방향
        laserTransform.setRotation(laserQ);
        br.sendTransform(tf::StampedTransform(laserTransform, ros::Time::now(), "base_link", "laser"));
    }
};

int main(int argc, char** argv)
//- argc는 운영체제가 이 프로그램을 실행했을 때 전달되는 인수의 갯수이다. 
//- 즉, main()함수에 전달되는 데이터의 갯수를 의미한다.
//- char* argv[]: 문자열의 주소를 저장하는 포인터 배열
//- argv[0]은 프로그램의 실행경로이다.
//- argv[1], argv[2] ... 에는 순서대로 사용자가 입력한 argument가 저장된다.
{
    ros::init(argc, argv, "simple_slam");
    ros::NodeHandle nh;
    //ros::init 로스의 시스템 초기화 로스마스터와 통신 시작
    //ros::NodeHandle nh 객체는 ROS 시스템과의 통신을 관리 pub, sub

    SimpleSLAM slam(nh);
    slam.run();

    //SimpleSLAM 객체 생성 nh 가 생성자로 전달
    //slam.run()은 SLAM 시스템을 실행 ros::spin()을 호출하여 ROS 이벤트 루프를 실행

    return 0;
}
