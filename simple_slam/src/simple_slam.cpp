#include <ros/ros.h> // ROS 헤더 파일
#include <sensor_msgs/LaserScan.h> // 라이다 스캔 메시지 헤더
#include <nav_msgs/OccupancyGrid.h> // 점유 격자 맵 메시지 헤더
#include <vector>
#include <cmath>

// 라이다 데이터를 처리하는 함수
std::vector<std::pair<float, float>> processScan(const sensor_msgs::LaserScan::ConstPtr& scan) {
    std::vector<std::pair<float, float>> obstacles;
    for (unsigned int i = 0; i < scan->ranges.size(); ++i) {
        float angle = scan->angle_min + i * scan->angle_increment;
        float distance = scan->ranges[i];
        if (distance > scan->range_min && distance < scan->range_max) {
            float x = distance * cos(angle);
            float y = distance * sin(angle);
            obstacles.push_back(std::make_pair(x, y));
        }
    }
    return obstacles;
}

// 맵을 관리하고 업데이트하는 함수
void updateMap(std::vector<int8_t>& map, int width, int height, const std::vector<std::pair<float, float>>& obstacles) {
    for (auto& obstacle : obstacles) {
        int x = static_cast<int>(obstacle.first + width / 2);
        int y = static_cast<int>(obstacle.second + height / 2);
        if (x >= 0 && x < width && y >= 0 && y < height) {
            map[y * width + x] = 100;
        }
    }
}

// 맵을 발행하는 함수
void publishMap(ros::Publisher& map_pub, const std::vector<int8_t>& map, int width, int height) {
    nav_msgs::OccupancyGrid grid;
    grid.header.stamp = ros::Time::now();
    grid.header.frame_id = "map";
    grid.info.width = width;
    grid.info.height = height;
    grid.data = map;
    map_pub.publish(grid);
}

// 라이다 콜백 함수
void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan, ros::Publisher& map_pub, std::vector<int8_t>& map, int width, int height) {
    auto obstacles = processScan(scan);
    updateMap(map, width, height, obstacles);
    publishMap(map_pub, map, width, height);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "simple_slam");
    ros::NodeHandle nh;

    std::vector<int8_t> map(100 * 100, -1); // 맵 초기화
    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 10);
    ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, boost::bind(lidarCallback, _1, boost::ref(map_pub), boost::ref(map), 100, 100));

    ros::spin();
    return 0;
}
