#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <cmath>

class LidarProcessor {
public:
    LidarProcessor() {}

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
};


class MapManager {
public:
    MapManager(int width, int height) : width_(width), height_(height) {
        map_.resize(width_ * height_, -1);
    }

    void updateMap(const std::vector<std::pair<float, float>>& obstacles) {
        for (auto& obstacle : obstacles) {
            int x = static_cast<int>(obstacle.first + width_ / 2); // 맵 중앙을 기준으로 위치 조정
            int y = static_cast<int>(obstacle.second + height_ / 2); // 맵 중앙을 기준으로 위치 조정
            if (x >= 0 && x < width_ && y >= 0 && y < height_) {
                map_[y * width_ + x] = 100; 
            }
        }
    }

    void publishMap(ros::Publisher& map_pub) {
        nav_msgs::OccupancyGrid grid;
        grid.header.stamp = ros::Time::now();
        grid.header.frame_id = "map";
        grid.info.width = width_;
        grid.info.height = height_;
        grid.info.origin.position.x = 0; 
        grid.info.origin.position.y = 0;
        grid.data.assign(map_.begin(), map_.end());
        map_pub.publish(grid);
    }

private:
    int width_, height_;
    std::vector<int8_t> map_;
};

class SimpleSLAM {
public:
    SimpleSLAM(ros::NodeHandle& nh) : nh_(nh) {
        lidar_processor_ = std::make_unique<LidarProcessor>();
        map_manager_ = std::make_unique<MapManager>(100, 100); 
        lidar_sub_ = nh_.subscribe("/scan", 1000, &SimpleSLAM::lidarCallback, this);
        map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 10);
    }

    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        auto obstacles = lidar_processor_->processScan(scan);
        map_manager_->updateMap(obstacles);
        map_manager_->publishMap(map_pub_);
    }

    void run() {
        ros::spin();
    }

private:
    ros::NodeHandle nh_;
    std::unique_ptr<LidarProcessor> lidar_processor_;
    std::unique_ptr<MapManager> map_manager_;
    ros::Subscriber lidar_sub_;
    ros::Publisher map_pub_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "simple_slam");
    ros::NodeHandle nh;

    SimpleSLAM slam(nh);
    slam.run();

    return 0;
}
