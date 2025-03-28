
# 🗺️ Simple Lidar SLAM (ROS)

Lidar 스캔만을 이용하여 실시간으로 장애물을 추정하고  
Occupancy Grid 맵을 생성하며,  
`base_link`, `map`, `laser` 프레임 간 TF를 브로드캐스팅하는  
**간단한 ROS 기반 SLAM 시스템**입니다.

---

## 🧭 주요 기능 요약

- ✅ `LaserScan` 수신 및 가우시안 필터 적용
- ✅ 장애물 위치 추정 및 Occupancy Grid Map 업데이트
- ✅ 맵 `/map` 토픽으로 퍼블리시 (`nav_msgs/OccupancyGrid`)
- ✅ `base_link → map`, `base_link → laser` 간 TF 변환 브로드캐스트

---

## 🧩 구조 구성도

```txt
/scan (LaserScan)
   ↓
LidarProcessor  ← 가우시안 필터 + 장애물 위치 계산
   ↓
MapManager     ← Occupancy Grid Map 갱신
   ↓
/map (nav_msgs/OccupancyGrid)

TF:
map → base_link → laser
