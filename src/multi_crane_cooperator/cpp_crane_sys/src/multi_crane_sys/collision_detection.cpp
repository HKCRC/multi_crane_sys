#include <iomanip>
#include "multi_crane_sys/crane_utility.h"
#include "multi_crane_sys/collision_detection.hpp"

bool CraneAntiCollision::loadConfigFile(std::string file_name)
{
    YAML::Node config = YAML::LoadFile(file_name);

    if (!config)
    {
        std::cout << "Open config file: " << file_name << " failed" << std::endl;
        return false;
    }

    if (config["crane_list"])
    {
        CraneConfig crane_config;
        int ljc_num = 0, tc_num = 0;
        for (const auto &crane : config["crane_list"])
        {
            crane_config.type = crane["parameters"]["type"].as<int>();
            crane_config.x = crane["parameters"]["x"].as<double>();
            crane_config.y = crane["parameters"]["y"].as<double>();
            crane_config.h = crane["parameters"]["h"].as<double>();
            crane_config.d = crane["parameters"]["d"].as<double>();
            crane_config.jib_length = crane["parameters"]["jib_length"].as<double>();
            crane_config.slewing_angle = 0;
            crane_config.trolley_radius_jib_angle = 0;
            crane_config.hoisting_height = crane_config.h;
            crane_config.slewing_velocity = 0;
            crane_list_.push_back(crane_config);
            if(crane_config.type == 0)
                ljc_num++;
            else
                tc_num++;
        }
        std::cout << "Load " << ljc_num << " luffing jib cranes and " << tc_num << " tower cranes successfully!" << std::endl;  
    }

    return true;
}

double CraneAntiCollision::getDistanceBetweenID(u_int craneID1, u_int craneID2)
{
    if(craneID1 >= crane_list_.size() || craneID2 >= crane_list_.size())
    {
        std::cout<<"The crane ID is out of range"<<std::endl;
        return 0;
    }

    CraneConfig crane1 = crane_list_[craneID1];
    CraneConfig crane2 = crane_list_[craneID2];

    double distance = getDistanceBetweenCranes(crane1, crane2);
    
    return distance;
}

bool CraneAntiCollision::checkCollisionBetweenID(u_int craneID1, u_int craneID2, double threshold)
{ 
    if(craneID1 >= crane_list_.size() || craneID2 >= crane_list_.size())
    {
        std::cout<<"The crane ID is out of range"<<std::endl;
        return false;
    }

    CraneConfig crane1 = crane_list_[craneID1];
    CraneConfig crane2 = crane_list_[craneID2];
    double distance = getDistanceBetweenCranes(crane1, crane2);

    if(distance < threshold)
        return true;
    else
        return false;
}

bool CraneAntiCollision::predictCollisionBetweenID(u_int craneID1, u_int craneID2, double threshold)
{ 
    if(craneID1 >= crane_list_.size() || craneID2 >= crane_list_.size())
    {
        std::cout<<"The crane ID is out of range"<<std::endl;
        return false;
    }

    CraneConfig crane1 = crane_list_[craneID1];
    CraneConfig crane2 = crane_list_[craneID2];

    std::vector<CraneConfig> crane1_sequence;
    std::vector<CraneConfig> crane2_sequence;
    generate_prediction_sequence(crane1, crane1_sequence);
    generate_prediction_sequence(crane2, crane2_sequence);

    int size = std::min(crane1_sequence.size(), crane2_sequence.size()); 
    for(int i = 0; i < size; i++)
    {
        if(checkCollisionBetweenCranes(crane1_sequence[i], crane2_sequence[i], threshold))
            return true;
    }
    return false;
}

std::vector<std::pair<u_int, u_int>> CraneAntiCollision::checkCollisionAll(double threshold, bool show_results)
{
    std::vector<std::pair<u_int, u_int>> crane_pairs;

    for(long unsigned int i = 0; i < crane_list_.size(); i++)
    {
        for(long unsigned int j = i+1; j < crane_list_.size(); j++)
        {
            if(checkCollisionBetweenID(i, j, threshold))
                crane_pairs.push_back(std::make_pair(i+1, j+1));
        }
    }
    if(show_results)
    {
        for(auto pair : crane_pairs)
        {
            std::cout<<"WARNNING: Crane "<<pair.first<<" and Crane "<<pair.second<<" are too close!"<<std::endl;
        }
    }
    return crane_pairs;
}

std::vector<std::pair<u_int, u_int>> CraneAntiCollision::predictCollisionAll(double threshold, bool show_results)
{
    std::vector<std::pair<u_int, u_int>> crane_pairs;

    for(long unsigned int i = 0; i < crane_list_.size(); i++)
    {
        for(long unsigned int j = i+1; j < crane_list_.size(); j++)
        {
            if(predictCollisionBetweenID(i, j, threshold))
                crane_pairs.push_back(std::make_pair(i+1, j+1));
        }
    }
    if(show_results)
    {
        for(auto pair : crane_pairs)
        {
            std::cout<<"WARNNING: Crane "<<pair.first<<" and Crane "<<pair.second<<" are predicted to be too close!"<<std::endl;
        }
    }
    return crane_pairs;
}

void CraneAntiCollision::showDistanceAll()
{
    double dist;
    std::cout<< "cranes \t";
    for(long unsigned int i = 0; i < crane_list_.size(); i++)
        std::cout<< "Crane " << i+1 << "\t";
    std::cout<<std::endl;

    for(long unsigned int i = 0; i < crane_list_.size(); i++)
    {
      std::cout<< "Crane " << i+1 << ": ";
      for(long unsigned int j = 0; j < crane_list_.size(); j++)
      {
        dist = getDistanceBetweenID(i, j);
        std::cout<< std::fixed << std::setprecision(3) << dist << "\t";
      }
      std::cout<<std::endl;
    }
      
}
bool CraneAntiCollision::checkCollisionBetweenCranes(const CraneConfig& crane1, const CraneConfig& crane2, double threshold)
{
    double distance = getDistanceBetweenCranes(crane1, crane2);

    if(distance < threshold)
        return true;
    else
        return false;
}

bool CraneAntiCollision::predictCollisionBetweenCranes(const CraneConfig& crane1, const CraneConfig& crane2, double threshold)
{
    std::vector<CraneConfig> crane1_sequence;
    std::vector<CraneConfig> crane2_sequence;
    generate_prediction_sequence(crane1, crane1_sequence);
    generate_prediction_sequence(crane2, crane2_sequence);

    int size = std::min(crane1_sequence.size(), crane2_sequence.size()); 
    for(int i = 0; i < size; i++)
    {
        if(checkCollisionBetweenCranes(crane1_sequence[i], crane2_sequence[i], threshold))
            return true;
    }
    return false;
}

double CraneAntiCollision::getDistanceBetweenCranes(const CraneConfig& crane1, const CraneConfig& crane2)
{
    Segment3D jib_seg1, hook_seg1, jib_seg2, hook_seg2;

    if(crane1.type == 0)
    {
        obtainSegmentsLJC(crane1, jib_seg1, hook_seg1);
    }
    else
    {
        obtainSegmentsTC(crane1, jib_seg1, hook_seg1);
    }
    if(crane2.type == 0)
    {
        obtainSegmentsLJC(crane2, jib_seg2, hook_seg2);
    }
    else
    {
        obtainSegmentsTC(crane2, jib_seg2, hook_seg2);
    }

    return calculateMinimalDistance(jib_seg1, hook_seg1, jib_seg2, hook_seg2);
}

void CraneAntiCollision::updateCraneState(u_int craneID, double slew, double jib_trolley, double hoist, double slewing_velocity)
{
    if (craneID >= crane_list_.size())
    {
        std::cout << "The crane ID is out of range" << std::endl;
        return;
    }
    crane_list_[craneID].slewing_angle = slew;
    crane_list_[craneID].trolley_radius_jib_angle = jib_trolley;
    crane_list_[craneID].hoisting_height = hoist;
    crane_list_[craneID].slewing_velocity = slewing_velocity;
}


double CraneAntiCollision::distance(const Point3D &p1, const Point3D &p2)
{
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));
}

double CraneAntiCollision::dotProduct(const Point3D &u, const Point3D &v)
{
    return u.x * v.x + u.y * v.y + u.z * v.z;
}

double CraneAntiCollision::distancePointToSegment3D(const Point3D &P, const Point3D &A, const Point3D &B, Point3D &closest_Q)
{
    double ABx = B.x - A.x;
    double ABy = B.y - A.y;
    double ABz = B.z - A.z;
    double APx = P.x - A.x;
    double APy = P.y - A.y;
    double APz = P.z - A.z;

    double AB_squared = ABx * ABx + ABy * ABy + ABz * ABz; // 线段 AB 的平方长度
    if (AB_squared == 0.0)
    {
        // A 和 B 重合，直接返回 P 到 A 的距离
        return std::sqrt(APx * APx + APy * APy + APz * APz);
    }

    // 计算参数 t
    double t = (APx * ABx + APy * ABy + APz * ABz) / AB_squared;
    t = std::max(0.0, std::min(1.0, t)); // 限制 t 在 [0, 1] 范围内

    // 计算最近点
    closest_Q.x = A.x + t * ABx;
    closest_Q.y = A.y + t * ABy;
    closest_Q.z = A.z + t * ABz;

    // 返回点 P 到最近点的距离
    double dx = P.x - closest_Q.x;
    double dy = P.y - closest_Q.y;
    double dz = P.z - closest_Q.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double CraneAntiCollision::shortestDistanceBetweenSegments(const Point3D &p1, const Point3D &p2, const Point3D &q1, const Point3D &q2)
{
    Point3D u = {p2.x - p1.x, p2.y - p1.y, p2.z - p1.z}; // L1 的方向向量
    Point3D v = {q2.x - q1.x, q2.y - q1.y, q2.z - q1.z}; // L2 的方向向量
    Point3D w = {p1.x - q1.x, p1.y - q1.y, p1.z - q1.z}; // L1 起点到 L2 起点的向量

    double a = dotProduct(u, u); // u·u
    double b = dotProduct(u, v); // u·v
    double c = dotProduct(v, v); // v·v
    double d = dotProduct(u, w); // u·w
    double e = dotProduct(v, w); // v·w

    double D = a * c - b * b; // 标量三重积
    double sc, tc;
    // 如果 D 很小，说明两线段平行或接近平行
    if (D < 1e-8)
    {
        sc = 0.0;                     // 假设 sc = 0
        tc = (b > c ? d / b : e / c); // 投影到 v 上
    }
    else
    {
        // 计算参数 sc 和 tc
        sc = (b * e - c * d) / D;
        tc = (a * e - b * d) / D;
    }

    // 限制 sc 和 tc 在 [0, 1] 之间
    sc = std::max(0.0, std::min(1.0, sc));
    tc = std::max(0.0, std::min(1.0, tc));

    Point3D closest_p = {p1.x + sc * u.x, p1.y + sc * u.y, p1.z + sc * u.z};
    Point3D closest_q = {q1.x + tc * v.x, q1.y + tc * v.y, q1.z + tc * v.z};
    Point3D dP = {closest_p.x - closest_q.x,
                  closest_p.y - closest_q.y,
                  closest_p.z - closest_q.z};
    double distance = std::sqrt(dotProduct(dP, dP));
    double dist_tmp;
    Point3D update_p, update_q;
    // if closest points are out of the segments
    if (sc == 0 || sc == 1)
    {
        dist_tmp = distancePointToSegment3D(closest_p, q1, q2, update_q);
        if (dist_tmp < distance)
        {
            distance = dist_tmp;
            closest_q = update_q;
        }
    }
    if (tc == 0 || tc == 1)
    {
        dist_tmp = distancePointToSegment3D(closest_q, p1, p2, update_p);
        if (dist_tmp < distance)
        {
            distance = dist_tmp;
            closest_p = update_p;
        }
    }

    dP.x = closest_p.x - closest_q.x;
    dP.y = closest_p.y - closest_q.y;
    dP.z = closest_p.z - closest_q.z;

    // std::cout<<"closest P: " << closest_p.x << ", " << closest_p.y << ", "<<closest_p.z << std::endl;
    // std::cout<<"closest Q: " << closest_q.x << ", " << closest_q.y << ", "<<closest_q.z << std::endl;
    return std::sqrt(dotProduct(dP, dP));
}

void CraneAntiCollision::obtainSegmentsLJC(const CraneConfig &crane, Segment3D &jib_line, Segment3D &hook_cable)
{
    jib_line.p1.x = crane.x + crane.d * cos(crane.slewing_angle / 180 * M_PI);
    jib_line.p1.y = crane.y + crane.d * sin(crane.slewing_angle / 180 * M_PI);
    jib_line.p1.z = crane.h;

    jib_line.p2.x = jib_line.p1.x + crane.jib_length * cos(crane.trolley_radius_jib_angle / 180 * M_PI) * cos(crane.slewing_angle / 180 * M_PI);
    jib_line.p2.y = jib_line.p1.y + crane.jib_length * cos(crane.trolley_radius_jib_angle / 180 * M_PI) * sin(crane.slewing_angle / 180 * M_PI);
    jib_line.p2.z = jib_line.p1.z + crane.jib_length * sin(crane.trolley_radius_jib_angle / 180 * M_PI);

    hook_cable.p1 = jib_line.p2;
    hook_cable.p2.x = hook_cable.p1.x;
    hook_cable.p2.y = hook_cable.p1.y;
    hook_cable.p2.z = hook_cable.p1.z - crane.hoisting_height;
}

void CraneAntiCollision::obtainSegmentsTC(const CraneConfig &crane, Segment3D &jib_line, Segment3D &hook_cable)
{
    jib_line.p1.x = crane.x;
    jib_line.p1.y = crane.y;
    jib_line.p1.z = crane.h;

    jib_line.p2.x = crane.x + crane.jib_length * cos(crane.slewing_angle / 180 * M_PI);
    jib_line.p2.y = crane.y + crane.jib_length * sin(crane.slewing_angle / 180 * M_PI);
    jib_line.p2.z = crane.h;

    hook_cable.p1.x = crane.x + crane.trolley_radius_jib_angle * cos(crane.slewing_angle / 180 * M_PI);
    hook_cable.p1.y = crane.y + crane.trolley_radius_jib_angle * sin(crane.slewing_angle / 180 * M_PI);
    hook_cable.p1.z = crane.h;

    hook_cable.p2.x = hook_cable.p1.x;
    hook_cable.p2.y = hook_cable.p1.y;
    hook_cable.p2.z = hook_cable.p1.z - crane.hoisting_height;
}

double CraneAntiCollision::calculateMinimalDistance(const Segment3D &jib_seg1, const Segment3D &hook_seg1, const Segment3D &jib_seg2, const Segment3D &hook_seg2)
{
    double min_dist = 1000000.0;

    double tmp = shortestDistanceBetweenSegments(jib_seg1.p1, jib_seg1.p2, jib_seg2.p1, jib_seg2.p2);
    if (tmp < min_dist)
        min_dist = tmp;

    tmp = shortestDistanceBetweenSegments(jib_seg1.p1, jib_seg1.p2, hook_seg2.p1, hook_seg2.p2);
    if (tmp < min_dist)
        min_dist = tmp;

    tmp = shortestDistanceBetweenSegments(hook_seg1.p1, hook_seg1.p2, jib_seg2.p1, jib_seg2.p2);
    if (tmp < min_dist)
        min_dist = tmp;

    tmp = shortestDistanceBetweenSegments(hook_seg1.p1, hook_seg1.p2, hook_seg2.p1, hook_seg2.p2);
    if (tmp < min_dist)
        min_dist = tmp;

    return min_dist;
}
// this function is assume cranes in crame swarm has the same braking_time (is not correct, need refine)
void CraneAntiCollision::generate_prediction_sequence(const CraneConfig& crane, std::vector<CraneConfig>& crane_sequence) 
{   

    double slewing_position = crane.slewing_angle;
    double slewing_velocity = crane.slewing_velocity;
    // double braking_distance;

    //case1: no consider deceleration 
    double braking_time = 3.5284*abs(slewing_velocity) + 2.3791; //fitted by Boyuan's 3 data pairs,need to be verified
    
    //case2: cnsider deceleration; gear1 dec = 0.2068 deg/s^2, gear2 = 0.2949 deg/s^2, gear3= 0.3090 deg/s^2
    // double deceleration = -0.2068;
    // double braking_time = slewing_velocity + sqrt(pow(slewing_velocity,2) - 2*deceleration*slewing_position); 
    
    double sequence_timestep = 1.0f;
    int sequence_size = int(braking_time / sequence_timestep);
    crane_sequence.resize(sequence_size);

    for (int i = 0; i <= sequence_size-1; i++) {
        crane_sequence[i] = crane;
        // only predict slewing_angle element
        slewing_position =  slewing_position + slewing_velocity*sequence_timestep;
        crane_sequence[i].slewing_angle = slewing_position;
    }
    // std::cout<<"crane_state:" << crane <<std::endl; 
    // print_TC_stdVector(tower_crane_sequence);
}