#ifndef COLLISION_DETECTION
#define COLLISION_DETECTION

#include <iostream>
#include <cmath>

#include "multi_crane_cooperator/crane_utility.h"

struct Point3D
{
    double x;
    double y;
    double z;
};

struct Segment3D
{
    Point3D p1;
    Point3D p2;
};

double distance(const Point3D& p1, const Point3D& p2) 
{
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));
}

double dotProduct(const Point3D& u, const Point3D& v) 
{
    return u.x * v.x + u.y * v.y + u.z * v.z;
}

double distancePointToSegment3D(const Point3D& P, const Point3D& A, const Point3D& B, Point3D& closest_Q)
{
    double ABx = B.x - A.x;
    double ABy = B.y - A.y;
    double ABz = B.z - A.z;
    double APx = P.x - A.x;
    double APy = P.y - A.y;
    double APz = P.z - A.z;

    double AB_squared = ABx * ABx + ABy * ABy + ABz * ABz; // 线段 AB 的平方长度
    if (AB_squared == 0.0) {
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

double shortestDistanceBetweenSegments(const Point3D& p1, const Point3D& p2, const Point3D& q1, const Point3D& q2) 
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
        sc = 0.0; // 假设 sc = 0
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

    Point3D closest_p = {p1.x + sc*u.x, p1.y + sc*u.y, p1.z + sc*u.z};
    Point3D closest_q = {q1.x + tc*v.x, q1.y + tc*v.y, q1.z + tc*v.z};
    Point3D dP = {closest_p.x - closest_q.x,
                  closest_p.y - closest_q.y,
                  closest_p.z - closest_q.z};
    double distance = std::sqrt(dotProduct(dP, dP));
    double dist_tmp;
    Point3D update_p, update_q;
    // if closest points are out of the segments
    if(sc == 0 || sc == 1)
    {
        dist_tmp = distancePointToSegment3D(closest_p, q1, q2, update_q);
        if(dist_tmp < distance)
        {
            distance = dist_tmp;
            closest_q = update_q;
        }     
    }
    if(tc == 0 || tc == 1)
    {
        dist_tmp = distancePointToSegment3D(closest_q, p1, p2, update_p);
        if(dist_tmp < distance)
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

void obtainSegmentsLJC(const LuffingJibCraneConfig& crane, Segment3D & jib_line, Segment3D & hook_cable)
{
    jib_line.p1.x = crane.x + crane.d * cos(crane.slewing_angle / 180 * M_PI);
    jib_line.p1.y = crane.y + crane.d * sin(crane.slewing_angle / 180 * M_PI);
    jib_line.p1.z = crane.h;

    jib_line.p2.x = jib_line.p1.x + crane.jib_length * cos(crane.jib_angle / 180 * M_PI) * cos(crane.slewing_angle / 180 * M_PI);
    jib_line.p2.y = jib_line.p1.y + crane.jib_length * cos(crane.jib_angle / 180 * M_PI) * sin(crane.slewing_angle / 180 * M_PI);
    jib_line.p2.z = jib_line.p1.z + crane.jib_length * sin(crane.jib_angle / 180 * M_PI);

    hook_cable.p1 = jib_line.p2;
    hook_cable.p2.x = hook_cable.p1.x;
    hook_cable.p2.y = hook_cable.p1.y;
    hook_cable.p2.z = hook_cable.p1.z - crane.hoisting_height;
}

void obtainSegmentsTC(const TowerCraneConfig& crane, Segment3D & jib_line, Segment3D & hook_cable)
{
    jib_line.p1.x = crane.x;
    jib_line.p1.y = crane.y;
    jib_line.p1.z = crane.h;

    jib_line.p2.x = crane.x + crane.jib_length * cos(crane.slewing_angle / 180 * M_PI);
    jib_line.p2.y = crane.y + crane.jib_length * sin(crane.slewing_angle / 180 * M_PI);
    jib_line.p2.z = crane.h;

    hook_cable.p1.x = crane.x + crane.trolley_radius * cos(crane.slewing_angle / 180 * M_PI);
    hook_cable.p1.y = crane.y + crane.trolley_radius * sin(crane.slewing_angle / 180 * M_PI);
    hook_cable.p2.z = crane.h;

    hook_cable.p2.x = hook_cable.p1.x;
    hook_cable.p2.y = hook_cable.p1.y;
    hook_cable.p2.z = hook_cable.p1.z - crane.hoisting_height;
}

double calculateMinimalDistance(const Segment3D& jib_seg1, const Segment3D& hook_seg1, const Segment3D& jib_seg2, const Segment3D& hook_seg2)
{
    double min_dist = 1000000.0;
    
    double tmp = shortestDistanceBetweenSegments(jib_seg1.p1, jib_seg1.p2, jib_seg2.p1, jib_seg2.p2); 
    if(tmp < min_dist)
        min_dist = tmp;

    tmp = shortestDistanceBetweenSegments(jib_seg1.p1, jib_seg1.p2, hook_seg2.p1, hook_seg2.p2);
    if(tmp < min_dist)
        min_dist = tmp;

    tmp = shortestDistanceBetweenSegments(hook_seg1.p1, hook_seg1.p2, jib_seg2.p1, jib_seg2.p2);
    if(tmp < min_dist)
        min_dist = tmp;

    tmp = shortestDistanceBetweenSegments(hook_seg1.p1, hook_seg1.p2, hook_seg2.p1, hook_seg2.p2);
    if(tmp < min_dist)
        min_dist = tmp;

    return min_dist;
}

double checkCollisionBetweenLJCs(const LuffingJibCraneConfig& crane1,const LuffingJibCraneConfig& crane2)
{
    // obtain the position of the jib and the hook
    Segment3D jib_seg1, hook_seg1, jib_seg2, hook_seg2;

    obtainSegmentsLJC(crane1, jib_seg1, hook_seg1);

    obtainSegmentsLJC(crane2, jib_seg2, hook_seg2);

    double distance =  calculateMinimalDistance(jib_seg1, hook_seg1, jib_seg2, hook_seg2);

    return distance;
    // if(distance < threshold)
    //     return true;
    // else
    //     return false;
}

double checkCollisionBetweenTCs(const TowerCraneConfig& crane1, const TowerCraneConfig& crane2)
{
    // obtain the position of the jib and the hook
    Segment3D jib_seg1, hook_seg1, jib_seg2, hook_seg2;

    obtainSegmentsTC(crane1, jib_seg1, hook_seg1);

    obtainSegmentsTC(crane2, jib_seg2, hook_seg2);

    double distance =  calculateMinimalDistance(jib_seg1, hook_seg1, jib_seg2, hook_seg2);

    // if(distance < threshold)
    //     return true;
    // else
    //     return false;
    return distance;
}

double checkCollisionBetweenMixCranes(const LuffingJibCraneConfig& crane1, const TowerCraneConfig& crane2)
{
        // obtain the position of the jib and the hook
    Segment3D jib_seg1, hook_seg1, jib_seg2, hook_seg2;

    obtainSegmentsLJC(crane1, jib_seg1, hook_seg1);

    obtainSegmentsTC(crane2, jib_seg2, hook_seg2);

    double distance =  calculateMinimalDistance(jib_seg1, hook_seg1, jib_seg2, hook_seg2);

    // if(distance < threshold)
    //     return true;
    // else
    //     return false;
    return distance;
}

#endif