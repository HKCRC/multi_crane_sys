#ifndef COLLISION_DETECTION
#define COLLISION_DETECTION

#include <iostream>
#include <cmath>
#include <utility> 

#include <yaml-cpp/yaml.h>
#include "multi_crane_sys/crane_utility.h"


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

class CraneAntiCollision
{
public:
    CraneAntiCollision() {}
    ~CraneAntiCollision() {}
    bool loadConfigFile(std::string file_name);
    void findMainCraneID(void);
    // void initAdjacencyMatrix(void);
    bool checkCollisionBetweenCranes(const CraneConfig& crane1, const CraneConfig& crane2, double threshold);
    bool predictCollisionBetweenCranes(const CraneConfig& crane1, const CraneConfig& crane2, double threshold);
    double getDistanceBetweenCranes(const CraneConfig& crane1, const CraneConfig& crane2);
    bool checkCollisionBetweenID(u_int craneID1, u_int craneID2, double threshold);
    bool predictCollisionBetweenID(u_int craneID1, u_int craneID2, double threshold);
    double getDistanceBetweenID(u_int c1, u_int c2);
    std::vector<std::pair<u_int, u_int>> checkCollisionAll(double threshold, bool show_results);
    std::vector<std::pair<u_int, u_int>> predictCollisionAll(double threshold, bool show_results);
    std::vector<std::pair<u_int, u_int>> predictCollisionMainCraneNeighbor(double threshold, bool show_results);
    void showDistanceAll();
    void updateCraneState(u_int craneID, double slew, double jib_trolley, double hoist, double slewing_velocity);


private:
    double distance(const Point3D &p1, const Point3D &p2);

    double dotProduct(const Point3D &u, const Point3D &v);

    double distancePointToSegment3D(const Point3D &P, const Point3D &A, const Point3D &B, Point3D &closest_Q);

    double shortestDistanceBetweenSegments(const Point3D &p1, const Point3D &p2, const Point3D &q1, const Point3D &q2);

    void obtainSegmentsLJC(const CraneConfig &crane, Segment3D &jib_line, Segment3D &hook_cable);

    void obtainSegmentsTC(const CraneConfig &crane, Segment3D &jib_line, Segment3D &hook_cable);

    double calculateMinimalDistance(const Segment3D &jib_seg1, const Segment3D &hook_seg1, const Segment3D &jib_seg2, const Segment3D &hook_seg2);

    // void generate_prediction_sequence(const CraneConfig& crane, std::vector<CraneConfig>& crane_sequence);

    void generate_prediction_sequence(const CraneConfig& crane, std::vector<CraneConfig>& crane_sequence, double sequence_time);

    void calculate_braking_time(const CraneConfig& crane1, const CraneConfig& crane2, double& braking_time);

public:
 std::vector<CraneConfig> crane_list_;
 long unsigned int crane_num=0, ljc_num=0, tc_num=0;
 long unsigned int main_crane_id;
 std::vector<std::pair<u_int, u_int>> predict_collision_crane_pairs;
//  std::vector<std::vector<double>> braking_time_adj_mat; //adjacency matrix for braking time
//  std::vector<std::vector<double>> collision_prediction_adj_mat; //adjacency matrix for ollision prediction status
};

#endif