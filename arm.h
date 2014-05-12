#ifndef inversekin_arm_h
#define inversekin_arm_h

#include <vector>
#include <Eigen/Dense>

#include "segment.h"

class Arm {
private:
    std::vector<Segment> segments;
    
    double fullLength();
    Eigen::Vector3f normalizeGoal(Eigen::Vector3f goal);
    Eigen::Vector3f getEnd();
    double getError(Eigen::Vector3f goal);

    Eigen::MatrixXf calculateTransformations(Eigen::Vector3f goal);
    Eigen::Vector3f jacobianColumn(int segmentNumber, Eigen::Vector3f angle);
    
    void applyTransformations(Eigen::MatrixXf transformations);
    void revertTransformations();
    void saveTransformations();
    
    void drawBase();
    void drawSegments();
    
public:
    Arm();
    void solve(Eigen::Vector3f goal);
    void draw();
    void addSegment(Segment s);
};

#endif