#ifndef inversekin_segment_h
#define inversekin_segment_h

#include <Eigen/Dense>

class Segment {
private:
    double length;
    Eigen::AngleAxisf transformation, savedTransformation;
    
    Eigen::Vector3f getX();
    Eigen::Vector3f getY();
    Eigen::Vector3f getZ();

    Eigen::Vector3f jacobianColumn(Eigen::Vector3f angle);
    
public:
    Segment(double l);
    
    double getLength();
    Eigen::Vector3f getEnd();

    void applyTransformation(Eigen::Vector3f v);
    void saveTransformation();
    void revertTransformation();
    
    Eigen::Matrix3f jacobianMatrix();
    Eigen::Vector3f draw(Eigen::Vector3f origin);
};

#endif
