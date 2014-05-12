#include <GLUT/glut.h>
#include <OpenGL/glu.h>

#include "segment.h"

#define PI 3.1415926535897

using namespace std;
using namespace Eigen;

Segment::Segment(double l) : length(l) {
    savedTransformation = transformation = transformation.Identity();
}

double Segment::getLength() {
    return length;
}

Vector3f Segment::getEnd() {
    return transformation * Vector3f(0, 0, length);
}

Vector3f Segment::getX() {
    return transformation * Vector3f(1, 0, 0);
}

Vector3f Segment::getY() {
    return transformation * Vector3f(0, 1, 0);
}

Vector3f Segment::getZ() {
    return transformation * Vector3f(0, 0, 1);
}

void Segment::applyTransformation(Vector3f v) {
    AngleAxisf xTransformation(v.x(), getX());
    AngleAxisf yTransformation(v.y(), getY());
    AngleAxisf zTransformation(v.z(), getZ());
    transformation = xTransformation * yTransformation * zTransformation * transformation;
}

void Segment::saveTransformation() {
    savedTransformation = transformation;
}

void Segment::revertTransformation() {
    transformation = savedTransformation;
}

Vector3f Segment::jacobianColumn(Vector3f angle) {
    const double increment = 0.0005;
    AngleAxisf t = AngleAxisf(increment, angle);
    return (t * getEnd() - getEnd()) / increment;
}

Matrix3f Segment::jacobianMatrix() {
    Matrix3f result;
    result.col(0) = jacobianColumn(getX());
    result.col(1) = jacobianColumn(getY());
    result.col(2) = jacobianColumn(getZ());
    return result;
}

Vector3f Segment::draw(Vector3f origin) {
    const int sides = 5;
    
    Vector3f a = getEnd();
    Vector3f aNorm = a.normalized();
    a = a + origin;
    
    for (int i = 0; i < sides; i++) {
        Vector3f b = transformation * Vector3f(cos((i + 0) * 2 * PI / sides), sin((i + 0) * 2 * PI / sides), 0);
        Vector3f c = transformation * Vector3f(cos((i + 1) * 2 * PI / sides), sin((i + 1) * 2 * PI / sides), 0);
        
        Vector3f bNorm = b.normalized();
        Vector3f cNorm = c.normalized();
        
        b = b + origin;
        c = c + origin;
        
        glBegin(GL_TRIANGLES);
        glColor3f(0, 1, 0);

        glNormal3f(aNorm.x(), aNorm.y(), aNorm.z());
        glVertex3f(a.x(), a.y(), a.z());
        
        glNormal3f(bNorm.x(), bNorm.y(), bNorm.z());
        glVertex3f(b.x(), b.y(), b.z());
        
        glNormal3f(cNorm.x(), cNorm.y(), cNorm.z());
        glVertex3f(c.x(), c.y(), c.z());
        
        glEnd();
    }
    
    return a;
}