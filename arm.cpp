#include <cfloat>
#include <Eigen/Dense>
#include <GLUT/glut.h>
#include <OpenGL/glu.h>

#include "arm.h"

using namespace Eigen;
using namespace std;

// Constructors
// ============

Arm::Arm() {}

// Utilities
// =========

void Arm::addSegment(Segment s) {
    segments.push_back(s);
}

double Arm::fullLength() {
    double result = 0.0;
    for (int i = 0; i < segments.size(); i++) {
        result += segments[i].getLength();
    }
    
    return result;
}

Vector3f Arm::getEnd() {
    Vector3f result(0, 0, 0);
    for (int i = 0; i < segments.size(); i++) {
        result += segments[i].getEnd();
    }
    
    return result;
}

double Arm::getError(Vector3f goal) {
    return (goal - getEnd()).norm();
}

// Solving
// =======

Vector3f Arm::normalizeGoal(Vector3f goal) {
    if (goal.norm() > fullLength()) {
        return goal.normalized() * fullLength();
    } else {
        return goal;
    }
}

void Arm::applyTransformations(MatrixXf transformations) {
    for (int i = 0; i < segments.size(); i++) {
        Segment &segment = segments[i];
        segment.applyTransformation(transformations.col(i));
    }
}

void Arm::revertTransformations() {
    for (int i = 0; i < segments.size(); i++) {
        Segment &segment = segments[i];
        segment.revertTransformation();
    }
}

void Arm::saveTransformations() {
    for (int i = 0; i < segments.size(); i++) {
        Segment &segment = segments[i];
        segment.saveTransformation();
    }
}

MatrixXf Arm::calculateTransformations(Vector3f goal) {
    MatrixXf jacobian(3, 3 * segments.size());
    MatrixXf pseudoinverse(3 * segments.size(), 3);
    VectorXf transformationsVector(3 * segments.size());

    for (int i = 0; i < segments.size(); i++) {
        Segment &segment = segments[i];
        jacobian.block<3, 3>(0, i * 3) = segment.jacobianMatrix();
    }

    // Based on the pseudoinverse function at http://eigen.tuxfamily.org/bz/show_bug.cgi?id=257
    JacobiSVD<MatrixXf> svd(jacobian, ComputeThinU | Eigen::ComputeThinV);
    double tolerance = std::numeric_limits<double>::epsilon() * std::max(jacobian.cols(), jacobian.rows()) * svd.singularValues().array().abs()(0);
    pseudoinverse = svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
    
    transformationsVector = pseudoinverse * (goal - getEnd());
    
    MatrixXf transformations(3, segments.size());
    for (int i = 0; i < segments.size(); i++) {
        transformations.col(i) = transformationsVector.segment<3>(i * 3);
    }
    
    return transformations;
}

void Arm::solve(Vector3f goal) {
    const double errorMargin = 0.01;
    const int maxIterations = 100;
    const int maxHalvingIterations = 100;
    
    goal = normalizeGoal(goal);
    double bestError = getError(goal);
    
    for (int it = 0; it < maxIterations && bestError > errorMargin; it++) {
        MatrixXf transformations = calculateTransformations(goal);

        double error = DBL_MAX;
        for (int halvingIt = 0; halvingIt <= maxHalvingIterations; halvingIt++) {
            applyTransformations(transformations);
            error = getError(goal);
            
            if (error <= bestError) {
                saveTransformations();
                bestError = error;
                break;
            } else {
                revertTransformations();
                transformations /= 2.0;
            }
        }

        if (error > bestError) return;
    }
}

// Drawing
// =======

void Arm::draw() {
    drawBase();
    drawSegments();
}

void Arm::drawBase() {
    const double baseLength = 5;
    
    glBegin(GL_QUADS);
    glColor3f(0, 0, 1);

    glNormal3f(0, 0, 1);
    glVertex3f(-baseLength, -baseLength, 0);
    
    glNormal3f(0, 0, 1);
    glVertex3f(baseLength, -baseLength, 0);
    
    glNormal3f(0, 0, 1);
    glVertex3f(baseLength, baseLength, 0);
    
    glNormal3f(0, 0, 1);
    glVertex3f(-baseLength, baseLength, 0);
    
    glEnd();
}

void Arm::drawSegments() {
    Vector3f origin(0, 0, 0);
    for (int i = 0; i < segments.size(); i++) {
        origin = segments[i].draw(origin);
    }
}