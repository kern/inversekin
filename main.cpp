#include <GLUT/glut.h>
#include <OpenGL/glu.h>

#include "arm.h"
#include "segment.h"

using namespace Eigen;

Arm arm;
double t = 0;
double cameraX = 0;
double cameraY = -25;
double cameraZ = 5;

void initialize() {
    glEnable(GL_POINT_SMOOTH);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glPointSize(8);
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60, 1, 0.5, 50);
    
    arm.addSegment(Segment(3));
    arm.addSegment(Segment(4));
    arm.addSegment(Segment(5));
    arm.addSegment(Segment(1));
}

Vector3f updateGoal() {
    const double tInc = .01;
    const int maxT = 10000;
    const double length = 4.5;
    const Vector3f offset = Vector3f(1, 0, 1);
    
    t += tInc;
    if (t > maxT) t = 0;
    
    double x = (2 + cos(2 * t)) * cos(3 * t) * length;
    double y = (2 + cos(2 * t)) * sin(3 * t) * length;
    double z = sin(4 * t) * length;
    
    Vector3f goal(x, y, z);
    return goal;
}

void drawGoal(Vector3f goal) {
    glBegin(GL_POINTS);
    glColor3f(1, 0, 0);
    glVertex3f(goal.x(), goal.y(), goal.z());
    glEnd();
}

void displayHandler() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(cameraX, cameraY, cameraZ, cameraX, cameraY + 1, cameraZ, 0, 0, 1);
    
    Vector3f goal = updateGoal();
    drawGoal(goal);
    arm.solve(goal);
    arm.draw();
    
    glFlush();
    glutSwapBuffers();
    glutPostRedisplay();
}

void reshapeHandler(int width, int height) {
	glViewport(0, 0, width, height);
}

void keyboardBasicHandler(unsigned char key, int x, int y) {
    switch (key) {
        case ' ': exit(0); break;
        case '+': cameraY += 0.5; break;
        case '-': cameraY -= 0.5; break;
        default: return;
    }
}

void keyboardSpecialHandler(int key, int x, int y) {
    switch (key) {
        case GLUT_KEY_LEFT:  cameraX += 0.25; break;
        case GLUT_KEY_RIGHT: cameraX -= 0.25; break;
        case GLUT_KEY_UP:    cameraZ -= 0.25; break;
        case GLUT_KEY_DOWN:  cameraZ += 0.25; break;
    }
}

int main(int argc, char **argv) {
	glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(640, 480);
    glutInitWindowPosition(0, 0);
    
    glutCreateWindow("inversekin");
    glutDisplayFunc(displayHandler);
    glutReshapeFunc(reshapeHandler);
    glutKeyboardFunc(keyboardBasicHandler);
    glutSpecialFunc(keyboardSpecialHandler);
    
    initialize();
    glutMainLoop();
    return 0;  
}
