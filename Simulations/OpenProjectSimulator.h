#ifndef OpenProjectSimulator_h
#define OpenProjectSimulator_h
#include "Simulator.h"

struct MassPoint {
    MassPoint(Vec3 position, Vec3 velocity, bool isFixed)
        : position(position), velocity(velocity), isFixed(isFixed) {
    }
    Vec3 position, oldPosition;  // old position and velocity needed for midpoint
    Vec3 velocity, oldVelocity;
    Vec3 force;
    bool isFixed;
};

struct Spring {
    Spring(MassPoint& mp1, MassPoint& mp2, float initialLength) :
        mp1(mp1), mp2(mp2), initialLength(initialLength) {
    }
    MassPoint& mp1, & mp2;
    float initialLength;
};


class OpenProjectSimulator:public Simulator{
public:
    // Construtors
    OpenProjectSimulator();
    
    // UI Functions
    void initUI(DrawingUtilitiesClass * DUC);
    void reset();
    void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
    void notifyCaseChanged(int testCase);
    void externalForcesCalculations(float timeElapsed);
    void computeForces();
    void simulateTimestep(float timeStep);
    void onClick(int x, int y);
    void onMouse(int x, int y);

    // Specific Functions
    const char* getTestCasesStr();
    void applyExternalForce(Vec3 force);
    
private:
    // Data Attributes

    // UI Attributes
    Point2D m_mouse;
    Point2D m_trackmouse;
    Point2D m_oldtrackmouse;
};
#endif
