#ifndef OpenProjectSimulator_h
#define OpenProjectSimulator_h
#include "Simulator.h"
#include "collisionDetect.h"


struct RigidBody {
    RigidBody(Vec3 position, Vec3 size, float mass)
        : position(position), size(size), mass(mass) {
    }
    Vec3 position;
    Vec3 linearVelocity;
    Vec3 angularVelocity;
    Vec3 size;  //  (width, depth, height)
    Vec3 angularMomentum;
    float mass;
    Quat orientation;
    Vec3 totalForce; //  total force acting on center of mass
    bool isFixed = false;
    bool hasGravity = false;
    Mat4 worldMatrix;
    bool canCollide = true;
    bool destroyed = false;
};

struct Spring {
    Spring(RigidBody& rb1, RigidBody& rb2, float initialLength) :
        rb1(rb1), rb2(rb2), initialLength(initialLength) {
    }
    RigidBody& rb1, & rb2;
    float initialLength;
};

struct force {
    force(Vec3 forceApplied, Vec3 applicationPoint, int applicationBody)
        : forceApplied(forceApplied), applicationPoint(applicationPoint), applicationBody(applicationBody) {
    }
    Vec3 forceApplied;
    Vec3 applicationPoint;
    int applicationBody;
};


class OpenProjectSimulator :public Simulator {
public:
    // Construtors
    OpenProjectSimulator();

    // UI Functions
    void initUI(DrawingUtilitiesClass* DUC);
    void reset();
    void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
    void notifyCaseChanged(int testCase);
    void externalForcesCalculations(float timeElapsed);
    void computeForces();
    void simulateTimestep(float timeStep);
    void addSpring(int masspoint1, int masspoint2, float initialLength);
    int addRigidBody(Vec3 position, Vec3 size, int mass);
    void calcImpulse(CollisionInfo info, RigidBody& rbA, RigidBody& rbB, int c);
    void onClick(int x, int y);
    void onMouse(int x, int y);

    // Specific Functions
    const char* getTestCasesStr();
    void applyExternalForce(Vec3 force);

private:
    // Attributes
    // add your RigidBodySystem data members, for e.g.,
    // RigidBodySystem * m_pRigidBodySystem; 
    Vec3 m_CmPosition;
    Vec3 m_CmVelocity;
    float m_fMass;
    float m_fStiffness;
    float m_fDamping;
    float m_fGravity;
    Vec3 m_externalForce;
    int castlesDestroyed;

    std::vector<Spring> springs;
    std::vector<RigidBody> RigidBodies;
    std::vector<force> forces;

    // UI Attributes
    Vec3 m_springColor;
    Point2D m_mouse;
    Point2D m_trackmouse;
    Point2D m_oldtrackmouse;
};
#endif