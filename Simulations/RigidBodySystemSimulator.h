#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h
#include "Simulator.h"
//add your header for your rigid body system, for e.g.,
//#include "rigidBodySystem.h" 

#define TESTCASEUSEDTORUNTEST 2

struct RigidBody {
	RigidBody(Vec3 position, Vec3 size, int mass)
		: position(position), size(size), mass(mass) {
	}
	Vec3 position;
	Vec3 linearVelocity;
	Vec3 angularVelocity;
	Vec3 size;  //  (width, depth, height)
	Vec3 angularMomentum;
	int mass;
	Quat orientation;
	Vec3 totalForce; //  total force acting on center of mass
};

struct force {
	force(Vec3 forceApplied, Vec3 applicationPoint, int applicationBody)
		: forceApplied(forceApplied), applicationPoint(applicationPoint), applicationBody(applicationBody){
	}
	Vec3 forceApplied;
	Vec3 applicationPoint;
	int applicationBody;
};

class RigidBodySystemSimulator:public Simulator{
public:
	// Construtors
	RigidBodySystemSimulator();
	
	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// ExtraFunctions
	int getNumberOfRigidBodies();
	Vec3 getPositionOfRigidBody(int i);
	Vec3 getLinearVelocityOfRigidBody(int i);
	Vec3 getAngularVelocityOfRigidBody(int i);
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);
	void addRigidBody(Vec3 position, Vec3 size, int mass);
	void setOrientationOf(int i,Quat orientation);
	void setVelocityOf(int i, Vec3 velocity);

private:
	// Attributes
	// add your RigidBodySystem data members, for e.g.,
	// RigidBodySystem * m_pRigidBodySystem; 
	Vec3 m_CmPosition;
	Vec3 m_CmVelocity;

	float m_fGravity;
	Vec3 m_externalForce;
	std::vector<RigidBody> RigidBodies;
	std::vector<force> forces;

	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	};
#endif