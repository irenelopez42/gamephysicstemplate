#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h
#include "Simulator.h"


inline Real to_degrees(Real radians) {
	return radians * (180.0 / M_PI);
}

inline Real to_radians(Real degrees) {
	return M_PI * (degrees / 180.0);
}

inline XMFLOAT3X3 rotation_matrix(const Quat& orientation) {
	double angle = 2 * acos(orientation.w);
	Real x = orientation.x, y = orientation.y, z = orientation.z;
	double c = cos(angle), s = sin(angle);
	return XMFLOAT3X3(
		c + (1-c)*pow(x, 2), (1-c)*x*y - s*z, (1-c)*x*z - s*y,
		(1-c)*x*y - s*z, c + (1-c)*pow(y, 2), (1-c)*y*z - s*x,
		(1-c)*x*z - s*y, (1-c)*y*z - s*x, c + (1-c)*pow(z, 2)
	);
}

struct RigidBody {
	RigidBody(Vec3 position, Vec3 size, int mass)
		: position(position), size(size), mass(mass) {
		linear_velocity = Vec3(0, 0, 0);

		orientation = Quat();
		angular_velocity = Vec3(0, 0, 0);
		I_0 = XMFLOAT3X3();
		I_0_inv = XMFLOAT3X3();
	}

	Vec3 size;

	struct ExternalForce {
		Vec3 position, direction;

		ExternalForce(Vec3 position, Vec3 direction)
			: position(position), direction(direction) {
		}
	};

	std::vector<ExternalForce> external_forces;

	Vec3 position, linear_velocity;
	int mass;
	
	Quat orientation;
	Vec3 angular_velocity;
	XMFLOAT3X3 I_0, I_0_inv;
};

#define TESTCASEUSEDTORUNTEST 2

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
	int testCase;
	bool debugPrint;

	int N_bodies;
	std::vector<RigidBody> rigidBodies;

	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	};
#endif