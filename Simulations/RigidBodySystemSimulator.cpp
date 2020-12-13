#include "..\Simulations\RigidBodySystemSimulator.h"

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	m_externalForce = Vec3(0, 0, 0);
	m_fGravity = 0;

	m_mouse = Point2D();
	m_trackmouse = Point2D();
	m_oldtrackmouse = Point2D();
}

const char* RigidBodySystemSimulator::getTestCasesStr()
{
	return "1-Step, Simple, Collision, Complex";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_FLOAT, &m_fGravity, "min=0.00 max=100.00 step=0.01");
}

void RigidBodySystemSimulator::reset()
{
	(this->rigidBodys).clear();
	(this->forces).clear();
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, Vec3(0.5, 0.5, 0.5));

	for (RigidBody& rb : this->rigidBodys) {
		double x[] = { rb.size.x,0.0,0.0,0.0,
				       0.0,rb.size.y,0.0,0.0,
				       0.0,0.0,rb.size.z,0.0,
				       0.0,0.0,0.0,0.0 };
		double y[] = { 0.0,0.0,0.0,rb.position.x,
					  0.0,0.0,0.0,rb.position.y,
					  0.0,0.0,0.0,rb.position.z,
					  0.0,0.0,0.0,0.0 };
		rb.scalingMatrix = Mat4(0.0);
		rb.translationMatrix = Mat4(0.0);
		rb.scalingMatrix.initFromArray(x);
		rb.translationMatrix.initFromArray(y);
		rb.worldMatrix = rb.scalingMatrix * rb.rotationMatrix * rb.translationMatrix;
		DUC->drawRigidBody(rb.worldMatrix);
	}
}



void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	this->reset();
	switch (testCase) {
	case 0:
		addRigidBody(Vec3(0.0,0.0,0.0), Vec3(1.0, 0.6, 0.5), 2);
		(this->forces).push_back(Force(Vec3(1.0,1.0,1.0), Vec3(0.3, 0.5, 0.25)));
		break;
	case 1:
		addRigidBody(Vec3(0.0, 0.0, 0.0), Vec3(1.0, 0.6, 0.5), 2);
		(this->forces).push_back(Force(Vec3(1.0, 1.0, 1.0), Vec3(0.3, 0.5, 0.25)));
		//set timestep
		break;
	case 2:

		break;
	case 3:

		break;
	}
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	//do the Simulation Algorithm 3D

	//calculate F and torque q

	Vec3 F = Vec3(0, 0, 0);
	Vec3 q = Vec3(0, 0, 0);
	for (Force& f_i : this->forces) {
		F = F + f_i.force;
		q = q + cross(f_i.position, f_i.force);
	}

	for (RigidBody& rb : this->rigidBodys) {

		//Euler step

		rb.position = rb.position + timeStep * rb.linearVelocity;
		rb.linearVelocity = rb.linearVelocity + timeStep * F / rb.mass;

		//Orientation

		rb.orientation = rb.orientation + timeStep / 2 * Quat(0, rb.angularVelocity.x, 
			rb.angularVelocity.y, rb.angularVelocity.z) * rb.orientation;

		//Angular momentum

		rb.angularMomentum = rb.angularMomentum * timeStep / q;

		//Rotation Matrix

		calculateRotationMatrix(rb);

		//Inverse Intertia Tensor

		//calculate Initial Inverse Inertia Tensor after creating the rigidbody
		Mat4 rotationMatrixTransposed = rb.rotationMatrix;
		rotationMatrixTransposed.transpose();
		rb.InvertedInertialTensor = rb.rotationMatrix * rb.InitialInvertedInertiaTensor * rotationMatrixTransposed;
		
		//Angular velocity

		rb.angularVelocity = rb.InvertedInertialTensor * rb.angularMomentum;
	}
}

void RigidBodySystemSimulator::calculateInitialInertiaTensor(RigidBody rb) {
	double x[] = {
		1/12 * rb.mass * (pow(rb.size.y,2) + pow(rb.size.z,2)),
		0.0,
		0.0,
		0.0,
		0.0,
		1 / 12 * rb.mass * (pow(rb.size.x,2) + pow(rb.size.z,2)),
		0.0,
		0.0,
		0.0,
		0.0,
		1 / 12 * rb.mass * (pow(rb.size.x,2) + pow(rb.size.y,2)),
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
	};
	Mat4 C = Mat4(0.0);
	C.initFromArray(x);
	double trace_C = C.value[0][0] + C.value[1][1] + C.value[2][2];
	double y[] = {
		trace_C,
		0.0,
		0.0,
		0.0,
		0.0,
		trace_C,
		0.0,
		0.0,
		0.0,
		0.0,
		trace_C,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0
	};
	Mat4 traceMatrix = Mat4(0.0);
	traceMatrix.initFromArray(y);
	Mat4 initialInertiaTensor = traceMatrix - C;
	rb.InitialInvertedInertiaTensor = initialInertiaTensor.inverse();
}

void RigidBodySystemSimulator::calculateRotationMatrix(RigidBody rb) {
	double x[] = { 
		1.0 - 2*pow(rb.orientation.y,2)-2*pow(rb.orientation.z,2),
		2*rb.orientation.x * rb.orientation.y - 2*rb.orientation.w * rb.orientation.z,
		2*rb.orientation.x * rb.orientation.z + 2*rb.orientation.w * rb.orientation.y,
		0.0,
		2*rb.orientation.x * rb.orientation.y + 2*rb.orientation.w * rb.orientation.z,
		1 - 2*pow(rb.orientation.x,2) - 2* pow(rb.orientation.z,2),
		2 * rb.orientation.y * rb.orientation.z - 2 * rb.orientation.w * rb.orientation.x,
		0.0,
		2 * rb.orientation.x * rb.orientation.z - 2 * rb.orientation.w * rb.orientation.y,
		2 * rb.orientation.y * rb.orientation.z + 2 * rb.orientation.w * rb.orientation.x,
		1 - 2 * pow(rb.orientation.x,2) - 2 * pow(rb.orientation.y,2),
		0.0,
		0.0,
		0.0,
		0.0,
		0.0 };
	Mat4 rotationMatrix = Mat4(0.0);
	rotationMatrix.initFromArray(x);
}

void RigidBodySystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
	return rigidBodys.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
	return (this->rigidBodys)[i].position;
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	return (this->rigidBodys)[i].linearVelocity;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	return (this->rigidBodys)[i].angularVelocity;
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	(this->rigidBodys).push_back(RigidBody(position, size, mass));
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	(this->rigidBodys)[i].orientation = orientation;
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	(this->rigidBodys)[i].angularVelocity = velocity;
}
