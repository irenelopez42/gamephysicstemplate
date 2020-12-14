#include "RigidBodySystemSimulator.h"

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
    m_fMass = 2;
    m_fGravity = 0;

    m_mouse = Point2D();
    m_trackmouse = Point2D();
    m_oldtrackmouse = Point2D();
}

const char* RigidBodySystemSimulator::getTestCasesStr()
{
    return "Single body, Two bodies, Complex";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
    this->DUC = DUC;
    TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "min=0.1 max=100.0 step=0.1");
    //TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "min=0.00 max=5.00 step=0.05");
    TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_FLOAT, &m_fGravity, "min=0.00 max=100.00 step=0.01");
}
void RigidBodySystemSimulator::reset()
{
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
    this->DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, Vec3(0.5, 0.5, 0.5));

    for (RigidBody& rb : this->RigidBodies) {
        Mat4 scalingMatrix = Mat4(0.0);
        Mat4 translationMatrix = Mat4(0.0);
        translationMatrix.initTranslation(rb.position[0], rb.position[1], rb.position[2]);
        scalingMatrix.initScaling(rb.size[0], rb.size[1], rb.size[2]);
        Mat4 rotationMatrix = rb.orientation.getRotMat();
        Mat4 worldMatrix = scalingMatrix * rotationMatrix * translationMatrix;
        this->DUC->drawRigidBody(worldMatrix);
        }
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
    this->reset();
    switch (testCase) {
    case 0:
        addRigidBody(Vec3(0.0, 0.0, 0.0), Vec3(1.0, 0.6, 0.5), 2);
        (this->forces).push_back(force(Vec3(1,1,0), Vec3(0.3, 0.5, 0.25)));
        break;
    }
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
    //for (Force& f_i : this->forces) {
       // F = F + f_i.force;
       // q = q + cross(f_i.position, f_i.force);
    //}
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
    return RigidBodies.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
    return (this->RigidBodies)[i].position;
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
    return (this->RigidBodies)[i].linearVelocity;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
    return (this->RigidBodies)[i].angularVelocity;
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
    //(this->RigidBodies)[i].angularMomentum += timestep * cross(loc, force);
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
    (this->RigidBodies).push_back(RigidBody(position, size, mass));
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
    (this->RigidBodies)[i].orientation = orientation;
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
    (this->RigidBodies)[i].linearVelocity = velocity;
}
