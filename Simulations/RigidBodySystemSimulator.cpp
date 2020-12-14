#include "RigidBodySystemSimulator.h"

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
    m_fGravity = 0;

    m_mouse = Point2D();
    m_trackmouse = Point2D();
    m_oldtrackmouse = Point2D();
    m_externalForce = Vec3(0, 0, 0);
}

const char* RigidBodySystemSimulator::getTestCasesStr()
{
    return "Single body, Two bodies, Complex";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
    this->DUC = DUC;
    //TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "min=0.00 max=5.00 step=0.05");
    TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_FLOAT, &m_fGravity, "min=0.00 max=100.00 step=0.01");
}
void RigidBodySystemSimulator::reset()
{
    (this->RigidBodies).clear();
    (this->forces).clear();
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
    this->DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, Vec3(0.5, 0.5, 0.5));

    for (RigidBody& rb : this->RigidBodies) {
        Mat4 scalingMatrix = Mat4(0.0);
        scalingMatrix.initScaling(rb.size[0], rb.size[1], rb.size[2]);  //scaling matrix according to size of object

        Mat4 translationMatrix = Mat4(0.0);
        translationMatrix.initTranslation(rb.position[0], rb.position[1], rb.position[2]);  //translation matrix according to position

        Mat4 rotationMatrix = rb.orientation.getRotMat();  // rotation matrix from orientation quaternion

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
        (this->RigidBodies)[0].orientation = Quat(Vec3(0, 0, 1), 1.57079);
        (this->forces).push_back(force(Vec3(1,1,0), Vec3(0.3, 0.5, 0.25), 0));
        break;
    }
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
    // Apply a force to one of the bodies according to mouse movement (move along cameras view plane)
    Point2D mouseDiff;
    mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
    mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
    if (mouseDiff.x != 0 || mouseDiff.y != 0)
    {
        Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
        worldViewInv = worldViewInv.inverse();
        Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
        Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
        // find a proper scale!
        float inputScale = 0.05f;
        inputWorld = inputWorld * inputScale;
        m_externalForce = inputWorld;
        (this->RigidBodies)[0].linearVelocity += timeElapsed * m_externalForce / (this->RigidBodies)[0].mass;

    }
    else {
        m_externalForce = Vec3(0,0,0);
    }
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
    for (force& f_i : this->forces) {
        Vec3 q = cross(f_i.applicationPoint, f_i.forceApplied);
        (this->RigidBodies)[f_i.applicationBody].angularMomentum += timeStep * q;
        (this->RigidBodies)[f_i.applicationBody].totalForce += f_i.forceApplied;
    }

    for (RigidBody& rb : this->RigidBodies) {
        int mass = rb.mass;
        // using inertia tensor assuming uniform distribution of mass along box
        Vec3 diagInertia = Vec3(1.0 / 12 * mass * (rb.size[2] * rb.size[2] + rb.size[1] * rb.size[1]),
            1.0 / 12 * mass * (rb.size[0] * rb.size[0] + rb.size[1] * rb.size[1]),
            1.0 / 12 * mass * (rb.size[0] * rb.size[0] + rb.size[2] * rb.size[2]));
        // compute angular velocity w = I^(-1) L
        rb.angularVelocity = Vec3(rb.angularMomentum[0] / diagInertia[0],
            rb.angularMomentum[1] / diagInertia[1],
            rb.angularMomentum[2] / diagInertia[2]);
        // update orientation
        Quat wQuat = Quat(rb.angularVelocity, 1.57079);
        rb.orientation += timeStep / 2 * wQuat * rb.orientation;
        rb.orientation /= sqrtf(rb.orientation.x * rb.orientation.x + rb.orientation.y * rb.orientation.y
            + rb.orientation.z * rb.orientation.z + rb.orientation.w * rb.orientation.w);

        // apply gravity
        rb.totalForce += Vec3(0, 0, -m_fGravity * mass);

        // update velocity and position of center of mass
        rb.position += timeStep * rb.linearVelocity;
        rb.linearVelocity += timeStep * rb.totalForce / mass;

        // Clear force before new time-step
        rb.totalForce = Vec3(0, 0, 0);
    }
    (this->forces).clear();
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
