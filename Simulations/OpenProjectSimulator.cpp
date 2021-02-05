#include "OpenProjectSimulator.h"

OpenProjectSimulator::OpenProjectSimulator() {
    m_fGravity = 0.0;
    m_mouse = Point2D();
    m_trackmouse = Point2D();
    m_oldtrackmouse = Point2D();
    m_externalForce = Vec3(0, 0, 0);
}

const char* OpenProjectSimulator::getTestCasesStr()
{
    return "Basic";
}

void OpenProjectSimulator::initUI(DrawingUtilitiesClass* DUC)
{
    this->DUC = DUC;
    TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_FLOAT, &m_fGravity, "min=0.00 max=100.00 step=0.01");
}

void OpenProjectSimulator::reset()
{
    (this->RigidBodies).clear();
    (this->forces).clear();
}

void OpenProjectSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
    this->DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, Vec3(0.5, 0.5, 0.5));

    for (RigidBody& rb : this->RigidBodies) {
        Mat4 scalingMatrix = Mat4(0.0);
        scalingMatrix.initScaling(rb.size[0], rb.size[1], rb.size[2]);  //scaling matrix according to size of object

        Mat4 translationMatrix = Mat4(0.0);
        translationMatrix.initTranslation(rb.position[0], rb.position[1], rb.position[2]);  //translation matrix according to position

        Mat4 rotationMatrix = rb.orientation.getRotMat();  // rotation matrix from orientation quaternion

        rb.worldMatrix = scalingMatrix * rotationMatrix * translationMatrix;
        this->DUC->drawRigidBody(rb.worldMatrix);
    }
}

void OpenProjectSimulator::notifyCaseChanged(int testCase)
{
    this->reset();
    switch (testCase) {
    case 0:
        addRigidBody(Vec3(1.0, -0.5, 0.0), Vec3(1.0, 0.6, 0.5), 2);
        (this->RigidBodies)[0].orientation = Quat(Vec3(0, 0, 1), 1.57079);
        addRigidBody(Vec3(2.0, -0.5, 0.0), Vec3(1.0, 0.6, 0.5), 2);
        (this->RigidBodies)[1].orientation = Quat(Vec3(0, 0, 1), 1.57079);

        addRigidBody(Vec3(1.5, 0.301, 0.0), Vec3(2.0, 0.6, 0.5), 2);
        (this->RigidBodies)[2].orientation = Quat(Vec3(0, 0, 1), 0.0);

        addRigidBody(Vec3(2.0, 1.101, 0.0), Vec3(1.0, 0.6, 0.5), 2);
        (this->RigidBodies)[3].orientation = Quat(Vec3(0, 0, 1), 1.57079);
        addRigidBody(Vec3(1.0, 1.101, 0.0), Vec3(1.0, 0.6, 0.5), 2);
        (this->RigidBodies)[4].orientation = Quat(Vec3(0, 0, 1), 1.57079);

        addRigidBody(Vec3(1.5, 1.902, 0.0), Vec3(2.0, 0.6, 0.5), 2);
        (this->RigidBodies)[5].orientation = Quat(Vec3(0, 0, 1), 0.0);

        break;
    }
}

void OpenProjectSimulator::externalForcesCalculations(float timeElapsed)
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
        m_externalForce = Vec3(0, 0, 0);
    }
}

void OpenProjectSimulator::computeForces()
{
}

void OpenProjectSimulator::simulateTimestep(float timeStep)
{
    for (force& f_i : this->forces) {
        Vec3 q = cross(f_i.applicationPoint, f_i.forceApplied);
        (this->RigidBodies)[f_i.applicationBody].angularMomentum += timeStep * q;
        (this->RigidBodies)[f_i.applicationBody].totalForce += f_i.forceApplied;
    }

    for (RigidBody& rb : this->RigidBodies) {
        if (rb.isFixed) continue;
        int mass = rb.mass;
        // update orientation
        Quat wQuat = Quat(rb.angularVelocity, 1.57079);
        rb.orientation += timeStep / 2 * wQuat * rb.orientation;
        rb.orientation /= sqrtf(rb.orientation.x * rb.orientation.x + rb.orientation.y * rb.orientation.y
            + rb.orientation.z * rb.orientation.z + rb.orientation.w * rb.orientation.w);

        // using inertia tensor assuming uniform distribution of mass along box
        Vec3 diagInertia = Vec3(1.0 / 12 * mass * (rb.size[2] * rb.size[2] + rb.size[1] * rb.size[1]),
            1.0 / 12 * mass * (rb.size[0] * rb.size[0] + rb.size[1] * rb.size[1]),
            1.0 / 12 * mass * (rb.size[0] * rb.size[0] + rb.size[2] * rb.size[2]));
        // compute angular velocity w = I^(-1) L
        rb.angularVelocity = Vec3(rb.angularMomentum[0] / diagInertia[0],
            rb.angularMomentum[1] / diagInertia[1],
            rb.angularMomentum[2] / diagInertia[2]);

        // apply gravity
        rb.totalForce += Vec3(0, -m_fGravity * mass, 0);

        // update velocity and position of center of mass
        rb.position += timeStep * rb.linearVelocity;
        rb.linearVelocity += timeStep * rb.totalForce / mass;

        // Clear force before new time-step
        rb.totalForce = Vec3(0, 0, 0);

        // Floor collision
        float floor_level = -0.5;
        if (rb.position.y < floor_level) {
            rb.position.y = floor_level + (floor_level - rb.position.y);
            rb.linearVelocity.y *= -1;
        }

    }

    // Check for Collisions
    for (int i = 0; i < RigidBodies.size(); i++) {
        for (int j = i + 1; j < RigidBodies.size(); j++) {
            CollisionInfo collisionInfo = checkCollisionSAT(RigidBodies[i].worldMatrix, RigidBodies[j].worldMatrix);
            calcImpulse(collisionInfo, RigidBodies[i], RigidBodies[j], 1);
        }
    }

    (this->forces).clear();
}

void OpenProjectSimulator::calcImpulse(CollisionInfo info, RigidBody& rbA, RigidBody& rbB, int c) {
    //Check if there is a Collision
    if (info.isValid == true)
    {
        //Calculate v_i for the Collision Point for both objects
        Vec3 velA = rbA.linearVelocity + cross(rbA.angularVelocity, info.collisionPointWorld);
        Vec3 velB = rbB.linearVelocity + cross(rbB.angularVelocity, info.collisionPointWorld);
        Vec3 velRel = (velA - velB);
        double imp = 0.0;
        //check if Objects are moving towards eachother

        if (dot(velRel, info.normalWorld) < 0) {
            float impNum = -(1 + c) * dot(velRel, info.normalWorld);
            Vec3 diagInertiaA = Vec3(1.0 / 12 * rbA.mass * (rbA.size[2] * rbA.size[2] + rbA.size[1] * rbA.size[1]),
                1.0 / 12 * rbA.mass * (rbA.size[0] * rbA.size[0] + rbA.size[1] * rbA.size[1]),
                1.0 / 12 * rbA.mass * (rbA.size[0] * rbA.size[0] + rbA.size[2] * rbA.size[2]));
            Vec3 diagInertiaB = Vec3(1.0 / 12 * rbB.mass * (rbB.size[2] * rbB.size[2] + rbB.size[1] * rbB.size[1]),
                1.0 / 12 * rbB.mass * (rbB.size[0] * rbB.size[0] + rbB.size[1] * rbB.size[1]),
                1.0 / 12 * rbB.mass * (rbB.size[0] * rbB.size[0] + rbB.size[2] * rbB.size[2]));
            Vec3 crossproductA = cross(rbA.position, info.normalWorld);
            Vec3 crossproductB = cross(rbB.position, info.normalWorld);
            Vec3 rbAMoving = cross(Vec3(crossproductA[0] / diagInertiaA[0], crossproductA[1] / diagInertiaA[1], crossproductA[2] / diagInertiaA[2]), rbA.position);
            Vec3 rbBMoving = cross(Vec3(crossproductB[0] / diagInertiaB[0], crossproductB[1] / diagInertiaB[1], crossproductB[2] / diagInertiaB[2]), rbB.position);

            float impDen = 0.0;

            // denominator is adjusted regarding which rb can actually move. If Fixed then inverse inertia and mass are set to 0
            if (rbA.isFixed && !rbB.isFixed) {
                impDen = 1.0 / rbB.mass + dot(rbBMoving, info.normalWorld);
            }
            if (!rbA.isFixed && rbB.isFixed) {
                impDen = 1.0 / rbA.mass + dot(rbAMoving, info.normalWorld);
            }
            if (!rbA.isFixed && !rbB.isFixed) {
                impDen = 1.0 / rbA.mass + 1 / rbB.mass + dot(rbAMoving + rbBMoving, info.normalWorld);
            }

            imp = impNum / impDen;
        }

        // Update Linear Velocity
        rbA.linearVelocity += imp * info.normalWorld / rbA.mass;
        rbB.linearVelocity -= imp * info.normalWorld / rbB.mass;

        //Update  Angular Momentum
        rbA.angularMomentum += cross(rbA.position, (imp * info.normalWorld));
        rbB.angularMomentum -= cross(rbB.position, (imp * info.normalWorld));

    }

}

void OpenProjectSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
    (this->RigidBodies).push_back(RigidBody(position, size, mass));
}

void OpenProjectSimulator::onClick(int x, int y)
{
    m_trackmouse.x = x;
    m_trackmouse.y = y;
}

void OpenProjectSimulator::onMouse(int x, int y)
{
    m_oldtrackmouse.x = x;
    m_oldtrackmouse.y = y;
    m_trackmouse.x = x;
    m_trackmouse.y = y;
}
