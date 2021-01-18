#include "RigidBodySystemSimulator.h"


RigidBodySystemSimulator::RigidBodySystemSimulator()
{
    testCase = 0;
    debugPrint = false;
    m_mouse = Point2D();
    m_oldtrackmouse = Point2D();
    m_trackmouse = Point2D();

    N_bodies = 0;
}

const char* RigidBodySystemSimulator::getTestCasesStr()
{
    return "Demo 1: One step, Demo 2: Simple single body, Demo 3: Two rigid-body collision, Demo 4: Complex";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
    this->DUC = DUC;
    TwAddVarRW(DUC->g_pTweakBar, "[Complex] # of bodies", TW_TYPE_INT16, &N_bodies, "min=0 max=1000");
}

void RigidBodySystemSimulator::reset()
{
    rigidBodies.clear();
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
    // Draw each rigid body with position & size
    // Color the collision status of each rigid body
    for (RigidBody& body : rigidBodies) {
        Mat4 scaling, rotation, translation;
        scaling.initScaling(body.size.x, body.size.y, body.size.z);
        rotation = body.orientation.getRotMat();
        translation.initTranslation(body.position.x, body.position.y, body.position.z);

        DUC->drawRigidBody(scaling * rotation * translation);
    }
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
    (this->testCase) = testCase;
    switch (testCase) {
    case 0: // Demo 1: One step
    case 1: // Demo 2: Simple single body
    {
        // For Demo 1 & 2, create the sample object from the exercise sheet
        rigidBodies.push_back(RigidBody(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5), 2));
        RigidBody& rigidBody = rigidBodies[0];
        rigidBody.orientation = Quat(Vec3(0.0, 0.0, 1.0), to_radians(90.0)).unit();

        rigidBody.external_forces.push_back(RigidBody::ExternalForce(Vec3(0.3, 0.5, 0.25), Vec3(1, 1, 0)));
        break;
    }
    case 2: // Demo 3: Two rigid-body collision
        // Create two rigid bodies
        break;
    case 3: // Demo 4: Complex
        // Create N bodies in locations where touching each other is impossible, with same distance
        // Give random initial orientation, random initial angular velocity
        // Give linear velocity for all to meet in the center point
        break;
    default:
        break;
    }

    // Initial computations for each rigid body
    for (RigidBody& body : rigidBodies) {
        float c = body.mass / 12.0;
        body.I_0 = XMFLOAT3X3(
            c * (pow(body.size.z, 2) + pow(body.size.y, 2)), 0.0, 0.0,
            0.0, c * (pow(body.size.x, 2) + pow(body.size.y, 2)), 0.0,
            0.0, 0.0, c * (pow(body.size.x, 2) + pow(body.size.z, 2))
        );
        XMMATRIX I_0 = XMLoadFloat3x3(&body.I_0);
        XMMATRIX rot_r = XMLoadFloat3x3(&rotation_matrix(body.orientation));
        body.I_0_inv = (rot_r * XMMatrixInverse(nullptr, I_0) * XMMatrixTranspose(rot_r)); // TODO: Convert to 3x3
        // I-1 = rot_r * (I_0)^-1 * rot_r.transpose
        // L = Iw
    }
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
    for (RigidBody& rigid_body : this->rigidBodies) {
        // External force & torque calculation
        Vec3 F(0.0, 0.0, 0.0), q(0.0, 0.0, 0.0);
        for (RigidBody::ExternalForce& external_force : rigid_body.external_forces) {
            F += external_force.direction;
            q += cross(external_force.position, external_force.direction);
        }

        // Translations (Euler step)
        rigid_body.position += rigid_body.linear_velocity * timeStep;
        rigid_body.linear_velocity += (F / rigid_body.mass) * timeStep;

        // Rotations
        //   - Quaternion integration
        //orientation = np.normalized(orientation + np.quaternion(0, *w) * orientation * (h / 2))
        //L += q * h
        //rot_r = rotation_matrix(orientation)

        //   - I-1 = rot_r * (I_0)^-1 * rot_r.transpose
        //I_inv = rot_r @ I_0_inv @ rot_r.T

        //w = I_inv @ L
    }

    // For any point:
    // point.world_position = body.position + rot_r * (point w.r.t. center)
    // point.world_velocity = body.velocity + cross(angular_velocity, point w.r.t. center)
}

void RigidBodySystemSimulator::onClick(int x, int y)
{
}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
}

int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
    return rigidBodies.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
    return rigidBodies[i].position;
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
    return rigidBodies[i].linear_velocity;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
    return rigidBodies[i].angular_velocity;
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
    RigidBody& body = rigidBodies[i];

}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
    rigidBodies.push_back(RigidBody(position, size, mass));
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
    rigidBodies[i].orientation = orientation;
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
    rigidBodies[i].linear_velocity = velocity;
}
