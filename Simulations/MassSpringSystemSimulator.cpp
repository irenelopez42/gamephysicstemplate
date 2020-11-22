#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	m_fMass = 10;
	m_fStiffness = 40;
	m_fDamping = 0;
	m_iIntegrator = 0;
	m_externalForce = 0;
	m_mouse = Point2D();
	m_trackmouse = Point2D();
	m_oldtrackmouse = Point2D();

	addSpring(addMassPoint({ 0,0,0 }, { -1,0,0 }, FALSE), addMassPoint({ 0, 2, 0 }, { 1,0,0 }, FALSE), 1);
}

// UI Functions

const char * MassSpringSystemSimulator::getTestCasesStr()
{
	return "2-point, Complex";
}

const char * MassSpringSystemSimulator::getIntegratorStr()
{
	return "Euler, Leapfrog, Midpoint";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	TwType TW_TYPE_INTEGRATOR = TwDefineEnumFromString("Integrator", getIntegratorStr());
	TwAddVarRW(DUC->g_pTweakBar, "Integrator", TW_TYPE_INTEGRATOR, &m_iIntegrator, "");
	TwAddVarRW(DUC->g_pTweakBar, "Mass Spheres", TW_TYPE_FLOAT, &m_fMass, "min=0.1 step=0.1");
	TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness, "min=0.5 step=0.5");
	TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "min=0.01 step=0.01");
}

void MassSpringSystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext * pd3dImmediateContext)
{
	//DUC->setUpLighting(Vec3(), 0.4*Vec3(1, 1, 1), 100, 0.6*Vec3(0.97, 0.86, 1));

	for (int i = 0; i < myMassPointVector.size(); i++) {
		Vec3 pos_vector = myMassPointVector[i].position;
		Vec3 scale = { 0.1f, 0.1f, 0.1f};
		DUC->drawSphere(pos_vector, scale);
	}

	for (int i = 0; i < mySpringVector.size(); i++) {
		MassPoint mp1 = myMassPointVector[mySpringVector[i].mp1];
		MassPoint mp2 = myMassPointVector[mySpringVector[i].mp2];
		Vec3 pos_mp1 = mp1.position;
		Vec3 pos_mp2 = mp2.position;
		DUC->beginLine();
		DUC->drawLine(pos_mp1, Vec3(50, 50, 50), pos_mp2, Vec3(50, 50, 50));
		DUC->endLine();
	}
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	if (m_iIntegrator == 0) {
		// EULER
		for (int i = 0; i < myMassPointVector.size(); i++) {
			int indexConnectedMassPoint = -1;
			int springLength = -1;
			for (int j = 0; j < mySpringVector.size(); j++) {
				if (mySpringVector[j].mp1 == i) {
					indexConnectedMassPoint = mySpringVector[j].mp2;
					springLength = mySpringVector[j].initLength;
				}
				if (mySpringVector[j].mp2 == i) {
					indexConnectedMassPoint = mySpringVector[j].mp1;
					springLength = mySpringVector[j].initLength;
				}
			}
			Vec3 position_mp_i = myMassPointVector[i].position;
			Vec3 position_mp_connected = myMassPointVector[indexConnectedMassPoint].position;
			int l_i = sqrt((position_mp_i.x - position_mp_connected.x) * (position_mp_i.x - position_mp_connected.x) +
				(position_mp_i.y - position_mp_connected.y) * (position_mp_i.y - position_mp_connected.y) +
				(position_mp_i.z - position_mp_connected.z) * (position_mp_i.z - position_mp_connected.z));
			Vec3 a_i = -m_fStiffness * (l_i - springLength) / m_fMass * (position_mp_i - position_mp_connected) / l_i;
			//gravity?
			Vec3 v_i = myMassPointVector[i].velocity + timeStep * a_i;
			Vec3 x_i = myMassPointVector[i].position + timeStep * myMassPointVector[i].velocity;
			myMassPointVector[i].position = x_i;
			myMassPointVector[i].velocity = v_i;

		}
	}
	if (m_iIntegrator == 1) {
		//MIDSTEP

	}
}

void MassSpringSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

// Specific Functions

void MassSpringSystemSimulator::setMass(float mass)
{
	m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness)
{
	m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping)
{
	m_fDamping = damping;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed)
{
	MassPoint mp = {position, Velocity, isFixed};
	myMassPointVector.push_back(mp);
	return myMassPointVector.size() - 1;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	Spring s = { masspoint1, masspoint2, initialLength };
	mySpringVector.push_back(s);
}

int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	return  myMassPointVector.size();
}

int MassSpringSystemSimulator::getNumberOfSprings()
{
	return mySpringVector.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	MassPoint mp = myMassPointVector[index];
	return mp.position;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	MassPoint mp = myMassPointVector[index];
	return mp.velocity;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
}
