#include "OpenProjectSimulator.h"

OpenProjectSimulator::OpenProjectSimulator() {
}

const char* OpenProjectSimulator::getTestCasesStr()
{
    return "Basic";
}

void OpenProjectSimulator::initUI(DrawingUtilitiesClass* DUC)
{
    this->DUC = DUC;
}

void OpenProjectSimulator::reset()
{
}

void OpenProjectSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
}

void OpenProjectSimulator::notifyCaseChanged(int testCase)
{
}

void OpenProjectSimulator::externalForcesCalculations(float timeElapsed)
{
}

void OpenProjectSimulator::computeForces()
{
}

void OpenProjectSimulator::simulateTimestep(float timeStep)
{
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
