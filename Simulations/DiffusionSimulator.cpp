#include "DiffusionSimulator.h"
#include "pcgsolver.h"
using namespace std;

//Grid::Grid() {
//}


DiffusionSimulator::DiffusionSimulator()
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();
	m_sizeM = 16;
	m_sizeN = 16;
	T = new Grid(m_sizeM, m_sizeN);
	for (int i = 0; i < m_sizeM; i++) {
		for (int j = 0; j < m_sizeN; j++) {
			T->vect2d[i][j] = 0;
		}
	}
	// to be implemented
}

const char * DiffusionSimulator::getTestCasesStr(){
	return "Explicit_solver, Implicit_solver";
}

void DiffusionSimulator::reset(){
		m_mouse.x = m_mouse.y = 0;
		m_trackmouse.x = m_trackmouse.y = 0;
		m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
		m_sizeM = 16;
		m_sizeN = 16;
		for (int i = 0; i < m_sizeM; i++) {
			for (int j = 0; j < m_sizeN; j++) {
				T->vect2d[i][j] = 0;
			}
		}

}

void DiffusionSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	// to be implemented
	//TwAddVarRW(DUC->g_pTweakBar, "Size m", TW_TYPE_FLOAT, &m_sizeM, "min=2 max=30 step=1");
	//TwAddVarRW(DUC->g_pTweakBar, "Size n", TW_TYPE_FLOAT, &m_sizeN, "min=2 max=30 step=1");
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_vfMovableObjectPos = Vec3(0, 0, 0);
	m_vfRotate = Vec3(0, 0, 0);
	//
	//to be implemented
	//
	switch (m_iTestCase)
	{
	case 0:
		cout << "Explicit solver!\n";
		break;
	case 1:
		cout << "Implicit solver!\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

Grid* DiffusionSimulator::diffuseTemperatureExplicit(float timeStep) {
	//add your own parameters
	int m = T->m;
	int n = T->n;
	Grid* newT = new Grid(m,n);
	
	for (int i = 1; i < m - 1; i++) {
		for (int j = 1; j < n - 1; j++) {
			float alpha = 1;
			newT->vect2d[i][j] = T->vect2d[i][j] + timeStep * alpha * (
				((T->vect2d[i+1][j] - 2*T->vect2d[i][j]+T->vect2d[i-1][j]) / pow(1,2)) +
				((T->vect2d[i][j+1] - 2*T->vect2d[i][j]+T->vect2d[i][j-1]) / pow(1,2))
			);
		}
	}
	
	return newT;
}

void setupB(std::vector<Real>& b) {//add your own parameters
	// to be implemented
	//set vector B[sizeX*sizeY]
	for (int i = 0; i < 25; i++) {
		b.at(i) = 0;
	}
}

void fillT() {//add your own parameters
	// to be implemented
	//fill T with solved vector x
	//make sure that the temperature in boundary cells stays zero
}

void setupA(SparseMatrix<Real>& A, double factor) {//add your own parameters
	// to be implemented
	//setup Matrix A[sizeX*sizeY*sizeZ, sizeX*sizeY*sizeZ]
	// set with:  A.set_element( index1, index2 , value );
	// if needed, read with: A(index1, index2);
	// avoid zero rows in A -> set the diagonal value for boundary cells to 1.0
	for (int i = 0; i < 25; i++) {
			A.set_element(i, i, 1); // set diagonal
	}
}


void DiffusionSimulator::diffuseTemperatureImplicit() {//add your own parameters
	// solve A T = b
	// to be implemented
	const int N = 25;//N = sizeX*sizeY*sizeZ
	SparseMatrix<Real> *A = new SparseMatrix<Real> (N);
	std::vector<Real> *b = new std::vector<Real>(N);

	setupA(*A, 0.1);
	setupB(*b);

	// perform solve
	Real pcg_target_residual = 1e-05;
	Real pcg_max_iterations = 1000;
	Real ret_pcg_residual = 1e10;
	int  ret_pcg_iterations = -1;

	SparsePCGSolver<Real> solver;
	solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

	std::vector<Real> x(N);
	for (int j = 0; j < N; ++j) { x[j] = 0.; }

	// preconditioners: 0 off, 1 diagonal, 2 incomplete cholesky
	solver.solve(*A, *b, x, ret_pcg_residual, ret_pcg_iterations, 0);
	// x contains the new temperature values
	fillT();//copy x to T
}



void DiffusionSimulator::simulateTimestep(float timeStep)
{
	// update current setup for each frame
	switch (m_iTestCase)
	{
	case 0:
		T = diffuseTemperatureExplicit(timeStep);
		break;
	case 1:
		diffuseTemperatureImplicit();
		break;
	}
}

void DiffusionSimulator::drawObjects()
{
	//visualization
	Vec3 scale = Vec3(0.1f, 0.1f, 0.1f);
	for (int i = 0; i < m_sizeM; i++) {
		for (int j = 0; j < m_sizeN; j++) {
			this->DUC->drawSphere(Vec3(-0.5 + i / 10.0, -0.5 + j / 10.0, 0), scale);
		}
	}
	


}


void DiffusionSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	drawObjects();
}

void DiffusionSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void DiffusionSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}
