#include "Simulation.h"

Simulation::Simulation()
{
}



Simulation::~Simulation()
{

}

void Simulation::Init()
{
	std::cout << "Initializing Simulation" << std::endl;

//	for (int i = 0; i < mesh.size(); i++) {
	//	EigenMatrixXs* a;
	//	m_V.push_back(&(mesh[i]->m_V));

	// set pointer of m_V, m_F, m_T
	m_V = &mesh->m_V;
	m_F = &mesh->m_F;
	m_T = &mesh->m_T;

	m_Vel.resize(mesh->m_vert_num, 3); m_Vel.setZero();

	m_Inertia.resize(mesh->m_vert_num, 3);       m_Inertia.setZero();
	m_ExternalForce.resize(mesh->m_vert_num, 3); m_ExternalForce.setZero();

	setReprecomputeFlag();
	setReprefactorFlag();

	convertlameConstant();

	preComputation();
	prefactorize();
//	/
	//add manual parameter
	m_iterations_per_frame = 3;
	m_damping_coefficient = 0.001;

	m_gravity_constant = 0.0;
	m_young = 100.0;
	m_poisson = 0.5;


}


void Simulation::update() {
	std::cout << "Update Simulation ..." << std::endl;


	//update iertial term
	computeInertia();

	//update external term
	computeExternalForce();

	integrateOptimizationMethod();

	dampVelocity();

}
#include<igl/massmatrix.h>
void Simulation::preComputation()
{
	std::cout << "preComputing..." << std::endl;
// set m_B, m_W
	m_B.clear();
	m_W.clear();
	// Inprement Here !! //
	// Hint: compute and push_back "m_B" and "m_W//
	int  j, p;
	for (int i = 0; i < m_T->rows(); ++i) {
		EigenMatrix3 m_B_element;
		ScalarType   m_W_element;
		for (j = 0; j < 3; j++) {
			for (p = 0; p < 3; p++) {
				m_B_element(p, j) = (*m_V)((*m_T)(i, j), p) - (*m_V)((*m_T)(i, 3), p);

			}
		}

		m_W_element = 1.0 / 6.0*  fabs((m_B_element).determinant());//*/
		m_B.push_back(m_B_element.inverse());
		m_W.push_back(m_W_element);

	}
	//set MassMatrix
	igl::massmatrix(*m_V, *m_T, igl::MASSMATRIX_TYPE_DEFAULT, m_MassMat);
	ScalarType ModelVolume = 0.0;
	for (uint i = 0; i < m_T->rows(); ++i) { ModelVolume += m_W[i]; }

	std::cout << ModelVolume << std::endl;
	system("pause");
	m_MassMat *= (mesh->m_total_mass) / ModelVolume;
	std::cout << mesh->m_total_mass << std::endl;
	system("pause");

	m_precomputation_flag = true;
	std::cout << "preComputing...end" << std::endl;

}

void Simulation::convertlameConstant()
{
	m_young = 100;
	m_poisson = 0.5;
	m_myu = m_young / (2.f * (1.f + m_poisson));
	m_lambda = m_young * m_poisson / ((1.f + m_poisson)*(1.f - 2.f*m_poisson));
}


void Simulation::computeRotMat(EigenMatrixXs& RotMat, const EigenMatrixXs& Jv)
{
	//// Inprement Here !! //
	//// Hint: use "SVD"

	//#pragma omp parallel {
	int num = 0;
#pragma omp parallel for

	for (num = 0; num < m_T->rows(); num++) {

		EigenMatrix3 F;
		EigenMatrix3 U, V;
		F.setZero();
		F = (Jv.block(num * 3, 0, 3, 3)).transpose();


		///constraintつけるjvが更新される

		Eigen::JacobiSVD< EigenMatrix3 >svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
		U = svd.matrixU();
		V = svd.matrixV();
		RotMat.block(3 * num, 0, 3, 3) = U * (V.transpose());


	}
}

void Simulation::volumeconservation(EigenMatrix3 &F, EigenMatrix3 &B, const unsigned int tet_list[], EigenMatrixXs &X)//Bにだいれくとにいれないように注意
{

	EigenMatrix3 U, V;
	EigenVector3 SIGMA;

	singularValueDecomp(U, SIGMA, V, F);

	double det_F = F.determinant();

	if (det_F < 0.0) {

		SIGMA[2] *= -1.0;


		double high_val = SIGMA[0];
		double mid_val = SIGMA[1];
		double low_val = SIGMA[2];

		if (mid_val < low_val) {
			double temp = low_val;
			low_val = mid_val;
			mid_val = temp;

		}
		if (high_val < low_val) {
			double temp = low_val;
			low_val = high_val;
			high_val = temp;

		}
		if (high_val < mid_val) {

			double temp = mid_val;
			mid_val = high_val;
			high_val = temp;
		}


		SIGMA[0] = high_val;
		SIGMA[1] = mid_val;
		SIGMA[2] = low_val;
	}

	double min = 0.95;
	double max = 1.05;

	EigenMatrix3 SIGMA_new, DS;
	//clamp
	//glm::clamp CLAMP;
	SIGMA_new << clamp(SIGMA(0, 0), min, max), 0.0, 0.0,
		0.0, clamp(SIGMA(1, 0), min, max), 0.0,
		0.0, 0.0, clamp(SIGMA(2, 0), min, max);


	EigenMatrix3  F_new;



	F_new = V.transpose();
	SIGMA_new.applyOnTheLeft(F_new);
	U.applyOnTheLeft(F_new);

	DS = B;

	F_new.applyOnTheLeft(DS);

	EigenVector3 meanpos = 1 / 4.0*((X.row(tet_list[0])).transpose() + (X.row(tet_list[1])).transpose() + (X.row(tet_list[2])).transpose() + (X.row(tet_list[3])).transpose());
	EigenVector3 newpos[4];
	//check
	newpos[3] = meanpos - 1 / 4.0*(DS.col(0) + DS.col(1) + DS.col(2));


	newpos[0] = newpos[3] + DS.col(0) - X.row(tet_list[0]).transpose();
	newpos[1] = newpos[3] + DS.col(1) - X.row(tet_list[1]).transpose();
	newpos[2] = newpos[3] + DS.col(2) - X.row(tet_list[2]).transpose();
	newpos[3] = newpos[3] - X.row(tet_list[3]).transpose();

	for (int i = 0; i < 4; i++) {

		X.row(tet_list[i]) += newpos[i].transpose();
	}
}



void Simulation::prefactorize()
{
	SparseMatrix A;
	ScalarType h2 = m_h*m_h;
	A.resize(mesh->m_vert_num, mesh->m_vert_num);
//	A.setZero();
	std::cout << "preFactorizing ..." << std::endl;


	// Hint: "A = ?"
	setJacobianMat();
	setLaplacianMat();
	//jacobian 計算
	A = m_LaplacianMat + m_MassMat / h2;

	factorizeDirectSolverLLT(A, m_prefactored_LLTsolver);

	m_prefactorization_flag = true;
}


void Simulation::dampVelocity()
{
	std::cout << "dampvelocity" << std::endl;

//	if (std::abs(m_damping_coefficient) < EPSILON)
//		return;

	// post-processing damping
	EigenVector3 pos_mc(0.0, 0.0, 0.0), vel_mc(0.0, 0.0, 0.0);
	unsigned int i, size;
	ScalarType denominator(0.0), mass(0.0);
	size = mesh->m_vert_num;

	{
		for (int i = 0; i < size; ++i)
		{
			mass = m_MassMat.coeff(i, i);

			pos_mc += mass*(((*m_V).row(i)).transpose());
			vel_mc += mass*(((m_Vel).row(i)).transpose());
			denominator += mass;
		}
		std::cout << denominator << std::endl;
		assert(denominator != 0.0);
		pos_mc /= denominator;
		vel_mc /= denominator;

		EigenVector3 angular_momentum(0.0, 0.0, 0.0), r(0.0, 0.0, 0.0);
		EigenMatrix3 inertia, r_mat;

		inertia.setZero(); r_mat.setZero();

		for (int i = 0; i < size; ++i)
		{
			mass = m_MassMat.coeff(i, i);

			r = ((*m_V).row(i)).transpose() - pos_mc;


			//r_mat = EigenMatrix3(0.0,  r.z, -r.y,
			//                    -r.z, 0.0,  r.x,
			//                    r.y, -r.x, 0.0);

			r_mat.coeffRef(0, 1) = r[2];
			r_mat.coeffRef(0, 2) = -r[1];
			r_mat.coeffRef(1, 0) = -r[2];
			r_mat.coeffRef(1, 2) = r[0];
			r_mat.coeffRef(2, 0) = r[1];
			r_mat.coeffRef(2, 1) = -r[0];

			inertia += r_mat * r_mat.transpose() * mass;
		}
		EigenVector3 angular_vel = inertia.inverse() * angular_momentum;

		EigenVector3 delta_v(0.0, 0.0, 0.0);

		for (int i = 0; i < size; ++i)
		{
			r = ((*m_V).row(i)).transpose() - pos_mc;
			delta_v = vel_mc + angular_vel.cross(r) - ((m_Vel.row(i)).transpose());
			m_Vel.row(i) += m_damping_coefficient * (delta_v.transpose());
		}
	}
}

void Simulation::computeInertia()
{
	// Inprement Here !! //
	// Hint: use "m_Inertia = ?"
	std::cout << "compute inertial" << std::endl;

	m_Inertia.setZero();
	for (int y = 0; y < mesh->m_vert_num; ++y) {
		for (int x = 0; x < 3; ++x) {
			m_Inertia.coeffRef(y, x) = m_Vel(y, x) *m_h + (*m_V)(y, x);// +
		}
	}
}


void Simulation::computeExternalForce()
{
	std::cout << "computingexternal" << std::endl;

	m_ExternalForce.setZero();

	// gravity
	// Inprement Here !! //
	// Hint: use "m_ExternalForce(i, 1) = ?"

	for (unsigned int i = 0; i < mesh->m_vert_num; ++i)
	{
		m_ExternalForce(i, 1) = -1.0*m_MassMat.coeff(i, i) *  m_gravity_constant;
	}
}

void Simulation::integrateOptimizationMethod()
{
	// check if precomputation is done (for Debag)
	if (m_precomputation_flag == false) { fprintf(stdout, "precompution dosen't work\n"); }
	// take a initial guess
	EigenMatrixXs pos_next = m_Inertia;//感性力のみ
										 // while loop until converge or exceeds maximum iterations
	bool converge = false;

	for (unsigned int iteration_num = 0; !converge && iteration_num < m_iterations_per_frame; ++iteration_num)
	{
			converge = integrateLocalGlobalOneIteration(pos_next);
	}
	std::cout << "update" << std::endl;

	// update q_{n+1}
	updatePosAndVel(pos_next);

}

// main

bool Simulation::integrateLocalGlobalOneIteration(EigenMatrixXs& X)
{
	//// local step
	EigenMatrixXs RotMat(m_T->rows() * 3, 3);       // (#TetNum*dim) * dim
	EigenMatrixXs Jv = m_JacobianMat * X;      // (#TetNum*dim) * dim
	computeRotMat(RotMat, Jv);

	//	Xの更新体積保存
	int ini;
	for (ini = 0; ini< m_T->rows(); ini++) {


		EigenMatrix3 F = (Jv.block(ini * 3, 0, 3, 3)).transpose();
		EigenMatrix3 B = m_B[ini].inverse();
		uint tet_list[4] = { (*m_T)(ini, 0), (*m_T)(ini, 1),(*m_T)(ini, 2), (*m_T)(ini, 3) };
		volumeconservation(F, B, tet_list, X);

	}

	std::cout << "local step end" << std::endl;
	// global step
	// Inprement Here !! //
	// Hint: use "b = ?"
	// Hint: you should not add inertia yet
	//  b=jrを入れて
	EigenMatrixXs b(mesh->m_vert_num, 3);
	b.setZero();

	int i, j, k, p;
	for (i = 0; i < m_T->rows(); ++i)
	{
		for (j = 0; j < 3; j++) {//xyz
			for (k = 0; k < 3; k++) {//ヤコビアンの行情報
				double rjk = RotMat(3 * i + j, k);
				for (p = 0; p < mesh->m_vert_num; p++) {
					b(p, j) = b(p, j) + m_JacobianMat.coeff(3 * i + k, p) *rjk;
				}
			}
		}
	}


	std::cout << "writingX" << std::endl;//sum1:x2x4の係数


										 // add effect of inertia
										 // Inprement Here !! //
										 // Hint: use "b = ?"
										 // Hint: add intertia here


	EigenMatrixXs inertial(mesh->m_vert_num, 3);
	std::cout << "computing inertia" << std::endl;//sum1:x2x4の係数
	inertial = m_MassMat*m_Inertia / (m_h*m_h);
	b+= inertial+ m_ExternalForce;
	std::cout << "computing X" << std::endl;//sum1:x2x4の係数

	for (int i = 0; i < 3; i++) {
		X.col(i) = m_prefactored_LLTsolver.solve(b.col(i));
	}
	return false;
}


#pragma region matrices and prefactorization
void Simulation::setLaplacianMat()
{


	m_LaplacianMat.resize(mesh->m_vert_num, mesh->m_vert_num);
	m_LaplacianMat.setZero();

	std::vector<SparseMatrixTriplet> l_triplets;
	l_triplets.clear();
	for (uint i = 0; i<m_T->rows(); ++i)
	{
		uint tet_list[4] = { (*m_T)(i, 0), (*m_T)(i, 1),(*m_T)(i, 2), (*m_T)(i, 3) };
		// Inprement Here !! //
		// Hint: use "computeElementLaplacianMat"
		computeElementLaplacianMat(m_B[i], m_W[i], tet_list, l_triplets);
	}

	// add attachment constraint
	//for (std::vector<Constraint*>::iterator it_c = m_constraints.begin(); it_c != m_constraints.end(); ++it_c)
	//{
	//	(*it_c)->computeLaplacianMat(l_triplets);
	//}

	m_LaplacianMat.setFromTriplets(l_triplets.begin(), l_triplets.end());
}

void Simulation::setJacobianMat()
{
	m_JacobianMat.resize(m_T->rows() * 3, mesh->m_vert_num);
	std::cout << "ヤコビアン計算" << std::endl;

	m_JacobianMat.setZero();


	std::vector<SparseMatrixTriplet> j_triplets;
	j_triplets.clear();

	for (uint i = 0; i<m_T->rows(); ++i)
	{
		uint tet_list[4] = { (*m_T)(i,0), (*m_T)(i,1), (*m_T)(i,2), (*m_T)(i,3) };
		// Inprement Here !! //
		// Hint: use "computeElementJacobianMat"
		computeElementJacobianMat(m_B[i], m_W[i], tet_list, i, j_triplets);
	}

	m_JacobianMat.setFromTriplets(j_triplets.begin(), j_triplets.end());

}
#pragma region utilities
void Simulation::updatePosAndVel(const EigenMatrixXs NewPos)
{
	m_Vel = (NewPos - (*m_V)) / m_h;//vを更新
	*m_V = NewPos;//次のステップのひとつ前になる

}



void Simulation::singularValueDecomp(EigenMatrix3& U, EigenVector3& SIGMA, EigenMatrix3& V, const EigenMatrix3& A)
{
	Eigen::JacobiSVD<EigenMatrix3> svd;
	svd.compute(A, Eigen::ComputeFullU | Eigen::ComputeFullV);

	U = svd.matrixU();
	V = svd.matrixV();
	SIGMA = svd.singularValues();

	ScalarType detU = U.determinant();
	ScalarType detV = V.determinant();

	// make sure that both U and V are rotation matrices without reflection
	if (detU < 0)
	{
		U.block<3, 1>(0, 2) *= -1;
		SIGMA[2] *= -1;
	}
	if (detV < 0)
	{
		V.block<3, 1>(0, 2) *= -1;
		SIGMA[2] *= -1;
	}

}


void Simulation::computeElementLaplacianMat(const EigenMatrix3 &B, const ScalarType W, const unsigned int tet_list[], std::vector<SparseMatrixTriplet>& l_triplets)
{
	//#pragma omp parallel for
	//m_myu = 2.0;
	for (int i = 0; i < 3; i++) {
		for (int j = i; j < 3; j++) {
			if (i == j) {
				for (int k = 0; k < 3; k++) {
					l_triplets.push_back(SparseMatrixTriplet(tet_list[i], tet_list[i], m_myu * 1.0 * W  * B(i, k) * B(i, k)));
				}
			}
			else {
				for (int k = 0; k < 3; k++) {
					l_triplets.push_back(SparseMatrixTriplet(tet_list[j], tet_list[i], m_myu * 1.0 * W  * B(i, k) * B(j, k)));
					l_triplets.push_back(SparseMatrixTriplet(tet_list[i], tet_list[j], m_myu * 1.0 * W  * B(i, k) * B(j, k)));
				}
			}
		}
	}
	for (int k = 0; k < 3; k++) {
		double sum = 0;
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				sum += B(k, i)*B(j, i);
			}
		}
		l_triplets.push_back(SparseMatrixTriplet(tet_list[k], tet_list[3], m_myu * W * -1.0 * sum));
		l_triplets.push_back(SparseMatrixTriplet(tet_list[3], tet_list[k], m_myu * W * -1.0 * sum));
	}
	for (int i = 0; i < 3; i++) {
		double beki_sum = 0;
		for (int j = 0; j < 3; j++) {
			beki_sum += B(j, i);
		}
		l_triplets.push_back(SparseMatrixTriplet(tet_list[3], tet_list[3], m_myu * W * 1.0*beki_sum *beki_sum));
	}





}


void Simulation::computeElementJacobianMat(const EigenMatrix3 &B, const ScalarType W, const unsigned int tet_list[], const unsigned int ele_num, std::vector<SparseMatrixTriplet>& j_triplets)
{
	//	// Inprement Here !! //
	//	// Small Hint: use "m_myu"

	int i, j;
	//	m_myu = 2.0;

	for (j = 0; j< 3; j++) {//xyz

		for (i = 0; i < 3; i++) {//1234
			j_triplets.push_back(SparseMatrixTriplet(ele_num * 3 + i, tet_list[j], -1.0* m_myu * W * B(j, i)));
			j_triplets.push_back(SparseMatrixTriplet(ele_num * 3 + i, tet_list[3], 1.0 * m_myu * W * B(j, i)));
		}
	}

}
double Simulation::clamp(double n, double lower, double upper) {

	return std::max(lower, std::min(n, upper));
}
void Simulation::factorizeDirectSolverLLT(const SparseMatrix& Mat_A, Eigen::SimplicialLLT<SparseMatrix, Eigen::Upper>& lltSolver)
{
	SparseMatrix A_prime = Mat_A;
	SparseMatrix I(Mat_A.cols(), Mat_A.rows());
	//	I.setIdentity();
	lltSolver.analyzePattern(A_prime);
	lltSolver.factorize(A_prime);
	ScalarType Regularization = 0.00001;
	bool success = true;
	while (lltSolver.info() != Eigen::Success)
	{
		Regularization *= 10;
		A_prime = A_prime + Regularization*I;
		lltSolver.factorize(A_prime);
		success = false;
	}
	char* warning_msg = "";
	if (!success)
		std::cout << "Warning: " << warning_msg << " adding " << Regularization << " identites.(llt solver)" << std::endl;
}