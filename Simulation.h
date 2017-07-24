#pragma once
#include "mesh.h"

class Mesh;

// for projective dyanmics

class Simulation
{
public:
	Simulation();
	~Simulation();

	//std::vector<Mesh*> mesh;
	Mesh* mesh;
	//void SetMesh(std::vector<Mesh*> i_meshs) { mesh = i_meshs; } // add the mesh all
	void SetMesh(Mesh* i_meshs) { mesh = i_meshs; } 
	void Init();

	double mass = 1.0;
	double time = 0.33;

	//set parameter

	// linline function

	// inline functions
	inline void setReprecomputeFlag() { m_precomputation_flag = false; }
	inline void setReprefactorFlag() { m_prefactorization_flag = false; }

	// main function 
	void update();


protected:

	// flags
	bool m_precomputation_flag;
	bool m_prefactorization_flag;


	//-----------------------Parameter (Always calculated)-----------------------------------------//

	//std::vector<EigenMatrixXs*>   m_V;						//current position        (vertnum,3)
	//std::vector<EigenMatrixXi*>   m_F;						//current velocity      (vertnum,3)
	//std::vector<EigenMatrixXi*>   m_T;						//external force
	//std::vector<EigenMatrixXs>  m_Vel;					//gravity force

	EigenMatrixXs*   m_V;						//current position        (vertnum,3)
	EigenMatrixXi*   m_F;						//current velocity      (vertnum,3)
	EigenMatrixXi*   m_T;						//external force
	EigenMatrixXs  m_Vel;					//gravity force



	SparseMatrix m_MassMat;					//Mass Matrix
	SparseMatrix m_Inertia;					//Inertial Force
	EigenMatrixXs m_ExternalForce;			//External Force

	
	//-------------------------for Precomputation------------------------------------------------//

	std::vector<EigenMatrix3> m_B; // Dm inverses
	std::vector<ScalarType>   m_W; // volume of Te


	
								  
										// for prefactorization
	SparseMatrix m_LaplacianMat;
	SparseMatrix m_JacobianMat;
	Eigen::SimplicialLLT<SparseMatrix, Eigen::Upper> m_prefactored_LLTsolver;

	// simulation constants
	ScalarType   m_h; // time_step
	uint m_iterations_per_frame; // for optimization method, number of iterations

	 
    // ------------------------------simulation constants-------------------------------------------//
	ScalarType m_gravity_constant;
	ScalarType m_damping_coefficient;


	ScalarType m_young, m_young_old;
	ScalarType m_poisson, m_poisson_old;
	ScalarType m_myu;
	ScalarType m_lambda;


	//----------------------------Functions for update simulation ----------------------------------//
private:
	
	void preComputation();
	void convertlameConstant();

	void dampVelocity();
	void computeInertia(); // calculate the inertia term: y = current_pos + current_vel*h
	void computeExternalForce(); // wind force is propotional to the area of triangles projected on the tangential plane

	void singularValueDecomp(EigenMatrix3& U, EigenVector3& SIGMA, EigenMatrix3& V, const EigenMatrix3& A);


	// integration scheme
	void integrateOptimizationMethod();

	bool integrateLocalGlobalOneIteration(EigenMatrixXs& X);

	// for local global method 
	void computeRotMat(EigenMatrixXs& RotMat, const EigenMatrixXs& Jv);
	void computeElementLaplacianMat(const EigenMatrix3 &B, const ScalarType W, const unsigned int tet_list[], std::vector<SparseMatrixTriplet>& l_triplets);
	void computeElementJacobianMat(const EigenMatrix3 &B, const ScalarType W, const unsigned int tet_list[], const unsigned int ele_num, std::vector<SparseMatrixTriplet>& j_triplets);
	void volumeconservation(EigenMatrix3 &F, EigenMatrix3 &B, const unsigned int tet_list[], EigenMatrixXs &X);

	void setLaplacianMat();
	void setJacobianMat();


	void prefactorize();


	double clamp(double n, double lower, double upper);

	// utility functions
	void updatePosAndVel(const EigenMatrixXs NewPos);
	void factorizeDirectSolverLLT(const SparseMatrix& Mat_A, Eigen::SimplicialLLT<SparseMatrix, Eigen::Upper>& lltSolver);//, char* warning_msg = "");

};

