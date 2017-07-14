#pragma once
#include "mesh.h"

class Simulation
{
public:
	Simulation();
	~Simulation();

	Mesh *mesh;
	void SetMesh(Mesh * i_mesh) { mesh = i_mesh; }
	void Init();

	double mass = 1.0;
	double time = 0.033;

	//set parameter

	//-----------------------Parameter (Always calculated)-----------------------------------------//

	std::vector<Eigen::Vector3d>          pos;							//current velocity      (vertnum,3)
	std::vector<Eigen::Vector3d>        D_pos;							//vertex of init        (vertnum,3)
	std::vector<Eigen::Vector3d>     velocity;						    //current velocity      (vertnum,3)
	std::vector<Eigen::Vector3d>        f_ext;							//external force
	std::vector<Eigen::Vector3d>          f_g;							//gravity force

	//-------------------------for element------------------------------------------------//

	//Position for element
	std::vector<std::vector<Eigen::Vector3d>>   element_pos;		    //vertex of current for element   (vertnum,Oneringsize,3)
	std::vector<std::vector<Eigen::Vector3d>> D_element_pos;			//vertex of init for element      (vertnum,Oneringsize,3)
																		//Center of mass for element
	std::vector<Eigen::Vector3d> Mg_element_pos;			//element vertex (center of mass//current)        (vertnum,3)
	std::vector<Eigen::Vector3d> Dg_element_pos;			//element vertex (center of mass//init)           (vertnum,3)

	
   //--------------------------Region Calcurate ------------------------------------//
	
   // Make Matrix Rotmat and Matrix A

	std::vector<Eigen::Quaterniond> RotMat;					//(vertnum,3*3)
	std::vector<Eigen::Matrix3d>  Matrix_A;					//(vertnum,3*3)
	std::vector<Eigen::Matrix3d>  Matrix_Apq;				//(vertnum,3*3)
	std::vector<Eigen::Matrix3d>  Matrix_Aqq;				//(vertnum,3*3)

	//-------------------------------compute one rings--------------------------------//

	void ComputeOnering();
	std::vector<std::vector<int>> Onering;

	//--------------------------------SHape Matching-----------------------------------//

	//-------------main solution---------------------//
	void Update();


	double SetWeight(int count, int region, double beta);
	void GetCenterOfGravity(std::vector<Eigen::Vector3d> &vec, Eigen::Vector3d &v, int ele, double _beta);
	//void SetBasisMatrix(int ele, double _beta);

	//---------setting matrix A-----------------------------//
	void GetManipulateMatrix(int ele, double _beta);

	//------------for solve 
	void extractRotation(const Eigen::Matrix3d &A, Eigen::Quaterniond &q, const unsigned int maxIter);


	//-----------------prepare calculation for next------------------//
	void DeformVector();

	
	//-------------gola position-------------------------------------//
	void GetGoalPosition(Eigen::Vector3d &goal, int ele, double _beta);
	void Integration_bone(int ele, double alpha);
	void Integration(int ele, double alpha, double _beta);



	//for fiber vector
	std::vector<Eigen::Vector3d>   weight_vec;					        //weight vector         (vertnum,3)   illustrationg as a color in
	std::vector<Eigen::Vector3d>   weight_vec_init;					    //weight vector init       (vertnum,3)

	//--------------------set region 
	std::vector <int > _Bone;


	//-----------------------for draw
	void Positon_to_mesh();

};

