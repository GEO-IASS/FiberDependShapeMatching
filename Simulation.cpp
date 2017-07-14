#include "Simulation.h"



Simulation::Simulation()
{
}



Simulation::~Simulation()
{

}


void Simulation:: Init() {//should after meshs

	//set size

	pos.resize(mesh->GetVertexNum());
	D_pos.resize(mesh->GetVertexNum());
	velocity.resize(mesh->GetVertexNum());
	f_ext.resize(mesh->GetVertexNum());
	f_g.resize(mesh->GetVertexNum());

	

	element_pos.resize(mesh->GetVertexNum());	//Rest gravity element pos		(mesh->GetVertexNum(),3)
	D_element_pos.resize(mesh->GetVertexNum());	//Current gravity element pos	(mesh->GetVertexNum(),3)

	Mg_element_pos.resize(mesh->GetVertexNum());	//Current center of mass element pos	(mesh->GetVertexNum(),3)
	Dg_element_pos.resize(mesh->GetVertexNum());	//   Rest center of mass element pos		(mesh->GetVertexNum(),3)


	for (uint i = 0; i<mesh->GetVertexNum(); ++i)
	{
		pos[i]   = mesh->GetVertex(i);
		D_pos[i] = mesh->GetVertex(i);
		velocity[i].setZero();
		f_ext[i].setZero();
		f_g[i].setZero();
	}


	Matrix_A.resize(mesh->GetVertexNum());				//(mesh->GetVertexNum(), 3,3)
	Matrix_Aqq.resize(mesh->GetVertexNum());				//(mesh->GetVertexNum(), 3,3)
	Matrix_Apq.resize(mesh->GetVertexNum());				//(mesh->GetVertexNum(), 3,3)
	RotMat.resize(mesh->GetVertexNum());				//(mesh->GetVertexNum(), quartanion)


	for (int i = 0; i < mesh->GetVertexNum(); i++) {
		Matrix_A[i].setZero();
		Matrix_Apq[i].setZero();
		Matrix_Aqq[i].setZero();

		RotMat[i].setIdentity();
	

		//set position

		_Bone.push_back(i);
	}
}

void Simulation::ComputeOnering() {

	Onering.clear();
	Onering.resize(mesh->GetVertexNum());

	for (int i = 0; i < mesh->GetTetNum(); i++) {

		int index0 = mesh->GetTetra(i,0) +  1;
		for (int j = 0; j < 4; j++) {
			Onering[mesh->GetTetra(i,j)].push_back(index0);
		}


		int index1 = mesh->GetTetra(i, 1) +  1;
		for (int j = 0; j < 4; j++) {
			Onering[mesh->GetTetra(i, j)].push_back(index1);
		}

		int index2 = mesh->GetTetra(i, 2) +  1;
		for (int j = 0; j < 4; j++) {
			Onering[mesh->GetTetra(i , j)].push_back(index2);
		}

		int index3 = mesh->GetTetra(i , 3) +  1;
		for (int j = 0; j < 4; j++) {
			Onering[mesh->GetTetra(i , j)].push_back(index3);
		}

	}

	for (int i = 0; i < mesh->GetVertexNum(); i++) {
		sort(Onering[i].begin(), Onering[i].end());
		Onering[i].erase(std::unique(Onering[i].begin(), Onering[i].end()), Onering[i].end());
	}


	for (int i = 0; i < mesh->GetVertexNum(); i++) {
		for (int j = 0; j < Onering[i].size(); j++) {
			Onering[i][j] = Onering[i][j] - 1;
		}
	}

}


void Simulation::GetCenterOfGravity(std::vector<Eigen::Vector3d> &vec, Eigen::Vector3d &v, int ele, double _beta) {
	Eigen::Vector3d  g_pos = { 0.0, 0.0, 0.0 };

	double sum = 0.0;

	for (int i = 0; i < Onering[ele].size(); i++) {  //ele :: region
		int index = Onering[ele][i];

		g_pos += vec[i] * SetWeight(index, ele, _beta);// 
		sum += SetWeight(index, ele, _beta);//

	}

	if (sum < 0.001) {
		std::cout << "errorGCG" << std::endl;
		system("pause");
	}
	else {
		v = g_pos / sum;
	}
}



void Simulation::GetManipulateMatrix(int ele, double _beta) {
	//Init every Matrix
	Matrix_A[ele].setZero();
	Matrix_Apq[ele].setZero();
	Matrix_Aqq[ele].setZero();
	//calcurate matrix or each region

	//	double qq[3][3];
	Eigen::Matrix3d  pq;
	Eigen::Matrix3d  qq;
	Eigen::Vector3d _pi;
	Eigen::Vector3d _qi;

//	SetBasisMatrix(ele);
	for (int i = 0; i < Onering[ele].size(); i++)
	{

		int index = Onering[ele][i];

		_pi = (pos[index] - Mg_element_pos[ele]);
		_qi = (D_pos[index] - Dg_element_pos[ele]);
	//	_qi = Matrix_TOT[ele] * _qi;

		pq = _pi*_qi.transpose();
		qq = _qi*_qi.transpose();
		Matrix_Apq[ele] += SetWeight(index, ele, _beta) *pq;
		Matrix_Aqq[ele] += SetWeight(index, ele, _beta) *qq;

	}
	Matrix_A[ele] = Matrix_Apq[ele] * Matrix_Aqq[ele].inverse();

}


double Simulation::SetWeight(int count, int region, double beta) { //count is i (vert num ) & region is r beta is depend on region

	double sum = 0.0;
	Eigen::Vector3d _qi;
	_qi.setZero();

//	SetBasisMatrix(region);

	//	std::cout <<"after"<< Matrix_T_ele[region].col(0) << std::endl;

	for (int k = 0; k < Onering[region].size(); k++) {
		_qi -= D_pos[Onering[region][k]] * mass;
		sum += mass;
	}
	_qi /= sum;
	_qi += D_pos[count];

	if (_qi.norm() < 0.00001) { //stem("pause");
		_qi = Eigen::Vector3d(1.0, 0.0, 0.0);
	}


	/*if (abs(weight_vec[region].norm()) < 0.000001)
	{
		std::cout << "WeightVec" << std::endl;
		system("pause");
		return mass;
	}
	else {
	*/	
	Eigen::Vector3d unit1;
	unit1 = { weight_vec[region](0) ,weight_vec[region](1),weight_vec[region](2) };
		/*if (Matrix_T_ele[region].col(0) != unit1) {
			std::cout << "MatT_ele" << std::endl;

			system("pause");
		}
*/

		double b = 1.0 / (sqrt(beta * beta + (1 - beta)*(1 - beta)))* (beta *weight_vec[region] + (1.0 - beta) / _qi.norm()*_qi).dot(_qi);
		return mass*((b*b / (_qi.norm() *  _qi.norm())));//  *a + (1 - a) *Matrix_T[region]);

}


void Simulation::extractRotation(const Eigen::Matrix3d &A, Eigen::Quaterniond &q, const unsigned int maxIter)
{
	//Eigen::AngleAxis
	for (unsigned int iter = 0; iter < maxIter; iter++)
	{
		Eigen::Matrix3d R = q.matrix();
		Eigen::Vector3d omega = (R.col(0).cross(A.col(0)) + R.col
		(1).cross(A.col(1)) + R.col(2).cross(A.col(2))
			) * (1.0 / fabs(R.col(0).dot(A.col(0)) + R.col
			(1).dot(A.col(1)) + R.col(2).dot(A.col(2))) + 1.0e-9);
		double w = omega.norm();
		if (w < 1.0e-9)
			break;
		q = Eigen::Quaterniond(Eigen::AngleAxisd(w, (1.0 / w) * omega)) *	q;
		q.normalize();
	}
}


void Simulation::DeformVector() {
	////calcurate gravity of element


	for (int ele = 0; ele < mesh->GetVertexNum(); ele++) {
		weight_vec[ele] = RotMat[ele] * weight_vec_init[ele];
		//std::cout << RotMat[ele].toRotationMatrix() << std::endl;
	}

//should re calculate gravity of center
	for (int i = 0; i < _Bone.size(); i++) {
		int index = _Bone[i];
		GetCenterOfGravity(D_element_pos[index], Dg_element_pos[index], index, 1.0);
	}


	//for (int i = 0; i < Muscle.size(); i++) {
	//	int index = Muscle[i];
	//	GetCenterOfGravity(D_element_pos[index], Dg_element_pos[index], index, beta_M);	// initial
	//}
}
void Simulation::Integration(int ele, double alpha, double _beta) {

	Eigen::Vector3d goal = { 0.0, 0.0, 0.0 };
	GetGoalPosition(goal, ele, _beta); //add the weight

	velocity[ele] += alpha * (goal - pos[ele]) / time + (f_ext[ele] + f_g[ele]) / mass*time;// +time *Rot_Mat[i][j] / mass*ganma;// +f_coll[i][j] / mass*time;		//各頂点速度の更新
	pos[ele] += time * velocity[ele];																			//各頂点位置の更新	
	velocity[ele] *= 0.8;

}

void Simulation::Integration_bone(int ele, double alpha) {
	int index = _Bone[ele];

	/*VERTEX3D V = dst.GetVertex(index);
	Eigen::Vector3d goal;

	goal(0) = V.x;  goal(1) = V.y;   goal(2) = V.z;		*/	// 変形後の座標格納
	
	//velocity[index] += alpha * (goal - pos[index]) / time + (f_ext[index] + f_g[index]) / mass*time;// +time *Rot_Mat[i][j] / mass*ganma;// +f_coll[i][j] / mass*time;		//各頂点速度の更新
	//pos[index] += time * velocity[index];																			//各頂点位置の更新	
	//velocity[index] *= 0.8;
}

void Simulation::GetGoalPosition(Eigen::Vector3d &goal, int ele, double _beta) {//write for element (ele is vertex)

	goal.setZero();
	double sum = 0.0;
	for (int i = 0; i < Onering[ele].size(); i++) // summation  for r including i 
	{

		int index = Onering[ele][i];	//region index
		Eigen::Vector3d qir;
		qir = D_pos[ele] - Dg_element_pos[index];
		goal += (RotMat[index] * qir + Mg_element_pos[index]) *SetWeight(ele, index, _beta);
		sum += SetWeight(ele, index, _beta);

	}
	if (sum < 0.001) {
		std::cout << "errorGGP" << std::endl;
		system("pause");
	}
	else {
		goal = goal / sum;
	}
}


void Simulation::Update() {
	
	// Calculation current center of gravity
	for (int i = 0; i < _Bone.size(); i++) {
		int index = _Bone[i];
		GetCenterOfGravity(element_pos[index], Mg_element_pos[index], index, 1.0);
	}

	for (int i = 0; i < _Bone.size(); i++) {
		int index = _Bone[i];
		GetManipulateMatrix(index, 1.0);
	}

	for (int i = 0; i < _Bone.size(); i++) {
		int index = _Bone[i];
		extractRotation(Matrix_A[index], RotMat[index], 20);
	}

	//---no constraint
	for (int i = 0; i < _Bone.size(); i++) {
		int index = _Bone[i];
		Integration(index, 1.0, 1.0);
	}


}

void Simulation::Positon_to_mesh() {

	for (uint i = 0; i<mesh->GetVertexNum(); ++i)
	{
		mesh->GetVertex(i) = pos[i] ;
	}

}
//void Simulation::SetBasisMatrix(int i) {

//Matrix_T[i](0, 0) = weight_vec[i](0);
//Matrix_T[i](1, 0) = weight_vec[i](1);
//Matrix_T[i](2, 0) = weight_vec[i](2);
//Matrix_T_ele[i].col(0) = weight_vec[i];

//if (weight_vec[i](1) < 0.00001 && weight_vec[i](2) < 0.00001) {

//	Matrix_T[i](0, 1) = weight_vec[i](1);
//	Matrix_T[i](1, 1) = -weight_vec[i](0);
//	Matrix_T[i](2, 1) = 0.0;
//	Matrix_T[i].col(1) = Matrix_T[i].col(1) / (Matrix_T[i].col(1)).norm();
//	Matrix_T_ele[i].col(1) = Matrix_T[i].col(1);
//}
//else {
//	Matrix_T[i](0, 1) = 0.0;
//	Matrix_T[i](1, 1) = weight_vec[i](2);
//	Matrix_T[i](2, 1) = -weight_vec[i](1);
//	//正規化
//	Matrix_T[i].col(1) = Matrix_T[i].col(1) / Matrix_T[i].col(1).norm();
//	Matrix_T_ele[i].col(1) = Matrix_T[i].col(1);
//}

//Matrix_T[i].col(2) = (Matrix_T[i].col(0)).cross(Matrix_T[i].col(1));
//Matrix_T[i].col(2) = Matrix_T[i].col(2) / (Matrix_T[i].col(2)).norm();
//Matrix_T_ele[i].col(2) = Matrix_T[i].col(2);
//Matrix_T[i] = Matrix_T[i] * Matrix_T[i].transpose();
//Matrix_T[i] = Matrix_T[i] / Matrix_T[i].determinant();

//}
