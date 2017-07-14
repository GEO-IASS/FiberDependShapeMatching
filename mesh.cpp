#pragma warning( disable : 4267)

#include <fstream>
#include "mesh.h"
#include "common.h"
#include "io_mesh.h"

#pragma Mesh
void Mesh::reset()
{
	cleanup();
	init();
}

void Mesh::cleanup()
{
	m_positions.clear();
	m_normals.clear();
	m_colors.clear();
	m_texcoords.clear();
	m_triangle_list.clear();
}

void Mesh::initialset() {
	//initial set parameter

}
void Mesh::draw(const VBO& vbos, bool wire_frame, int show_texture)// draw using VBo
{
	m_dim = 3;
	for (uint i = 0; i<m_vert_num; ++i)
	{
		for (uint j = 0; j<m_dim; ++j)
		{
			m_positions[i][j] = m_V[i][j];
		}
	}


	computeNormal();

	glPolygonMode(GL_FRONT_AND_BACK, (wire_frame ? GL_LINE : GL_FILL));

	uint size = m_vert_num;
	uint element_num = m_triangle_list.size();


	// position
	glBindBuffer(GL_ARRAY_BUFFER, vbos.m_vbo);
	glBufferData(GL_ARRAY_BUFFER, 3 * size * sizeof(float), &m_positions[0], GL_DYNAMIC_DRAW);

	// color
	glBindBuffer(GL_ARRAY_BUFFER, vbos.m_cbo);
	glBufferData(GL_ARRAY_BUFFER, 3 * size * sizeof(float), &m_colors[0], GL_STATIC_DRAW);
	// normal
	glBindBuffer(GL_ARRAY_BUFFER, vbos.m_nbo);
	glBufferData(GL_ARRAY_BUFFER, 3 * size * sizeof(float), &m_normals[0], GL_DYNAMIC_DRAW);
	// texture
	glBindBuffer(GL_ARRAY_BUFFER, vbos.m_tbo);
	glBufferData(GL_ARRAY_BUFFER, 2 * size * sizeof(float), &m_texcoords[0], GL_STATIC_DRAW);

	// indices
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbos.m_ibo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, element_num * sizeof(uint), &m_triangle_list[0], GL_STATIC_DRAW);

	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);
	glEnableVertexAttribArray(2);
	glEnableVertexAttribArray(3);

	glBindBuffer(GL_ARRAY_BUFFER, vbos.m_vbo);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, vbos.m_cbo);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, vbos.m_nbo);
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, vbos.m_tbo);
	glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, 0, 0);

	glm::mat4 identity = glm::mat4(); // identity matrix
	glUniformMatrix4fv(vbos.m_uniform_transformation, 1, false, &identity[0][0]);

	glUniform1i(vbos.m_uniform_enable_texture, show_texture); // enable/disable texture

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbos.m_ibo);
	glDrawElements(GL_TRIANGLES, element_num, GL_UNSIGNED_INT, 0);

	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);
	glDisableVertexAttribArray(2);
	glDisableVertexAttribArray(3);
	glUniform1i(vbos.m_uniform_enable_texture, 0); // disable texture

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

void Mesh::computeNormal()
{
	// reset all the normal.
	glm::vec3 zero(0.0);
	for (std::vector<glm::vec3>::iterator n = m_normals.begin(); n != m_normals.end(); ++n)
	{
		*n = zero;
	}
	// calculate normal for each individual triangle
	uint triangle_num = m_triangle_list.size() / 3;
	uint id0,id1, id2;
	EigenVector3 p0, p1, p2;
	EigenVector3 normal;
	for (uint i = 0; i < triangle_num; ++i)
	{
		id0 = m_triangle_list[3 * i];
		id1 = m_triangle_list[3 * i + 1];
		id2 = m_triangle_list[3 * i + 2];

		p0 = { m_positions[id0][0], m_positions[id0][1], m_positions[id0][2] };
		p1 = { m_positions[id1][0], m_positions[id1][1], m_positions[id1][2] };
		p2 = { m_positions[id2][0], m_positions[id2][1], m_positions[id2][2] };
		normal = (p1 - p0).cross(p2 - p1);
		normal.normalize();
		glm::vec3 glm_normal = glm::vec3(normal[0], normal[1], normal[2]);

		m_normals[id0] += glm_normal;
		m_normals[id1] += glm_normal;
		m_normals[id2] += glm_normal;
	}
	// re-normalize all the normals.
	for (std::vector<glm::vec3>::iterator n = m_normals.begin(); n != m_normals.end(); ++n)
	{
		if (glm::length(*n) > EPSILON) {// skip if norm is a zero vector
			*n = glm::normalize(*n);
		}
	}
}
#pragma endregion

#pragma region TetMesh
bool TetMesh::Init()
{
	strcpy_s(m_mesh_file_path, MESHMODEL);
	m_dim = 3;
	LoadTetMesh();
	initialset();
	generateParticleList();
	generateTriangleList();
	
	//edgelist‚Æ‚Í
	return true;
}

//#include <igl/read_triangle_mesh.h>

bool TetMesh::LoadTetMesh() {
	m_loaded_mesh = new MeshLoader(m_mesh_file_path, m_tet_scaling);
	if (m_loaded_mesh->Info() == false)
	{
		std::cout << "Load mesh error. Using regular Mesh." << std::endl;
		delete m_loaded_mesh;

		m_mesh_type = MESH_TYPE_TET;
		Init();
		return false;
	}

	//// libigl
	//std::cout << "load by libigl..." << std::endl;
	//igl::readMESH(m_mesh_file_path, m_V, m_T, m_F);
	//m_V *= m_tet_scaling; // adjast scale
	//m_V0 = m_V;

	//m_vert_num = m_V.rows();
	//m_system_dim = m_vert_num * 3;

	//ScalarType unit_mass = m_total_mass / m_system_dim;
	return true;

}


void TetMesh::generateParticleList()
{
	std::cout << "TetMesh::ParticleList Generating..." << std::endl;
	m_vert_num = m_loaded_mesh->m_vertices.size();
	unsigned int m_faces =  m_loaded_mesh->m_faces.size();
	unsigned int m_tets = m_loaded_mesh->m_tets.size();


	m_system_dim = m_vert_num * 3;
	ScalarType unit_mass = m_total_mass / m_system_dim;
	
	//set initial parameter for e

	m_V0.resize(m_vert_num);	// rest pose
	m_V.resize(m_vert_num);	// current pose
//	m_N.resize(m_vert_num, 3);	// current normal
	m_F.resize(m_faces);  // tri index
	m_T.resize(m_tets);  // tet index

	for (uint i = 0; i<m_vert_num; ++i)
	{
		for (uint j = 0; j<m_dim; ++j)
		{
			m_V0[i] [j] = m_loaded_mesh->m_vertices[i][j];
			m_V[i][j] = m_loaded_mesh->m_vertices[i][j];
		}
	}

	for (uint i = 0; i<m_faces; ++i)
	{
		m_F[i].resize(3);
			m_F[i] [0] = m_loaded_mesh->m_faces[i].id1;
			m_F[i] [1] = m_loaded_mesh->m_faces[i].id2;
			m_F[i] [2] = m_loaded_mesh->m_faces[i].id3;

	}

	for (uint i = 0; i<m_tets; ++i)
	{
		m_T[i].resize(4);
		m_T[i][ 0] = m_loaded_mesh->m_tets[i].id1;
		m_T[i][ 1] = m_loaded_mesh->m_tets[i].id2;
		m_T[i][ 2] = m_loaded_mesh->m_tets[i].id3;
		m_T[i][ 3] = m_loaded_mesh->m_tets[i].id4;

	}

	// resize variables for render
	m_positions.resize(m_vert_num);

	m_normals.resize(m_vert_num);
	m_colors.resize(m_vert_num);
	m_texcoords.resize(m_vert_num);

	// color
	glm::vec3 mesh_color(0.3, 0.8, 1);
	for (uint i = 0; i<m_vert_num; ++i)
	{
		m_colors[i] = mesh_color;
	}
}

void TetMesh::generateTriangleList()
{
	std::cout << "TetMesh::TriangleList Generating..." << std::endl;

	m_triangle_list.resize(m_F.size() * 3); // #triangle * 3

	for (int i = 0; i != m_F.size(); ++i)
	{
		m_triangle_list[3 * i + 0] = m_F[i][ 0];
		m_triangle_list[3 * i + 1] = m_F[i][ 1];
		m_triangle_list[3 * i + 2] = m_F[i][ 2];
	}
}
#pragma endregion