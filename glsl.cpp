#include "glsl.h"

Render::Render()
{
}

Render::~Render()
{
}

void Render::InitShader(const char* vert_path, const char* frag_path) {

	vert_ShaderId = glCreateShader(GL_VERTEX_SHADER);
	frag_ShaderId = glCreateShader(GL_FRAGMENT_SHADER);


	static const std::string VERT_SHADER_FILE = std::string(SHADER_DIRECTORY_V);
	static const std::string FRAG_SHADER_FILE = std::string(SHADER_DIRECTORY_F);


	//vert_shaderの読み込み
	std::ifstream vertShaderFile(VERT_SHADER_FILE.c_str(), std::ios::in);
	if (!vertShaderFile.is_open()) {
		fprintf(stderr, "Failed to load vertex shader: %s\n", VERT_SHADER_FILE.c_str());
		exit(1);
	}

	std::string line;

	std::string vertShaderBuffer;
	while (!vertShaderFile.eof()) {
		std::getline(vertShaderFile, line);
		vertShaderBuffer += line + "\r\n";
	}

	const char *vertShaderCode = vertShaderBuffer.c_str();


	//fragment shader の読み込み



	std::ifstream fragShaderFile(FRAG_SHADER_FILE.c_str(), std::ios::in);
	if (!fragShaderFile.is_open()) {
		fprintf(stderr, "Failed to load fragment shader: %s\n", FRAG_SHADER_FILE.c_str());
		exit(1);
	}

	std::string fragShaderBuffer;
	while (!fragShaderFile.eof()) {
		std::getline(fragShaderFile, line);
		fragShaderBuffer += line + "\n";
	}
	const char *fragShaderCode = fragShaderBuffer.c_str();


	//shaderのコンパイル

	GLint compileStatus;
	glShaderSource(vert_ShaderId, 1, &vertShaderCode, NULL);
	glCompileShader(vert_ShaderId);
	glGetShaderiv(vert_ShaderId, GL_COMPILE_STATUS, &compileStatus);
	if (compileStatus == GL_FALSE) {
		fprintf(stderr, "Failed to compile vertex shader!\n");

		GLint logLength;
		glGetShaderiv(vert_ShaderId, GL_INFO_LOG_LENGTH, &logLength);
		if (logLength > 0) {
			GLsizei length;
			char *errmsg = new char[logLength + 1];
			glGetShaderInfoLog(vert_ShaderId, logLength, &length, errmsg);

			std::cerr << errmsg << std::endl;
			fprintf(stderr, "%s", vertShaderCode);
		}
	}

	glShaderSource(frag_ShaderId, 1, &fragShaderCode, NULL);
	glCompileShader(frag_ShaderId);
	//エラー処理
	glGetShaderiv(frag_ShaderId, GL_COMPILE_STATUS, &compileStatus);
	if (compileStatus == GL_FALSE) {
		fprintf(stderr, "Failed to compile fragment shader!\n");

		GLint logLength;
		glGetShaderiv(frag_ShaderId, GL_INFO_LOG_LENGTH, &logLength);
		if (logLength > 0) {
			GLsizei length;
			char *errmsg = new char[logLength + 1];
			glGetShaderInfoLog(frag_ShaderId, logLength, &length, errmsg);

			std::cerr << errmsg << std::endl;
			fprintf(stderr, "%s", vertShaderCode);
			delete[] errmsg;
		}
	}


	//shaderプログラムの準備
	programId = glCreateProgram();


	// 0 for position, 1 for color, 2 for normal.
	glBindAttribLocation(programId, 0, "v_position");
	glBindAttribLocation(programId, 1, "v_color");
	glBindAttribLocation(programId, 2, "v_normal");
	glBindAttribLocation(programId, 3, "v_texcoord");


	glAttachShader(programId, vert_ShaderId);
	glAttachShader(programId, frag_ShaderId);
	// bind attribute locations for the shaders


	/* シェーダオブジェクトの削除 */
	glDeleteShader(vert_ShaderId);
	glDeleteShader(frag_ShaderId);

	//link
	GLint linkState;
	glLinkProgram(programId);
	//エラー処理
	glGetProgramiv(programId, GL_LINK_STATUS, &linkState);
	if (linkState == GL_FALSE) {
		fprintf(stderr, "Failed to compile shader!\n");

		GLint logLength;
		glGetProgramiv(programId, GL_INFO_LOG_LENGTH, &logLength);
		if (logLength > 0) {
			GLsizei length;
			char *errmsg = new char[logLength];
			glGetProgramInfoLog(programId, logLength, &length, errmsg);

			std::cerr << errmsg << std::endl;
			delete[] errmsg;
		}

		exit(1);
	}

	//// query uniform locations from openGL.
	m_vbo_handle.m_uniform_modelview = glGetUniformLocation(programId, "u_modelviewMatrix");
	m_vbo_handle.m_uniform_projection = glGetUniformLocation(programId, "u_projMatrix");
	m_vbo_handle.m_uniform_transformation = glGetUniformLocation(programId, "u_transformMatrix");
	m_vbo_handle.m_uniform_enable_texture = glGetUniformLocation(programId, "u_choose_tex");
	m_vbo_handle.m_uniform_texture_sampler = glGetUniformLocation(programId, "u_sampler1");
	//VBO使ってるからなんちゃら～

	glUseProgram(programId);
}


void Render::CleanupShader()
{
	glDetachShader(programId,frag_ShaderId);
	glDetachShader(programId,vert_ShaderId);
	glDeleteShader(frag_ShaderId);
	glDeleteShader(vert_ShaderId);
	glDeleteProgram(programId);
}


void Render::ActivateShaderprog()
{
	GLint current_prog;
	glGetIntegerv(GL_CURRENT_PROGRAM, &current_prog);
	if (current_prog != (GLint)programId)
		glUseProgram(programId);
}

void Render::DeactivateShaderprog()
{
	GLint current_prog;
	glGetIntegerv(GL_CURRENT_PROGRAM, &current_prog);
	if (current_prog == (GLint)programId)
		glUseProgram(0);
}


void Render::SetCameraProjection(glm::mat4 projection)
{
	glMatrixMode(GL_PROJECTION);
	glLoadMatrixf(&projection[0][0]);

	ActivateShaderprog();
	glUniformMatrix4fv(m_vbo_handle.m_uniform_projection, 1, false, &projection[0][0]);
}

void Render::SetCameraModelview(glm::mat4 modelview)
{
	glMatrixMode(GL_MODELVIEW);
	glLoadMatrixf(&modelview[0][0]);

	ActivateShaderprog();
	glUniformMatrix4fv(m_vbo_handle.m_uniform_modelview, 1, false, &modelview[0][0]);
}

