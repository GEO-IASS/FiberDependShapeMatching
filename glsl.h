
#ifndef _GLSL_WRAPPER_H_
#define _GLSL_WRAPPER_H_
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
//#include "stb_image.h"


#include "opengl_headers.h"
#include "common.h"

class Render
{
public:
	Render(void);
	virtual ~Render(void);

	void InitShader(const char* vert_path, const char* frag_path);
//	bool InitTexture(const char* tex_path);
	void CleanupShader();

	void SetCameraProjection(glm::mat4 projection);
	void SetCameraModelview(glm::mat4 modelview);

	void ActivateShaderprog();
	void DeactivateShaderprog();

public: // inlines
	inline VBO& getVBO() { return m_vbo_handle; }

private:
	VBO m_vbo_handle;
	GLuint vert_ShaderId, frag_ShaderId, programId;
//	GLuint m_texture;

private: // private methods

		 //helper function to read shader source and put it in a char array
		 //thanks to Swiftless
//	char* textFileRead(const char*);

	//some other helper functions from CIS 565
//	void printLinkInfoLog(int);
//	void printShaderInfoLog(int);
};

#endif
