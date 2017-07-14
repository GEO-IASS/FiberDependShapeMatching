#include <iostream>


#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include"mesh.h"
#include"glsl.h"
#include "common.h"
#include "camera.h"
#include "Simulation.h"

static int WIN_WIDTH = 1280;                       // ウィンドウの幅
static int WIN_HEIGHT = 720;                       // ウィンドウの高さ
static const char *WIN_TITLE = "BOYOB_BOYON";

int count = 1.0;

Render* render;
TetMesh * mesh;
Camera *camera;
Simulation *simulation;

//----------Control--------------------//
int mouse_old_x, mouse_old_y;
int mouse_wheel_pos;
unsigned char g_button_mask = 0x00;

//----------global parameter -------------------------//

int screen_width = DEFAULT_SCREEN_WIDTH;
int screen_height = DEFAULT_SCREEN_HEIGHT;
 

//---------------
void resize(GLFWwindow *const window, int width, int height);
void mouse_motion(GLFWwindow *const window);
void wheel(GLFWwindow *const window, double x, double y);


void paintGL() {
	//背景色と深度地のクリア frame
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

	render->SetCameraModelview(camera->GetViewMatrix());
	render->SetCameraProjection(camera->GetProjectionMatrix());

	render->ActivateShaderprog();
	mesh->draw(render->getVBO(), false, 0);
	render->DeactivateShaderprog();



}

void update() {
	simulation->Update();
	simulation->SetElement();
	simulation->Positon_to_mesh();
	std::cout << count << std::endl;
	count++;
}

void initializeGL() {
	//背景色の決定
	glClearColor(0.0, 0.0, 0.0, 1.0); 

	//depth の有効か
	glEnable(GL_DEPTH_TEST);


	//render の初期化
	fprintf(stdout, "Initializing render wrapper...\n");
	mesh = new TetMesh();
	render = new Render();
	render->InitShader(SHADER_DIRECTORY_V, SHADER_DIRECTORY_F);
	// camera init
	fprintf(stdout, "Initializing camera...\n");
	camera = new Camera();

	//file loading and set scaling for mesh model

	mesh->set_scale(1.0);
	mesh->Init();
	//mesh->
	camera->Reset(DEFAULT_SCREEN_WIDTH,DEFAULT_SCREEN_HEIGHT);

	simulation = new Simulation();
	simulation->SetMesh(mesh);
	simulation->Init();

}



int main(int argc, char **argv) {

	// OpenGLを初期化する
	if (glfwInit() == GL_FALSE) {
		fprintf(stderr, "Initialization failed!\n");
		return 1;
	}
//	atexit(glfwTerminate);
	//Window making

	GLFWwindow *window = glfwCreateWindow(WIN_WIDTH, WIN_HEIGHT, WIN_TITLE,	NULL, NULL);
	glfwMakeContextCurrent(window);

	glewExperimental = GL_TRUE;
	if (glewInit() != GLEW_OK) {
		std::cerr << "Can't initialixe GLEW" << std::endl;
		system("pause");
		exit(1);
		
	}


	//OPrn Glの初期化関数
	initializeGL();

	//set call back
	glfwSetWindowSizeCallback(window, resize);
	glfwSetScrollCallback(window, wheel);
	//glfw


	while (glfwWindowShouldClose(window) == GL_FALSE) {
		// アニメーション
		update();
		// 描画
		paintGL();
		// 描画用バッファの切り替え
		glfwSwapBuffers(window);
		glfwWaitEvents();
		mouse_motion(window);
		glfwPollEvents();
//		system("pause");
	}




}

void mouse_motion(GLFWwindow *const window)
{
	if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_1) != GLFW_RELEASE) {
		double x, y;
		glfwGetCursorPos(window, &x, &y);
		float dx, dy;
		dx = (float)(x - mouse_old_x);
		dy = (float)(y - mouse_old_y);

//		if (g_button_mask & 0x01)
//		{// left button
			camera->MouseChangeHeadPitch(0.2f, dx, dy);
//		}
		//else if (g_button_mask & 0x02)
		//{// middle button
		//	camera->MouseChangeLookat(0.01f, dx, dy);
		//}
		//else if (g_button_mask & 0x04)
		//{// right button
		//	camera->MouseChangeDistance(0.05f, dx, dy);
		//}
		

		mouse_old_x = x;
		mouse_old_y = y;
	}
}

void wheel(GLFWwindow *const window, double x, double y)
{
	mouse_wheel_pos+= y;// glfwGetMouseWheel();
	//std::cout<< glfwSetScrollCallback
	{
		camera->MouseChangeDistance(1.0f, 0, (ScalarType)(y));
	}
}

void resize(GLFWwindow *const window, int width, int height) {
	
	screen_width = width;
	screen_height = height;
	glViewport(0, 0, width, height);
	camera->ResizeWindow(width, height);

}


