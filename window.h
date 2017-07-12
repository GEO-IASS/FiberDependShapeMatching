#pragma once
#include <iostream>


#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/type_ptr.hpp>

class Window
{
	GLFWwindow *const window;

	GLfloat aspect;

public:
	//コンストラクタ
	Window(int width = 640, int height = 480, const char *title = "Heloo")
		:window(glfwCreateWindow(width, height, title, NULL, NULL))
	{
		if (window == NULL) {
			fprintf(stderr, "Window creation failed!");
			glfwTerminate();
			exit(1);
		}

		//target for current window

		// OpenGLの描画対象にWindowを追加
		glfwMakeContextCurrent(window);

		// GLEWを初期化する (glfwMakeContextCurrentの後でないといけない)
		glewExperimental = true;

		if (glewInit() != GLEW_OK) {
			fprintf(stderr, "GLEW initialization failed!\n");
			exit(1);
		}

		glfwSwapInterval(1);
		glfwSetWindowUserPointer(window, this);

		glfwSetWindowSizeCallback(window, resize);

		resize(window, width, height);



	}


	//de structor
	virtual ~Window()
	{
		glfwDestroyWindow(window);


	}

	GLfloat getAspect() const { return aspect; }

	int shouldClose() const
	{

		return glfwWindowShouldClose(window);
	}

	void swapBuffers()
	{
		glfwSwapBuffers(window);

		glfwWaitEvents();//イベントが発生するまでプログラムを停止させる
	}



	void resizeGL(GLFWwindow *window, int width, int height) {

		WIN_WIDTH = width;
		WIN_HEIGHT = height;

		glfwSetWindowSize(window, WIN_WIDTH, WIN_HEIGHT);

		//実際のウィンドウサイズ（ピクセルサイズ）を取得
		int renderBufferWidth, renderBufferHeight;
		glfwGetFramebufferSize(window, &renderBufferWidth, &renderBufferHeight);


		//ビューポート変換の更新
		glViewport(0, 0, renderBufferWidth, renderBufferHeight);//全体を設定
		camera->ResizeWindow(width, height);

	}




};