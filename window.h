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
	//�R���X�g���N�^
	Window(int width = 640, int height = 480, const char *title = "Heloo")
		:window(glfwCreateWindow(width, height, title, NULL, NULL))
	{
		if (window == NULL) {
			fprintf(stderr, "Window creation failed!");
			glfwTerminate();
			exit(1);
		}

		//target for current window

		// OpenGL�̕`��Ώۂ�Window��ǉ�
		glfwMakeContextCurrent(window);

		// GLEW������������ (glfwMakeContextCurrent�̌�łȂ��Ƃ����Ȃ�)
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

		glfwWaitEvents();//�C�x���g����������܂Ńv���O�������~������
	}



	void resizeGL(GLFWwindow *window, int width, int height) {

		WIN_WIDTH = width;
		WIN_HEIGHT = height;

		glfwSetWindowSize(window, WIN_WIDTH, WIN_HEIGHT);

		//���ۂ̃E�B���h�E�T�C�Y�i�s�N�Z���T�C�Y�j���擾
		int renderBufferWidth, renderBufferHeight;
		glfwGetFramebufferSize(window, &renderBufferWidth, &renderBufferHeight);


		//�r���[�|�[�g�ϊ��̍X�V
		glViewport(0, 0, renderBufferWidth, renderBufferHeight);//�S�̂�ݒ�
		camera->ResizeWindow(width, height);

	}




};