/*=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP, Shanghai Jiao Tong University, under MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

常量定义，首字母大写
#include"nabo_nabo_config.h"
=====================================================*/
#include<sstream>
#include<string>
#include<chrono>
#include<thread>
#include"mujoco/mujoco.h"
#include"glfw3.h"
#include"nabo.h"
#ifdef _WIN32
#include<windows.h>
#endif

#define For(x) for(int i=0;i<x;++i)
using namespace std;

int sleepMs=20;
double dt;
Nabo::inputStruct in;
Nabo::outputStruct out;
bool floatBaseFlag=0, slowRanderFlag=1;
stringstream msg;

inline void clip(double&x, double lim){
	if(lim<0){lim=-lim;}
	if(x<-lim){x=-lim;}
	else if(x>lim){x=lim;}
}
void runCtrl(mjData &d){
	in.cnt=(int)(d.time/dt);
	if(floatBaseFlag){
		mjtNum*q=d.qpos+3;
		in.rpy[0]=atan2(2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]*q[1]+q[2]*q[2]));
		in.rpy[1]=asin(2*(q[0]*q[2]-q[3]*q[1]));
		in.rpy[2]=atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]*q[2]+q[3]*q[3]));
		for(int i=0;i<3;i++){
			in.gyr[i]=d.qvel[i+3];
			in.acc[i]=d.qacc[i];
			in.supP[i]=d.qpos[i];
			in.supV[i]=d.qvel[i];
		}
		in.supP[2]-=0.04;//ankle到脚底高度0.04，W系
		For(NMot){
			in.j[i]=d.qpos[i+7];//当存在浮基，qpos前7个数为【浮动基3d位置+四元数】
			in.w[i]=d.qvel[i+6];//当存在浮基，qvel前6个数为【浮动基3d速度+角速度】
			in.t[i]=d.actuator_force[i];
		}
	}else{
		in.supP[2]=0.65-(1e-4);
		For(NMot){
			in.j[i]=d.qpos[i];
			in.w[i]=d.qvel[i];
			in.t[i]=d.actuator_force[i];
		}
	}
	Nabo::run(in,out);
	For(NMot){
		d.ctrl[i]=out.t[i];
	}
}
//=========================
mjModel *m;
mjData *d;
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context
GLFWwindow* window;
//==mouse interaction======
bool button_left=false;
bool button_middle=false;
bool button_right= false;
double lastx=0;
double lasty=0;
//==keyboard callback======
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods){
	// backspace: reset simulation
	if(act==GLFW_PRESS){
		switch(key){
		case GLFW_KEY_BACKSPACE:
			mj_resetData(m, d);
			mj_forward(m, d);
			// break;//没有break
		case GLFW_KEY_SPACE:
			in.cmdVx=0;in.cmdVy=0;in.cmdWz=0;break;
		case GLFW_KEY_W:
			in.cmdVx+=0.1;break;
		case GLFW_KEY_S:
			in.cmdVx-=0.1;break;
		case GLFW_KEY_A:
			in.cmdVy+=0.1;break;
		case GLFW_KEY_D:
			in.cmdVy-=0.1;break;
		case GLFW_KEY_J:
			in.cmdWz+=0.1;break;
		case GLFW_KEY_L:
			in.cmdWz-=0.1;break;
		}
		if(abs(in.cmdVx)<1e-4){in.cmdVx=0;}
		if(abs(in.cmdVy)<1e-4){in.cmdVy=0;}
		if(abs(in.cmdWz)<1e-4){in.cmdWz=0;}
		clip(in.cmdVx,0.5);
		clip(in.cmdVy,0.2);
		clip(in.cmdWz,0.5);
		msg.clear();
		msg.str("");
		msg<<"cmd: vx="<<in.cmdVx<<", vy="<<in.cmdVy<<", wz="<<in.cmdWz;
	}
}
//==mouse button callback=============
void mouse_button(GLFWwindow* window, int button, int act, int mods){
	// update button state
	button_left=(glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
	button_middle=(glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
	button_right=(glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);
	// update mouse position
	glfwGetCursorPos(window, &lastx, &lasty);
}
//=mouse move callback=================
void mouse_move(GLFWwindow* window, double xpos, double ypos){
	if(!button_left && !button_middle && !button_right){return;}
	// compute mouse displacement, save
	double dx=xpos-lastx, dy=ypos-lasty;
	lastx=xpos; lasty=ypos;
	// get current window size
	int width, height;
	glfwGetWindowSize(window, &width, &height);
	// get shift key state
	bool mod_shift=(glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);
	// determine action based on mouse button
	mjtMouse action;
	if(button_right){
		action=mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
	}else if(button_left){
		action=mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
	}else{
		action=mjMOUSE_ZOOM;
	}
	mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}
//==scroll callback=============
void scroll(GLFWwindow* window, double xoffset, double yoffset){
  // emulate vertical mouse motion=5% of window height
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, 0.05*yoffset, &scn, &cam);
}


int main(int argc, char* argv[]){
	char errorMsg[64]{"fail to load xml!\n"};
	#ifdef _WIN32
	LoadKeyboardLayout("0x409", KLF_ACTIVATE | KLF_SETFORPROCESS);
	m=mj_loadXML("../../nabo.xml",0,errorMsg,64);
	#else
	m=mj_loadXML("../nabo.xml",0,errorMsg,64);
	#endif
	d=mj_makeData(m);
	if(!glfwInit()){mju_error("Could not initialize GLFW");}
	// create window, make OpenGL context current, request v-sync
	window=glfwCreateWindow(1200, 800, "nabo", NULL, NULL);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);
	// initialize visualization data structures
	mjv_defaultCamera(&cam);
	cam.lookat[2]=0.5;
	cam.azimuth=60;
	cam.elevation=0;
	cam.distance=3;
	mjv_defaultOption(&opt);
	mjv_defaultScene(&scn);
	mjr_defaultContext(&con);
	// create scene and context
	mjv_makeScene(m, &scn, 2000);
	mjr_makeContext(m, &con, mjFONTSCALE_100);
	// install GLFW mouse and keyboard callbacks
	glfwSetKeyCallback(window, keyboard);
	glfwSetCursorPosCallback(window, mouse_move);
	glfwSetMouseButtonCallback(window, mouse_button);
	glfwSetScrollCallback(window, scroll);

	dt=m->opt.timestep;
	Nabo::setDt(dt);
	if(m->nq>NMot){floatBaseFlag=1;}
	msg<<"keyboard: {w s a d j l} for movments";

	while(!glfwWindowShouldClose(window)){
		mjtNum simstart=d->time;
		//小循环内mj_step按硬件能力进行场景计算，循环外按30帧更新显示
		while(d->time-simstart < 1.0/30){
			runCtrl(*d);
			mj_step(m, d);
		}
		//延时为了人眼看清楚。另：不同旋转视角渲染速度也不同
		if(slowRanderFlag){this_thread::sleep_for(chrono::milliseconds(sleepMs));}
		// if(d->time>0.99){exit(0);}
		mjrRect viewport ={0, 0, 0, 0};
		glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
		mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
		mjr_render(viewport, &scn, &con);

		mjrRect rect{0,0,200,50};
		mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, rect, msg.str().data(), 0, &con);
		glfwSwapBuffers(window);// swap OpenGL buffers(blocking call due to v-sync)
		glfwPollEvents();// process pending GUI events, call GLFW callbacks
	}
	mjv_freeScene(&scn);
	mjr_freeContext(&con);
	mj_deleteData(d);
	mj_deleteModel(m);
	// terminate GLFW(crashes with Linux NVidia drivers)
	#if defined(__APPLE__) || defined(_WIN32)
	glfwTerminate();
	#endif
	return 0;
}