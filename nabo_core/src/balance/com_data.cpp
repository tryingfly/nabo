/*=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP, Shanghai Jiao Tong University, under MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

=====================================================*/
#include "com_data.h"
namespace WBC{
	comDataStruct::comDataStruct(){
		kpA=vec3d(40,40,40);
		kiA=vec3d(0.02,0.02,0.02);
		kdA=vec3d(10,10,10);
		kpP=vec3d(30,30,40);
		kiP=vec3d(0.02,0.02,0.02);
		kdP=vec3d(10,10,10);
		setZero();
	}
	void comDataStruct::setZero(){
		aAcc.setZero();
		acc.setZero();
		errASum.setZero();
		errPSum.setZero();
		for(int i=0;i!=6;++i){
			fil[i].reset(30,0);
		}
	}
	void comDataStruct::pid(const vec3d &errA,const vec3d &errP,const vec3d &errW,const vec3d &errV){
		errASum+=errA;
		errPSum+=errP;
		errASum*=0.999;
		errPSum*=0.998;
		aAcc=kpA.cwiseProduct(errA)+ kiA.cwiseProduct(errASum)+ kdA.cwiseProduct(errW);
		acc=kpP.cwiseProduct(errP)+ kiP.cwiseProduct(errPSum)+ kdP.cwiseProduct(errV);
	}
	void comDataStruct::filt(){
		aAcc[0]=fil[0].filt(aAcc[0]);
		aAcc[1]=fil[1].filt(aAcc[1]);
		aAcc[2]=fil[2].filt(aAcc[2]);
		acc[0]=fil[3].filt(acc[0]);
		acc[1]=fil[4].filt(acc[1]);
		acc[2]=fil[5].filt(acc[2]);
	}
}