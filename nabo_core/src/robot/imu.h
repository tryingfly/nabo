/*=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP, Shanghai Jiao Tong University, under MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

=====================================================*/
#pragma once
#include"eigen.h"
namespace Rbt{
class imuClass{
public:
	double rpy[3]{};//imu的yaw存在漂移，此处yaw为tgt.rpy[2]
	double yawAct;//imu的实际yaw，处理init为0
	vec3d w,acc;
	mat3d Rxy,Rz,R;//R内包含tgt.rpy[2]
	imuClass(){
		init(0,0,0);
	}
	//rol、pit会直接set，但yawRead将作为基准，让此刻yawAct=0
	void init(const double (&rpy)[3]){
		init(rpy[0],rpy[1],rpy[2]);
	}
	void init(const double &rol,const double &pit,const double &yawRead){
		w.setZero();
		acc.setZero();
		rpy[0]=rol+rolAjd;
		rpy[1]=pit+pitAdj;
		rpy[2]=0;
		yawAct=0;
		yawRaw=0;
		imuYawLast=yawRead;

		rolFilter.reset(35,0,rol);//rolAjd加在外面
		pitFilter.reset(35,0,pit);//pitAjd加在外面
		yawActFilter.reset(35,0);
		Rxy=aAxisY(rpy[1])*aAxisX(rpy[0]);
		Rz.setIdentity();
		R=Rxy;
	}
	void update(const double (&rpy)[3],const double yawImage,
				const double (&w)[3],
				const double (&acc)[3])
	{
		update(rpy[0],rpy[1],rpy[2],yawImage,w[0],w[1],w[2],acc[0],acc[1],acc[2]);
	}
	//yaw通常会有零飘，yawImage会写入imu.rpy[2], yawRead会多圈处理后写入imu.yawAct
	void update(const double rol,const double pit,const double yawRead,const double yawImage,
				const double wx,const double wy,const double wz,
				const double ax,const double ay,const double az){
		rpy[0]=rolFilter.filt(rol)+rolAjd;
		rpy[1]=pitFilter.filt(pit)+pitAdj;
		rpy[2]=yawImage;
		double tmp=yawRead-imuYawLast;
		imuYawLast=yawRead;
		// yawRaw+= tmp<5?(tmp>-5?tmp:tmp+Pi2):tmp-Pi2;//处理多圈突变
		yawRaw+= tmp-round(tmp/Pi2)*Pi2;//处理多圈突变
		yawAct=yawActFilter.filt(yawRaw);

		w=wFilter.filt(vec3d(wx,wy,wz));
		acc=aFilter.filt(vec3d(ax,ay,az));
		Rxy=aAxisY(rpy[1])*aAxisX(rpy[0]);
		Rz=aAxisZ(rpy[2]);
		R=Rz*Rxy;
	};
	void setAdjust(double rolAdj,double pitAdj){
		this->rolAjd=rolAdj;
		this->pitAdj=pitAdj;
	}
private:
	Alg::filterOneClass pitFilter;
	Alg::filterOneClass rolFilter;
	Alg::filterOneClass yawActFilter;
	Ei::filterOneClass aFilter;
	Ei::filterOneClass wFilter;
	double rolAjd=0,pitAdj=0;
	double imuYawLast=0,yawRaw=0;
};
}//namespace