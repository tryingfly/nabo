/*=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP, Shanghai Jiao Tong University, under MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

简化Eigen类名，以及常用函数
// #include"eigen.h"
=====================================================*/
#pragma once
#include<Eigen/Dense>
#include"algorithms.h"

#define vec2d Eigen::Vector2d
#define vec3d Eigen::Vector3d
#define vec6d Eigen::Matrix<double,6,1>
#define vecXd(r) Eigen::Matrix<double,r,1>

#define mat2d Eigen::Matrix2d
#define mat3d Eigen::Matrix3d
#define matXd(r,c) Eigen::Matrix<double,r,c>

#define quat4d Eigen::Quaterniond

#define aAxis  Eigen::AngleAxisd
#define aAxisX(rol) Eigen::AngleAxisd(rol,Eigen::Vector3d::UnitX())
#define aAxisY(pit) Eigen::AngleAxisd(pit,Eigen::Vector3d::UnitY())
#define aAxisZ(yaw) Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())


namespace Ei{
//==上下限区间==========
	inline void clip(vec3d &x,const vec3d &a, const vec3d &b){
		double*xx=x.data();
		const double*aa=a.data();
		const double*bb=b.data();
		For3{Alg::clip(xx[i],aa[i],bb[i]);}
	}
	inline void clip(vec3d &x,double lim){
		if(lim<0){lim=-lim;}
		For3{
			if(x[i]>lim){x[i]=lim;}
			else if(x[i]<-lim){x[i]=-lim;}
		}
	}
	inline void clip(vec3d &x,double a, double b){
		if(a<b){
			For3{
				if(x[i]>b){x[i]=b;}
				else if(x[i]<a){x[i]=a;}
			}
		}else{
			For3{
				if(x[i]>a){x[i]=a;}
				else if(x[i]<b){x[i]=b;}
			}
		}
	}
//==阀值=============
	inline void thresh(vec3d &x,double threshold){
		if(threshold<0){threshold=-threshold;}
		For3{
			if(x[i]>threshold){x[i]-=threshold;}
			else if(x[i]<-threshold){x[i]+=threshold;}
			else{x[i]=0;}
		}
	}
	inline vec3d threshed(vec3d &x,double threshold){
		if(threshold<0){threshold=-threshold;}
		For3{
			if(x[i]>threshold){x[i]-=threshold;}
			else if(x[i]<-threshold){x[i]+=threshold;}
			else{x[i]=0;}
		}
		return x;
	}
//==递增函数=============当cmd=tgt时return 1
	inline bool cmd2out1step(vec3d cmd,vec3d &tgt,double step){
		double*c=cmd.data();
		double*t=tgt.data();
		bool isDone(1);
		for(int i=0;i<3;++i){
			isDone &=Alg::cmd2out1step(c[i],t[i],step);
		}
		return isDone;
	}
//==反对称矩阵===============
	inline mat3d skew(const vec3d &v){
		mat3d M;
		M<<.0,-v[2],v[1], v[2],.0,-v[0], -v[1],v[0],.0;
		return M;
	};
	inline void skew(mat3d &M,const vec3d &v){
		M<<.0,-v[2],v[1], v[2],.0,-v[0], -v[1],v[0],.0;
	};
//==rpy转R==============
// eigen自带需要轴角乘3次，重写
	inline mat3d rpy2R(double rol,double pit,double yaw){
		mat3d R;
		double cx=cos(rol),cy=cos(pit),cz=cos(yaw);
		double sx=sin(rol),sy=sin(pit),sz=sin(yaw);
		R<< cz*cy,-sz*cx+cz*sy*sx, sz*sx+cz*sy*cx,
			sz*cy, cz*cx+sz*sy*sx,-cz*sx+sz*sy*cx,
			  -sy,          cy*sx,          cy*cx;
		return R;
	}
	inline mat3d rpy2R(vec3d rpy){
		mat3d R;
		double cx=cos(rpy[0]),cy=cos(rpy[1]),cz=cos(rpy[2]);
		double sx=sin(rpy[0]),sy=sin(rpy[1]),sz=sin(rpy[2]);
		R<< cz*cy,-sz*cx+cz*sy*sx, sz*sx+cz*sy*cx,
			sz*cy, cz*cx+sz*sy*sx,-cz*sx+sz*sy*cx,
			  -sy,          cy*sx,          cy*cx;
		return R;
	}
	inline mat3d rpy2R(double rpy[3]){
		mat3d R;
		double cx=cos(rpy[0]),cy=cos(rpy[1]),cz=cos(rpy[2]);
		double sx=sin(rpy[0]),sy=sin(rpy[1]),sz=sin(rpy[2]);
		R<< cz*cy,-sz*cx+cz*sy*sx, sz*sx+cz*sy*cx,
			sz*cy, cz*cx+sz*sy*sx,-cz*sx+sz*sy*cx,
			  -sy,          cy*sx,          cy*cx;
		return R;
	}
//==R转rpy==============
	inline mat2d rot2R2d(double rotZ){
		mat2d R;
		double c=cos(rotZ),s=sin(rotZ);
		R<<c,-s,
			s,c;
		return R;
	}
//==R转rpy==============
// eigen自带欧拉角范围不对，重写
	inline void R2rpy(const mat3d &R,double rpy[3]){
		rpy[0]=atan2(R(2,1),R(2,2));
		rpy[1]=asin(-R(2,0));
		rpy[2]=atan2(R(1,0),R(0,0));
	}
	inline vec3d R2rpy(const mat3d &R){
		vec3d rpy;
		rpy[0]=atan2(R(2,1),R(2,2));
		rpy[1]=asin(-R(2,0));
		rpy[2]=atan2(R(1,0),R(0,0));
		return rpy;
	}
//==伪逆===============
	inline mat3d pInv(const mat3d &A){
		return A.completeOrthogonalDecomposition().pseudoInverse();
	}
//==一阶滤波============
class filterOneClass{
public:
	filterOneClass(){};
	filterOneClass(double cutF){
		k=cutF*dt;
		Alg::clip(k,0,1);
		ks=k/50;
	};
	void init(double dt,double cutF,vec3d y0){
		this->dt=dt;
		k=cutF*dt;
		Alg::clip(k,0,1);
		ks=k/50;
		y=y0;
	};
	//默认50
	void setCutF(double cutF){
		k=cutF*dt;
		Alg::clip(k,0,1);
		ks=k/50;
	};
	const vec3d & filt(const vec3d &x){
		y+=k*(x-y)-ks*ySum;
		ySum+=y-x;
		return y;
	};
private:
	vec3d y=vec3d::Zero(), ySum=vec3d::Zero();
	double dt=0.001, k=50*dt, ks=k/50;
};
}
