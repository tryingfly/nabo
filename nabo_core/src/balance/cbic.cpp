/*=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP, Shanghai Jiao Tong University, under MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

wbic比wbc多出来的impulse，是指前馈fr
=====================================================*/

#include"eiquadprog/eiquadprog-fast.hpp"
#include"cbic.h"
#include"robot.h"

// #define DefCbicTest

#ifdef DefCbicTest
#include"iopack.h"
#pragma message("cbic.cpp: test模式")
#endif

namespace Blc{
class cbicClass::impClass{
public:
	impClass(cbicClass *omp);
	Rbt::rbtClass &rbt=Rbt::rbtClass::instance();
	cbicClass &omp;
	eiquadprog::solvers::EiquadprogFast qp;
	double mu,muz;//摩擦系数mu，旋转摩擦系数muz，用于摩擦锥约束。
	double lx,ly;//脚面半长lx，脚面半宽ly，用于zmp约束。if脚长不对称需改lx

	const mat3d I3=mat3d::Identity();
	matXd(6,12) A;
	matXd(6,1) b;
	matXd(18,18) H;
	matXd(18,1) h;
	matXd(6,18) CE;
	matXd(6,1) ce;
	matXd(22,18) CI;
	matXd(22,1) ci;
	Eigen::VectorXd fr;//前进坐标下mpc f
	Eigen::VectorXd u;//前进坐标下df、dacc

	Alg::filterOneClass fFilter[12];
	void init(double dt);
	void setZero();
	void setCiParam(double mu,double muz,double lx,double ly);
	void inline cal(vec3d &aAcc,vec3d &acc);
	void inline setCi(int leg,double maxFz);
	void inline setCiRight(double maxFz);
	void inline setCiLeft(double maxFz);
};

	cbicClass::impClass::impClass(cbicClass *omp):omp(*omp){
		qp.reset(18,6,22);
		mat3d Z3;
		Z3.setZero();
		A<< I3,Z3,I3,Z3,
			Z3,I3,Z3,I3;
		H.setIdentity();
		for(int i=12;i<18;i++){
			H(i,i)=0.2;
		}
		H(13,13)=0.5;//pit
		h.setZero();

		CE<<I3,Z3,I3,Z3,I3,Z3,
			Z3,I3,Z3,I3,Z3,I3;
		CI.setZero();
		ci.setZero();
		setCiParam(1, 0.1, 0.08, 0.03);

		fr.resize(12);
		u.resize(18);
		u.setZero();
		setZero();
	}
	void cbicClass::impClass::init(double dt){
		for(int i=0;i!=12;++i){
			fFilter[i].init(dt,200,0,0);
		}
		setZero();
	}
	void cbicClass::impClass::setZero(){
		fr.setZero();
		fr[5]=-rbt.getMass()*5;
		fr[11]=-rbt.getMass()*5;
		for(int i=0;i!=12;++i){
			fFilter[i].setBase(fr[i]);
		}
	};
	void cbicClass::impClass::setCiParam(double mu,double muz,double lx,double ly){
		Alg::clip(mu,0,10);
		Alg::clip(muz,0,1);
		Alg::clip(lx,0,1);
		Alg::clip(ly,0,1);
		this->mu=mu;
		this->muz=muz;
		this->lx=lx;
		this->ly=ly;
		//== CI*x+ci>0
		//左前11行右腿
		CI.block<5,5>(0,0).setIdentity();
		CI.block<5,1>(0,5)<<-ly,-lx,-muz,-mu,-mu;
		CI.block<5,5>(5,0).setIdentity();
		CI.block<5,5>(5,0)*=-1;
		CI.block<6,1>(5,5)<<-ly,-lx,-muz,-mu,-mu,1;
		//右后11行左腿
		CI.block<5,5>(11,6).setIdentity();
		CI.block<5,1>(11,11)<<-ly,-lx,-muz,-mu,-mu;
		CI.block<5,5>(16,6).setIdentity();
		CI.block<5,5>(16,6)*=-1;
		CI.block<6,1>(16,11)<<-ly,-lx,-muz,-mu,-mu,1;
		// cout<<CI<<endl;//CI期望结果附于最后
	}
	void cbicClass::impClass::setCi(int i,double maxFz){
		if(i){
			setCiLeft(maxFz);
		}else{
			setCiRight(maxFz);
		}
	}
	void cbicClass::impClass::setCiRight(double maxFz){
		//== CI*x+ci>0
		ci[0]= fr[0] -ly*fr[5];
		ci[1]= fr[1] -lx*fr[5];
		ci[2]= fr[2]-muz*fr[5];
		ci[3]= fr[3] -mu*fr[5];
		ci[4]= fr[4] -mu*fr[5];
		ci[5]=-fr[0] -ly*fr[5];
		ci[6]=-fr[1] -lx*fr[5];
		ci[7]=-fr[2]-muz*fr[5];
		ci[8]=-fr[3] -mu*fr[5];
		ci[9]=-fr[4] -mu*fr[5];
		ci[10]=maxFz+fr[5];
	}
	void cbicClass::impClass::setCiLeft(double maxFz){
		ci[11]= fr[6] -ly*fr[11];
		ci[12]= fr[7] -lx*fr[11];
		ci[13]= fr[8]-muz*fr[11];
		ci[14]= fr[9] -mu*fr[11];
		ci[15]= fr[10]-mu*fr[11];
		ci[16]=-fr[6] -ly*fr[11];
		ci[17]=-fr[7] -lx*fr[11];
		ci[18]=-fr[8]-muz*fr[11];
		ci[19]=-fr[9] -mu*fr[11];
		ci[20]=-fr[10]-mu*fr[11];
		ci[21]=maxFz+fr[11];
	}
	void cbicClass::impClass::cal(vec3d &aAcc,vec3d &acc){
		A.block<3,3>(0,3)<<Ei::skew(rbt.imu.Rxy*rbt.getAnkleP2B(0));
		A.block<3,3>(0,9)<<Ei::skew(rbt.imu.Rxy*rbt.getAnkleP2B(1));
		CE.block<3,3>(0,3)=A.block<3,3>(0,3);
		CE.block<3,3>(0,9)=A.block<3,3>(0,9);
		b<<rbt.imu.Rxy*rbt.getInertia()*rbt.imu.Rxy.transpose()*aAcc, rbt.getMass()*(acc+vec3d(0,0,10));
		ce=A*fr+b;//f是向下的，b取负
		qp.solve_quadprog(H,h,CE,ce,CI,ci,u);

		for(int i=0;i!=6;++i){
			double ff=u[i]+fr[i];
			omp.mf[0][i]=fFilter[i].filt(ff);

			ff=u[i+6]+fr[i+6];
			omp.mf[1][i]=fFilter[i+6].filt(ff);
		}

		#ifdef DefCbicTest
		cout<<"=======* wbic test *=======\nu[f]:\n";
		printEi(u.segment<6>(0));
		printEi(u.segment<6>(6));
		cout<<"\nu[acc]:\n";
		printEi(u.segment<6>(12));
		cout<<"\nf=u+fr:\n";
		printEi(u.segment<6>(0)+fr.segment<6>(0));
		printEi(u.segment<6>(6)+fr.segment<6>(6));
		cout<<"\n===========================\n";
		#endif
	}
//==========================================
	cbicClass& cbicClass::instance(){
		static cbicClass singtn;
		return singtn;
	}
	cbicClass::cbicClass():imp(*new impClass(this)){}
	void cbicClass::init(double dt){
		imp.init(dt);
	}
	void cbicClass::setZero(){
		imp.setZero();
		mf[0].setZero();
		mf[1].setZero();
		mf[0][5]=-imp.rbt.getMass()*5;
		mf[1][5]=-imp.rbt.getMass()*5;
	}
	void cbicClass::setCiParam(double mu,double muz,double lx,double ly){
		imp.setCiParam(mu,muz,lx,ly);
	}
	void cbicClass::setFr(const vec6d (&mf)[2]){
		memcpy(imp.fr.data(), mf[0].data(), 48);
		memcpy(imp.fr.data()+6, mf[1].data(), 48);
	};
	//==根据相位分配力==
	void cbicClass::run(vec3d &aAcc,vec3d &acc){//前进坐标下
		For(2){
			if(imp.rbt.isSwing(i)){
				imp.setCi(i,0);
			}else{
				imp.setCi(i,maxFz);
			}
		}
		imp.cal(aAcc,acc);
	}
	//==根据相位分配力==支撑腿平滑
	void cbicClass::run(vec3d &aAcc,vec3d &acc,const double (&stPgs)[2],bool isLock){
		For(2){
			if(isLock){
				imp.setCi(i, maxFz*erf(15*stPgs[i]) );
			}else if(stPgs[i]){
				imp.setCi(i, maxFz*( erf(15*stPgs[i]) +erf(15*(1-stPgs[i])) -1) );
			}else{
				imp.setCi(i,0);
			}
		}
		imp.cal(aAcc,acc);
	}
	//==根据相位分配力==支撑摆动分别平滑
	void cbicClass::run(vec3d &aAcc,vec3d &acc,const double (&stPgs)[2],const double (&swPgs)[2]){
		For(2){
			if(stPgs[i]){
				imp.setCi(i, maxFz*erf(stPgs[i]*16) );
			}else{
				imp.setCi(i, maxFz*(erf(-swPgs[i]*16) +1));
			}
		}
		imp.cal(aAcc,acc);
	}
}//namespace


/*============CI期望结果:===============
   1    0    0    0    0  -ly    0    0    0    0    0    0    0    0    0    0    0    0
   0    1    0    0    0  -lx    0    0    0    0    0    0    0    0    0    0    0    0
   0    0    1    0    0 -muz    0    0    0    0    0    0    0    0    0    0    0    0
   0    0    0    1    0  -mu    0    0    0    0    0    0    0    0    0    0    0    0
   0    0    0    0    1  -mu    0    0    0    0    0    0    0    0    0    0    0    0
  -1    0    0    0    0  -ly    0    0    0    0    0    0    0    0    0    0    0    0
   0   -1    0    0    0  -lx    0    0    0    0    0    0    0    0    0    0    0    0
   0    0   -1    0    0 -muz    0    0    0    0    0    0    0    0    0    0    0    0
   0    0    0   -1    0  -mu    0    0    0    0    0    0    0    0    0    0    0    0
   0    0    0    0   -1  -mu    0    0    0    0    0    0    0    0    0    0    0    0
   0    0    0    0    0    1    0    0    0    0    0    0    0    0    0    0    0    0
   0    0    0    0    0    0    1    0    0    0    0  -ly    0    0    0    0    0    0
   0    0    0    0    0    0    0    1    0    0    0  -lx    0    0    0    0    0    0
   0    0    0    0    0    0    0    0    1    0    0 -muz    0    0    0    0    0    0
   0    0    0    0    0    0    0    0    0    1    0  -mu    0    0    0    0    0    0
   0    0    0    0    0    0    0    0    0    0    1  -mu    0    0    0    0    0    0
   0    0    0    0    0    0   -1    0    0    0    0  -ly    0    0    0    0    0    0
   0    0    0    0    0    0    0   -1    0    0    0  -lx    0    0    0    0    0    0
   0    0    0    0    0    0    0    0   -1    0    0 -muz    0    0    0    0    0    0
   0    0    0    0    0    0    0    0    0   -1    0  -mu    0    0    0    0    0    0
   0    0    0    0    0    0    0    0    0    0   -1  -mu    0    0    0    0    0    0
   0    0    0    0    0    0    0    0    0    0    0    1    0    0    0    0    0    0
========================================*/