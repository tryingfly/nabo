/*=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP, Shanghai Jiao Tong University, under MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

腿位置线性预测，忽略pit、rol对惯量影响
=====================================================*/
#include"mpc.h"
#include"robot.h"
#include"cpg.h"
#include"eiquadprog/eiquadprog-fast.hpp"
#include<thread>
#include<chrono>
#include<mutex>
#include<condition_variable>
#include"iopack.h"

// #define DefMpcTest
// #define DefMpcTiming

#ifdef DefMpcTest
#pragma message("mpc.cpp: test模式")
#endif

using namespace std;
namespace Blc{
const int xDim=12,uDim=12;
const int Horizon=10;
const int Horizon12=Horizon*12;
const double maxFz=300;

class mpcClass::impClass{
public:
	impClass();
	void setCiParam(double mu,double muz,double lx,double ly);
	void loop();
	void runOnce();
	void test();
	Rbt::rbtClass &rbt=Rbt::rbtClass::instance();
	Cpg::cpgClass &cpg=Cpg::cpgClass::instance();
	double pcT,stT,g=10;
	double mu,muz;//摩擦系数mu，旋转摩擦系数muz，用于摩擦锥约束。
	double lx,ly;//脚面半长lx，脚面半宽ly，用于zmp约束。if脚长不对称需改lx
	int stStage[Horizon*2];
	double q[12],r[12];

	stateStruct stt,stt0;
	double mass=20;
	mat3d Inertia0=mat3d::Identity(),InertiaInv=mat3d::Identity();

	matXd(12,12) B;
	vecXd(12) g0;
	vecXd(Horizon12) AG,xd,h;
	matXd(Horizon12,Horizon12) BB,H;//AA已简化到A*x0-tgt+BB*g
	Eigen::VectorXd u;
	vecXd(12) mf,x1;

	matXd(0,Horizon12) CE;
	matXd(0,1) ce;
	matXd(-1,-1) CI;//直接给size太大超限
	vecXd(-1) ci;
	eiquadprog::solvers::EiquadprogFast qp;

	bool isStarted{0},isStop{1};
	bool isWait{0},needSleep{1};
	mutex mut;
	condition_variable cv;
};
	mpcClass::impClass::impClass(){
		isWait=1;
		qp.reset(Horizon12, 0, Horizon*22);
		u.resize(Horizon12);
		CI.setZero(Horizon*22,Horizon12);
		ci.setZero(Horizon*22);
		setCiParam(1, 0.1, 0.08, 0.03);

		mass=rbt.getMass();
		Inertia0=rbt.getInertia();
		InertiaInv=Inertia0.inverse();

		B.setZero();
		BB.setZero();
		g0.setZero();
		stt.setZero();
		stt0.setZero();

		x1.setZero();
		mf.setZero();
		mf[5]=mass*5;mf[11]=mass*5;//5=g/2

		ini.getArray("mpcQ",q);
		ini.getArray("mpcR",r);
	};
	void mpcClass::impClass::setCiParam(double mu,double muz,double lx,double ly){
		Alg::clip(mu,0,10);
		Alg::clip(muz,0,1);
		Alg::clip(lx,0,1);
		Alg::clip(ly,0,1);
		this->mu=mu;
		this->muz=muz;
		this->lx=lx;
		this->ly=ly;
		// //== CI*x+ci>0，参考cbic.cpp拓，【但mpc的f方向是需求方向】
		For(Horizon){
			int i12=i*12;
			int i22=i*22;
			CI.block<5,5>(i22, i12).setIdentity();
			CI.block<5,1>(i22, i12+5)<<ly,lx,muz,mu,mu;
			CI.block<5,5>(i22+5, i12).setIdentity();
			CI.block<5,5>(i22+5, i12)*=-1;
			CI.block<6,1>(i22+5, i12+5)<<ly,lx,muz,mu,mu,-1;
			//左腿
			CI.block<5,5>(i22+11, i12+6).setIdentity();
			CI.block<5,1>(i22+11, i12+11)<<ly,lx,muz,mu,mu;
			CI.block<5,5>(i22+16, i12+6).setIdentity();
			CI.block<5,5>(i22+16, i12+6)*=-1;
			CI.block<6,1>(i22+16, i12+11)<<ly,lx,muz,mu,mu,-1;
		}
		// fprintEi(CI);
	}
	void mpcClass::impClass::loop(){
		auto refClock=chrono::high_resolution_clock::now();
		int period=10000;//微秒
		auto tag0=refClock,tag1=refClock;
		while(isStarted){
			while(needSleep){
				this_thread::sleep_for(chrono::microseconds(period));//微秒
				refClock=chrono::high_resolution_clock::now();
			}
			refClock+=chrono::microseconds(period);//微秒

			#ifdef DefMpcTiming
			tag0=chrono::high_resolution_clock::now();
			#endif

			unique_lock<mutex> lock(mut);
			isWait=1;
			while(isWait){cv.wait(lock);}
			stt=stt0;
			lock.unlock();

			#ifdef DefMpcTiming
			tag1=chrono::high_resolution_clock::now();
			#endif

			runOnce();
			//==平均耗时计算（20个点）==
			#ifdef DefMpcTiming
			static double rcd0[20]{},average0=0,rcd1[20]{},average1=0;
			static int idx=0;
			auto dddt0=chrono::duration_cast<chrono::nanoseconds>(chrono::high_resolution_clock::now()-tag0);
			auto dddt1=chrono::duration_cast<chrono::nanoseconds>(chrono::high_resolution_clock::now()-tag1);
			rcd0[idx]=dddt0.count()/1e6;
			rcd1[idx]=dddt1.count()/1e6;
			average0+=rcd0[idx]/20;
			average1+=rcd1[idx]/20;
			idx++; idx%=20;
			average0-=rcd0[idx]/20;
			average1-=rcd1[idx]/20;
			if(idx==0){cout<<"mpc循环平均耗时(ms)="<<average0<<"，计算平均耗时(ms)="<<average1<<endl;}
			#endif
			//=================
			// this_thread::sleep_until(refClock);
		}
		isStop=1;
	}
	void mpcClass::impClass::runOnce(){
		pcT=cpg.getPeriod()/Horizon;
		stT=cpg.getStT(0);
		For(Horizon){
			double t=pcT*i;
			int i2{i*2},i12{i*12};
			cpg.isFutureSt(t,stStage+i2);
			double MaxMoment=20;
			if(stStage[i2]){
				ci[i*22+10]=maxFz;
			}else{
				ci[i*22+10]=0;
			}
			if(stStage[i2+1]){
				ci[i*22+21]=maxFz;
			}else{
				ci[i*22+21]=0;
			}
		}
		g0[5]=-10*pcT*pcT*0.5;
		g0[11]=-10*pcT;

		AG.head<12>()<<stt.aDlt+stt.w*pcT, stt.pDlt+stt.v*pcT, stt.w, stt.v;
		AG.head<12>()+=g0;
		xd.head<12>()<<stt.w*pcT, stt.v*pcT, stt.w, stt.v;
		for(int i=1;i<Horizon;i++){
			int i12{i*12};
			AG.segment<12>(i12)=AG.segment<12>(i12-12);
			AG.segment<6>(i12)+=AG.segment<6>(i12+6)*pcT;
			AG.segment<12>(i12)+=g0;
			xd.segment<12>(i12)=xd.segment<12>(i12-12);
			xd.segment<6>(i12)+=xd.segment<6>(i12+6)*pcT;
		}
		//========================
		vec3d tip[2]{stt.leg0,stt.leg1};
		//B角速度项
		B.block<3,3>(6,0)=InertiaInv*pcT;
		B.block<3,3>(6,3)=InertiaInv*Ei::skew(tip[0])*pcT;
		B.block<3,3>(6,6)=InertiaInv*pcT;
		B.block<3,3>(6,9)=InertiaInv*Ei::skew(tip[1])*pcT;
		//B角度项
		B.topRows<3>()=B.middleRows<3>(6)*pcT*0.5;
		//B速度项
		double tmp=pcT/mass;
		B(9,3)=tmp;B(10,4)=tmp;B(11,5)=tmp;
		B(9,9)=tmp;B(10,10)=tmp;B(11,11)=tmp;
		//B位置项
		tmp=0.5*pcT*tmp;
		B(3,3)=tmp;B(4,4)=tmp;B(5,5)=tmp;
		B(3,9)=tmp;B(4,10)=tmp;B(5,11)=tmp;

		BB.topLeftCorner<12,12>()=B;
		for(int i=1;i<Horizon;i++){
			int i12{i*12};
			mat2d Rz2d=Ei::rot2R2d(stt.wz*pcT*i);
			vec2d dXY=Rz2d*vec2d(stt.vx, stt.vy)*pcT;
			for(int l=0;l<2;l++){
				if(stStage[i*2+l]==2 && stStage[i*2+l-2]==0){//由sw首次落地预测落脚点
					vec2d tmp(stt.vx*stT/2, stt.vy*stT/2 +0.1*(2*l-1));
					tmp=Rz2d*tmp;
					tip[l][0]=tmp[0];//运动学预测是在当前F系，但动力学发生在未来的F系，故无需添加body位移
					tip[l][1]=tmp[1];
				}else{
					tip[l][0]-=dXY[0];
					tip[l][1]-=dXY[1];
				}
				mat3d tmp=InertiaInv*Ei::skew(tip[l])*pcT;
				B.block<3,3>(6,l*6+3)=tmp;
				B.block<3,3>(0,l*6+3)=tmp*pcT*0.5;
			}
			BB.block<12,12>(i12,i12)=B;
			for(int j=0;j<i;j++){
				BB.block<6,12>(i12+6,j*12)=BB.block<6,12>(i12-6,j*12);
				BB.block<6,12>(i12,j*12)=BB.block<6,12>(i12-12,j*12)
										+BB.block<6,12>(i12+6,j*12)*pcT;
			}
		}
		h=AG-xd;
		For(Horizon12){
			double qi=q[i%12];
			H.row(i)=BB.row(i)*qi;
			h[i]*=qi*qi;
		}
		H=H.transpose()*H;
		h=BB.transpose()*h;
		For(Horizon12){
			H(i,i)+=r[i%12];//R权重
		}
		For(Horizon){
			int i2{i*2},i12{i*12};
			if(stStage[i2] & stStage[i2+1]){
				h[i12+5]-=r[5]*mass*5;//5=g/2
				h[i12+11]-=r[11]*mass*5;//5=g/2
			}else{
				if(stStage[i2]){
					h[i12+5]-=r[5]*mass*10;
				}else if(stStage[i2+1]){
					h[i12+11]-=r[11]*mass*10;
				}
			}
		}
		qp.solve_quadprog(H,h,CE,ce,CI,ci,u);
		memcpy(mf.data(),u.data(),96);
		x1=AG.head<12>()+BB.topLeftCorner<12,12>()*mf;
	}
//============================
	mpcClass& mpcClass::instance(){
		static mpcClass singleton;
		return singleton;
	}
	mpcClass::mpcClass():imp(*new impClass()){}
	bool mpcClass::start(){
		if(!imp.isStarted & imp.isStop){
			imp.isStarted=1;
			thread thd(&mpcClass::impClass::loop,&(this->imp));//因为loop不是静态，写成这样就可以了
			thd.detach();
			imp.isStop=0;
			return 1;
		}
		return 0;
	}
	bool mpcClass::stop(){
		imp.isStarted=0;
		return 1;
	}
	bool mpcClass::wake(){
		imp.needSleep=0;
		if(imp.isStarted){return 1;}
		return 0;
	}
	bool mpcClass::sleep(){
		imp.needSleep=1;
		if(imp.isStarted){return 1;}
		return 0;
	}
	void mpcClass::setCiParam(double mu,double muz,double lx,double ly){
		imp.setCiParam(mu,muz,lx,ly);
	}
	void mpcClass::setState(stateStruct& stt){
		lock_guard<mutex> lock(imp.mut);
		if(imp.isWait){
			imp.stt0=stt;
			imp.isWait=0;
			imp.cv.notify_one();
		}
		#ifdef DefMpcTest
		imp.stt=stt;
		#endif
	}
	void mpcClass::getResult(vec3d &p,mat3d &R,vec6d &wv,vec6d &mf0,vec6d &mf1){
		p=imp.x1.segment<3>(3);
		R=Ei::rpy2R(imp.x1.segment<3>(0));
		wv=imp.x1.segment<6>(6);
		mf0=-imp.mf.head<6>();
		mf1=-imp.mf.tail<6>();
	}
	void mpcClass::getMF(vec6d &mf0,vec6d &mf1){
		mf0=-imp.mf.head<6>();
		mf1=-imp.mf.tail<6>();
	}
//==打印测试=============================
	#ifndef DefMpcTest
	void mpcClass::test(){}
	void mpcClass::testRun(){}
	#else
	void mpcClass::testRun(){imp.runOnce();}
	void mpcClass::test(){imp.test();}
	void mpcClass::impClass::test(){
		runOnce();
		cout<<"mpc: test 将写入 zzz.txt\n";
		fout<<"=======* mpc test *=======\nstage:\n";
		For(Horizon){
			fout<<stStage[i*2]<<", ";
		}
		fout<<endl;
		For(Horizon){
			fout<<stStage[i*2+1]<<", ";
		}
		// fout<<"\nci:\n";
		// For(Horizon){
		// 	fprintEi(ci.segment<22>(i*22));
		// }
		fout<<"\nu:\n";
		For(Horizon){
			fprintEi(u.segment<12>(i*12));
		}
		fout<<"\nAG:\n";
		For(Horizon){
			fprintEi(AG.segment<12>(i*12));
		}
		vecXd(Horizon12) x=BB*u;
		fout<<"\nBB*u:\n";
		For(Horizon){
			fprintEi(x.segment<12>(i*12));
		}
		x+=AG;
		fout<<"\nx:(A,p,w,v)\n";
		For(Horizon){
			fprintEi(x.segment<12>(i*12));
		}
		fout<<"\ncost=";
		auto cost1=u.transpose()*H*u;// +u.transpose()*h;
		auto cost2=u.transpose()*h;
		fout<<cost1<<", "<<cost2<<"\n合力: ";
		vec3d xyz(0,0,0);
		For(2){
			xyz+=u.segment<3>(i*6+3);
		}
		xyz-=mass*vec3d(0,0,10);
		fprintEi(xyz);
		fprintEi(xyz/mass);
	}
	#endif //DefMpcTest
}//namespace


