/*=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP, Shanghai Jiao Tong University, under MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

=====================================================*/
#include"robot.h"
#include<rbdl/rbdl.h>
#include"iopack.h"

namespace Rbt{
	using namespace RigidBodyDynamics;
	using namespace RigidBodyDynamics::Math;

	class rbtClass::impClass{
	public:
		Model model;
		unsigned int feetId[2];
		rbtClass &omp;
		double dt=0.001;//初始值会被rbt.setDt覆盖
		vec3d hip[2]{{0, -HipY, HipZ},{0, HipY, HipZ}};
		double maxL=LenThigh+LenShank-0.001;//最大腿长

		vecXd(-1) q,qd,qdLast,qdd,t;
		vec3d ankleP[2];
		mat3d ankleR[2];
		vec6d ankleV6d[2],ankleF6d[2];
		matXd(-1,-1) Jcb6d;
		matXd(NBaseDof,NLegDof) JcbAnkle6d[2];

		InverseKinematicsConstraintSet ikSet;
		vec6d tgtAnkV6d[2],tgtAnkF6d[2];
		vecXd(-1) tgtQ,tgtQd,tgtQdLast,tgtQdd,tgtT;

		vecXd(6) legKp,legKd;
		vecXd(NMot) jntKp,jntKd;
		vecXd(NGenDof) dnmKp,dnmKd;

		impClass(rbtClass *omp);
		void update(const double(&jj)[NMot], const double(&ww)[NMot], const double(&tt)[NMot]);
		void dwdate();
	};
	rbtClass::impClass::impClass(rbtClass *omp):omp(*omp){
		model.gravity=Vector3d(0,0,-9.8);
		Body body(10, Vector3d(0,0,0), Vector3d(0.283, 0.242, 0.108)),
			 hipYaw(0.5, Vector3d(0,0,0), Vector3d(0.00929, 0.00929, 0.00225)),
			 hipRol(0.5, Vector3d(0,0,0), Vector3d(0.00225, 0.00929, 0.00929)),
			 thigh(1, Vector3d(0,0,-LenThigh/2),Vector3d(0.00113, 0.00113, 0.000113)),
			 shank(1, Vector3d(0,0,-LenShank/2),Vector3d(0.000454, 0.000454, 7.5e-5)),
			 ankle(0.01, Vector3d(0,0,0),Vector3d(1e-8, 1e-8, 1e-8)),
			 foot(0.4, Vector3d(0,0,0), Vector3d(0.000291,0.000291,3.8e-5));
		Joint jx(JointTypeRevolute,Vector3d(1,0,0)),
			  jy(JointTypeRevolute,Vector3d(0,1,0)),
			  jz(JointTypeRevolute,Vector3d(0,0,1)),
			  //SpatialVector定义顺序为【rx,ry,rz,x,y,z】，希望的顺序为【x,y,z,rz,ry,rx】
			  floatBase(SpatialVector(0,0,0,1,0,0),
						SpatialVector(0,0,0,0,1,0),
						SpatialVector(0,0,0,0,0,1),
						SpatialVector(0,0,1,0,0,0),
						SpatialVector(0,1,0,0,0,0),
						SpatialVector(1,0,0,0,0,0));
		unsigned int bodyId =model.AddBody(0, Xtrans(Vector3d(0,0,0)), floatBase,body);
		//右腿
		model.AddBody(bodyId, Xtrans(Vector3d(0,-HipY,HipZ)), jz, hipYaw);
		model.AppendBody(Xtrans(Vector3d(0,0,0)), jx, hipRol);
		model.AppendBody(Xtrans(Vector3d(0,0,0)), jy, thigh);
		model.AppendBody(Xtrans(Vector3d(0,0,-LenThigh)), jy, shank);
		model.AppendBody(Xtrans(Vector3d(0,0,-LenShank)), jy, ankle);
		feetId[0]=model.AppendBody(Xtrans(Vector3d(0,0,0)), jx, foot);
		//左腿
		model.AddBody(bodyId, Xtrans(Vector3d(0,HipY,HipZ)), jz, hipYaw);
		model.AppendBody(Xtrans(Vector3d(0,0,0)), jx, hipRol);
		model.AppendBody(Xtrans(Vector3d(0,0,0)), jy, thigh);
		model.AppendBody(Xtrans(Vector3d(0,0,-LenThigh)), jy, shank);
		model.AppendBody(Xtrans(Vector3d(0,0,-LenShank)), jy, ankle);
		feetId[1]=model.AppendBody(Xtrans(Vector3d(0,0,0)), jx, foot);

		//逆解，右、左脚踝位、姿，另外身体6维冗余，在B系中应固定身体位姿为0
		ikSet.AddFullConstraint(feetId[0],{0,0,0},{0,-HipY,-maxL},mat3d::Identity());
		ikSet.AddFullConstraint(feetId[1],{0,0,0},{0, HipY,-maxL},mat3d::Identity());
		ikSet.AddFullConstraint(bodyId,{0,0,0},{0,0,0},mat3d::Identity());

		q.setZero(NGenDof);
		qd.setZero(NGenDof);
		qdLast.setZero(NGenDof);
		qdd.setZero(NGenDof);
		t.setZero(NGenDof);
		tgtQ.setZero(NGenDof);
		tgtQd.setZero(NGenDof);
		tgtQdLast.setZero(NGenDof);
		tgtQdd.setZero(NGenDof);
		tgtT.setZero(NGenDof);
		Jcb6d.setZero(6,NGenDof);

		double kpP=ini["kpP"],kdA=ini["kdA"];
		double kpA=ini["kpA"],kdP=ini["kdP"];
		double kpJ=ini["kpJ"],kdJ=ini["kdJ"];
		legKp<<kpA,kpA,kpA, kpP,kpP,kpP;
		legKd<<kdA,kdA,kdA, kdP,kdP,kdP;
		jntKp.setOnes();
		jntKd.setOnes();
		jntKp*=kpJ;
		jntKd*=kdJ;
		dnmKp.setZero();
		dnmKd.setZero();
		dnmKp.segment<NMot>(NBaseDof).setOnes();
		dnmKd.segment<NMot>(NBaseDof).setOnes();
		dnmKp*=20;
		dnmKd*=2;
	}
	void rbtClass::impClass::update(const double(&jj)[NMot], const double(&ww)[NMot], const double(&tt)[NMot]){
		memcpy(q.data()+NBaseDof,jj,MemMot);
		memcpy(qd.data()+NBaseDof,ww,MemMot);
		qdd=(qd-qdLast)/dt;
		memcpy(qdLast.data()+NBaseDof,ww,MemMot);
		memcpy(t.data()+NBaseDof,tt,MemMot);

		UpdateKinematics(model,q,qd,qdd);
		vec3d tmp{0,0,0};
		ankleP[0]=CalcBodyToBaseCoordinates(model,q,feetId[0],tmp,0);
		ankleP[1]=CalcBodyToBaseCoordinates(model,q,feetId[1],tmp,0);
		ankleR[0]=CalcBodyWorldOrientation(model,q,feetId[0],0).transpose();//转置前是身体相对脚，转置后是脚相对身体
		ankleR[1]=CalcBodyWorldOrientation(model,q,feetId[1],0).transpose();//转置前是身体相对脚，转置后是脚相对身体
		//ankleV[0]=CalcPointVelocity(model,q,qd,feetId[0],tmp,0);
		//ankleV[1]=CalcPointVelocity(model,q,qd,feetId[1],tmp,0);
		ankleV6d[0]=CalcPointVelocity6D(model,q,qd,feetId[0],tmp,0);
		ankleV6d[1]=CalcPointVelocity6D(model,q,qd,feetId[1],tmp,0);

		CalcPointJacobian6D(model,q,feetId[0],tmp,Jcb6d,0);
		JcbAnkle6d[0]=Jcb6d.block(0,NBaseDof,6,NLegDof);
		CalcPointJacobian6D(model,q,feetId[1],tmp,Jcb6d,0);
		JcbAnkle6d[1]=Jcb6d.block(0,NBaseDof+NLegDof,6,NLegDof);
		//fprintEi(Jcb6d);

		// InverseDynamics(model,q,qd,qdd,t);
		ankleF6d[0]=JcbAnkle6d[0].transpose().colPivHouseholderQr().solve(t.segment<NLegDof>(NBaseDof));
		ankleF6d[1]=JcbAnkle6d[1].transpose().colPivHouseholderQr().solve(t.segment<NLegDof>(NBaseDof+NLegDof));
	}
	void rbtClass::impClass::dwdate(){
		auto &tgtP=ikSet.target_positions;
		double tgtL=(tgtP[0]-hip[0]).norm();
		if(tgtL>maxL){
			tgtP[0]=hip[0] +maxL/tgtL*(tgtP[0]-hip[0]);
		}
		tgtL=(tgtP[1]-hip[1]).norm();
		if(tgtL>maxL){
			tgtP[1]=hip[1] +maxL/tgtL*(tgtP[1]-hip[1]);
		}
		InverseKinematics(model,q,ikSet,tgtQ);

		tgtQd.segment<NLegDof>(NBaseDof)=JcbAnkle6d[0].householderQr().solve(tgtAnkV6d[0]);
		tgtQd.segment<NLegDof>(NBaseDof+NLegDof)=JcbAnkle6d[1].householderQr().solve(tgtAnkV6d[1]);
		tgtQdd=(tgtQd-tgtQdLast)/dt*0.8;//前馈加速度
		tgtQdd+=dnmKp.cwiseProduct(tgtQ-q) +dnmKd.cwiseProduct(tgtQd-qd);//反馈闭环加速度

		For(NGenDof){Alg::clip(tgtQdd[i],200);}
		InverseDynamics(model,q,tgtQd,tgtQdd,tgtT);
		tgtQdLast=tgtQd;

		aAxis errA;
		vec6d err6d;
		errA=ikSet.target_orientations[0]*ankleR[0];//rbdl的tgtR是身体相对脚，已经包含所需的转置，但方向是反的，因此下一行tmp需取反
		err6d<<-errA.angle()*errA.axis(), tgtP[0]-ankleP[0];
		tgtAnkF6d[0]+=legKp.cwiseProduct(err6d) +legKd.cwiseProduct(tgtAnkV6d[0]-ankleV6d[0]);

		errA=ikSet.target_orientations[1]*ankleR[1];//rbdl的tgtR是身体相对脚，已经包含所需的转置，但方向是反的，因此下一行tmp需取反
		err6d<<-errA.angle()*errA.axis(), tgtP[1]-ankleP[1];
		tgtAnkF6d[1]+=legKp.cwiseProduct(err6d) +legKd.cwiseProduct(tgtAnkV6d[1]-ankleV6d[1]);

		tgtT.segment<NLegDof>(NBaseDof)+=JcbAnkle6d[0].transpose()*tgtAnkF6d[0];
		tgtT.segment<NLegDof>(NBaseDof+NLegDof)+=JcbAnkle6d[1].transpose()*tgtAnkF6d[1];
		// tgtT.setZero();
		memcpy(omp.tgtT,tgtT.data()+NBaseDof,MemMot);
		tgtT.segment<NMot>(NBaseDof)+=jntKp.cwiseProduct(tgtQ.segment<NMot>(NBaseDof)-q.segment<NMot>(NBaseDof))
									 +jntKd.cwiseProduct(tgtQd.segment<NMot>(NBaseDof)-qd.segment<NMot>(NBaseDof));
	}
//=========================================
	rbtClass& rbtClass::instance(){
		static rbtClass singtn;
		return singtn;
	}
	rbtClass::rbtClass():imp(*new impClass(this)){
		m=ini["mass"];
		inertia<<0.283, 0, 0
				,0, 0.242, 0
				,0, 0, 0.108;
		// inertia.setIdentity();
	}
	void rbtClass::update(const double(&j)[NMot], const double(&w)[NMot], const double(&t)[NMot]){
		imp.update(j,w,t);
	}
	const mat3d& rbtClass::getAnkleR2B(int legId){return imp.ankleR[legId];}
	const vec3d& rbtClass::getAnkleP2B(int legId){return imp.ankleP[legId];}
	const vec6d& rbtClass::getAnkleV6d2B(int legId){return imp.ankleV6d[legId];}
	const vec6d& rbtClass::getAnkleF6d2B(int legId){return imp.ankleF6d[legId];}//F系下足尖受力，反leg.f

	void rbtClass::dwdate(double(&tOut)[NMot]){
		imp.dwdate();
		memcpy(tOut,imp.tgtT.data()+NBaseDof,MemMot);
	}
	void rbtClass::dwdate(double(&jOut)[NMot], double(&wOut)[NMot], double(&tOut)[NMot]){
		imp.dwdate();
		memcpy(jOut,imp.tgtQ.data()+NBaseDof,MemMot);
		memcpy(wOut,imp.tgtQd.data()+NBaseDof,MemMot);
		memcpy(tOut,imp.tgtT.data()+NBaseDof,MemMot);
	}
	void rbtClass::setDt(double dt){
		imp.dt=dt;
	}
	//void rbtClass::setTip2B(vec6d tpP[2]){

	//}
	//void rbtClass::setTip2B(vec6d tpP[2],vec6d tpV[2]){

	//}
	void rbtClass::setTip2B(const mat3d(&tpR)[2],const vec3d(&tpP)[2],const vec6d(&tpV6d)[2],const vec6d(&tpF6d)[2]){
		imp.ikSet.target_orientations[0]=tpR[0].transpose();//rbdl是身体相对脚
		imp.ikSet.target_orientations[1]=tpR[1].transpose();//rbdl是身体相对脚
		imp.ikSet.target_positions[0]=tpP[0];//右踝
		imp.ikSet.target_positions[1]=tpP[1];//左踝
		imp.tgtAnkV6d[0]=tpV6d[0];
		imp.tgtAnkV6d[1]=tpV6d[1];
		imp.tgtAnkF6d[0]=tpF6d[0];
		imp.tgtAnkF6d[1]=tpF6d[1];
	}
}//namespace



//==测试用====
	// Rbt::rbtClass &rbt=Rbt::rbtClass::instance();
	// //double jj[10]{0,0,0.1,-0.2,0,  0,0.1,-0.1,-0.2,0.1};
	// double jj[10]{0,0,0.1,-0.2,0.1,  0,0,0.1,-0.2,0.3};
	// //double jj[10]{0,0,0,0.1,0,  0,0,0,0,0};
	// double ww[10]{0,1,0,0,0,  0,0,0.1,0,0};
	// double tt[10]{};
	// rbt.update(jj,ww,tt);

	// //Plan::walkPlanClass plan(dt);
	// // plan.data.j[]
	// mat3d tpR[2];
	// vec3d tpP[2];
	// vec6d tpV6d[2],tpF6d[2];
	// tpR[0]=rbt.getAnkleR2B(0);
	// tpR[1]=rbt.getAnkleR2B(1);
	// tpP[0]=rbt.getAnkleP2B(0);
	// tpP[1]=rbt.getAnkleP2B(1);
	// tpV6d[0]=rbt.getAnkleV6d2B(0);
	// tpV6d[1]=rbt.getAnkleV6d2B(1);
	// tpF6d[0]=rbt.getAnkleF6d2B(0);
	// tpF6d[1]=rbt.getAnkleF6d2B(1);

	// printEi(tpP[0]);
	// printEi(tpP[1]);
	// printEi(tpV6d[0]);
	// printEi(tpV6d[1]);
	// printEi(tpF6d[0]);
	// printEi(tpF6d[1]);

	// rbt.setTip2B(tpR,tpP,tpV6d,tpF6d);
	// rbt.dwdate(jj,ww,tt);
	// cout<<"逆解：\n";
	// print(jj);
	// print(ww);