/*=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP, Shanghai Jiao Tong University, under MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

=====================================================*/
#include"plan_walk.h"

namespace Plan{
	walkPlanClass::walkPlanClass(){
		logFlag=ini["wkLogFlag"];
		consoleFlag=ini["wkConsoleFlag"];
		pRate=ini["pRate"];
		vRate=ini["vRate"];
		cbicFlag=ini["cbicFlag"];
		cmdFocFlag=ini["cmdFocFlag"];
	}
	bool walkPlanClass::run(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef){
		in=inRef;
		if(in.cnt==0){
			baseInit();
			update();//确实多一次
			init();
		}
		update();
		plan();
		balance();
		dwdate(outRef);
		return quitFlag;
	}
	void walkPlanClass::init(){
		cmdTipP[0]=rbt.getAnkleP2B(0);
		cmdTipP[1]=rbt.getAnkleP2B(1);
		cmdTipR[0]=rbt.getAnkleR2B(0);
		cmdTipR[1]=rbt.getAnkleR2B(1);
		cmdTipV6d[0]=rbt.getAnkleV6d2B(0);
		cmdTipV6d[1]=rbt.getAnkleV6d2B(1);
		cmdTipF6d[0]<<0,0,0, 0,0,-80;
		cmdTipF6d[1]<<0,0,0, 0,0,-80;
		stdTipP[0]<<0,-HipY,-BodyH;
		stdTipP[1]<<0, HipY,-BodyH;
		swTrj6d[0].setRP0(cmdTipR[0],cmdTipP[0]);
		swTrj6d[0].setRP1(cmdTipR[0],cmdTipP[0]);
		swTrj6d[0].update(0);
		swTrj6d[1].setRP0(cmdTipR[1],cmdTipP[1]);
		swTrj6d[1].setRP1(cmdTipR[1],cmdTipP[1]);
		swTrj6d[1].update(0);

		des.init();
		tgt.init();
		lmt.init(dt);

		double duty[2]{0.5,0.5};
		double offset[2]{0.3,0.8};
		cpg.init(dt,ini["period"],duty,offset);
		if(ini["cpg"]){
			cpg.unlock();
			cout<<"cpg start\n";
		}
		est.init(dt);
		cbic.init(dt);
		cbic.setCiParam(1, 0.1, 0.08, 0.03);
		com.setZero();
		mpc.setCiParam(1, 0.1, 0.08, 0.03);
		mpc.start();
		mpc.wake();
		cout<<"walk init\n";
	}
	void walkPlanClass::update(){
		tim=in.cnt*dt;
		rbt.imu.update(in.rpy, in.rpy[2], in.gyr, in.acc);
		// For(NMot){
		// 	in.w[i]=wFil[i].filt(in.w[i]);
		// }
		rbt.update(in.j, in.w, in.t);
		cpg.update();
		// est.run();
		est.p<<in.supP[0], in.supP[1], in.supP[2]-BodyH;
		est.v<<in.supV[0], in.supV[1], in.supV[2];
		est.v2F=rbt.imu.Rz.transpose()*est.v;

		Alg::clip(in.cmdVx, lmt.vx);
		Alg::clip(in.cmdVy, lmt.vy);
		Alg::clip(in.cmdWz, lmt.wz);
		des.dVx=(in.cmdVx-des.v2S[0])*dt;
		des.dVy=(in.cmdVy-des.v2S[1])*dt;
		des.dWz=(in.cmdWz-des.wz)*dt;
		Alg::clip(des.dVx, lmt.dVx);
		Alg::clip(des.dVy, lmt.dVy);
		Alg::clip(des.dWz, lmt.dWz);

		Alg::cmd2out1step(in.cmdVx, des.v2S[0], des.dVx);
		Alg::cmd2out1step(in.cmdVy, des.v2S[1], des.dVy);
		Alg::cmd2out1step(in.cmdWz, des.wz, des.dWz);
		des.RxyS=aAxisY(est.slope[1])*aAxisX(est.slope[0]);//选落点，仅地形
		des.RxyA2S=aAxisY(des.rpy[1])*aAxisX(des.rpy[0]);//选落点，仅地形
		des.rpy[2]+=des.wz*dt;
		des.Rz=aAxisZ(des.rpy[2]);
		des.RS=des.Rz*des.RxyS;
		des.p+=des.RS*des.v2S*dt;

		tgt.Rxy=des.RxyA2S*des.RxyS;//主动(pit、rol)+地形
		tgt.R=des.Rz*tgt.Rxy;
		tgt.v2F=des.RxyS*des.v2S;
		tgt.v=des.RS*des.v2S;
		tgt.w<<0,0,des.wz;
		tgt.p=des.p;
		if(!cpg.isStop()){//zmp近似调整
			vec3d adj=des.Rz.col(1);
			tgt.p-=0.01*cos(cpg.getPgs()*Pi2)*adj;
		}
	}
	void walkPlanClass::plan(){
		auto &imu=rbt.imu;
		double stT=cpg.getStT(0);
		For(2){
			double swPgs2=cpg.getSwPgs(i);
			if(swPgs2>0){
				if(!rbt.isSwing(i)){//每迈步初始
					swTrj6d[i].setRP0now();
					swTrj6d[i].setH(0.05);
					rbt.setSwing(i,1);
				}
				mat3d Rdz(aAxisZ(tgt.w[2]*(0.5*stT+cpg.getSwTicRm(i))));
				mat3d R1(Rdz*des.RS);

				vec3d pBody(pRate*est.p +(1-pRate)*tgt.p);
				pBody[2]=tgt.p[2];
				vec3d vBody(vRate*est.v +(1-vRate)*tgt.v);
				vBody[2]=tgt.v[2];
				vec3d hip(0,stdTipP[i][1],0), leg(0,0,stdTipP[i][2]-tgt.zOff);
				vec3d p1=pBody +Rdz*(vBody*(cpg.getSwTicRm(i)+0.5*stT) +des.Rz*(tgt.Rxy*hip+leg));
				swTrj6d[i].setRP1(R1,p1,swPgs2);
				swTrj6d[i].update(swPgs2);

				swTrj6d[i].getWV6d(cmdTipV6d[i]);//sw和st的v不同，所以单独拿出来
				cmdTipV6d[i]/=cpg.getSwT(i);
			}else{//==st==
				if(rbt.isSwing(i)){
					rbt.setSwing(i,0);
				}
				cmdTipV6d[i].setZero();
			}
			swTrj6d[i].getRP(cmdTipR[i],cmdTipP[i]);
			cmdTipR[i]=tgt.R.transpose()*cmdTipR[i];
			cmdTipP[i]=tgt.R.transpose()*(cmdTipP[i]-tgt.p);//实机imu矫正会发散
			cmdTipV6d[i].head<3>()=tgt.R.transpose()*(cmdTipV6d[i].head<3>()-tgt.w);
			cmdTipV6d[i].tail<3>()=tgt.R.transpose()*(cmdTipV6d[i].tail<3>()-tgt.v -tgt.w.cross(tgt.R*rbt.getAnkleP2B(i)));
		}
	}
	void walkPlanClass::balance(){
		auto&imu=rbt.imu;
		mat3d RzT=imu.Rz.transpose();
		aAxis Tmp(tgt.R*imu.R.transpose());//W系
		vec3d errA(Tmp.angle()*Tmp.axis());
		errA=RzT*errA;//转到F系
		vec3d errP{RzT*(tgt.p-est.p)};
		vec3d errW(-imu.w);
		errW[2]+=tgt.w[2];
		errW=imu.Rxy*errW;
		vec3d errV(tgt.v2F-est.v2F);

		com.pid(errA,errP,errW,errV);
		com.acc[0]-=4;
		com.filt();

		stt.aDlt=-errA;
		stt.pDlt=-errP;
		stt.w=imu.Rxy*imu.w;
		stt.v=est.v2F;
		stt.wz=tgt.w[2];
		stt.vx=tgt.v2F[0];
		stt.vy=tgt.v2F[1];
		stt.leg0=cmdTipP[0];
		stt.leg1=cmdTipP[1];

		mpc.setState(stt);
		mpc.getMF(cmdTipF6d[0],cmdTipF6d[1]);
		if(cbicFlag){
			cbic.setFr(cmdTipF6d);
			cbic.run(com.aAcc, com.acc);
			cmdTipF6d[0]=cbic.mf[0];
			cmdTipF6d[1]=cbic.mf[1];
		}

		mat3d RxyT=tgt.Rxy.transpose();
		cmdTipF6d[0].head<3>()=RxyT*cmdTipF6d[0].head<3>();
		cmdTipF6d[0].tail<3>()=RxyT*cmdTipF6d[0].tail<3>();
		cmdTipF6d[1].head<3>()=RxyT*cmdTipF6d[1].head<3>();
		cmdTipF6d[1].tail<3>()=RxyT*cmdTipF6d[1].tail<3>();
	}
	void walkPlanClass::dwdate(Nabo::outputStruct &outRef){
		if(!cmdFocFlag){
			cmdTipF6d[0].setZero();
			cmdTipF6d[1].setZero();
		}
		rbt.setTip2B(cmdTipR,cmdTipP,cmdTipV6d,cmdTipF6d);
		rbt.dwdate(out.j, out.w, out.t);
		For(NMot){
			out.t[i]=tOutFil[i].filt(out.t[i]);
			Alg::clip(out.t[i],MaxMotToq);
		}
		outRef=out;
	}
	void walkPlanClass::log(){
		if(logFlag && in.cnt%logCnt==0){
			vec3d cmdTipA[2]{Ei::R2rpy(cmdTipR[0]),Ei::R2rpy(cmdTipR[1])};
			vec3d actTipP[2]{rbt.getAnkleP2B(0),rbt.getAnkleP2B(1)};
			vec3d actTipA[2]{Ei::R2rpy(rbt.getAnkleR2B(0)),Ei::R2rpy(rbt.getAnkleR2B(1))};
			fout<<"wk\t"<<tim<<"\t";

			fout<<"cmdA\t";
			For3{fout<<cmdTipA[0][i]<<"\t";}
			For3{fout<<cmdTipA[1][i]<<"\t";}
			fout<<"actA\t";
			For3{fout<<actTipA[0][i]<<"\t";}
			For3{fout<<actTipA[1][i]<<"\t";}
			fout<<"cmdP\t";
			For3{fout<<cmdTipP[0][i]<<"\t";}
			For3{fout<<cmdTipP[1][i]<<"\t";}
			fout<<"actP\t";
			For3{fout<<actTipP[0][i]<<"\t";}
			For3{fout<<actTipP[1][i]<<"\t";}

			// fout<<"cmdV6d\t";
			// For6{fout<<cmdTipV6d[0][i]<<"\t";}
			// For6{fout<<cmdTipV6d[1][i]<<"\t";}
			fout<<"cmdF6d\t";
			For6{fout<<cmdTipF6d[0][i]<<"\t";}
			For6{fout<<cmdTipF6d[1][i]<<"\t";}
			fout<<"actF6d\t";
			For6{fout<<rbt.getAnkleF6d2B(0)[i]<<"\t";}
			For6{fout<<rbt.getAnkleF6d2B(1)[i]<<"\t";}

			fout<<"swPgs\t"<<cpg.getSwPgs(0)<<"\t"<<cpg.getSwPgs(1)<<"\t";

			auto &imu=rbt.imu;
			fout<<"rpy\t";
			For3{fout<<imu.rpy[i]<<"\t";}
			fout<<imu.yawAct<<"\t";
			fout<<"w\t";
			For3{fout<<imu.w[i]<<"\t";}
			fout<<"est\t";
			For3{fout<<est.p[i]<<"\t";}
			For3{fout<<est.v2F[i]<<"\t";}

			// fout<<"mpc-stt\t";
			// For(3){fout<<stt.aDlt[i]<<"\t";}
			// For(3){fout<<stt.pDlt[i]<<"\t";}
			// For(3){fout<<stt.w[i]<<"\t";}
			// For(3){fout<<stt.v[i]<<"\t";}

			fout<<"cbcAcc\t";
			For3{fout<<com.aAcc[i]<<"\t";}
			For3{fout<<com.acc[i]<<"\t";}

			// fout<<"cmdJ\t";
			// For(12){fout<<out.j[i]<<"\t";}
			// fout<<"actJ\t";
			// For(12){fout<<in.j[i]<<"\t";}
			// fout<<"cmdW\t";
			// For(12){fout<<out.w[i]<<"\t";}
			// fout<<"actW\t";
			// For(12){fout<<in.w[i]<<"\t";}
			fout<<"cmdT\t";
			For(12){fout<<out.t[i]<<"\t";}
			// fout<<"tmpT\t";
			// For(12){fout<<rbt.tgtT[i]<<"\t";}
			// fout<<"actT\t";
			// For(12){fout<<in.t[i]<<"\t";}
			fout<<endl;
		}
		if(consoleFlag && in.cnt%consoleCnt==0){
			cout<<"------\ncnt="<<in.cnt<<", time="<<tim<<endl;
			cout<<"cmd: vx="<<in.cmdVx<<", vy="<<in.cmdVy<<", wz="<<in.cmdWz<<endl;
			// print(in.supP[0],in.supP[1],in.supP[2]
			// 	,in.supV[0],in.supV[1],in.supV[2],'\n'
			// );
			// print(out.t);
			//print(in.j);
			//printEi(cmdTipP[0]);
			// printEi(cmdTipF6d[0]);
			// printEi(cmdTipF6d[1]);
			//print(out.j);
			//print(out.w[0], out.w[1], out.w[2], out.w[3], out.w[4]);
		}
	}
}//namespace

