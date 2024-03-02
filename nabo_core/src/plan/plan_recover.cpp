/*=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP, Shanghai Jiao Tong University, under MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

=====================================================*/
#include"iopack.h"
#include"plan_recover.h"

namespace Plan{
	rcPlanClass::rcPlanClass(){
		logFlag=ini["rcLogFlag"];
		consoleFlag=ini["rcConsoleFlag"];
		kp=ini["kp"];
		ki=ini["ki"];
		kd=ini["kd"];
	}
	bool rcPlanClass::run(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef){
		in=inRef;
		if(in.cnt==0){
			baseInit();
			init();
		}
		plan();
		dwdate(outRef);
		return quitFlag;
	}
	void rcPlanClass::init(){
		memcpy(j0.data(),in.j,MemMot);
		j1={0,0,-0.5,1,-0.5,0,  0,0,-0.5,1,-0.5,0};
		j2={0,0,-0.2,0.4,-0.2,0,  0,0,-0.2,0.4,-0.2,0};
		memset(errSum,0,MemMot);
		tSit=1;tWait=1;tStand=1;
		cout<<"rc init\n";
	}
	void rcPlanClass::plan(){
		tim=in.cnt*dt;
		// For(NMot){
		// 	in.w[i]=wFil[i].filt(in.w[i]);
		// }
		if(tim<tSit){
			double pgs=tim/tSit;
			double s=0.5-cos(pgs*Pi)*0.5;
			double sd=Pi*0.5*sin(pgs*Pi)/tSit;
			For(NMot){
				out.j[i]=j0[i]+s*(j1[i]-j0[i]);
				out.w[i]=sd*(j1[i]-j0[i]);
			}
		// }else if(tim<tSit+tWait){
		// 	For(NMot){
		// 		out.j[i]=j1[i];
		// 		out.w[i]=0;
		// 	}
		// }else if(tim<tSit+tWait+tStand){
		// 	double pgs=(tim-tSit-tWait)/tStand;
		// 	double s=0.5-cos(pgs*Pi)*0.5;
		// 	double sd=Pi*0.5*sin(pgs*Pi)/tStand;
		// 	For(NMot){
		// 		out.j[i]=j1[i]+s*(j2[i]-j1[i]);
		// 		out.w[i]=sd*(j2[i]-j1[i]);
		// 	}
		}else{quitFlag=1;}
	}
	void rcPlanClass::dwdate(Nabo::outputStruct &outRef){
		For(NMot){
			double err{out.j[i]-in.j[i]};
			errSum[i]+=err;
			errSum[i]*=0.99;
			out.t[i]=kp*err +ki*errSum[i] +kd*(out.w[i]-in.w[i]);
			out.t[i]=tOutFil[i].filt(out.t[i]);
			Alg::clip(out.t[i],MaxMotToq);
		}
		outRef=out;
	}
	void rcPlanClass::log(){
		if(logFlag && in.cnt%logCnt==0){
			fout<<"rc\t"<<tim<<"\t";
			fout<<"tgtJ\t";
			For(12){fout<<out.j[i]<<"\t";}
			fout<<"actJ\t";
			For(12){fout<<in.j[i]<<"\t";}

			fout<<"tgtW\t";
			For(12){fout<<out.w[i]<<"\t";}
			fout<<"actW\t";
			For(12){fout<<in.w[i]<<"\t";}

			fout<<"tgtT\t";
			For(12){fout<<out.t[i]<<"\t";}
			fout<<"actT\t";
			For(12){fout<<in.t[i]<<"\t";}
			fout<<endl;
		}

		if(consoleFlag && in.cnt%consoleCnt==0){
			cout<<"rc: tim="<<tim<<endl;
		}
	}
}//namespace

