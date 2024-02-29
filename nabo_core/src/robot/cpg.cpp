/*=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP with MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

采用时钟计时生成步态周期
=====================================================*/
#include<cstring>
#include"cpg.h"
#include"algorithms.h"

#define For2 for(int i(0);i!=2;++i)
namespace Cpg{
	cpgClass& cpgClass::instance(){
		static cpgClass singtn;
		return singtn;
	};
	cpgClass::cpgClass(){

	}
	void cpgClass::init(const double dt,const double period, const double(&duty)[Num], const double(&offset)[Num]){
		this->dt=dt;
		this->period=period;
		cmd.period=period;
		memcpy(cmd.offset, offset, 8*Num);
		memcpy(cmd.duty, duty, 8*Num);
		For2{
			Alg::clip(cmd.duty[i],0.00001,0.99999);
			Alg::clip(cmd.offset[i],0,1);
			stT[i]=period*cmd.duty[i];
			swT[i]=period-stT[i];
		}
		memcpy(this->offset, cmd.offset, 8*Num);
		memcpy(this->duty, cmd.duty, 8*Num);
		setZero();
	}
	void cpgClass::setZero(){
		locking=1;
		shouldStart=0;
		pgs=pgsEarly;//在unlock后等一段时间再动，负数
		memset(isFirstStep,1,2);//在isAllSw(fT)内判断第一步
		memset(swTic,0,8*Num);
		memset(swPgs,0,8*Num);
		memset(isSw,0,2);//bool
		For2{
			stPgs[i]=1;
			stTic[i]=stT[i];
		}
	}
	void cpgClass::setParam(const double period,const double(&duty)[Num],const double(&offset)[Num]){
		cmd.period=period;
		memcpy(cmd.offset, offset, 8*Num);
		memcpy(cmd.duty, duty, 8*Num);
		Alg::clip(cmd.period,0.1,10);//保证安全
		For2{
			Alg::clip(cmd.offset[i],0,1);
			Alg::clip(cmd.duty[i],0.00001,0.99999);
		}
	}

	bool cpgClass::isAllSw(double fT){
		bool re=!isStop();
		double fp(fT/period);
		For2{
			double tmp(pgs+fp-offset[i]+1);//+1省去判断<0
			tmp-=int(tmp);
			re &= (tmp<(1-duty[i]));
			re &= !(isFirstStep[i]&(pgs+fp<offset[i]));
		}
		return re;
	}
	void cpgClass::isFutureSt(double fT,bool*isSt){
		double fp(fT/period);
		For2{
			isSt[i]=locking&(!isSw[i]);
			double tmp(pgs+fp-offset[i]+1);//+1省去判断<0
			tmp-=int(tmp);
			isSt[i] |= !(tmp<1-duty[i]);
			isSt[i] |= isFirstStep[i]&(pgs+fp<offset[i]);
			// cout<<"cpg:"<<fT<<", "<<tmp<<", "<<isSt[i]<<endl;
		}
	}
	void cpgClass::isFutureSt(double fT,int*stage){
		double fp(fT/period);
		For2{
			double tmp(pgs-offset[i]+2);//多+2保证为正
			tmp+=-int(tmp)+fp;
			if(isSw[i]){
				if(tmp>1-duty[i] && tmp<1){
					stage[i]=2;
				}else{stage[i]=0;}
			}else if(locking){
				stage[i]=1;
			}else{
				if(tmp<1){//好处是不用考虑第一步
					stage[i]=1;
				}else if(tmp<2-duty[i]){
					stage[i]=0;
				}else{
					stage[i]=2;
				}
			}
		}
	}
	double cpgClass::getNextEvent(double fT){
		double fp(fT/period),cmp=1;
		For2{
			double tmp(offset[i]-pgs-fp+100);//+100保证为正
			tmp-=int(tmp);
			if(tmp<cmp){cmp=tmp;}
			tmp+=100-duty[i];//+100保证为正
			tmp-=int(tmp);
			if(tmp<cmp){cmp=tmp;}
		}
		return cmp*period;
	}
	void cpgClass::update(){
		isSwAny=0;
		For2{
			isSwAny|=isSw[i];
		}
		if(!isSwAny && shouldStart){
			shouldStart=0;
			locking=0;
		}
		//==步态参数线性变化==
		Alg::cmd2out1step(cmd.period,period,0.0015);
		For2{
			Alg::cmd2out1step(cmd.offset[i],offset[i],0.0016);
			Alg::cmd2out1step(cmd.duty[i],duty[i],0.0008);
			stT[i]=period*duty[i];
			swT[i]=period-stT[i];
		}
		//=======
		pgs+=dt/period;
		pgs-=int(pgs);
		For2{
			if(isSw[i]){
				if(calSw(i)){
					calSt(i);
				}
			}else if(locking){
				stTic[i]=stT[i];
				stPgs[i]=1;
				swPgs2[i]=0;
				swTic2[i]=0;
				if(!isSwAny){pgs=pgsEarly;}//在unlock后等一段时间再动，负数
				isFirstStep[i]=1;//在isAllSw(fT)内判断第一步
			}else{//st
				if(pgs<0){//在unlock后等一段时间再动
					continue;
				}
				if(calSt(i)){
					calSw(i);
				}
			}
		}
	}
	bool cpgClass::calSw(int i){
		isFirstStep[i]=0;
		swPgs[i]=pgs-offset[i];
		if(swPgs[i]<0){swPgs[i]+=1;}
		swPgs[i]/=1-duty[i];
		swTic[i]=swPgs[i]*swT[i];

		swTic2[i]=swTic[i];//在sw时二者相等，避免同时清零，需放在sw>=1之前
		swPgs2[i]=swPgs[i];
		if(swPgs[i]>1 || duty[i]>0.99){
			swPgs[i]=0;
			swTic[i]=0;
			isSw[i]=0;
			return 1;
		}
		return 0;
	}
	bool cpgClass::calSt(int i){
		stPgs[i]=pgs-offset[i]+duty[i]-1;
		if(stPgs[i]<0){stPgs[i]+=1;}
		//让第一步不飞
		if(stPgs[i]<0){stPgs[i]=0;}
		stPgs[i]/=duty[i];
		stTic[i]=stPgs[i]*stT[i];
		swPgs2[i]=pgs-offset[i];
		if(isFirstStep[i]){swPgs2[i]=0;}
		if(swPgs2[i]<0){swPgs2[i]+=1;}

		swPgs2[i]/=1-duty[i];
		swTic2[i]=swPgs2[i]*swT[i];//在st时swTic2继续增长，在st>=1需要清零

		if(stPgs[i]>=1){
			stPgs[i]=0;
			stTic[i]=0;
			isSw[i]=1;
			return 1;
		}
		return 0;
	}

	void cpgClass::unlock(){
		if(locking){
			if(isSwAny){
				shouldStart=1;
			}else{
				locking=0;
			}
		}
	};
}//namespace



//==cpg测试===========
	// Cpg::cpgClass& cpg=Cpg::cpgClass::instance();
	// double period=1,dt=0.01;
	// double duty[Num]={0.6,0.6};
	// double offset[Num]={0, 0.5};
	// cpg.init(dt,period,duty,offset);

	// double pgs,swPgs[Num],stPgs[Num],swPgs2[Num],swTic[Num],stTic[Num],swTic2[Num],swTicRm[Num],stTicRm[Num];
	// double t=0;
	// cpg.unlock();

	// // int st[Num];
	// // For(10){
	// // 	cpg.isFutureSt(0.1*i,st);
	// // 	cout<<st[0]<<","<<st[1]<<","<<st[2]<<","<<st[3]<<","<<st[4]<<","<<st[5]<<endl;
	// // }

	// for(t=0;t<3;t+=dt){
	// 	// if(t>0.5){cpg.unlock();}
	// 	cpg.update();
	// 	pgs=cpg.getPgs();
	// 	for(int i=0;i<Num;i++){
	// 		swPgs[i]=cpg.getSwPgs(i);
	// 		stPgs[i]=cpg.getStPgs(i);
	// 		swPgs2[i]=cpg.getSwPgs2(i);
	// 		swTic[i]=cpg.getSwTic(i);
	// 		stTic[i]=cpg.getStTic(i);
	// 		swTic2[i]=cpg.getSwTic2(i);
	// 		swTicRm[i]=cpg.getSwTicRm(i);
	// 		stTicRm[i]=cpg.getStTicRm(i);
	// 	}
	// 	fout<<t<<","<<pgs<<",swPgs,";
	// 	for(int i=0;i<Num;i++){
	// 		fout<<swPgs[i]<<",";
	// 	}
	// 	fout<<"stPgs,";
	// 	for(int i=0;i<Num;i++){
	// 		fout<<stPgs[i]<<",";
	// 	}
	// 	fout<<"swPgs2,";
	// 	for(int i=0;i<Num;i++){
	// 		fout<<swPgs2[i]<<",";
	// 	}
	// 	fout<<"swTic,";
	// 	for(int i=0;i<Num;i++){
	// 		fout<<swTic[i]<<",";
	// 	}
	// 	fout<<"stTic,";
	// 	for(int i=0;i<Num;i++){
	// 		fout<<stTic[i]<<",";
	// 	}
	// 	fout<<"swTic2,";
	// 	for(int i=0;i<Num;i++){
	// 		fout<<swTic2[i]<<",";
	// 	}
	// 	fout<<"swTicRm,";
	// 	for(int i=0;i<Num;i++){
	// 		fout<<swTicRm[i]<<",";
	// 	}
	// 	fout<<"stTicRm,";
	// 	for(int i=0;i<Num;i++){
	// 		fout<<stTicRm[i]<<",";
	// 	}
	// 	fout<<endl;
	// }
