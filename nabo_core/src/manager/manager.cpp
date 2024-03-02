/*=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP, Shanghai Jiao Tong University, under MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

=====================================================*/
#include"manager.h"
#include"plan.h"
#include"plan_recover.h"
#include"plan_walk.h"

namespace Mng{
class manageClass::impClass{
public:
	int rcCnt;
	Plan::rcPlanClass rc;
	Plan::walkPlanClass wk;
};
//=====================================================
	manageClass::manageClass():imp(*new impClass()){}
	manageClass& manageClass::instance(){
		static manageClass singtn;
		return singtn;
	}
	void manageClass::setDt(double dt){
		imp.rcCnt=int(1/dt);
		imp.rc.setDt(dt);
		imp.wk.setDt(dt);
		cout<<"manager: dt="<<dt<<endl;
	}
	void manageClass::run(Nabo::inputStruct &in,Nabo::outputStruct &out){
		//todo：借plan的继承属性做系统调度
		if(in.cnt<imp.rcCnt){
			imp.rc.run(in,out);
			imp.rc.log();//涉及io，实机log应在实时线程之外，此处暂且不考虑
		}else{
			in.cnt-=imp.rcCnt;
			imp.wk.run(in,out);
			imp.wk.log();//涉及io，实机log应在实时线程之外，此处暂且不考虑
		}
	}
}//namespace
