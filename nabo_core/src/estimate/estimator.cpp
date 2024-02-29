/*=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP with MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

=====================================================*/
#include"estimator.h"
#include"robot.h"

namespace Est{
class estClass::impClass{
public:
	impClass(estClass *omp);
	Rbt::rbtClass &rbt=Rbt::rbtClass::instance();
	estClass &omp;
	double dt;


	const vec3d g=vec3d(0,0,9.8);
	mat3d I3=mat3d::Identity();

	void run();
};
	estClass::impClass::impClass(estClass *omp):omp(*omp){

	}
	void estClass::impClass::run(){
		auto&imu=rbt.imu;


	}
//=========================================
	estClass& estClass::instance(){
		static estClass singtn;
		return singtn;
	}
	estClass::estClass():imp(*new impClass(this)){}

	void estClass::init(double dt){
		imp.dt=dt;
		setZero();
	}
	void estClass::setZero(){
		p.setZero();
		v2F.setZero();
		slope[0]=0;
		slope[1]=0;
	}
	void estClass::run(){
		imp.run();
	}
}//namespace


