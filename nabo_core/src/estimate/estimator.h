/*=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP, Shanghai Jiao Tong University, under MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

#include"estimator.h"
	Est::estClass &est=Est::estClass::instance();
=====================================================*/
#pragma once
#include"eigen.h"

namespace Est{
class estClass{
public:
	static estClass& instance();
	void init(double dt);
	void setZero();
	void run();
	vec3d p,v,v2F;//p：xy+身高z
	double slope[2];//F系rol、pit
private:
	estClass();
	estClass(const estClass&)=delete;
	estClass& operator=(const estClass&)=delete;

	class impClass;
	impClass &imp;
};
}//namespace
