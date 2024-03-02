/*=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP, Shanghai Jiao Tong University, under MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

步态规划全隔离封装，提供外部接口
#include"manager.h"
	Mng::manageClass &mng=Mng::manageClass::instance();
=====================================================*/
#pragma once
#include"nabo_config.h"

namespace Mng{
class manageClass{
public:
	static manageClass& instance();
	void setDt(double dt);
	void run(Nabo::inputStruct &in,Nabo::outputStruct &out);
private:
	manageClass();
	manageClass(const manageClass&)=delete;
	manageClass & operator=(const manageClass&)=delete;
	class impClass;
	impClass&imp;
};
}//namespace
