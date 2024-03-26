/*=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP, Shanghai Jiao Tong University, under MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

常量定义，首字母大写
#include"nabo_nabo_config.h"
=====================================================*/
#pragma once
//涉及数组大小，必须编译前给定
static const int NLegDof       =6;
static const int NMot          =NLegDof*2;
static const int MemMot        =NMot*8;
static const int NBaseDof      =6;
static const int NGenDof       =NBaseDof +NMot;
//其他常量
static const double HipY       =0.1;
static const double HipZ       =-0.3;
static const double LenThigh   =0.2;
static const double LenShank   =0.2;
static const double BodyH      =0.65;
static const double MaxMotToq  =50;


namespace Nabo{

struct inputStruct{
	int cnt;
	double cmdVx,cmdVy,cmdWz;
	double supP[3],supV[3];
	double rpy[3],gyr[3],acc[3];
	double j[NMot],w[NMot],t[NMot];
};
struct outputStruct{
	double j[NMot],w[NMot],t[NMot];
};

}//namespace