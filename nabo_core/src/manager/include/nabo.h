/*=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP, Shanghai Jiao Tong University, under MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

项目最终封装，仅提供c接口
=====================================================*/
#pragma once
#include"nabo_config.h"

namespace Nabo{

extern "C" void setDt(double dt);
extern "C" void run(inputStruct &in,outputStruct &out);

}//namespace
