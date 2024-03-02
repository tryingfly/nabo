/*=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP, Shanghai Jiao Tong University, under MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

读取解析配置文件*.ini
不要在实时循环使用，可以在进实时循环前先找变量赋值
不能在全局变量使用
可以在类构造函数中使用(main之前执行)，但该类不能实例化全局变量
#include"ini.h"
=====================================================*/
#pragma once
#include<string>
#include<fstream>

using namespace std;

namespace Ini{
//不要在实时循环使用
class iniClass{
public:
	iniClass();//不要在实时循环使用
	iniClass(string fileName);//不要在实时循环使用
	double operator[](string key);//不要在实时循环使用
	string getStr(string key);//不要在实时循环使用
	template<typename T,int n>
	void getArray(string key,T (&value)[n]){getArray(key,value,n);};//不要在实时循环使用
	template<typename T>
	void getArray(string key,T *value,int n);//不要在实时循环使用
private:
	class impClass;
	impClass&imp;
};
}//namespace

extern ofstream fout;//放在这里原因：匹配“000.ini”目录
extern Ini::iniClass ini;//已实例化，默认打开“000.ini”
