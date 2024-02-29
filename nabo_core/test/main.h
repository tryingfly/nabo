/*=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP with MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

=====================================================*/
#include<chrono>
#include<thread>
#include"iopack.h"
#pragma once


void setup();
void tictoc();

int main(int argc, char* argv[]){
	#ifdef _WIN32
	system("CHCP 65001");//改变命令行为utf8编码，会被360报毒
	#endif

	cout<<endl;
	setup();
	auto clockTag=chrono::high_resolution_clock::now();
	tictoc();
	auto clockPeriod=chrono::duration_cast<chrono::nanoseconds>(chrono::high_resolution_clock::now() -clockTag);
	double clockPeriodMs=clockPeriod.count()/1000000.0;
	cout<<"\n------\ntime cost: "<<clockPeriodMs<<" ms"<<endl;
	fout.close();
	return 0;
}
