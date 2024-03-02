/*=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP, Shanghai Jiao Tong University, under MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

=====================================================*/
#include"ini.h"
#include<map>
#include<vector>
#include<iostream>
#include<sstream>

ofstream fout;//放在这里原因：在impClass构造函数中匹配“000.ini”目录

namespace Ini{
	inline void splitString(string &str, vector<string> &substrings, char token){
		string tmp;
		stringstream ss(str);
		while(getline(ss, tmp, token)){
			substrings.push_back(tmp);
		}
	}
//==========================
class iniClass::impClass{
public:
	impClass();
	impClass(string fileName);
	ifstream f;
	map<string,string>mp;

	void init();
	double operator[](string key);
	string getStr(string key);
	template<typename T,int n>
	void getArray(string key,T (&value)[n]){getArray(key,value,n);};
	template<typename T>
	void getArray(string key,T *value,int n);
};
	iniClass::impClass::impClass(){
		#ifdef _WIN32
		f.open("../../000.ini");
		if(f.is_open()){fout.open("../../zzz.txt", ios::trunc);}
		#else
		f.open("../000.ini");
		if(f.is_open()){fout.open("../zzz.txt", ios::trunc);}
		#endif
		if(!f.is_open()){
			f.open("000.ini");
			fout.open("zzz.txt", ios::trunc);
		}
		init();
	}
	iniClass::impClass::impClass(string fileName){
		f.open(fileName);
		init();
	}
	void iniClass::impClass::init(){
		string line;
		while(getline(f,line)){
			size_t id=0;
			//去除空格
			while((id=line.find(' ',id)) != string::npos){
				line.erase(id,1);
			}
			//去除tab
			id=0;
			while((id=line.find('\t',id)) != string::npos){
				line.erase(id,1);
			}
			//分号是ini的注释标志
			id=line.find(";");
			if(id != string::npos){
				line=line.substr(0,id);
			}
			id=line.find("=");
			if(id != string::npos){
				string key,value;
				key.assign(line,0,id);
				value.assign(line,id+1,line.size()-id);
				mp.insert(pair<string,string>(key,value));
			}
		}
	}
	double iniClass::impClass::operator[](string key){
		auto it=mp.find(key);
		if(it!=mp.end()){
			try{
				return stod(it->second);
			}catch(...){
				cout<<"\" ini[\""<<key<<"\"] \" 值类型不匹配"<<endl;
				throw runtime_error("\" ini[\""+key+"\"] \" 值类型不匹配");
			}
		}else{
			cout<<"\" ini[\""<<key<<"\"] \" 找不到key"<<endl;
			throw runtime_error("\" ini[\""+key+"\"] \" 找不到key");
		}
	};
	string iniClass::impClass::getStr(string key){
		auto it=mp.find(key);
		if(it!=mp.end()){
			return it->second;
		}else{
			cout<<"\" ini[\""<<key<<"\"] \" 找不到key"<<endl;
			throw runtime_error("\" ini[\""+key+"\"] \" 找不到key");
		}
	};
	template<typename T>
	void iniClass::impClass::getArray(string key,T *value,int n){
		auto it=mp.find(key);
		if(it!=mp.end()){
			std::vector<std::string> vs;
			splitString(it->second,vs,',');
			if(vs.size()<n){
				cout<<"ini.getArray数目不匹配，key="+key<<endl;
				throw runtime_error("ini.getArray数目不匹配，key="+key);
			}
			try{
				for(int i=0;i<n;i++){
					value[i]=(T)stod(vs[i]);
				}
			}catch(...){
				cout<<"ini.getArray值类型不匹配，key="+key<<endl;
				throw runtime_error("ini.getArray值类型不匹配，key="+key);
			}
		}else{
			cout<<"ini.getArray找不到key="+key<<endl;
			throw runtime_error("ini.getArray找不到key="+key);
		}
	}
//==========================
	iniClass::iniClass():imp(*new impClass()){}
	iniClass::iniClass(string fileName):imp(*new impClass(fileName)){}
	double iniClass::operator[](string key){
		return imp[key];
	};
	string iniClass::getStr(string key){
		return imp.getStr(key);
	};
	template<typename T>
	void iniClass::getArray(string key,T *value,int n){
		imp.getArray(key,value,n);
	}
	template void iniClass::getArray(string key,int *value,int n);
	template void iniClass::getArray(string key,float *value,int n);
	template void iniClass::getArray(string key,double *value,int n);
}//namespace

Ini::iniClass ini;//默认打开“000.ini”