#pragma once
#include "stdafx.h"
#include"deadlock.h"
using namespace std;

vector<AGVtd> AGVList;//创建AGV列表
vector<Map>topmap;//建立地图
AGVtrans getAGVtrans();//测试用
void getmap() {//测试用，获得地图
	path pathtemp;
	Json::CharReaderBuilder builder;
	Json::Value root;//定义根节点
	Json::CharReader* reader(builder.newCharReader());
	ifstream ifs(TOPMAP);
	builder["collectComments"] = false;
	JSONCPP_STRING errs;
	if (!ifs.is_open()) {
		cout << "文件打开失败";
	}
	if (parseFromStream(builder, ifs, &root, &errs)) {//开始读文件
		int size = root.size();
		for (int i = 0; i < size; i++)
		{
			Map temp;
			temp.pointID = root[i]["pointID"].asString();//保存节点名
			temp.pos_x = root[i]["pos_x"].asDouble();
			temp.pos_y = root[i]["pos_y"].asDouble();
			temp.block = root[i]["block"].asBool();
			for (int size2 = root[i]["linkpoint"].size(), j = 0; j < size2; j++) {
				pathtemp.endPoint = root[i]["linkpoint"][j]["linkPointID"].asString();
				pathtemp.lenth = root[i]["linkpoint"][j]["lenth"].asDouble();
				pathtemp.factor = root[i]["linkpoint"][j]["factor"].asDouble();
				temp.linkPoint.push_back(pathtemp);
			}
			topmap.push_back(temp);
		}
	}
	ifs.close();
	//测试用
	cout << "地图初始化完成\n";
	for (vector<Map>::iterator point = topmap.begin(); point != topmap.end(); point++) {
		cout << point->pointID << "->";
		for (vector<path>::iterator freepath = point->linkPoint.begin(); freepath != point->linkPoint.end(); freepath++) {
			cout << freepath->endPoint << ", ";
		}
		cout << "\n";
	}
}

int main() {
	getmap();//加载地图
	while (1) {//保持运行
		AGVtrans AGVMessage = getAGVtrans();//收到AGV信息后，开始
		AGVtd AGVinfo;
		AGVinfo.AGVID = AGVMessage.AGVID;
		AGVinfo.lastPoint = AGVMessage.lastPoint;
		AGVinfo.times = 0;
		Json::CharReaderBuilder builder;
		Json::Value root;//定义根节点
		Json::CharReader* reader(builder.newCharReader());
		ifstream ifs(AGVPATH);
		builder["collectComments"] = false;
		JSONCPP_STRING errs;
		bool deadlockflag = false;//死锁标记
		if (parseFromStream(builder, ifs, &root, &errs)) {
			root = root[AGVinfo.AGVID];
			int size = root.size();
			for (int i = 0; i < size; i++) {
				if (root[i] == AGVinfo.lastPoint) {
					if (i + 2 < size) {
						AGVinfo.nextPoint = root[i + 1].asString();
						AGVinfo.nnPoint = root[i + 2].asString();
					}
					else {
						deadlockflag = true;
					}
					break;
				}
			}
		}
		ifs.close();
		if (deadlockflag) {//如果前面出现AGV要到达终点情况，跳过后续部分
			continue;
		}
		if (empty(AGVList)) {//判断AGV列表是否为空，如果为空直接将AGV信息存入
			AGVList.push_back(AGVinfo);
			continue;
		}
		bool flag = false;//AGVList是否有该AGV的记录
		//故障死锁部分
		if (AGVinfo.AGVstatus == "error") {
			//向MM报错，重新分配任务
			//重写地图
			continue;
		}
		//AGV信息处理部分
		int lenth = AGVList.size();
		int i = 0;
		for (; i < lenth; i++) {//遍历vector
			if (AGVinfo.AGVID == AGVList[i].AGVID) {//找到AGV记录
				flag = true;
				//超时死锁部分
				if (AGVinfo.nextPoint == AGVList[i].nextPoint) {//AGV位置没有发生变化（仍然在原来的路径上）
					AGVList[i].times++;
					if (AGVList[i].times > TIMETD) {//超时
						//deadlockChange(true, AGVList[i].nextPoint, AGVList[i].lastPoint);
						//调用deadlock类
						cout << "Time out:" << AGVList[i].AGVID << ',' << AGVList[i].times << "\n";
						deadLock myunlock;
						myunlock.setLockpoint(AGVinfo.nextPoint);//设置死锁点
						myunlock.setListandmap(AGVList, topmap);//传入AGV列表
						deadlockflag = true;
						continue;
					}
				}
				else {//AGV位置发生变化
					AGVList[i] = AGVinfo;//更新AGV信息
				}
			}
		}
		if (deadlockflag) {//如果前面出现死锁情况，跳过后续部分
			continue;
		}
		if (!flag) {//如果没有记录
			AGVList.push_back(AGVinfo);
		}
		//测试用
		for (vector<AGVtd>::iterator agvtest = AGVList.begin(); agvtest != AGVList.end(); agvtest++) {
			cout << agvtest->AGVID << ", " << agvtest->lastPoint << " ," << agvtest->nextPoint << " ," << agvtest->nnPoint << "\n";
		}
		//路径占用死锁判定部分
		for (lenth = AGVList.size(), i = 0; i < lenth; i++) {//遍历list，查找是否有冲突AGV
			//if (AGVList[i].lastPoint == AGVinfo.nextPoint&&AGVList[i].nextPoint == AGVinfo.lastPoint) {//检查AGV是否相向运行，这种情况应该不会出现
				//	//deadlockChange(true, AGVinfo.lastPoint, AGVinfo.nextPoint);
				//	//调用deadlock类
				//	deadlockflag = true;
				//	continue;
				//}
			if (AGVList[i].nextPoint == AGVinfo.nextPoint) {//检查AGV是否向同一个节点运行
				//检查下一节点是否重复
				if (AGVList[i].lastPoint == AGVinfo.nnPoint || AGVList[i].nnPoint == AGVinfo.lastPoint) {
					//测试用
					cout << "死锁点：" << AGVList[i].nextPoint << "\n";
					deadLock myunlock;//调用deadlock类
					myunlock.setListandmap(AGVList, topmap);;
					myunlock.setLockpoint(AGVList[i].nextPoint);
					myunlock.unlock();
					deadlockflag = true;
					break;
				}
			}
		}
		if (deadlockflag) {//如果前面出现死锁情况，跳过后续部分
			continue;
		}
	}

}

AGVtrans getAGVtrans() {
	AGVtrans AGVinfo;
	cout << "请输入AGV信息：";
	cin >> AGVinfo.AGVID >> AGVinfo.lastPoint;
	return AGVinfo;
}
