#pragma once
#include "stdafx.h"
#include"deadlock.h"
using namespace std;

vector<AGVtd> AGVList;//创建AGV列表
vector<Map>map;//建立地图
vector<Map>backupmap;//备份拓扑图
void deadlockChange(bool status, string Point);//更改死锁状态并告知MM
AGVtrans getAGVtrans();//测试用
void resetmap(string point1, string point2);//死锁发生时重设地图
void resetmap(string point);//重载，只有一个死锁点
void getmap() {//测试用，获得地图
	Map temp;
	path pathtemp;
	temp.pointID = "2";
	pathtemp.endPoint = "1";
	temp.linkPoint.push_back(pathtemp);
	pathtemp.endPoint = "3";
	temp.linkPoint.push_back(pathtemp);
	pathtemp.endPoint = "4";
	temp.linkPoint.push_back(pathtemp);
	pathtemp.endPoint = "5";
	temp.linkPoint.push_back(pathtemp);
	map.push_back(temp);
	cout << "地图初始化完成\n";
	for (vector<Map>::iterator point = map.begin(); point != map.end(); point++) {
		cout << point->pointID<<"->";
		for (vector<path>::iterator freepath = point->linkPoint.begin(); freepath != point->linkPoint.end(); freepath++) {
			cout << freepath->endPoint << ", ";
		}
		cout << "\n";
	}
}

int main() {
	//测试用，伪造地图
	getmap();
	while (1) {//保持运行
		AGVtrans AGVMessage = getAGVtrans();//收到AGV信息后，开始
		AGVtd AGVinfo;
		AGVinfo.AGVID = AGVMessage.AGVID;
		AGVinfo.lastPoint = AGVMessage.lastPoint;
		AGVinfo.nextPoint = AGVMessage.nextPoint;
		AGVinfo.nnPoint = AGVMessage.nnPoint;
		AGVinfo.times = 0;
		if (empty(AGVList)) {//判断AGV列表是否为空，如果为空直接将AGV信息存入
			AGVList.push_back(AGVinfo);
			continue;
		}
		bool flag = false;//AGVList是否有该AGV的记录
		bool deadlockflag = false;//死锁标记
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
						myunlock.setListandmap(AGVList, map);//传入AGV列表
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
					myunlock.setListandmap(AGVList, map);;
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

void resetMap(string point1, string point2) {//重写地图
	//删路
}

void resetMap(string point) {//重写地图
	//删死锁点连通过的路径
}

void resetTask(bool status, string Point) {//更改死锁状态并告知MM

}
AGVtrans getAGVtrans() {
	AGVtrans AGVinfo;
	//AGVinfo.AGVID = "agv133";
	//AGVinfo.lastPoint = "11";
	//AGVinfo.nextPoint = "25";
	cout << "请输入AGV信息：";
	cin >> AGVinfo.AGVID >> AGVinfo.lastPoint >> AGVinfo.nextPoint >> AGVinfo.nnPoint;
	return AGVinfo;
}
