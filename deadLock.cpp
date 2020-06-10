#pragma once
#include "stdafx.h"
#include"deadlock.h"
using namespace std;


void deadlockChange(bool status, string Point);//更改死锁状态并告知MM
AGVtrans getAGVtrans();//测试用

int main() {
	vector<AGVtd> AGVList;//创建AGV列表
	while (1) {//保持运行
		AGVtrans AGVMessage=getAGVtrans();//收到AGV信息后，开始
		AGVtd AGVinfo;
		AGVinfo.AGVID = AGVMessage.AGVID;
		AGVinfo.lastPoint = AGVMessage.lastPoint;
		AGVinfo.nextPoint = AGVMessage.nextPoint;
		AGVinfo.xPosition = AGVMessage.xPosition;
		AGVinfo.yPosition = AGVMessage.yPosition;
		AGVinfo.times = 0;
		if (empty(AGVList)) {//判断AGV列表是否为空，如果为空直接将AGV信息存入
			AGVList.push_back(AGVinfo);
			continue;
		}
		bool flag = false;//AGVList是否有该AGV的记录
		bool deadlockflag = false;//死锁标记
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
						deadLock myunlock;
						myunlock.setLockpoint(AGVinfo.nextPoint);//设置死锁点
						myunlock.setAGVlist(AGVList);//传入AGV列表
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
		//路径占用死锁判定部分
		for (lenth = AGVList.size(), i = 0; i < lenth; i++) {//遍历list，查找是否有冲突AGV
			if (AGVList[i].lastPoint == AGVinfo.nextPoint&&AGVList[i].nextPoint == AGVinfo.lastPoint) {//检查AGV是否相向运行，这种情况应该不会出现
				//deadlockChange(true, AGVinfo.lastPoint, AGVinfo.nextPoint);
				//调用deadlock类
				deadlockflag = true;
				continue;
			}
			if (AGVList[i].nextPoint == AGVinfo.nextPoint) {//检查AGV是否向同一个节点运行
				//检查下一节点是否重复
				if (AGVList[i].lastPoint == AGVinfo.nnPoint || AGVList[i].nnPoint == AGVinfo.nextPoint) {
					deadLock myunlock;//调用deadlock类

				}
				deadlockflag = true;
				continue;
			}
		}
		if (deadlockflag) {//如果前面出现死锁情况，跳过后续部分
			continue;
		}
	}

}

void deadlockChange(bool status, string Point) {//更改死锁状态并告知MM

}
AGVtrans getAGVtrans() {
	AGVtrans AGVinfo;
	AGVinfo.AGVID = "agv133";
	AGVinfo.lastPoint = "11";
	AGVinfo.nextPoint = "25";
	AGVinfo.xPosition = 2.02;
	AGVinfo.yPosition = 4.39;
	return AGVinfo;
}
