#pragma once
#include"stdafx.h"

using namespace std;

#define TIMETD  10//超时阈值，需要讨论

//solving deadlock

struct mapPath
{
	double length; //path length
	double weight;// weight factor
	string type; //
	bool block; //Whether the AGV can pass the path
};

struct AGVtd {//AGV信息表格式
	string AGVID;//AGV的ID
	string lastPoint;//上一次经过的点
	string nextPoint;//下一个要经过的点
	string nnPoint;//下下个节点
	double xPosition;//MR的x坐标
	double yPosition;//MR的y坐标
	int times;//MR在当前位置的反馈次数，用来记录超时，阈值待确定
	string AGVstatus;//AGV当前状态，working，running，charging，waiting，error
	bool back;//MR是否可以后退
};

struct AGVjampath {//死锁路径信息
	string startPoint;//路径起始点
	string lockPoint;//路径终点（死锁点）
	double priority;//优先级
	double influence;//该路径对其他AGV的影响（即有多少辆AGV需要通过这条路径）
	bool block;//是否被遮挡
	bool back;//是否可后退
	vector<AGVtd>AGVpathlist;//在这条路径上的AGV列表
};

struct AGVtrans {//MR向TC定时发送的位置信息
	string AGVID;//AGV的ID
	string lastPoint;//上一次经过的点
	string nextPoint;//下一个要经过的点
	string nnPoint;//下下个节点
	float xPosition;//MR的x坐标
	float yPosition;//MR的y坐标
	string AGVstatus;//AGV当前状态，working，running，charging，waiting，error
	bool back;//AGV是否可以后退
};
inline bool compInflu(AGVjampath & path1, AGVjampath & path2)//sort排序用,降序排列
{
	return path1.influence > path2.influence;
}

class deadLock
{
public:
	deadLock() {//构造函数

	}
		deadLock(string lockPoint1);//构造函数重载，构造时传入一个死锁点
		void setLockpoint(string lockPoint1);//设定一个死锁点
		void setAGVlist(vector<AGVtd> list);//传入AGV列表
		void unlock();//解锁
	
	private:
		string deadlockPoint1;//deadlock point
		vector<AGVtd>AGVList;//AGV列表
		vector<AGVjampath>AGVJamlist;//冲突AGV列表
		void newAGVPath(vector<AGVjampath>& list, AGVtd AGVinfo);//将AGV加入jamlistpath
		void buildJamlist();//检查AGVlist，将陷入死锁的AGV加入jamlist
		void updateJamlist();//更新jamlist中的influence和block
		bool cleanBlock();//清理block为true的路径
		string searchFreepoint();//查找最近的空闲点
};


deadLock::deadLock(string point) {//构造函数重载，构造时传入一个死锁点
	deadlockPoint1 = point;
}

void deadLock::newAGVPath(vector<AGVjampath>& list, AGVtd AGVinfo) {//将AGV加入jamlistpath
	AGVjampath temp;
	temp.startPoint = AGVinfo.lastPoint;
	temp.lockPoint = AGVinfo.nextPoint;
	temp.AGVpathlist.push_back(AGVinfo);
	temp.back = AGVinfo.back;
	temp.block = true;
	temp.influence = 0;
	list.push_back(temp);
}

void deadLock::setLockpoint(string lockPoint1) {//设定一个死锁点
	deadlockPoint1 = lockPoint1;
}

void deadLock::setAGVlist(vector<AGVtd> list) {
	AGVList = list;
}
//待完成
string deadLock::searchFreepoint() {//查找最近的空闲点
	return;
}

void deadLock::updateJamlist() {//更新jamlist中的influence和block
	if (AGVJamlist.empty()) {
		return;
	}
	for (vector<AGVjampath>::iterator jampath = AGVJamlist.begin(); jampath != AGVJamlist.end(); jampath++) {
		jampath->block = true;
		jampath->influence = 0;
	}
	for (vector<AGVjampath>::iterator jampath = AGVJamlist.begin(); jampath != AGVJamlist.end(); jampath++) {//遍历AGVJamlist，更新block，influence信息
		for (vector<AGVtd>::iterator jamAGV = jampath->AGVpathlist.begin(); jamAGV != jampath->AGVpathlist.end(); jamAGV++) {//检查AGV的下下个节点
			for (vector<AGVjampath>::iterator jampathcheck = AGVJamlist.begin(); jampathcheck != AGVJamlist.end(); jampathcheck++) {
				if (jampathcheck->startPoint == jamAGV->nnPoint) {//检查AGV的下下个节点的路径是否有AGV
					jampath->block = false;//不可通行
					jampathcheck->influence++;//影响系数加一
					break;
				}
			}
		}
	}
	sort(AGVJamlist.begin(), AGVJamlist.end(), compInflu);//按influence排序
}

void deadLock::buildJamlist() {//检查AGVlist，将陷入死锁的AGV加入jamlist
	//确定陷入死锁的AGV
	int lenth, i;
	for (lenth = AGVList.size(), i = 0; i < lenth; i++) {//遍历list，查找冲突AGV
		if (AGVList[i].nextPoint == deadlockPoint1) {//检查AGV是否向冲突点运行
			if (AGVJamlist.empty()) {//检查AGVJamlist是否为空，空的话直接将AGV加入list
				newAGVPath(AGVJamlist, AGVList[i]);
			}
			else//AGVJamlist不为空
			{
				bool jampathflag = false;
				for (vector<AGVjampath>::iterator jampath = AGVJamlist.begin(); jampath != AGVJamlist.end(); jampath++)//使用迭代器遍历AGVJamlist
				{
					if (jampath->startPoint == AGVList[i].lastPoint) {//寻找到该AGV所在路径
																	  //将AGV加入该路径
						jampath->AGVpathlist.push_back(AGVList[i]);
						jampath->back = jampath->back ? AGVList[i].back : false;//如果路径的back属性为真，将agv的back属性赋给路径
						jampathflag = true;
						break;
					}
				}
				if (!jampathflag) {//若jamlist没有该agv路径
					newAGVPath(AGVJamlist, AGVList[i]);
				}
			}
		}
	}
	updateJamlist();
}

bool deadLock::cleanBlock() {//清理block为true的路径
	if (AGVJamlist.empty())
	{
		return true;
	}
	bool blockflag = true;//记录本次遍历中是否有block为true的路径,有的话为false，无为true
	for (vector<AGVjampath>::iterator jampath = AGVJamlist.begin(); !empty(AGVJamlist); )//遍历jamlist，不受干扰的MR首先通行
	{
		if (jampath->block)//有block为可行的路径
		{
			//waiting for reply
			jampath = AGVJamlist.erase(jampath);//删除该路径
			updateJamlist();//更新jamlist
			blockflag = false;
			if (jampath == AGVJamlist.end())
			{
				jampath = AGVJamlist.begin();
				blockflag = true;
			}
		}
		else
		{
			jampath++;
			if (jampath == AGVJamlist.end())
			{
				jampath = AGVJamlist.begin();
				if (blockflag)
				{
					return false;
				}
				blockflag = true;
			}
		}
	}
	return true;
}

void deadLock::unlock() {//解锁
	buildJamlist();//建立AGV冲突表
	//开始解锁
	if (cleanBlock()) {
		return;
	}
	else {//如果清理过后，jamlist仍有内容，说明有MR互不相让，需要规划新路径，执行后退
		sort(AGVJamlist.begin(), AGVJamlist.end(), compInflu);//按influence排序
		for (vector<AGVjampath>::iterator jampath = AGVJamlist.begin(); !empty(AGVJamlist);)
		{
			if (jampath->back) {
				for (vector<AGVtd>::iterator jamAGV = jampath->AGVpathlist.begin(); jamAGV != jampath->AGVpathlist.end(); jamAGV++) {
					//重新规划路径
					//读取jamAGV的终点
					//if (!repath(jamAGV->AGVID, jamAGV->nextPoint, destination)) {
					//	//找就近避让点
					//	repath(jamAGV->AGVID, jamAGV->nextPoint, searchFreepoint());
					//	//此处需要监听
					//}
				}
			 }
			jampath = AGVJamlist.erase(jampath);//删除该路径
			if (jampath == AGVJamlist.end())
			{
				jampath = AGVJamlist.begin();
			}
			updateJamlist();
			if (cleanBlock()) {
				return;
			}
		}
	}
}
