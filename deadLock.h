#pragma once
#include"stdafx.h"

using namespace std;

#define TIMETD  10//超时阈值，需要讨论
#define TOPMAP "./data/mapspec.json"//拓扑图路径
#define RASTERMAP "./data/Rastermap.json"//栅格图路径
#define AGVPATH "./data/agvpath.json"//AGV路径文件
#define RADIUS 1//锁区半径
//solving deadlock

struct path {
	string endPoint;//连通的点
	double lenth;
	double factor;//拥堵系数
};

struct Map
{
	string pointID;
	double pos_x; //x坐标
	double pos_y;// y坐标
	bool block; //Whether the AGV can pass the point
	vector<path>linkPoint;//连通的点
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
inline bool compInflu(AGVjampath& path1, AGVjampath& path2)//sort排序用,降序排列
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
	void setListandmap(vector<AGVtd>& list, vector<Map>& mymap);//传入AGV列表和地图
	void unlock();//解锁
	void lockmap();//根据死锁点，锁住栅格地图和拓扑图
	void unlockmap();//解锁地图
private:
	string deadlockPoint1;//deadlock point
	vector<AGVtd>AGVList;//AGV列表
	vector<Map>map;//地图
	vector<AGVjampath>AGVJamlist;//冲突AGV列表
	void newAGVPath(vector<AGVjampath>& list, AGVtd AGVinfo);//将AGV加入jamlistpath
	void buildJamlist();//检查AGVlist，将陷入死锁的AGV加入jamlist
	void updateJamlist();//更新jamlist中的influence和block
	bool cleanBlock();//清理block为true的路径
	bool gobackFreepoint(string AGVID, string pointID);//查找最近的空闲点
	void pauseAGV(string AGVID);//暂停AGV
	void sendNewpath(string AGVID, string nextpoint);//AGV按原路运行
	void sendNewpath(string AGVID, string nextpoint, string freepoint);//重载，告知规避点
	string searchFreepath();//寻找空闲路径
	void sendAGVpath(string AGVID,Json::Value root);//向AGV发送路径指令
	void updatefactor();//更新拥堵系数
};


deadLock::deadLock(string point) {//构造函数重载，构造时传入一个死锁点
	deadlockPoint1 = point;
}
//锁住地图
void deadLock::lockmap() {
	Json::CharReaderBuilder builder;
	Json::Value root;//定义根节点
	Json::CharReader* reader(builder.newCharReader());
	ifstream ifs(TOPMAP);
	builder["collectComments"] = false;
	JSONCPP_STRING errs;
	double pos_x, pos_y;
	if (parseFromStream(builder, ifs, &root, &errs)) {//读取拓扑图，查找死锁点坐标
		for (int i = 0, size = root.size(); i < size; i++) {
			if (root[i]["pointID"].asString() == deadlockPoint1) {
				pos_x = root[i]["pos_x"].asDouble();
				pos_y = root[i]["pos_y"].asDouble();
				break;
			}
		}
	}
	ifs.close();
	ifs.open(RASTERMAP);
	ofstream ofs;
	if (parseFromStream(builder, ifs, &root, &errs)) {
		Json::Value district, temp;
		Json::StreamWriterBuilder sw;
		temp.append(pos_x - RADIUS);
		temp.append(pos_y - RADIUS);
		district.append(temp);
		temp.clear();
		temp.append(pos_x - RADIUS);
		temp.append(pos_y + RADIUS);
		district.append(temp);
		temp.clear();
		temp.append(pos_x + RADIUS);
		temp.append(pos_y - RADIUS);
		district.append(temp);
		temp.clear();
		temp.append(pos_x + RADIUS);
		temp.append(pos_y + RADIUS);
		district.append(temp);
		root["obstacle"][deadlockPoint1] = district;
		ifs.close();
		sw.settings_["precision"] = 5;
		ofs.open(RASTERMAP);
		std::unique_ptr<Json::StreamWriter> writer(sw.newStreamWriter());
		writer->write(root, &ofs);
		ofs.close();
		cout << "节点" << deadlockPoint1 << "已加锁\n";
	}
}
//解锁地图
void deadLock::unlockmap() {
	Json::CharReaderBuilder builder;
	Json::Value root;//定义根节点
	Json::CharReader* reader(builder.newCharReader());
	ifstream ifs(RASTERMAP);
	builder["collectComments"] = false;
	JSONCPP_STRING errs;
	if (parseFromStream(builder, ifs, &root, &errs)) {
		ifs.close();
		root["obstacle"].removeMember(deadlockPoint1);
		ofstream ofs(RASTERMAP);
		Json::StreamWriterBuilder sw;
		sw.settings_["precision"] = 5;
		std::unique_ptr<Json::StreamWriter> writer(sw.newStreamWriter());
		writer->write(root, &ofs);
		ofs.close();
		cout << "节点" << deadlockPoint1 << "已解锁\n";
	}
}
//待完成部分，需要通信接口
void deadLock::pauseAGV(string AGVID) {//暂停AGV
	cout << AGVID << "暂停\n";
}
//待完成部分，需要通信接口
void deadLock::sendAGVpath(string AGVID,Json::Value root) {//向AGV发送路径指令
	cout << AGVID << "前往" << root << "\n";
}
//告知AGV规避点
void deadLock::sendNewpath(string AGVID, string nextpoint, string freepoint) {
	Json::Value root;
	root.append(nextpoint);
	root.append(freepoint);
	sendAGVpath(AGVID,root);
}
//告知AGV按原路径运行
void deadLock::sendNewpath(string AGVID, string nextpoint) {
	Json::CharReaderBuilder builder;
	Json::Value root;//定义根节点
	Json::Value jsondelete;
	Json::CharReader* reader(builder.newCharReader());
	ifstream ifs(AGVPATH);
	builder["collectComments"] = false;
	JSONCPP_STRING errs;
	if (parseFromStream(builder, ifs, &root, &errs)) {
		root = root[AGVID];
		for (int size = root.size(), i = 0; i < size;) {
			if (root[i] == nextpoint) {
				sendAGVpath(AGVID,root);
				return;
			}
			else {
				root.removeIndex(i,&jsondelete);
			}
		}
	}
	ifs.close();
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

void deadLock::setListandmap(vector<AGVtd>& list, vector<Map>& mymap) {
	AGVList = list;
	map = mymap;
	//测试用
	cout << "AGVList传入完成；\n";
}
//查找最近的空闲点
bool deadLock::gobackFreepoint(string AGVID, string pointID) {
	updatefactor();
	for (vector<Map>::iterator point = map.begin(); point != map.end(); point++) {
		if (point->pointID == pointID) {
			for (vector<path>::iterator freepath = point->linkPoint.begin(); freepath != point->linkPoint.end(); freepath++) {
				if (freepath->factor == 0 && freepath->endPoint != deadlockPoint1) {
					for (vector<Map>::iterator point2 = map.begin(); point2 != map.end(); point2++)
					{
						if (point2->pointID == freepath->endPoint) {
							for (vector<path>::iterator freepath2 = point->linkPoint.begin(); freepath2 != point->linkPoint.end(); freepath2++) {
								if (freepath2->factor == 0 && freepath2->endPoint != point2->pointID)
								{
									sendNewpath(AGVID, pointID, freepath2->endPoint);
									return true;
								}
							}
						}
					}
				}
			}
			return false;
		}
	}
}

void deadLock::updatefactor() {
	//清空拥堵系数
	for (vector<Map>::iterator point = map.begin(); point != map.end(); point++) {
		for (vector<path>::iterator freepath = point->linkPoint.begin(); freepath != point->linkPoint.end(); freepath++) {
			freepath->factor = 0;
		}
	}
	//更新拥堵系数
	for (vector<AGVtd>::iterator AGV = AGVList.begin(); AGV != AGVList.end(); AGV++)
	{
		for (vector<Map>::iterator point = map.begin(); point != map.end(); point++) {
			if (point->pointID == AGV->nextPoint) {
				for (vector<path>::iterator freepath = point->linkPoint.begin(); freepath != point->linkPoint.end(); freepath++) {//查找相邻路径
					if (freepath->endPoint == AGV->lastPoint) {//反向，拥堵系数加一
						freepath->factor++;
						break;
					}
				}
				break;
			}
		}
	}
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
														  //测试用
	cout << "Jamlist更新：\n";
	for (vector<AGVjampath>::iterator jampath = AGVJamlist.begin(); jampath != AGVJamlist.end(); jampath++) {
		cout << "JamPath:" << jampath->startPoint << "->" << jampath->lockPoint << ": influence--" << jampath->influence << "  , block" << jampath->block << "\n";

	}
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
			//AGV暂停
			//测试用
			pauseAGV(AGVList[i].AGVID);//暂停AGV
		}
	}
	//测试用
	for (vector<AGVjampath>::iterator jampath = AGVJamlist.begin(); jampath != AGVJamlist.end(); jampath++) {
		cout << "JamPath:" << jampath->startPoint << "->" << jampath->lockPoint << " :";
		for (vector<AGVtd>::iterator jamAGV = jampath->AGVpathlist.begin(); jamAGV != jampath->AGVpathlist.end(); jamAGV++) {
			cout << "    " << jamAGV->AGVID << ", ";
		}
		cout << "\n";
	}
}

bool deadLock::cleanBlock() {//清理block为true的路径
	if (AGVJamlist.empty())
	{
		return true;
	}
	updateJamlist();
	bool blockflag = true;//记录本次遍历中是否有block为true的路径,有的话为false，无为true
	for (vector<AGVjampath>::iterator jampath = AGVJamlist.begin(); !empty(AGVJamlist); )//遍历jamlist，不受干扰的MR首先通行
	{
		if (jampath->block)//有block为可行的路径
		{
			for (vector<AGVtd>::iterator jamAGV = jampath->AGVpathlist.begin(); jamAGV != jampath->AGVpathlist.end(); jamAGV++) {
				cout << "路径" << jampath->startPoint << "->" << jampath->lockPoint << "所属" << jamAGV->AGVID << "按原路通行\n";
				//告知AGV前往该路径
				sendNewpath(jamAGV->AGVID, jamAGV->nextPoint);
			}
			//waiting for reply
			for (vector<AGVtd>::iterator jamAGV = jampath->AGVpathlist.begin(); !empty(jampath->AGVpathlist);)//等待AGV到达避让路径，并更新jampath
			{
				//等待AGV回复
				//jamAGV->nnPoint=
				cout << jamAGV->AGVID << "到达" << jamAGV->nnPoint << "\n";
				jamAGV = jampath->AGVpathlist.erase(jamAGV);
				if (jamAGV == jampath->AGVpathlist.end()) {
					jamAGV = jampath->AGVpathlist.begin();
				}
			}
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


//寻找空闲路径
string deadLock::searchFreepath() {
	for (vector<Map>::iterator point = map.begin(); point != map.end(); point++) {//查找死锁点
		if (point->pointID == deadlockPoint1) {
			for (vector<path>::iterator freepath = point->linkPoint.begin(); freepath != point->linkPoint.end(); freepath++) {//查找相邻路径
				bool freePathflag = true;
				for (vector<AGVjampath>::iterator jampath = AGVJamlist.begin(); jampath != AGVJamlist.end(); jampath++) {
					if (jampath->startPoint == freepath->endPoint) {
						freePathflag = false;
						break;
					}
				}
				if (freePathflag) {
					return freepath->endPoint;
				}
			}
			return "false";
		}
	}
	return "false";
}

void deadLock::unlock() {//解锁
	buildJamlist();//建立AGV冲突表
	lockmap();//改写地图
			  //开始解锁
	if (cleanBlock()) {
		//测试用
		cout << "自解锁完成\n";
		unlockmap();
		return;
	}
	else {//如果清理过后，jamlist仍有内容，说明有MR互不相让，需要规划新路径，执行后退
		sort(AGVJamlist.begin(), AGVJamlist.end(), compInflu);//按influence排序
		for (vector<AGVjampath>::iterator jampath = AGVJamlist.begin(); !empty(AGVJamlist);)
		{
			string freePath = searchFreepath();//寻找可进行避让的空闲路径
			if (freePath != "false") {
				for (vector<AGVtd>::iterator jamAGV = jampath->AGVpathlist.begin(); jamAGV != jampath->AGVpathlist.end(); jamAGV++) {
					cout << "路径" << jampath->startPoint << "->" << jampath->lockPoint << "所属" << jamAGV->AGVID << "前往" << freePath << "等待\n";
					//告知AGV前往该路径
					sendNewpath(jamAGV->AGVID, jamAGV->nextPoint, freePath);
				}
				AGVjampath temp;
				temp.startPoint = freePath;
				temp.lockPoint = deadlockPoint1;
				for (vector<AGVtd>::iterator jamAGV = jampath->AGVpathlist.begin(); !empty(jampath->AGVpathlist);)//等待AGV到达避让路径，并更新jampath
				{
					//等待AGV回复
					jamAGV->lastPoint = freePath;
					jamAGV->nextPoint = deadlockPoint1;
					//jamAGV->nnPoint=
					temp.AGVpathlist.push_back(*jamAGV);
					cout << jamAGV->AGVID << "到达" << freePath << "\n";
					jamAGV = jampath->AGVpathlist.erase(jamAGV);
					if (jamAGV == jampath->AGVpathlist.end()) {
						jamAGV = jampath->AGVpathlist.begin();
					}
				}
				jampath = AGVJamlist.erase(jampath);
				AGVJamlist.push_back(temp);
			}
			else if (jampath->back) {
				bool backflag = true;
				for (vector<AGVtd>::iterator jamAGV = jampath->AGVpathlist.begin(); jamAGV != jampath->AGVpathlist.end(); jamAGV++) {
					cout << "无可避让路径，路径" << jampath->startPoint << "->" << jampath->lockPoint << "所属" << jamAGV->AGVID << "后退避让\n";
					if (!gobackFreepoint(jamAGV->AGVID, jamAGV->lastPoint)) {
						cout << jamAGV->AGVID << "无法后退\n";
						backflag = false;
						break;
					}
				}
				if (backflag) {
					jampath = AGVJamlist.erase(jampath);//删除该路径,注意：erase方法会把迭代器指向下一个元素
				}
			}
			//防止溢出
			if (jampath == AGVJamlist.end())
			{
				jampath = AGVJamlist.begin();
			}
			if (cleanBlock()) {
				unlockmap();
				return;
			}
		}
	}
}
