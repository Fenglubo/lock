#pragma once
#include <string>
#include <vector>
#include <iostream>
using namespace std;
//solving deadlock
struct mapPath
{
	double length; //path length
	double weight;// weight factor
	string type; //
	bool block; //Whether the AGV can pass the path
	int* timewindow; //Time period when the path is occupied

};
struct AGVinfo {
	float speed;
	bool backup;//whether AGV can back up
};
class deadLock
{
public:
	int* newPath1;//解除死锁的新路径
	int* newPath2;

	deadLock();
	void setMap(mapPath& map);//接受地图数据
	void setLockpoint(int lockPoint);
	void setLockAGV(string AGV);
	void resetmap(string lockpint);//重设地图
	void newpath();//给出新路径
	string unlock(int status);//判断应该避让的AGV
	string predict(int AGV1position, float angle);//预测

private:
	int deadlockPoint;//deadlock point
	string lockAGV1;//deaklock AGV1
	string lockAGV2;//deadlock AGV2
	int AGV2position;//current position
	int AGV1position;
	mapPath *map; //map array


};
deadLock::deadLock(void) {

}
void deadLock::setMap(mapPath &myMap) {
	map = &myMap;
}
void deadLock::setLockpoint(int lockPoint) {
	deadlockPoint = lockPoint;
}