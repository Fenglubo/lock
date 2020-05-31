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
	int* newPath1;//�����������·��
	int* newPath2;

	deadLock();
	void setMap(mapPath& map);//���ܵ�ͼ����
	void setLockpoint(int lockPoint);
	void setLockAGV(string AGV);
	void resetmap(string lockpint);//�����ͼ
	void newpath();//������·��
	string unlock(int status);//�ж�Ӧ�ñ��õ�AGV
	string predict(int AGV1position, float angle);//Ԥ��

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