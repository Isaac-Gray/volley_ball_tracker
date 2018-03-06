#pragma once
#ifndef A2DD_H
#define A2DD_H
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv/cv.h>
#include <iostream>
#include <tchar.h>
#include <windows.h>
#include <string>

using namespace cv;
using namespace std;

DWORD WINAPI CamAquire(LPVOID lpParam);
DWORD WINAPI Processing(LPVOID lpParam);

const int FRAME_WIDTH = 1280;
const int FRAME_HEIGHT = 720;
const int MIN_OBJECT_AREA = 15 * 15;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH / 1.5;

class FPS {
private:
	/*LARGE_INTEGER startTime;
	LARGE_INTEGER endTime;
	LARGE_INTEGER frequancy;*/
	uint numberOfFrames = 0;
public:
	/*void start();
	void endnow();
	void startFreq();*/
	void undateFrame(uint size);
	double totalFps(LARGE_INTEGER s, LARGE_INTEGER e, LARGE_INTEGER f);
	uint returnNumberOfFrames();
	//LARGE_INTEGER getStartTime();
	//LARGE_INTEGER getEndTime();
};
uint FPS::returnNumberOfFrames() {
	return numberOfFrames;
}
/*
LARGE_INTEGER FPS::getStartTime() {
return startTime;
}
LARGE_INTEGER FPS::getEndTime() {
return endTime;
}*/
#endif