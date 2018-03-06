// Disable deprecated function warnings with Visual Studio 2005
#if defined(_MSC_VER) && _MSC_VER >= 1400
#pragma warning(disable: 4995)
#endif

#include <iostream>
#include <string>
#include <sstream>

#include "stdio.h"
#include "conio.h"
#include "GrabFramer.h"

#include "FPS.h"
#include <mutex>
#include <queue>
#include <atomic>
#include <vector>

using namespace cv;
using namespace std;

// Restore deprecated function warnings with Visual Studio 2005
#if defined(_MSC_VER) && _MSC_VER >= 1400
#pragma warning(default: 4995)
#endif

#define STRING_LENGTH		256 
HANDLE camAquire, processing, output;
queue<Mat> buffer;
std::mutex camLock;
std::atomic<bool> grabLock;

bool starting = true;
FPS keepTrack;
LARGE_INTEGER startTime;
LARGE_INTEGER endTime;
LARGE_INTEGER frequancy;

grabFrame *CamObj = new grabFrame();

void drawObject(int x, int y, Mat &cFrame);
void morphOps(Mat &thresh);
void trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed);
void courtCalibration();
BOOL white_tester(Vec3b x);

void grabFrame::XferCallback(SapXferCallbackInfo *pInfo)
{
	// Display the last transferred frame
	grabFrame* pDlg = (grabFrame *)pInfo->GetContext();
	void *pData = NULL;
	pDlg->pBuffer->GetAddress(&pData);
	Mat image(1024, 1280, CV_8UC3, pData);
	camLock.lock();
	buffer.push(Mat(image.size(), image.type()));
	image.copyTo(buffer.back());
	camLock.unlock();
}

void cameraName(char* cameraName);

int main()
{
	grabLock.store(true);
	courtCalibration();
	//grabFrame *Everything = new grabFrame();
	//char serverName[CORSERVER_MAX_STRLEN];
	
	//cameraName(serverName);
	//printf("/n starting : %s ", serverName);
	//if (serverName) {
	//	Everything->DefineObjects(serverName);
	//	Everything->CreateObjects();
		
		//Everything->onGrab();
	/*
	camAquire = CreateThread(NULL, 0, CamAquire, NULL, 0, NULL);
	if (camAquire == NULL) {
		printf("CreateThread error: %d\n", GetLastError());
	}

	processing = CreateThread(NULL, 0, Processing, NULL, 0, NULL);
	if (camAquire == NULL) {
		printf("CreateThread error: %d\n", GetLastError());
	}
		CamObj->stopPlease();
		CamObj->DestroyObjects();
	*/	
	printf("\nPress any key to terminate\n");
	printf("\n number of frames in the buffer = %d", buffer.size());
	grabLock.store(false);
	CorGetch();
	SapManager::Close();
	return 0;
}
		
void cameraName(char* cameraName) {
	// Get total number of boards in the system
	int serverCount = SapManager::GetServerCount();
	int GenieIndex = 0;
	BOOL serverFound = FALSE;
	char serverName[CORSERVER_MAX_STRLEN];
	SapManager::SetDisplayStatusMode(SapManager::StatusLog);
	if (serverCount == 0)
	{
		printf("No device found!\n");
	}

	for (int serverIndex = 0; serverIndex < serverCount; serverIndex++)
	{
		if (SapManager::GetResourceCount(serverIndex, SapManager::ResourceAcqDevice) != 0)
			serverFound = TRUE;
	}

	if (!serverFound)
	{
		printf("No camera found!\n");
		
	}
	for (int serverIndex = 0; serverIndex < serverCount; serverIndex++)
	{
		if (SapManager::GetResourceCount(serverIndex, SapManager::ResourceAcqDevice) != 0)
		{
			// Get Server Name Value
			SapManager::GetServerName(serverIndex, serverName, sizeof(serverName));
			printf("%s\n", serverName);
			for (int n = 0; n<sizeof(serverName); n++) {
				cameraName[n] = serverName[n];
			}
		}
	}
}

BOOL grabFrame::DefineObjects(char* cameraName) {
	// Define objects
	const char settings[] = "C:\\Program Files\\Teledyne DALSA\\Sapera\\CamFiles\\User\\T_Nano-C1280_RBG_color.ccf";
	AcqDevice = new SapAcqDevice(cameraName, settings);
	pBuffer = new SapBufferWithTrash(30, AcqDevice);
	View = new SapView(pBuffer);
	Xfer = new SapAcqDeviceToBuf(AcqDevice, pBuffer, XferCallback, this);
	return true;
}

BOOL grabFrame::CreateObjects()
{
	// Create acquisition object
	if (AcqDevice && !*AcqDevice && !AcqDevice->Create())
	{
		DestroyObjects();
		return FALSE;
	}

	// Create buffer object
	if (pBuffer && !*pBuffer)
	{
		if (!pBuffer->Create())
		{
			DestroyObjects();
			return FALSE;
		}
		// Clear all buffers
		pBuffer->Clear();
	}
	
	// Create Colour Converstion object
	if (m_Conv && !*m_Conv)
	{
		if (!m_Conv->Create())
		{
			DestroyObjects();
			return FALSE;
		}
	}

	// Create view object
	if (View && !*View)
	{
		if (!View->Create()) {
			DestroyObjects();
			return FALSE;
		}
	}
	
	// Set next empty with trash cycle mode for transfer
	if (Xfer && Xfer->GetPair(0))
	{
		if (!Xfer->GetPair(0)->SetCycleMode(SapXferPair::CycleNextWithTrash))
		{
			DestroyObjects();
			return FALSE;
		}
	}

	// Create transfer object
	if (Xfer && !*Xfer && !Xfer->Create())
	{
		DestroyObjects();
		return FALSE;
	}

	return TRUE;
}

BOOL grabFrame::DestroyObjects()
{
	// Destroy transfer object
	if (Xfer && *Xfer) Xfer->Destroy();

	// Destroy view object
	if (View && *View) View->Destroy();

	// Destroy bayer object
	if (m_Conv && *m_Conv) m_Conv->Destroy();

	// Destroy buffer object
	if (pBuffer && *pBuffer) pBuffer->Destroy();

	// Destroy buffer object
	if (outBuffer && *outBuffer) outBuffer->Destroy();

	// Destroy acquisition object
	if (AcqDevice && *AcqDevice) AcqDevice->Destroy();

	delete Xfer;
	delete m_Conv;
	delete View;
	delete pBuffer;
	delete outBuffer;
	delete AcqDevice;

	return TRUE;
}

void grabFrame::onGrab() {
	Xfer->Grab();
}

void grabFrame::onSnap() {
	Xfer->Snap();
}

void grabFrame::stopPlease() {
	printf("Press any key to stop grab\n");
	CorGetch();
	// Stop the transfer and wait (timeout = 5 seconds)
	BOOL status = Xfer->Freeze();
	status = Xfer->Wait(5000);
	grabLock.store(false);
	CorGetch();
}

void grabFrame::stopCalibration() {
	printf("ok got Picture \n");
	CorGetch();
	// Stop the transfer and wait (timeout = 5 seconds)
	BOOL status = Xfer->Freeze();
	status = Xfer->Wait(5000);
	grabLock.store(false);
	CorGetch();
}

//author Isaac
// this is a function to grab a pic of the court 
//and find where the corners of the court are and the height of the camera
void courtCalibration() {
	Mat courtFrame = imread("C:\\Users\\Isaac\\Pictures\\opencv_tests\\court1.jpg", 1);;
	/*
	//grabFrame *CamCourtOb = new grabFrame();
	char serverName[CORSERVER_MAX_STRLEN]; //holds the name of the camera if found
	cameraName(serverName); //looks for camera puts location name in serverName
	printf("/n starting : %s \n", serverName);
	
	if (serverName) {
		CamObj->DefineObjects(serverName);
		CamObj->CreateObjects();
	}
	if (grabLock.load() == true) {
		CamObj->onSnap();

	}
	
	CamObj->stopCalibrate();
	CamObj->DestroyObjects();
	//pop it off the global buffer
	if (buffer.size() > 0) {
		buffer.front().copyTo(courtFrame);
		buffer.pop();
		printf("\n somthings in buffer \n");

	}*/
	//flate feild correction
	Mat map1, map2, correct_src;
	Size image_size = courtFrame.size();
	initUndistortRectifyMap(camera_matrix, distortion_coef, Mat(),
		getOptimalNewCameraMatrix(camera_matrix, distortion_coef, image_size, 1, image_size, 0),
		image_size, CV_32FC1, map1, map2);
	remap(courtFrame, correct_src, map1, map2, INTER_LINEAR);

	calibration_scan_image(correct_src);
	
	imwrite("C:\\Users\\Isaac\\Pictures\\opencv_tests\\after_flat_field.jpg", correct_src);
	imshow("track", courtFrame);
	imshow("corrected", correct_src);
	waitKey(10);

}
void calibration_scan_image(Mat& I) {
	//Accept only char type matrices
	CV_Assert(I.depth() != sizeof(uchar));

	int i, j, corner;
	Point seed;

	int dilation_size = 1;
	Mat element = getStructuringElement(MORPH_CROSS,
		Size(2 * dilation_size + 1, 2 * dilation_size + 1),
		Point(dilation_size, dilation_size));
	dilate(I, I, element);
	for (i = 0; i < I.rows; ++i) {
		for (j = 0; j < I.col; ++j) {
			// looking for the line
			if (white_tester(I.at<Vec3b>(Point(j, i)))) { // found a white pixel
				corner = white_look_around(I, Point(j, i));
				if (corner == 1) { // top left corner
					if (tL.x > j && tL.y > i) {
						tL = Point(j, i);
						I.at<Vec3b>(Point(j, i)) = wipe2;
						cout << "Top Left Corner Found: " << I.at<Vec3b>(Point(j, i)) << " | Point: " << Point(j, i) << endl;
					}
				}
				else if (corner == 2) { // top right corner 
					if (tR.x < j && tR.y > i) {
						tR = Point(j, i);
						I.at<Vec3b>(Point(j, i)) = wipe2;
						cout << "Top Right Corner Found: " << I.at<Vec3b>(Point(j, i)) << " | Point: " << Point(j, i) << endl;
					}
				}
				else if (corner == 3) { // bottom left corner
					if (bL.x > j && bL.y < i) {
						bL = Point(j, i);
						I.at<Vec3b>(Point(j, i)) = wipe2;
						cout << "Bottom Left Corner Found: " << I.at<Vec3b>(Point(j, i)) << " | Point: " << Point(j, i) << endl;
					}
				}
				else if (corner == 4) { // bottom right corner
					if (bR.x < j && bR.y < i) {
						bR = Point(j, i);
						I.at<Vec3b>(Point(j, i)) = wipe2;
						cout << "Bottom Right Corner Found: " << I.at<Vec3b>(Point(j, i)) << " | Point: " << Point(j, i) << endl;
					}
				}
			}
		}
	}
}
BOOL white_tester(Vec3b x) {
	if (x[0] >= white_min[0] && x[0] <= white_max[0] &&
		x[1] >= white_min[1] && x[1] <= white_max[1] &&
		x[2] >= white_min[2] && x[2] <= white_max[2]) { // found a white pixel
		return true;
	}
	else {
		return false;
	}
}

int white_look_around(Mat &I, Point start) {
	int corner = 0; // none = 0, top left = 1, top right = 2, bottom left = 3, bottom right = 4
	if (white_tester(I.at<Vec3b>(Point(min(start.x + spacing, 639), start.y))) &&
		white_tester(I.at<Vec3b>(Point(start.x, min(start.y + spacing, 479)))) &&
		start.x < 220 && start.y < 180 && start.y > 40) { // top left pixel
		corner = 1;
	}
	else if (white_tester(I.at<Vec3b>(Point(max(start.x - spacing, 0), start.y))) &&
		white_tester(I.at<Vec3b>(Point(start.x, min(start.y + spacing, 479)))) &&
		start.x > 420 && start.y < 180 && start.y > 40) { // top right pixel
		corner = 2;
	}
	else if (white_tester(I.at<Vec3b>(Point(min(start.x + spacing, 639), start.y))) &&
		white_tester(I.at<Vec3b>(Point(start.x, max(start.y - spacing, 0)))) &&
		start.x < 220 && start.y > 300) { // bottom left pixel
		corner = 3;
	}
	else if (white_tester(I.at<Vec3b>(Point(max(start.x - spacing, 0), start.y))) &&
		white_tester(I.at<Vec3b>(Point(start.x, max(start.y - spacing, 0)))) &&
		start.x > 420 && start.y > 300) { // bottom right pixel
		corner = 4;
	}

	return corner;
}

DWORD WINAPI CamAquire(LPVOID lpParam) {
	Mat frame;
	char serverName[CORSERVER_MAX_STRLEN]; //holds the name of the camera if found
	cameraName(serverName); //looks for camera puts location name in serverName
	printf("/n starting : %s ", serverName);
	if (serverName) {
		CamObj->DefineObjects(serverName);
		CamObj->CreateObjects();
	}
	//if (starting) {
	//	QueryPerformanceCounter(&startTime);
	//	QueryPerformanceCounter(&frequancy);
	//	starting = false;
	//}
	
	while (grabLock.load() == true) {
		CamObj->onGrab();
	
	}
	printf ("Exiting camera thread \n");
	return 0;
}
DWORD WINAPI Processing(LPVOID lpParam) {
	Mat frame;
	Mat HSVBlue;
	Mat HSVYellow;
	Mat threshold;
	int x, y;
	while (grabLock.load() == true) {
		camLock.lock();
		int sizeBuf = buffer.size();
		if (sizeBuf > 0) {
			buffer.front().copyTo(frame);
			buffer.pop();

		}
		camLock.unlock();
		if (sizeBuf > 0) {
			cvtColor(frame, HSVBlue, COLOR_BGR2HSV);
			HSVYellow = HSVBlue;
			inRange(HSVBlue, Scalar(107, 84, 0), Scalar(146, 256, 256), HSVBlue);
			inRange(HSVYellow, Scalar(25, 76, 176), Scalar(31, 255, 255), HSVYellow);
			addWeighted(HSVBlue, .5, HSVYellow, .5, 0.0, threshold);
			morphOps(threshold);
			trackFilteredObject(x, y, threshold, frame);
			imshow("threshold", threshold);
			imshow("track", frame);
			waitKey(10);
			sizeBuf--;
		}

	}
	if (grabLock.load() == false) {
		cout << "exiting processing thread \n " << endl;
		return 0;
	}
}
string intToString(int number) {


	std::stringstream ss;
	ss << number;
	return ss.str();
}
void drawObject(int x, int y, Mat &frame) {

	//use some of the openCV drawing functions to draw crosshairs
	//on your tracked image!

	//UPDATE:JUNE 18TH, 2013
	//added 'if' and 'else' statements to prevent
	//memory errors from writing off the screen (ie. (-25,-25) is not within the window!)

	circle(frame, Point(x, y), 20, Scalar(0, 255, 0), 2);
	if (y - 25>0)
		line(frame, Point(x, y), Point(x, y - 25), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(x, 0), Scalar(0, 255, 0), 2);
	if (y + 25<FRAME_HEIGHT)
		line(frame, Point(x, y), Point(x, y + 25), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(x, FRAME_HEIGHT), Scalar(0, 255, 0), 2);
	if (x - 25>0)
		line(frame, Point(x, y), Point(x - 25, y), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(0, y), Scalar(0, 255, 0), 2);
	if (x + 25<FRAME_WIDTH)
		line(frame, Point(x, y), Point(x + 25, y), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(FRAME_WIDTH, y), Scalar(0, 255, 0), 2);

	putText(frame, intToString(x) + "," + intToString(y), Point(x, y + 30), 1, 1, Scalar(0, 255, 0), 2);

}
void morphOps(Mat &thresh) {

	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle

	Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3, 3));
	//dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement(MORPH_RECT, Size(8, 8));

	erode(thresh, thresh, erodeElement);
	erode(thresh, thresh, erodeElement);


	dilate(thresh, thresh, dilateElement);
	dilate(thresh, thresh, dilateElement);



}

void trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed) {

	Mat temp;
	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	//use moments method to find our filtered object
	double refArea = 0;
	int largestIndex = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
		//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
		if (numObjects<45) {
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {

				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

				//if the area is less than 15 px by 15px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we save a reference area each
				//iteration and compare it to the area in the next iteration.
				if (area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea) {
					x = moment.m10 / area;
					y = moment.m01 / area;
					objectFound = true;
					refArea = area;
					//save index of largest contour to use with drawContours
					largestIndex = index;
				}
				else objectFound = false;


			}
			//let user know you found an object
			if (objectFound == true) {
				putText(cameraFeed, "Tracking Object", Point(0, 50), 2, 1, Scalar(0, 255, 0), 2);
				//draw object location on screen
				drawObject(x, y, cameraFeed);
			}

		}
		else putText(cameraFeed, "TOO MUCH NOISE! ADJUST FILTER", Point(0, 50), 1, 2, Scalar(0, 0, 255), 2);
	}
}