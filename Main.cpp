// OpenCV includes
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// System includes
#include <iostream>
#include <chrono>

// Project includes
#include "main.h"
#include "Runtime.h"

typedef std::chrono::high_resolution_clock Clock;
using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
	VideoCapture cap(CAMERA_COM);

	if (!cap.isOpened())
	{
		cout << "Cannot open camera!";
		return -1;
	}

#if TIMED_DEBUG
	//TESTING BLOCK
	auto t1 = Clock::now();
#endif

	Runtime mainExecution(cap);
	mainExecution.MainLoop();

#if TIMED_DEBUG
	auto t2 = Clock::now();
	std::cout << "Delta t2-t1: "
		<< std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count()
		<< " nanoseconds" << std::endl;
	// END OF TESTING BLOCK
#endif

	return 0;
}