#pragma once

// OpenCV Includes
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Project Includes
#include "Runtime.h"
#include "Main.h"

using namespace std;
using namespace cv;

// Initiallizers
Runtime::Runtime()
{

}

Runtime::Runtime(VideoCapture aCap)
{
	mCap = aCap;
}

// Destructor
Runtime::~Runtime()
{
	mCap.~VideoCapture();
}

// Getters
VideoCapture Runtime::GetCap()
{
	return mCap;
}

// Setters
void Runtime::SetCap(VideoCapture aCap)
{
	mCap = aCap;
}

// Member Functions

// Takes in an image and returns a binary image where the red from the original image is outlined
Mat Runtime::DetectRed(Mat aImg)
{
	Mat img, lowerRedOutlineFrame, upperRedOutlineFrame;

	GaussianBlur(aImg, img, DETECT_RED_BLURR_KERNEL_SIZE, DETECT_RED_BLURR_STD_DIV);
	cvtColor(img, img, COLOR_BGR2HSV);
	inRange(img, DETECT_RED_HSV_LOWER_RED_LOWER_BOUNDARY, DETECT_RED_HSV_LOWER_RED_UPPER_BOUNDARY, lowerRedOutlineFrame);
	inRange(img, DETECT_RED_HSV_UPPER_RED_LOWER_BOUNDARY, DETECT_RED_HSV_UPPER_RED_UPPER_BOUNDARY, upperRedOutlineFrame);
	addWeighted(lowerRedOutlineFrame, DETECT_RED_WA_ARRAY_1_WEIGHT, upperRedOutlineFrame, DETECT_RED_WA_ARRAY_2_WEIGHT, DETECT_RED_WA_SCALAR, img);
	Canny(img, img, DETECT_RED_CANNY_THRESHOLD_1, DETECT_RED_CANNY_THRESHOLD_2);

	return img;
}

// Where everything is done lol #goodArchitecture
void Runtime::MainLoop()
{
	while (true)
	{
		Mat cameraFrame, redOutlineFrame, mainROI_M, centerROI_M;

		mCap.read(cameraFrame);

		Rect mainROI_R((int)((cameraFrame.cols / 2) - ((cameraFrame.cols / 2) / ROI_M_HORI)), (int)((cameraFrame.rows / 2) - ((cameraFrame.rows / 2) / ROI_M_VERT)), (int)(cameraFrame.cols / ROI_M_HORI), (int)(cameraFrame.rows / ROI_M_VERT)),
			 centerROI_R((int)((cameraFrame.cols / 2) - ((cameraFrame.cols / 2) / ROI_C_HORI)), (int)((cameraFrame.rows / 2) - ((cameraFrame.rows / 2) / ROI_C_VERT)), (int)(cameraFrame.cols / ROI_C_HORI), (int)(cameraFrame.rows / ROI_C_VERT));

		mainROI_M = cameraFrame(mainROI_R);
		centerROI_M = cameraFrame(centerROI_R);

#if DEBUG
		Mat detectedLines;
		detectedLines = Mat::zeros(mainROI_M.rows, mainROI_M.cols, CV_64F);
#endif

		redOutlineFrame = DetectRed(mainROI_M);
		//redOutlineFrame = Mat::zeros(mainROI_M.rows, mainROI_M.cols, CV_8U);

		vector<Vec2f> lines; // Array of detected lines, used to determine the angle of the line

		int iCount = 0;
		double iSum = 0, iAvg = 0;
		uchar* M = redOutlineFrame.data;

		for (int i = 0; i < redOutlineFrame.rows; i++)
		{
			int jCount = 0;
			double jSum = 0;

			for (int j = 0; j < redOutlineFrame.cols; j++)
			{
				if (M[i*redOutlineFrame.step + j] > 0)
				{
					jSum += j;
					jCount++;
				}
			}

			if (jCount > 0)
			{
				iSum += (jSum / jCount);
				iCount++;
				//cout << "i: " << i << " Row Avg: " << (jSum / jCount) << endl;
			}
		}

		iAvg = (iSum / iCount);

		// TO DO: use this value to set the roll setpoint

		HoughLines(redOutlineFrame, lines, DETECT_LINE_LENGTH_RES, DETECT_LINE_ANGLE_RES, DETECT_LINE_THRESHOLD);

		int count = 0;
		double rhoAvg = 0, rhoSum = 0, thetaAvg = -1, thetaSum = 0;
		Point centerOfLine;

		for (size_t i = 0; i < lines.size(); i++)
		{
			double rho = lines[i][0], theta = lines[i][1];

#if DEBUG
			Point pt1, pt2;
			double a = cos(theta), b = sin(theta);
			double x0 = a*rho, y0 = b*rho;
			pt1.x = cvRound(x0 + 1000 * (-b));
			pt1.y = cvRound(y0 + 1000 * (a));
			pt2.x = cvRound(x0 - 1000 * (-b));
			pt2.y = cvRound(y0 - 1000 * (a));
			line(detectedLines, pt1, pt2, Scalar(255, 255, 255), 3, CV_AA);
#endif

			// Converts the angle into 0 being left, PI/2 being straight, and PI being right
			// This math breaks if the yaw is off by more than PI/2 compared to the line
			if (theta > CV_PI / 2)
			{
				theta = abs(CV_PI - theta) + CV_PI / 2;
			}
			else
			{
				theta = abs(theta - CV_PI / 2);
			}

			thetaSum += theta;
			rhoSum += rho;
			count++;
		}

		if (count > 0)
		{
			rhoAvg = rhoSum / count;
			thetaAvg = thetaSum / count;
		}

#if DEBUG
		//cout //<< "Angle: " << thetaAvg << endl
			 //<< "iAvg: " << iAvg << endl;
#endif

		// TO DO: use this angle (theta) to calculate a yaw setpoint...

#if DEBUG
		if (thetaAvg >= 0)
		{
			double a = (50 * cos(thetaAvg));
			double b = (50 * sin(thetaAvg));

			Point averageLineBot(cvRound(mainROI_R.x + (iAvg) - (50 * cos(thetaAvg))), cvRound(mainROI_R.y + (mainROI_R.height / 2) - (50 * sin(-thetaAvg)))),
				  averageLineTop(cvRound(mainROI_R.x + (iAvg) + (50 * cos(thetaAvg))), cvRound(mainROI_R.y + (mainROI_R.height / 2) + (50 * sin(-thetaAvg))));

			arrowedLine(cameraFrame, averageLineBot, averageLineTop, Scalar(0, 0, 255), 2);
		}

		rectangle(cameraFrame, mainROI_R, Scalar(255, 0, 0), 2);
		rectangle(cameraFrame, centerROI_R, Scalar(0, 255, 0), 2);

		imshow("Captured Image", cameraFrame);
		imshow("Red Outlines", redOutlineFrame);
		imshow("Detected Lines", detectedLines);
#endif

		if (waitKey(MAIN_LOOP_WAIT_TIME) >= 0)
		{
			break;
		}
	}
}