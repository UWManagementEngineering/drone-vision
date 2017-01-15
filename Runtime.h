#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

// MAIN LOOP CONSTANTS
const int MAIN_LOOP_WAIT_TIME = 30;

// REGINE OF INTEREST CONSTANTS: used to divide the area of calculations. Larger # = Smaller area
	// MAIN REGION OF INTEREST
	const double ROI_M_VERT = 3;
	const double ROI_M_HORI = 3;

	// CENTER REGION OF INTEREST
	const double ROI_C_VERT = 5;
	const double ROI_C_HORI = 5;

// RED DETECTION CONSTANTS
	// GAUSIAN BLURR
	const Size DETECT_RED_BLURR_KERNEL_SIZE = Size(7, 7);
	const double DETECT_RED_BLURR_STD_DIV = 3;

	// HSV THRESHOLDING
	const Scalar DETECT_RED_HSV_LOWER_RED_LOWER_BOUNDARY = Scalar(0, 100, 100);
	const Scalar DETECT_RED_HSV_LOWER_RED_UPPER_BOUNDARY = Scalar(10, 255, 255);
	const Scalar DETECT_RED_HSV_UPPER_RED_LOWER_BOUNDARY = Scalar(160, 100, 100);
	const Scalar DETECT_RED_HSV_UPPER_RED_UPPER_BOUNDARY = Scalar(179, 255, 255);

	// WEIGHTED ADDITION
	const double DETECT_RED_WA_ARRAY_1_WEIGHT = 1.0;
	const double DETECT_RED_WA_ARRAY_2_WEIGHT = 1.0;
	const double DETECT_RED_WA_SCALAR = 0;

	// CANNY EDGE DETECTION
	const double DETECT_RED_CANNY_THRESHOLD_1 = 1;
	const double DETECT_RED_CANNY_THRESHOLD_2 = 0;

// LINEDETECTION CONSTANTS
const int DETECT_LINE_THRESHOLD = 25;
const double DETECT_LINE_LENGTH_RES = 1; // 1 pixel resolution
const double DETECT_LINE_ANGLE_RES = CV_PI / 180; // 1 degree resolution
const double DETECT_LINE_SAME_LINE_TOLORANCE = CV_PI / 9; // the # of degrees within which lines will be considered the same

class Runtime
{
private:
	VideoCapture mCap;

public:
	Runtime();
	Runtime(VideoCapture);
	~Runtime();
	VideoCapture GetCap();
	void SetCap(VideoCapture);
	void MainLoop();
	Mat DetectRed(Mat);
};