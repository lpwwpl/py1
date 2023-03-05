//Opencv Eye_In_Hand Calibration 
#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>
// #pragma comment( lib, "opencv_world430.lib" )
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <fstream>
#include <opencv2/imgproc/types_c.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/hpp/rs_types.hpp>
#include <librealsense2/hpp/rs_sensor.hpp>
#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RAD 57.29577951f
using namespace std;
using namespace cv;

Mat QuaternionToMatrix(const Mat& m, bool useQuaternion);
Mat R_T2HomogeneousMatrix(const Mat& R, const Mat& T);
void HomogeneousMtr2RT(Mat& HomoMtr, Mat& R, Mat& T);
bool isRotatedMatrix(Mat& R);
Mat eulerAngleToRotateMatrix(const Mat& eulerAngle, const std::string& seq);
Mat quaternionToRotatedMatrix(const Vec4d& q);
Mat attitudeVectorToMatrix(const Mat& m, bool useQuaternion, const string& seq);
Mat change_mat(Mat &R);
Mat Change_Mat(Mat &R);

void QuaternionToEulerAngles(double qx, double qy, double qz, double qw, double& rrx,double& rry,double& rrz)
{
	double rx, ry, rz;//俯仰角pitch,滚动角roll,航向角yaw
	rx = atan2f(2.f*(qw*qx + qy * qz), 1 - 2 * (qx*qx + qy * qy));
	ry = asinf(2.f*(qw*qy - qz*qx));
	rz = atan2f(2.f*(qw*qz + qx * qy), 1 - 2 * (qy*qy + qz * qz));
	//printf("rz=%lf deg\nry=%lf deg\nrx=%lf deg\n\n", rz*RAD, ry*RAD, rx*RAD);
	rrx = rx*RAD;
	rry = ry*RAD;
	rrz = rz*RAD;

}
void EulerAnglesToQuaternion(  float rz ,float ry,float rx,double& qx,double &qy,double &qz,double &qw)
{
	float x, y, z, w;
	w = cosf(rx*0.5f)*cosf(ry*0.5f)*cosf(rz*0.5f) + sinf(rx*0.5f)*sinf(ry*0.5f)*sinf(rz*0.5f);
	x = sinf(rx*0.5f)*cosf(ry*0.5f)*cosf(rz*0.5f) - cosf(rx*0.5f)*sinf(ry*0.5f)*sinf(rz*0.5f);
	y = cosf(rx*0.5f)*sinf(ry*0.5f)*cosf(rz*0.5f) + sinf(rx*0.5f)*cosf(ry*0.5f)*sinf(rz*0.5f);
	z = cosf(rx*0.5f)*cosf(ry*0.5f)*sinf(rz*0.5f) - sinf(rx*0.5f)*sinf(ry*0.5f)*cosf(rz*0.5f);
	//printf("%f,%f,%f,%f\n\n", x, y, z, w);

    qx = x;
    qy = y;
    qz = z;
    qw = w;
}
/*
int main()
{
   Mat_<double> Tool_Pose_abb = (cv::Mat_<double>(20, 7) <<
		19.784002132482312, -395.8032890167017, 288.6296361174512, 0.0740686764948415, -3.115885908055233, -0.12304546049499021,0,
		44.58661591970266, -379.2128474373975, 247.34781257005412, 0.08351331808583341, -2.975196860853366, 0.04085239274661643,0,
		44.60077369871478, -356.82340082232833, 289.5922902263236, 0.0887676946148333, -3.10108855130095, 0.015199714145750432,0,
		22.2055837423845, -367.6657275982245, 289.57488306632645, 0.08465589534417017, 2.982868456141706, 0.07379791941278338,0,
		68.92165801477529, -486.2578756020861, 289.5713521156477, 0.008193119459214445, -3.078157377456846, -0.5673511259713441,0,
		87.68665499497214, -429.33624258812414, 261.75646460754687, 0.1477484551955931, -3.0895506487463424, -0.145318116931678,0,
		87.6941256249296, -415.0018815125193, 234.82380360169228, 0.2603311866546789, -3.078824406924351, -0.14251985682891685,0,
		87.7084609153225, -448.661347492412, 292.8498877227447, 0.006519481372969573, -3.078535263280345, -0.37417254681221734,0,
		79.9770052922205, -481.1211454080794, 292.8181316919798, 0.6479828784866073, -2.961122271091746, -0.34714260484873444,0,
		113.7023619491926, -506.4949594534295, 342.6441695512624, 0.35253687604230843, -2.918651355049048, -0.4885664622460769,0,
		117.03491545482955, -425.21089011629915, 342.6410527527508, 0.8141759717088761, 2.985802288575681, 0.2675941078555598,0,
		55.99828511406704, -337.99751422907487, 291.20316657296913, 0.30215914274023076, 3.087435346786426, -0.17420275902591836,0,
		141.97430073273543, -380.42311823960134, 279.16062065937186, 0.12161788886640432, -3.0348454535572533, -0.15293902592613376,0,
		130.87132543067057, -356.41802508437936, 315.4987411658488, 0.23466274470322698, -3.022813329572303, -0.051717337141226304,0,
		44.79907069273472, -366.20848017920365, 220.34112241277518, 0.16011901211605092, -3.092715863835859, 0.11413351580204856,0,
		100.72414549098482, -323.23006945666805, 220.36209695857895, 0.5089451774030227, 2.9992014466380343, -0.1954205591486002,0,
		100.47350449380629, -299.7336402628107, 276.38146203324676, 0.5033446924560535, 2.9535311679611844, -0.26149684617218494,0,
		42.265424418073176, -283.32288554833274, 276.35071392731536, 0.34332053775145965, 2.9681899081348138, -0.24952116865428264,0,
		42.261397671940995, -283.3328555479276, 297.41837700643103, 0.532431390219277, 2.9499763189588424, -0.2635611187215266,0,
		38.6086162787127, -341.127312091404, 297.39580423022205, 0.4436307771471771, 2.940182426305739, 0.011924961895736141,0);


	// Robot poses
	Mat_<double> Tool_Pose = (cv::Mat_<double>(20, 6) <<
		19.784002132482312, -395.8032890167017, 288.6296361174512, 0.0740686764948415, -3.115885908055233, -0.12304546049499021,
		44.58661591970266, -379.2128474373975, 247.34781257005412, 0.08351331808583341, -2.975196860853366, 0.04085239274661643,
		44.60077369871478, -356.82340082232833, 289.5922902263236, 0.0887676946148333, -3.10108855130095, 0.015199714145750432,
		22.2055837423845, -367.6657275982245, 289.57488306632645, 0.08465589534417017, 2.982868456141706, 0.07379791941278338,
		68.92165801477529, -486.2578756020861, 289.5713521156477, 0.008193119459214445, -3.078157377456846, -0.5673511259713441,
		87.68665499497214, -429.33624258812414, 261.75646460754687, 0.1477484551955931, -3.0895506487463424, -0.145318116931678,
		87.6941256249296, -415.0018815125193, 234.82380360169228, 0.2603311866546789, -3.078824406924351, -0.14251985682891685,
		87.7084609153225, -448.661347492412, 292.8498877227447, 0.006519481372969573, -3.078535263280345, -0.37417254681221734,
		79.9770052922205, -481.1211454080794, 292.8181316919798, 0.6479828784866073, -2.961122271091746, -0.34714260484873444,
		113.7023619491926, -506.4949594534295, 342.6441695512624, 0.35253687604230843, -2.918651355049048, -0.4885664622460769,
		117.03491545482955, -425.21089011629915, 342.6410527527508, 0.8141759717088761, 2.985802288575681, 0.2675941078555598,
		55.99828511406704, -337.99751422907487, 291.20316657296913, 0.30215914274023076, 3.087435346786426, -0.17420275902591836,
		141.97430073273543, -380.42311823960134, 279.16062065937186, 0.12161788886640432, -3.0348454535572533, -0.15293902592613376,
		130.87132543067057, -356.41802508437936, 315.4987411658488, 0.23466274470322698, -3.022813329572303, -0.051717337141226304,
		44.79907069273472, -366.20848017920365, 220.34112241277518, 0.16011901211605092, -3.092715863835859, 0.11413351580204856,
		100.72414549098482, -323.23006945666805, 220.36209695857895, 0.5089451774030227, 2.9992014466380343, -0.1954205591486002,
		100.47350449380629, -299.7336402628107, 276.38146203324676, 0.5033446924560535, 2.9535311679611844, -0.26149684617218494,
		42.265424418073176, -283.32288554833274, 276.35071392731536, 0.34332053775145965, 2.9681899081348138, -0.24952116865428264,
		42.261397671940995, -283.3328555479276, 297.41837700643103, 0.532431390219277, 2.9499763189588424, -0.2635611187215266,
		38.6086162787127, -341.127312091404, 297.39580423022205, 0.4436307771471771, 2.940182426305739, 0.011924961895736141);
/*
    for (int i=0; i<Tool_Pose_abb.rows; i++)
    {
        Mat m_abb = Tool_Pose_abb.row(i);
        double qx = m_abb.at<double>(3);
        double qy = m_abb.at<double>(4);
        double qz = m_abb.at<double>(5);
        double qw = m_abb.at<double>(6);
        double rx = 0.0f;
        double ry = 0.0f;
        double rz = 0.0f;

        QuaternionToEulerAngles(qx,qy,qz,qw,rx,ry,rz);

        Mat m = Tool_Pose_abb.row(i);
        m.at<double>(3) = rx;
        m.at<double>(4) = ry;
        m.at<double>(5) = rz;
    }

std::cout<<Tool_Pose.rows<<std::endl;
    for (int i=0; i<Tool_Pose.rows; i++)
    {
        Mat m = Tool_Pose.row(i);
        double rx = m.at<double>(3);
        double ry = m.at<double>(4);
        double rz = m.at<double>(5);

        double qx = 0.0f;
        double qy = 0.0f;
        double qz = 0.0f;
        double qw = 0.0f;
        EulerAnglesToQuaternion(rx,ry,rz,qx,qy,qz,qw);

        Mat m_abb = Tool_Pose_abb.row(i);
        m_abb.at<double>(3) = qx;
        m_abb.at<double>(4) = qy;
        m_abb.at<double>(5) = qz;
        m_abb.at<double>(6) = qw;
    }
    std::cout<<Tool_Pose_abb<<std::endl;
    return 1;
}
*/
int main()
{
	//EulerAnglesToQuaternion(30 / RAD, 40 / RAD, 50 / RAD);
	//return 1;

	string dataPath="/home/jihua/catkin_ur/src/OpenCV_Hand_Eye_Cal/pics/";
	ifstream fin(dataPath+"/dataPath.txt"); // image paths

    if(access((dataPath+"calibResults").c_str(),F_OK)==0)	// remove if directory already exists
    {
        system(("rm -rf "+dataPath+"calibResults").c_str());
    }
	system(("mkdir -p "+dataPath+"calibResults").c_str());

	string fileName;      //  File name of an image
	int image_count = 0;  
	Size image_size;      
	Size board_size = Size(7, 9);             // Chess board size Size(7, 9);//8, 11
	int CornerNum = board_size.width * board_size.height;  // Total corner number on chess board

	vector<Mat> images;
	Mat temp3;
	bool rst;

	vector<Point2f> image_points_buf;         // Buffer for image chess board corners
	vector<vector<Point2f>> image_points_seq; // Vector of chess board corner vectors

	vector<Mat> R_gripper2base;
	vector<Mat> T_gripper2base;
	vector<Mat> R_target2cam;
	vector<Mat> T_target2cam;
	Mat rotation_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0));
	Mat R_cam2gripper = Mat(3, 3, CV_64FC1);				// Camera rotation matrix w.r.t gripper
	Mat T_cam2gripper = Mat(3, 1, CV_64FC1);
	Mat Homo_cam2gripper = Mat(4, 4, CV_64FC1);
	Mat T_Matrix= Mat(3, 1, CV_64FC1);
	Mat tempR, tempT, temp;

	vector<Mat> Homo_target2cam;
	vector<Mat> Homo_gripper2base;

	// Chess board info 
	Size square_size = Size(16, 16);         // Square size of chess board
	vector<vector<Point3f>> object_points;   // Vector of chess board corner points

	// Mat cameraMatrix = (cv::Mat_<float>(3, 3) << 614.400696, 0, 316.469086, 0, 612.924072, 238.549133, 0, 0, 1);  // Camera intrinsics
	Mat cameraMatrix = (cv::Mat_<float>(3, 3) << 613.366943, 0, 320.741455,	// Camera intrinsics
												0, 613.469727, 237.568481,
												0, 0, 1);  
	vector<int> point_counts;   // Point counts in each image
	Mat distCoeffs = (cv::Mat_<float>(1, 5) << 0, 0, 0, 0, 0);       // Distortion coefficients k1,k2,p1,p2,k3
	vector<Mat> rvecsMat;      // Rodrigues rotation vector
	Mat rvecsMat_temp;
	vector<Mat> tvecsMat;      // Transition vector
	Mat tvecsMat_temp;

	printf("Processing images");
	while (getline(fin, fileName))
	{

		++image_count;

		Mat imageInput = imread(fileName.c_str(),IMREAD_COLOR);
        //imshow("imageInput", imageInput);
        //waitKey(0);
		// Get image size
		if (image_count == 1)
		{
			image_size.width = imageInput.cols;
			image_size.height = imageInput.rows;
		}

		// Find board corners
		rst=findChessboardCorners(imageInput, board_size, image_points_buf);
		 cout << rst<<endl;
		if (0 == rst)
		{
			cout << "can not find chessboard corners!\n";  
		}
		else
		{
			Mat view_gray;			
			cvtColor(imageInput, view_gray, COLOR_RGB2GRAY);  

			TermCriteria criteria = TermCriteria(TermCriteria::EPS+TermCriteria::COUNT,30,DBL_EPSILON);
			cornerSubPix(view_gray, image_points_buf, Size(5, 5), Size(-1, -1), criteria);
			//find4QuadCornerSubpix(view_gray, image_points_buf, Size(3, 3));
			image_points_seq.push_back(image_points_buf); 

			// Draw chess board corners
			drawChessboardCorners(imageInput, board_size, image_points_buf, rst); 

			putText(imageInput, to_string(image_count), Point(100,100), FONT_HERSHEY_COMPLEX, 0.8, Scalar(0, 0, 255));

			images.push_back(imageInput);
		}
		printf(".");

		fflush(stdout);
	}
	printf("\n");

	// Coordinates of chess board corners in World coordination
	int i, j, t;
	for (t = 0; t < image_count; t++)
	{
		vector<Point3f> tempPointSet;
		for (i = 0; i < board_size.height; i++)
		{
			for (j = 0; j < board_size.width; j++)
			{
				Point3f realPoint;

				realPoint.x = i * square_size.width;
				realPoint.y = j * square_size.height;
				realPoint.z = 0;
				tempPointSet.push_back(realPoint);
			}

		}
		object_points.push_back(tempPointSet);
	}

	// Camera calibration: calculate camera intrinsics and object to camera poses
    cout<<"\n149:"<<endl;
	TermCriteria	criteria = TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, DBL_EPSILON);
	calibrateCamera(object_points, image_points_seq, image_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, CALIB_USE_INTRINSIC_GUESS, criteria);
	cout<<"\n152:"<<endl;

	cout<<"\nDistortion coefficients:"<<endl;
	cout<<distCoeffs<<endl<<endl;
	cout<<"Camera intrinsics:"<<endl;
	cout<<cameraMatrix<<endl<<endl;

	// Calculation of Rotation matrix and Homogeneous matrix
	for (int i = 0; i < rvecsMat.size(); i++)
	{
		Rodrigues(rvecsMat[i], rotation_matrix);
		
		R_target2cam.push_back(rotation_matrix.clone());
		T_target2cam.push_back(tvecsMat[i].clone());

		temp = R_T2HomogeneousMatrix(R_target2cam[i], T_target2cam[i]);
		Homo_target2cam.push_back(temp.clone());
	}

	// Draw the chess board World Coordinates on images and show them
	Mat axisXYZ,temp1,temp2;
	Point p0,p1;
	axisXYZ = (cv::Mat_<double>(4, 4) << 0, 75, 0, 0,	// 75 mm
										0, 0, 75, 0,
										0, 0, 0, 75,
										1, 1, 1, 1);
	for(int i=0; i<images.size(); i++)
	{		
		temp1=cv::Mat_<double>(cameraMatrix)*Homo_target2cam[i].rowRange(0,3)*axisXYZ;
		temp1.colRange(0,1)=temp1.colRange(0,1)/temp1.at<double>(2,0);
		temp1.colRange(1,2)=temp1.colRange(1,2)/temp1.at<double>(2,1);
		temp1.colRange(2,3)=temp1.colRange(2,3)/temp1.at<double>(2,2);
		temp1.colRange(3,4)=temp1.colRange(3,4)/temp1.at<double>(2,3);

		temp2=images[i].clone();
		cv::undistort(temp2,images[i],cameraMatrix,distCoeffs);

		p0=cv::Point(int(temp1.at<double>(0,0)),int(temp1.at<double>(1,0)));
		p1=cv::Point(int(temp1.at<double>(0,1)),int(temp1.at<double>(1,1)));
		line(images[i],p0,p1,cv::Scalar(0,0,255),2,4);	//draw X axis (in red)
		p1=cv::Point(int(temp1.at<double>(0,2)),int(temp1.at<double>(1,2)));
		line(images[i],p0,p1,cv::Scalar(0,255,0),2,4);	//draw Y axis (in green)
		p1=cv::Point(int(temp1.at<double>(0,3)),int(temp1.at<double>(1,3)));
		line(images[i],p0,p1,cv::Scalar(255,0,0),2,4);	//draw Z axis (in blue)


		cv::resize(images[i],temp2, Size(), 0.75, 0.75, INTER_LINEAR);
		imshow("Draw axis", temp2);
		waitKey(0);
	}

   Mat_<double> Tool_Pose_abb = (cv::Mat_<double>(24, 7));


	// Robot poses
	Mat_<double> Tool_Pose = (cv::Mat_<double>(24, 6) <<
		19.784002132482312, -395.8032890167017, 388.6296361174512, 0.0740686764948415, -3.115885908055233, -0.12304546049499021,
		44.60077369871478, -356.82340082232833, 389.5922902263236, 0.0887676946148333, -3.10108855130095, 0.015199714145750432,
		22.2055837423845, -367.6657275982245, 389.57488306632645, 0.08465589534417017, 2.982868456141706, 0.07379791941278338,
		68.92165801477529, -486.2578756020861, 389.5713521156477, 0.008193119459214445, -3.078157377456846, -0.5673511259713441,
		87.68665499497214, -429.33624258812414, 361.75646460754687, 0.1477484551955931, -3.0895506487463424, -0.145318116931678,
		87.6941256249296, -415.0018815125193, 334.82380360169228, 0.2603311866546789, -3.078824406924351, -0.14251985682891685,
		87.7084609153225, -448.661347492412, 392.8498877227447, 0.006519481372969573, -3.078535263280345, -0.37417254681221734,
		130.87132543067057, -356.41802508437936, 415.4987411658488, 0.23466274470322698, -3.022813329572303, -0.051717337141226304,
		19.784002132482312, -395.8032890167017, 388.6296361174512, 0.0740686764948415, -3.115885908055233, -0.12304546049499021,
		44.60077369871478, -356.82340082232833, 389.5922902263236, 0.0887676946148333, -3.10108855130095, 0.015199714145750432,
		22.2055837423845, -367.6657275982245, 389.57488306632645, 0.08465589534417017, 2.982868456141706, 0.07379791941278338,
		68.92165801477529, -486.2578756020861, 389.5713521156477, 0.008193119459214445, -3.078157377456846, -0.5673511259713441,
		87.68665499497214, -429.33624258812414, 361.75646460754687, 0.1477484551955931, -3.0895506487463424, -0.145318116931678,
		87.6941256249296, -415.0018815125193, 334.82380360169228, 0.2603311866546789, -3.078824406924351, -0.14251985682891685,
		87.7084609153225, -448.661347492412, 392.8498877227447, 0.006519481372969573, -3.078535263280345, -0.37417254681221734,
		130.87132543067057, -356.41802508437936, 415.4987411658488, 0.23466274470322698, -3.022813329572303, -0.051717337141226304,
		19.784002132482312, -395.8032890167017, 388.6296361174512, 0.0740686764948415, -3.115885908055233, -0.12304546049499021,
		44.60077369871478, -356.82340082232833, 389.5922902263236, 0.0887676946148333, -3.10108855130095, 0.015199714145750432,
		22.2055837423845, -367.6657275982245, 389.57488306632645, 0.08465589534417017, 2.982868456141706, 0.07379791941278338,
		68.92165801477529, -486.2578756020861, 389.5713521156477, 0.008193119459214445, -3.078157377456846, -0.5673511259713441,
		87.68665499497214, -429.33624258812414, 361.75646460754687, 0.1477484551955931, -3.0895506487463424, -0.145318116931678,
		87.6941256249296, -415.0018815125193, 334.82380360169228, 0.2603311866546789, -3.078824406924351, -0.14251985682891685,
		87.7084609153225, -448.661347492412, 392.8498877227447, 0.006519481372969573, -3.078535263280345, -0.37417254681221734,
		130.87132543067057, -356.41802508437936, 415.4987411658488, 0.23466274470322698, -3.022813329572303, -0.051717337141226304);
/*
    for (int i=0; i<Tool_Pose_abb.rows; i++)
    {
        Mat m_abb = Tool_Pose_abb.row(i);
        double qx = m_abb.at<double>(3);
        double qy = m_abb.at<double>(4);
        double qz = m_abb.at<double>(5);
        double qw = m_abb.at<double>(6);
        double rx = 0.0f;
        double ry = 0.0f;
        double rz = 0.0f;

        QuaternionToEulerAngles(qx,qy,qz,qw,rx,ry,rz);

        Mat m = Tool_Pose_abb.row(i);
        m.at<double>(3) = rx;
        m.at<double>(4) = ry;
        m.at<double>(5) = rz;
    }
*/

    for (int i=0; i<Tool_Pose.rows; i++)
    {
        Mat m = Tool_Pose.row(i);
        double rx = m.at<double>(3);
        double ry = m.at<double>(4);
        double rz = m.at<double>(5);

        double qx = 0.0f;
        double qy = 0.0f;
        double qz = 0.0f;
        double qw = 0.0f;
        EulerAnglesToQuaternion(rx,ry,rz,qx,qy,qz,qw);

        Mat m_abb = Tool_Pose_abb.row(i);
        m_abb.at<double>(3) = qx;
        m_abb.at<double>(4) = qy;
        m_abb.at<double>(5) = qz;
        m_abb.at<double>(6) = qw;
    }
    std::cout<<Tool_Pose_abb<<std::endl;

	Mat ToolPose = change_mat(Tool_Pose);

	// Calculate gripper to base Rotation and Transition matrixes
	for (int j = 0; j < ToolPose.rows; j++)
	{
		temp = attitudeVectorToMatrix(ToolPose.row(j), false,"xyz");
		Homo_gripper2base.push_back(temp.clone());
		
		HomogeneousMtr2RT(temp, tempR, tempT);
		R_gripper2base.push_back(tempR.clone());
		T_gripper2base.push_back(tempT.clone());
	}

	// Eye in hand calibration
	calibrateHandEye(R_gripper2base, T_gripper2base, R_target2cam, T_target2cam, R_cam2gripper, T_cam2gripper, CALIB_HAND_EYE_TSAI);//CALIB_HAND_EYE_TSAI  CALIB_HAND_EYE_PARK CALIB_HAND_EYE_HORAUD CALIB_HAND_EYE_ANDREFF

	Homo_cam2gripper = R_T2HomogeneousMatrix(R_cam2gripper, T_cam2gripper);

	// Print Opencv results
	Mat Homo_gripper2cam=Homo_cam2gripper.inv();
	Mat R_gripper2cam,T_gripper2cam,Rg_gripper2cam;
	HomogeneousMtr2RT(Homo_gripper2cam,R_gripper2cam,T_gripper2cam);
	Rodrigues(R_gripper2cam,Rg_gripper2cam);

	cout<<"OpenCV rodrigues vector - gripper to camera (in degree):"<<endl;
	cout<<Rg_gripper2cam/CV_PI*180<<endl<<endl;
	cout<<"OpenCV transition vector - gripper to camera (in mm):"<<endl;
	cout<<T_gripper2cam<<endl;

	// Print Halcon results
	Mat Homo_gripper2cam_halcon=(cv::Mat_<double>(4,4)<<0.926111,0.377251,-0.000286293,0.0346663,
														-0.372626,0.914636,-0.156813,0.0922973,
														-0.0588962,0.145333,0.987628,-0.0311515,
														0,0,0,1);
	Mat R_gripper2cam_halcon,T_gripper2cam_halcon,Rg_gripper2cam_halcon;
	HomogeneousMtr2RT(Homo_gripper2cam_halcon,R_gripper2cam_halcon,T_gripper2cam_halcon);
	Rodrigues(R_gripper2cam_halcon,Rg_gripper2cam_halcon);

	cout<<"\nHalcon rodrigues vector - gripper to camera (in degree):"<<endl;
	cout<<Rg_gripper2cam_halcon/CV_PI*180<<endl<<endl;
	cout<<"Halcon transition vector - gripper to camera (in mm):"<<endl;
	cout<<T_gripper2cam_halcon*1000<<endl;

	// Save results to file
	FILE *fp = fopen((dataPath+"calibResults/calibration_result.txt").c_str(), "w");
    fprintf(fp, "# Color camera intrinsic matrix\n");	// Save camera intrinsics of color sensor
    for (int i = 0; i < 3; ++i)
        fprintf(fp, "%14.8e\t%14.8e\t%14.8e\n", cameraMatrix.at<float>(i,0),cameraMatrix.at<float>(i,1), cameraMatrix.at<float>(i,2));

	fprintf(fp, "\n# Distortion coefficients [k1,k2,p1,p2,k3]\n");	// Save distortion coefficients
    fprintf(fp, "%14.8e\t%14.8e\t%14.8e\t%14.8e\t%14.8e\n", distCoeffs.at<float>(0,0),distCoeffs.at<float>(0,1),distCoeffs.at<float>(0,2),distCoeffs.at<float>(0,3),distCoeffs.at<float>(0,4));

	fprintf(fp, "\n# Homogeneous matrix camera to gripper\n");	// Save homogeneous matrix camera to gripper
	for (int i = 0; i < 4; ++i)
        fprintf(fp, "%14.8e\t%14.8e\t%14.8e\t%14.8e\n", Homo_cam2gripper.at<double>(i,0),Homo_cam2gripper.at<double>(i,1),Homo_cam2gripper.at<double>(i,2),Homo_cam2gripper.at<double>(i,3));	
	fclose(fp);

	for(int i=0;i<images.size();i++)
	{	
		string str=format("calibResults/Color_%d.png",i+1);
		imwrite(dataPath+str,images[i]);
	}

	return 0;
}

Mat change_mat(Mat &R)
{
	for (int i = 0; i < R.rows; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			R.at<double>(i, j) = 1 * R.at<double>(i, j);

		}
		for(int j=3;j<R.cols;j++)
		{
		    R.at<double>(i,j)=180/CV_PI*R.at<double>(i,j);
		}
	}
	return R;
}

Mat Change_Mat(Mat &R)
{
	for (int i = 0; i < R.rows-1; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			if (j == 3)
			{
			  R.at<double>(i, j) = 1000 * R.at<double>(i, j);
			}	
		}
	
	}
	return R;
}
bool isRotatedMatrix(Mat& R)		//旋转矩阵的转置矩阵是它的逆矩阵，逆矩阵 * 矩阵 = 单位矩阵
{
	Mat temp33 = R({ 0,0,3,3 });	//无论输入是几阶矩阵，均提取它的三阶矩阵
	Mat Rt;
	transpose(temp33, Rt);  //转置矩阵
	Mat shouldBeIdentity = Rt * temp33;//是旋转矩阵则乘积为单位矩阵
	Mat I = Mat::eye(3, 3, shouldBeIdentity.type());

	return cv::norm(I, shouldBeIdentity) < 1e-6;
}

Mat attitudeVectorToMatrix(const Mat& m, bool useQuaternion, const string& seq)
{
	CV_Assert(m.total() == 6 || m.total() == 10);

	Mat temp = Mat::eye(4, 4, CV_64FC1);

	if (useQuaternion)
	{
		Vec4d quaternionVec = m({ 3,0,4,1 });   //读取存储的四元数
		quaternionToRotatedMatrix(quaternionVec).copyTo(temp({ 0,0,3,3 }));
	}
	else
	{
		Mat rotVec;
		if (m.total() == 6)
		{
			rotVec = m({ 3,0,3,1 });//读取存储的欧拉角

			//cout<<"欧拉角为:"<<rotVec<<endl<<endl;
		}
		if (m.total() == 10)
		{
			rotVec = m({ 7,0,3,1 });
		}
		//如果seq为空，表示传入的是3*1旋转向量，否则，传入的是欧拉角
		if (0 == seq.compare(""))
		{
			Rodrigues(rotVec, temp({ 0,0,3,3 }));   //罗德利斯转换
		}
		else
		{
			eulerAngleToRotateMatrix(rotVec, seq).copyTo(temp({ 0,0,3,3 }));
		}
	}
	//存入平移矩阵
	temp({ 3,0,1,3 }) = m({ 0,0,3,1 }).t();
	return temp;   //返回转换结束的齐次矩阵
}

Mat QuaternionToMatrix(const Mat& m, bool useQuaternion)//四元素转换为矩阵
{
	CV_Assert(m.total() == 7);
	Mat temp = Mat::eye(4, 4, CV_64FC1);
	if (useQuaternion)
	{
		Vec4d quaternionVec = m({ 3,0,4,1 });   //读取存储的四元数
		quaternionToRotatedMatrix(quaternionVec).copyTo(temp({ 0,0,3,3 }));
	}

	temp({ 3,0,1,3 }) = m({ 0,0,3,1 }).t();
	return temp;   //返回转换结束的齐次矩阵
}

Mat quaternionToRotatedMatrix(const Vec4d& q)
{
	double q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

	double q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;
	double q0q1 = q0 * q1, q0q2 = q0 * q2, q0q3 = q0 * q3;
	double q1q2 = q1 * q2, q1q3 = q1 * q3;
	double q2q3 = q2 * q3;
	//根据公式得来
	Mat RotMtr = (Mat_<double>(3, 3) << (q0q0 + q1q1 - q2q2 - q3q3), 2 * (q1q2 + q0q3), 2 * (q1q3 - q0q2),
		2 * (q1q2 - q0q3), (q0q0 - q1q1 + q2q2 - q3q3), 2 * (q2q3 + q0q1),
		2 * (q1q3 + q0q2), 2 * (q2q3 - q0q1), (q0q0 - q1q1 - q2q2 + q3q3));

	return RotMtr;
}


/*
double x, y, z, w;
	double Φ, θ, Ψ;
	int a;
	while (1)
	{
		printf("【1:四元素->欧拉角，2:欧拉角->四元素】->>>");
		scanf("%d",&a);
		switch (a)
		{
		case 1:
			scanf("%lf,%lf,%lf,%lf",&x,&y,&z,&w);
			QuaternionToEulerAngles(x,y,z,w);
			break;
		case 2:
			scanf("%lf %lf %lf", &Ψ, &θ, &Φ);
			EulerAnglesToQuaternion(Ψ / RAD, θ / RAD, Φ / RAD);
			break;
		default:
			break;
		}
	}
	*/

Mat eulerAngleToRotateMatrix(const Mat& eulerAngle, const std::string& seq)
{
	CV_Assert(eulerAngle.rows == 1 && eulerAngle.cols == 3);//检查参数是否正确

	eulerAngle /= (180 / CV_PI);		//度转弧度

	Matx13d m(eulerAngle);				//<double, 1, 3>

	auto rx = m(0, 0), ry = m(0, 1), rz = m(0, 2);
	auto rxs = sin(rx), rxc = cos(rx);
	auto rys = sin(ry), ryc = cos(ry);
	auto rzs = sin(rz), rzc = cos(rz);

	//XYZ方向的旋转矩阵
	Mat RotX = (Mat_<double>(3, 3) << 1, 0, 0,
		0, rxc, -rxs,
		0, rxs, rxc);
	Mat RotY = (Mat_<double>(3, 3) << ryc, 0, rys,
		0, 1, 0,
		-rys, 0, ryc);
	Mat RotZ = (Mat_<double>(3, 3) << rzc, -rzs, 0,
		rzs, rzc, 0,
		0, 0, 1);
	//按顺序合成后的旋转矩阵
	cv::Mat rotMat;

	if (seq == "zyx") rotMat = RotX * RotY * RotZ;
	else if (seq == "yzx") rotMat = RotX * RotZ * RotY;
	else if (seq == "zxy") rotMat = RotY * RotX * RotZ;
	else if (seq == "yxz") rotMat = RotZ * RotX * RotY;
	else if (seq == "xyz") rotMat = RotZ * RotY * RotX;
	else if (seq == "xzy") rotMat = RotY * RotZ * RotX;
	else
	{
		cout << "Euler Angle Sequence string is wrong...";
	}
	if (!isRotatedMatrix(rotMat))		//欧拉角特殊情况下会出现死锁
	{
		cout << "Euler Angle convert to RotatedMatrix failed..." << endl;
		exit(-1);
	}
	return rotMat;
}

void HomogeneousMtr2RT(Mat& HomoMtr, Mat& R, Mat& T)
{
	Rect R_rect(0, 0, 3, 3);
	Rect T_rect(3, 0, 1, 3);
	R = HomoMtr(R_rect);
	T = HomoMtr(T_rect);
}


Mat R_T2HomogeneousMatrix(const Mat& R, const Mat& T)
{
	Mat HomoMtr;
	Mat_<double> R1 = (Mat_<double>(4, 3) <<
		R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
		R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
		R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2),
		0, 0, 0);
	Mat_<double> T1 = (Mat_<double>(4, 1) <<
		T.at<double>(0, 0),
		T.at<double>(1, 0),
		T.at<double>(2, 0),
		1);
	cv::hconcat(R1, T1, HomoMtr);		//矩阵拼接
	return HomoMtr;
}

