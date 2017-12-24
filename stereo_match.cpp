/*
 *  stereo_match.cpp
 *  calibration
 *
 *  Created by Victor  Eruhimov on 1/18/10.
 *  Copyright 2010 Argus Corp. All rights reserved.
 *
 */

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"

#pragma comment(lib,"opencv_core2413d.lib")	//顯示圖片使用
#pragma comment(lib,"opencv_highgui2413d.lib") 
#pragma comment(lib,"opencv_imgproc2413d.lib")
//#pragma comment(lib,"opencv_video2413d.lib")
#pragma comment(lib,"opencv_legacy2413d.lib")
//#pragma comment(lib,"opencv_objdetect2413d.lib")
#pragma comment(lib,"opencv_calib3d2413d.lib")
//#pragma comment(lib,"opencv_features2d2413d.lib")
#pragma comment(lib,"opencv_contrib2413d.lib")

#include <stdio.h>
#include <iostream>
using namespace cv;

static void print_help()
{
    printf("\nDemo stereo matching converting L and R images into disparity and point clouds\n");
    printf("\nUsage: stereo_match <left_image> <right_image> [--algorithm=bm|sgbm|hh|var] [--blocksize=<block_size>]\n"
           "[--max-disparity=<max_disparity>] [--scale=scale_factor>] [-i <intrinsic_filename>] [-e <extrinsic_filename>]\n"
           "[--no-display] [-o <disparity_image>] [-p <point_cloud_file>]\n");
}

static void saveXYZ(const char* filename, const Mat& mat)
{
    const double max_z = 1.0e4;
    FILE* fp = fopen(filename, "wt");
    for(int y = 0; y < mat.rows; y++)
    {
        for(int x = 0; x < mat.cols; x++)
        {
            Vec3f point = mat.at<Vec3f>(y, x);
            if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
            fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
        }
    }
    fclose(fp);
}

//int main(int argc, char** argv)
//{
//    const char* algorithm_opt = "--algorithm=";
//    const char* maxdisp_opt = "--max-disparity=";
//    const char* blocksize_opt = "--blocksize=";
//    const char* nodisplay_opt = "--no-display";
//    const char* scale_opt = "--scale=";
//
//    if(argc < 3)
//    {
//        print_help();
//        return 0;
//    }
//    const char* img1_filename = 0;
//    const char* img2_filename = 0;
//    const char* intrinsic_filename = 0;
//    const char* extrinsic_filename = 0;
//    const char* disparity_filename = 0;
//    const char* point_cloud_filename = 0;
//
//    enum { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_VAR=3 };
//    int alg = STEREO_SGBM;
//    int SADWindowSize = 0, numberOfDisparities = 0;
//    bool no_display = false;
//    float scale = 1.f;
//
//    StereoBM bm;
//    StereoSGBM sgbm;
//    StereoVar var;
//
//    for( int i = 1; i < argc; i++ )
//    {
//        if( argv[i][0] != '-' )
//        {
//            if( !img1_filename )
//                img1_filename = argv[i];
//            else
//                img2_filename = argv[i];
//        }
//        else if( strncmp(argv[i], algorithm_opt, strlen(algorithm_opt)) == 0 )
//        {
//            char* _alg = argv[i] + strlen(algorithm_opt);
//            alg = strcmp(_alg, "bm") == 0 ? STEREO_BM :
//                  strcmp(_alg, "sgbm") == 0 ? STEREO_SGBM :
//                  strcmp(_alg, "hh") == 0 ? STEREO_HH :
//                  strcmp(_alg, "var") == 0 ? STEREO_VAR : -1;
//            if( alg < 0 )
//            {
//                printf("Command-line parameter error: Unknown stereo algorithm\n\n");
//                print_help();
//                return -1;
//            }
//        }
//        else if( strncmp(argv[i], maxdisp_opt, strlen(maxdisp_opt)) == 0 )
//        {
//            if( sscanf( argv[i] + strlen(maxdisp_opt), "%d", &numberOfDisparities ) != 1 ||
//                numberOfDisparities < 1 || numberOfDisparities % 16 != 0 )
//            {
//                printf("Command-line parameter error: The max disparity (--maxdisparity=<...>) must be a positive integer divisible by 16\n");
//                print_help();
//                return -1;
//            }
//        }
//        else if( strncmp(argv[i], blocksize_opt, strlen(blocksize_opt)) == 0 )
//        {
//            if( sscanf( argv[i] + strlen(blocksize_opt), "%d", &SADWindowSize ) != 1 ||
//                SADWindowSize < 1 || SADWindowSize % 2 != 1 )
//            {
//                printf("Command-line parameter error: The block size (--blocksize=<...>) must be a positive odd number\n");
//                return -1;
//            }
//        }
//        else if( strncmp(argv[i], scale_opt, strlen(scale_opt)) == 0 )
//        {
//            if( sscanf( argv[i] + strlen(scale_opt), "%f", &scale ) != 1 || scale < 0 )
//            {
//                printf("Command-line parameter error: The scale factor (--scale=<...>) must be a positive floating-point number\n");
//                return -1;
//            }
//        }
//        else if( strcmp(argv[i], nodisplay_opt) == 0 )
//            no_display = true;
//        else if( strcmp(argv[i], "-i" ) == 0 )
//            intrinsic_filename = argv[++i];
//        else if( strcmp(argv[i], "-e" ) == 0 )
//            extrinsic_filename = argv[++i];
//        else if( strcmp(argv[i], "-o" ) == 0 )
//            disparity_filename = argv[++i];
//        else if( strcmp(argv[i], "-p" ) == 0 )
//            point_cloud_filename = argv[++i];
//        else
//        {
//            printf("Command-line parameter error: unknown option %s\n", argv[i]);
//            return -1;
//        }
//    }
//
//    if( !img1_filename || !img2_filename )
//    {
//        printf("Command-line parameter error: both left and right images must be specified\n");
//        return -1;
//    }
//
//    if( (intrinsic_filename != 0) ^ (extrinsic_filename != 0) )
//    {
//        printf("Command-line parameter error: either both intrinsic and extrinsic parameters must be specified, or none of them (when the stereo pair is already rectified)\n");
//        return -1;
//    }
//
//    if( extrinsic_filename == 0 && point_cloud_filename )
//    {
//        printf("Command-line parameter error: extrinsic and intrinsic parameters must be specified to compute the point cloud\n");
//        return -1;
//    }
//
//    int color_mode = alg == STEREO_BM ? 0 : -1;
//    Mat img1 = imread(img1_filename, color_mode);
//    Mat img2 = imread(img2_filename, color_mode);
//
//    if (img1.empty())
//    {
//        printf("Command-line parameter error: could not load the first input image file\n");
//        return -1;
//    }
//    if (img2.empty())
//    {
//        printf("Command-line parameter error: could not load the second input image file\n");
//        return -1;
//    }
//
//    if (scale != 1.f)
//    {
//        Mat temp1, temp2;
//        int method = scale < 1 ? INTER_AREA : INTER_CUBIC;
//        resize(img1, temp1, Size(), scale, scale, method);
//        img1 = temp1;
//        resize(img2, temp2, Size(), scale, scale, method);
//        img2 = temp2;
//    }
//
//    Size img_size = img1.size();
//
//    Rect roi1, roi2;
//    Mat Q;
//
//    if( intrinsic_filename )
//    {
//        // reading intrinsic parameters
//        FileStorage fs(intrinsic_filename, CV_STORAGE_READ);
//        if(!fs.isOpened())
//        {
//            printf("Failed to open file %s\n", intrinsic_filename);
//            return -1;
//        }
//
//        Mat M1, D1, M2, D2;
//        fs["M1"] >> M1;
//        fs["D1"] >> D1;
//        fs["M2"] >> M2;
//        fs["D2"] >> D2;
//
//        M1 *= scale;
//        M2 *= scale;
//
//        fs.open(extrinsic_filename, CV_STORAGE_READ);
//        if(!fs.isOpened())
//        {
//            printf("Failed to open file %s\n", extrinsic_filename);
//            return -1;
//        }
//
//        Mat R, T, R1, P1, R2, P2;
//        fs["R"] >> R;
//        fs["T"] >> T;
//
//        stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );
//
//        Mat map11, map12, map21, map22;
//        initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
//        initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);
//
//        Mat img1r, img2r;
//        remap(img1, img1r, map11, map12, INTER_LINEAR);
//        remap(img2, img2r, map21, map22, INTER_LINEAR);
//
//        img1 = img1r;
//        img2 = img2r;
//    }
//
//    numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width/8) + 15) & -16;
//
//    bm.state->roi1 = roi1;
//    bm.state->roi2 = roi2;
//    bm.state->preFilterCap = 31;
//    bm.state->SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 9;
//    bm.state->minDisparity = 0;
//    bm.state->numberOfDisparities = numberOfDisparities;
//    bm.state->textureThreshold = 10;
//    bm.state->uniquenessRatio = 15;
//    bm.state->speckleWindowSize = 100;
//    bm.state->speckleRange = 32;
//    bm.state->disp12MaxDiff = 1;
//
//    sgbm.preFilterCap = 63;
//    sgbm.SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 3;
//
//    int cn = img1.channels();
//
//    sgbm.P1 = 8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
//    sgbm.P2 = 32*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
//    sgbm.minDisparity = 0;
//    sgbm.numberOfDisparities = numberOfDisparities;
//    sgbm.uniquenessRatio = 10;
//    sgbm.speckleWindowSize = bm.state->speckleWindowSize;
//    sgbm.speckleRange = bm.state->speckleRange;
//    sgbm.disp12MaxDiff = 1;
//    sgbm.fullDP = alg == STEREO_HH;
//
//    var.levels = 3;                                 // ignored with USE_AUTO_PARAMS
//    var.pyrScale = 0.5;                             // ignored with USE_AUTO_PARAMS
//    var.nIt = 25;
//    var.minDisp = -numberOfDisparities;
//    var.maxDisp = 0;
//    var.poly_n = 3;
//    var.poly_sigma = 0.0;
//    var.fi = 15.0f;
//    var.lambda = 0.03f;
//    var.penalization = var.PENALIZATION_TICHONOV;   // ignored with USE_AUTO_PARAMS
//    var.cycle = var.CYCLE_V;                        // ignored with USE_AUTO_PARAMS
//    var.flags = var.USE_SMART_ID | var.USE_AUTO_PARAMS | var.USE_INITIAL_DISPARITY | var.USE_MEDIAN_FILTERING ;
//
//    Mat disp, disp8;
//    //Mat img1p, img2p, dispp;
//    //copyMakeBorder(img1, img1p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);
//    //copyMakeBorder(img2, img2p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);
//
//    int64 t = getTickCount();
//    if( alg == STEREO_BM )
//        bm(img1, img2, disp);
//    else if( alg == STEREO_VAR ) {
//        var(img1, img2, disp);
//    }
//    else if( alg == STEREO_SGBM || alg == STEREO_HH )
//        sgbm(img1, img2, disp);
//    t = getTickCount() - t;
//    printf("Time elapsed: %fms\n", t*1000/getTickFrequency());
//
//    //disp = dispp.colRange(numberOfDisparities, img1p.cols);
//    if( alg != STEREO_VAR )
//        disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));
//    else
//        disp.convertTo(disp8, CV_8U);
//    if( !no_display )
//    {
//        namedWindow("left", 1);
//        imshow("left", img1);
//        namedWindow("right", 1);
//        imshow("right", img2);
//        namedWindow("disparity", WINDOW_AUTOSIZE );
//        imshow("disparity", disp8);
//        printf("press any key to continue...");
//        fflush(stdout);
//        waitKey();
//        printf("\n");
//    }
//
//    if(disparity_filename)
//        imwrite(disparity_filename, disp8);
//
//    if(point_cloud_filename)
//    {
//        printf("storing the point cloud...");
//        fflush(stdout);
//        Mat xyz;
//        reprojectImageTo3D(disp, xyz, Q, true);
//		imshow("Depth", xyz);
//		waitKey();
//        saveXYZ(point_cloud_filename, xyz);
//        printf("\n");
//    }
//
//    return 0;
//}

//stanley
//static void onMouse( int event, int x, int y, int f, void* );

void on_TrackbarNumcharge(int,void*)
{
	int a=10;
	
}


Mat img1,img2;
Mat threeD;	

int main()
{
	int cameraID_L=0;
	int cameraID_R=1;

	VideoCapture Cap1,Cap2;

	

	Cap1.open(cameraID_L);
	Cap2.open(cameraID_R);

	StereoSGBM sgbm;
	StereoBM bm;
	float scale = 1.f;

	

	Cap1>>img1;
	Size img_size = img1.size();

	// reading intrinsic parameters
	const char* intrinsic_filename = "intrinsics.yml";
	const char* extrinsic_filename = "extrinsics.yml";
    FileStorage fs(intrinsic_filename, CV_STORAGE_READ);
    if(!fs.isOpened())
    {
        printf("Failed to open file %s\n", intrinsic_filename);
        return -1;
    }

    Mat M1, D1, M2, D2;
    fs["M1"] >> M1;
    fs["D1"] >> D1;
    fs["M2"] >> M2;
    fs["D2"] >> D2;

    M1 *= scale;
    M2 *= scale;

    fs.open(extrinsic_filename, CV_STORAGE_READ);
    if(!fs.isOpened())
    {
        printf("Failed to open file %s\n", extrinsic_filename);
        return -1;
    }

    Mat R, T, R1, P1, R2, P2;
	Mat Q;
    fs["R"] >> R;
    fs["T"] >> T;

	Rect roi1, roi2;

    stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );

    Mat map11, map12, map21, map22;
    initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
    initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);


	enum { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_VAR=3 };
    int alg = STEREO_BM;
    int SADWindowSize = 0, numberOfDisparities = 0;
    bool no_display = false;
    
	numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width/8) + 15) & -16;
	//sgbm.preFilterCap = 63;
	//sgbm.SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 3;

	int cn = img1.channels();

	//sgbm.P1 = 8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
	//sgbm.P2 = 32*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
	//sgbm.minDisparity = 1;
	//sgbm.numberOfDisparities = numberOfDisparities;
	//
	//sgbm.uniquenessRatio = 10;
	//sgbm.speckleWindowSize = 100;
	//sgbm.speckleRange = 32;
	//sgbm.disp12MaxDiff = 1;
	//sgbm.fullDP = alg == STEREO_HH;

	//bm.state->roi1 = roi1;
 //   bm.state->roi2 = roi2;
 //   bm.state->preFilterCap = 31;
 //   bm.state->SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 9;
 //   bm.state->minDisparity = 0;
 //   bm.state->numberOfDisparities = numberOfDisparities;
 //   bm.state->textureThreshold = 10;
 //   bm.state->uniquenessRatio = 15;
 //   bm.state->speckleWindowSize = 100;
 //   bm.state->speckleRange = 32;
 //   bm.state->disp12MaxDiff = 1;


	//stanley
	bm.state->roi1 = roi1;
	bm.state->roi2 = roi2;
	int preFilterCap=31;
	SADWindowSize=25;
	int minDisparity=0;
	numberOfDisparities=64;
	int textureThreshold=20;
	int uniquenessRatio=9;
	int speckleWindowSize=100;
	int speckleRange=32;
	int disp12MaxDiff = 1;


	//stanley near
	//int preFilterCap=4;
	//SADWindowSize=29;
	//int minDisparity=7;
	//numberOfDisparities=127;
	//int textureThreshold=16;
	//int uniquenessRatio=2;
	//int speckleWindowSize=50;
	//int speckleRange=43;
	//int disp12MaxDiff = 2;

	
	


	namedWindow("parameter",WINDOW_AUTOSIZE);//show mask
	resizeWindow("parameter",850,600);

	createTrackbar("SADWindowSize","parameter",&SADWindowSize,255,on_TrackbarNumcharge);
	createTrackbar("numberOfDisparities","parameter",&numberOfDisparities,255,on_TrackbarNumcharge);
	createTrackbar("preFilterCap","parameter",&preFilterCap,31,on_TrackbarNumcharge);
	createTrackbar("minDisparity","parameter",&minDisparity,255,on_TrackbarNumcharge);
	createTrackbar("textureThreshold","parameter",&textureThreshold,255,on_TrackbarNumcharge);
	createTrackbar("uniquenessRatio","parameter",&uniquenessRatio,255,on_TrackbarNumcharge);
	createTrackbar("speckleWindowSize","parameter",&speckleWindowSize,255,on_TrackbarNumcharge);
	createTrackbar("speckleRange","parameter",&speckleRange,255,on_TrackbarNumcharge);
	createTrackbar("disp12MaxDiff","parameter",&disp12MaxDiff,40,on_TrackbarNumcharge);
	

	Mat disp;
	Mat disp8;
	while(true)
	{
		Cap1>>img1;
		Cap2>>img2;

	
		//int color_mode = alg == STEREO_BM ? 0 : -1;
		//int color_mode=-1;
		//Mat img1 = imread("left17.jpg", color_mode);
		//Mat img2 = imread("right17.jpg", color_mode);
		

		//imshow("left1.jpg",img1);
		

		Mat img1r, img2r;
		remap(img1, img1r, map11, map12, INTER_LINEAR);
		remap(img2, img2r, map21, map22, INTER_LINEAR);

		img1 = img1r;
		img2 = img2r;

		Mat img1_gray,img2_gray;
		cvtColor(img1, img1_gray, COLOR_BGR2GRAY);
		cvtColor(img2, img2_gray, COLOR_BGR2GRAY);
	
	
		int64 t = getTickCount();

		if((SADWindowSize%2)==0)	//nust be odd
			SADWindowSize=SADWindowSize+1;
			if(SADWindowSize<5) SADWindowSize=5;
		
		if((numberOfDisparities%16)!=0)
		{
			numberOfDisparities=numberOfDisparities-numberOfDisparities%16;
			if (numberOfDisparities==0)
				numberOfDisparities=16;
		}

		//if((preFilterSize%2)==0)	//nust be odd
		//	preFilterSize=preFilterSize+1;
		//	if(preFilterSize<5) preFilterSize=5;


		if(preFilterCap==0)
			preFilterCap=1;


		bm.state->SADWindowSize = SADWindowSize;
		bm.state->numberOfDisparities = numberOfDisparities;
		
		bm.state->preFilterCap = preFilterCap;
		bm.state->minDisparity = minDisparity;
		bm.state->textureThreshold = textureThreshold;
		bm.state->uniquenessRatio = uniquenessRatio;
		bm.state->speckleWindowSize = speckleWindowSize;
		bm.state->speckleRange = speckleRange;
		bm.state->disp12MaxDiff = disp12MaxDiff;

		bm(img1_gray, img2_gray, disp);
		//sgbm(img1, img2, disp);
		//std::cout << "M = " << std::endl << " " << disp << std::endl << std::endl;
		t = getTickCount() - t;
		//printf("Time elapsed: %fms\n", t*1000/getTickFrequency());

		//disp = dispp.colRange(numberOfDisparities, img1p.cols);
		disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));

		//turn to 3d
		//threeD = cv2.reprojectImageTo3D(disparity.astype(np.float32)/16., camera_configs.Q)
		//reprojectImageTo3D(disp, threeD,Q,-1);
	
		

		namedWindow("left", 1);
		imshow("left", img1);
		namedWindow("right", 1);
		imshow("right", img2);
		namedWindow("disparity", WINDOW_AUTOSIZE );
		imshow("disparity", disp8);

		
		//setMouseCallback("left", onMouse, 0 );

		
		fflush(stdout);
		char c = (char)waitKey(10);
		if( c =='c' || c == 'C' )
		{
			break;
		}
	
	}

	return 0;
}

//static void onMouse( int event, int x, int y, int f, void* )
//{
//	if (event!=CV_EVENT_LBUTTONDOWN)
//		return;
//
//	Vec3f XYZ=threeD.at<Vec3f>(y,x);
//	float X=XYZ.val[0];
//	float Y=XYZ.val[1];
//	float Z=XYZ.val[2];
//
//	char name[30];
//	sprintf_s(name,"X=%f",X);
//	putText(img1,name, Point(150,40) , FONT_HERSHEY_SIMPLEX, .7, Scalar(0,255,0), 2,8,false );
//
//	sprintf_s(name,"Y=%f",Y);
//	putText(img1,name, Point(150,80) , FONT_HERSHEY_SIMPLEX, .7, Scalar(0,255,0), 2,8,false );
//
//	sprintf_s(name,"Z=%f",Z);
//	putText(img1,name, Point(150,120) , FONT_HERSHEY_SIMPLEX, .7, Scalar(0,255,0), 2,8,false );
//
//	sprintf_s(name,"X=%d",x);
//	putText(img1,name, Point(25,300) , FONT_HERSHEY_SIMPLEX, .7, Scalar(0,0,255), 2,8,false );
//
//	sprintf_s(name,"Y=%d",y);
//	putText(img1,name, Point(25,340) , FONT_HERSHEY_SIMPLEX, .7, Scalar(0,0,255), 2,8,false );
//
//	imwrite("hsv.jpg",image);
//	imshow("left",img1);
//	waitKey();
//}
