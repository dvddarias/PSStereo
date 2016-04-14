#include "stereo_alg.h"

void StereoAlgorithm::ReadCalibration(string calibration_path)
{
	string path = calibration_path;
	// reading intrinsic parameters
    FileStorage fs(path+string(intrinsic_filename), CV_STORAGE_READ);
    if(!fs.isOpened())
    {
        printf("Failed to open file %s\n", intrinsic_filename);
        return;
    }

    Mat M1, D1, M2, D2;
    fs["M1"] >> M1;
    fs["D1"] >> D1;
    fs["M2"] >> M2;
    fs["D2"] >> D2;

	double fovx;
	double fovy;
	double focalLength;
	Point2d pPoint;
	double aRatio;

	cout<<"-----------------------Left camera intrinsics-----------------------\n";
	calibrationMatrixValues(M1,img_size,3.840,2.880,fovx,fovy,focalLength,pPoint,aRatio);	
	cout<<"FOVx: "<<fovx<<endl;
	cout<<"FOVy: "<<fovy<<endl;
	cout<<"Focal Length: "<<focalLength<<endl;
	cout<<"Principal Point: "<<pPoint<<endl;
	cout<<"Aspect Ratio: "<<aRatio<<endl;
	cout<<"-----------------------Right camera intrinsics----------------------\n";
	calibrationMatrixValues(M2,img_size,3.840,2.880,fovx,fovy,focalLength,pPoint,aRatio);	
	cout<<"FOVx: "<<fovx<<endl;
	cout<<"FOVy: "<<fovy<<endl;
	cout<<"Focal Length: "<<focalLength<<endl;
	cout<<"Principal Point: "<<pPoint<<endl;
	cout<<"Aspect Ratio: "<<aRatio<<endl;
	cout<<"--------------------------------------------------------------------\n";

    M1 *= scale;
    M2 *= scale;

    fs.open(path+string(extrinsic_filename), CV_STORAGE_READ);
    if(!fs.isOpened())
    {
        printf("Failed to open file %s\n", extrinsic_filename);
        return;  
    }

    Mat R, T, R1, P1, R2, P2;
    fs["R"] >> R;
    fs["T"] >> T;

    stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, 0, img_size, &roiL, &roiR );

    initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
    initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);    
}

void StereoAlgorithm::ScaleImages()
{
	if( scale != 1.f )
    {
        Mat temp1, temp2;
		int method = scale < 1 ? CV_INTER_AREA: CV_INTER_LINEAR;
        resize(imgL, temp1, Size(), scale, scale, method);
        imgL = temp1;
        resize(imgR, temp2, Size(), scale, scale, method);
        imgR = temp2;
    }
}

void StereoAlgorithm::RectifyImages()
{
	if(_use_calibration)
	{
		Mat imgLr, imgRr;
		remap(imgL, imgLr, map11, map12, INTER_LINEAR);
		remap(imgR, imgRr, map21, map22, INTER_LINEAR);

		imgL = imgLr;
		imgR = imgRr;
	}
}

void StereoAlgorithm::UpdateParameters()
{
	if(Update_params)
	{
		SetParamaters();
		Update_params = false;
	}
}

void StereoAlgorithm::Load(Size s, string calibration_path)
{
    scale = WORK_SCALE;
	img_size = Size((int)(s.width*scale),(int)(s.height*scale));
	if(_use_calibration) ReadCalibration(calibration_path);
	BuildDefaultParameters();	
	cout<<"\n---------------Stereo algoritm parameters description---------------\n";
	PrintParamsDescription();
	cout<<"-------------------------Actual parameters--------------------------\n";
	SetParamaters();
	cout<<"--------------------------------------------------------------------\n\n";
}

void StereoAlgorithm::StoreResults(Mat disp, Mat left, Mat right)
{
	string path = string(GetExecutablePath()) + string("..\\..\\Results\\");
	//write disparity image.
	Mat aux = GetUserFirendlyDisparity(disp);
	imwrite(path+string(disparity_filename), aux);	

	//write stereo pair.
	Mat final(left.rows,left.cols*2,left.channels()==1?CV_8U:CV_8UC3);
    Mat roi1(final, Rect(0, 0, left.cols, left.rows ));
    left.copyTo(roi1);
    Mat roi2(final, Rect(left.cols, 0, left.cols, left.rows ));
    right.copyTo(roi2);
	imwrite(path+string(stereo_pair_filename), final );

	//write point cloud
    fflush(stdout);
	saveXYZ((path+string(point_cloud_filename)).c_str(), GetPointCloud(disp));
}

Mat StereoAlgorithm::GetPointCloud(Mat disp)
{
	Mat real_disp = GetFloatDisparity(disp);	
	Mat xyz;
    reprojectImageTo3D(real_disp, xyz, Q, false,-1);	
	return xyz*SQUARE_SIZE;
}

StereoAlgorithm::StereoAlgorithm()
{
	Update_params = false;
	_use_calibration = USE_CALIBRATION;

	intrinsic_filename = "intrinsics.yml";
    extrinsic_filename = "extrinsics.yml";
    disparity_filename = "disparity_map.bmp";
    point_cloud_filename = "point_cloud.asc";
	stereo_pair_filename = "stereo_pair.bmp";		
}

void saveXYZ(const char* filename, const Mat& mat)
{
    const double min_z = 1;
    FILE* fp;
	fopen_s(&fp,filename, "wt");
    for(int y = 0; y < mat.rows; y++)
    {
        for(int x = 0; x < mat.cols; x++)
        {			
			Vec3f point = mat.at<Vec3f>(y, x);
            if(fabs(point[2] - min_z) < FLT_EPSILON || point[2] < min_z) continue;
            fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);			
			/*
			if(x==mat.cols/2&&y==mat.rows/2)
			{				
				cout<<"coords: "<<point[0]<<", "<<point[1]<<", "<<point[2]<<endl;
				cout<<"distance: "<<point[2]<<"mm\n";
			}
			*/
        }
    }
    fclose(fp);
}

