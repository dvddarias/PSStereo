#include "new_stereo.h"

void NewStereo::Initialize(Size s,string calibration_path)
{
	name = "SmartMWS";
	LoadImage("cones","disp.png", ground_truth); 
	Load(s,calibration_path);

}

void NewStereo::SetParamaters()
{
	PrintParameters(parameters);
	
	disparities =		parameters->P0;
	min_disparity=		parameters->P1;
	windows =			parameters->P2;
	min_window_size=	parameters->P3;
	window_size_step=	parameters->P4;
}

void NewStereo::PrintParamsDescription()
{
	cout<<"P0: number of diparities, \n";
	cout<<"P1: min diparity, \n";
	cout<<"P2: number of windows, \n";
	cout<<"P3: min window size, \n";
	cout<<"P4: window size step, \n";
	cout<<"P5: unused, \n";
	cout<<"P6: unused, \n";
	cout<<"P7: unused, \n";
	cout<<"P8: unused, \n";
	cout<<"P9: unused."<<endl;	
}

void NewStereo::BuildDefaultParameters()
{
	this->parameters = new AlgParameters();

	parameters->P0 = 60;	//disps
	parameters->P1 = 0;		//min disp	
	parameters->P2 = 10;    //windows
	parameters->P3 = 3;		//min window size
	parameters->P4 = 2;		//window size step
	parameters->P5 = 0;		//
	parameters->P6 = 0;		//
	parameters->P7 = 0;		//
	parameters->P8 = 0;		//
	parameters->P9 = 0;		//
}

void NewStereo::Compute(Mat left,Mat right,Mat& disp)
{    
	UpdateParameters();
	
	if(left.channels()==1)
	{
		left.convertTo(imgL,CV_32F);
		right.convertTo(imgR,CV_32F);
	}
	else
	{			
		cvtColor(left,imgL,CV_BGR2GRAY);
		cvtColor(right,imgR,CV_BGR2GRAY);
		imgL.convertTo(imgL,CV_32F);
		imgR.convertTo(imgR,CV_32F);
	}

	ScaleImages();
	RectifyImages();

	disp = Mat::zeros(imgL.rows,imgL.cols,CV_8UC1);
	FillDisparities(imgL, imgR, disp);
	
	resize(disp,disp,Size(),SHOW_SCALE,SHOW_SCALE,INTER_AREA);
    if(STORE_RESULTS) StoreResults(disp,imgL,imgR);
	return;
}

void NewStereo::FillDisparities(Mat& reference, Mat& target, Mat& disp)
{
	int rows = reference.rows;
    int cols = reference.cols;

	int max_window_size = min_window_size+(windows-1)*window_size_step;
	int half_window = max_window_size/2;

	int col_start = disparities+min_disparity+half_window;
	int col_end = cols-half_window;
	int row_start = half_window;
	int row_end = rows-half_window;

	//the accumuator has two more final rows to sum the others and to see the gound truth
	accum = Mat::zeros(windows+2,disparities,CV_32F);

	int r,c;
    uchar* r_disp;
	for(r=row_start; r<row_end; ++r)
    {
		cout<<"row: "<<r<<endl;
        r_disp = disp.ptr<uchar>(r);
		for(c=col_start; c<col_end; ++c)
        {			
			r_disp[c] = GetDisparity(r,c,reference,target);
		}
	}

	return;
}

uchar NewStereo::GetDisparity(int row, int col, Mat& reference, Mat& target)
{		
	int step = window_size_step;
	int half_step = window_size_step/2;
	int total_disp = 0;
	float* r_accum;
	int w=0;
	int size = min_window_size;

	Mat base = GetWindow(row, col, size, reference); 

	for(; w<windows; ++w, size+=step)
	{
		r_accum = accum.ptr<float>(w);
		Mat actual = GetWindow(row, col-min_disparity, size, target);

		for(total_disp = 0; total_disp<disparities;++total_disp)
		{			
			r_accum[total_disp] = MatchingCost(base, actual);
			actual.adjustROI(0,0,1,-1);
		}

		base.adjustROI(half_step,half_step,half_step,half_step);
	}

	RowNormalize(accum);
	//ShowAccumulator(row, col, accum);
	
	//return ReadDisparity(row, col, ground_truth);
	return SelectDisparity(accum);
}

void NewStereo::RowNormalize(Mat& image)
{	
	Mat col_sum = Mat(image,Range(image.rows-1,image.rows),Range::all());
	col_sum *= 0;

	double max;
	Mat row = Mat(image,Range(0,1),Range::all());
	for(int r=0;r<image.rows-2;++r)
	{	
		divide(row,Scalar(min_window_size+r*window_size_step),row);
		minMaxLoc(row,NULL,&max);
		divide(row,Scalar(max),row);
		add(row,col_sum,col_sum);
		row.adjustROI(-1,1,0,0);
	}	
	minMaxLoc(col_sum,NULL,&max);
	divide(col_sum,Scalar(max),col_sum);
}

void NewStereo::ShowAccumulator(int row, int col, Mat& image)
{
	Mat pre_last = Mat(image,Range(image.rows-2,image.rows-1),Range::all());
	pre_last *= 0;
	float real_disp = ReadDisparity(row, col, ground_truth);
	pre_last.at<float>(Point(real_disp,0)) = 1.0f;

	Mat toshow;
	float scale = 10.0;
	resize(image,toshow,Size(),scale,scale,INTER_AREA);
	imshow("Accumulator",toshow); cvWaitKey(30000);
}

float NewStereo::SelectDisparity(Mat& accum)
{
	Mat col_sum = Mat(accum,Range(accum.rows-1,accum.rows),Range::all());
	Point p;
	double min;
	minMaxLoc(col_sum,&min,NULL,&p);	
	return min<0.2?p.x:0;
}

Mat NewStereo::GetWindow(int row, int col, int size, Mat& image)
{	
	return Mat(image,Rect(col-size/2,row-size/2,size,size));
}

float NewStereo::MatchingCost(Mat base, Mat actual)
{
	return sum(abs(base-actual))[0];
	//Mat aux;
	//pow(base-actual,2,aux);
	//return sum(aux)[0];
}

Mat NewStereo::GetUserFirendlyDisparity(Mat disp)
{
	return GetGrayRepresentation(disp);	
}

Mat NewStereo::GetFloatDisparity(Mat disp)
{
	Mat real_disp;
	disp.convertTo(real_disp,CV_32F);
	return real_disp;
}
