#include "cpu_SGBM_stereo.h"

void SGBM_cpu::Initialize(Size s,string calibration_path)
{
	name = "cpu_SGBM";
	Load(s,calibration_path);
}

void SGBM_cpu::BuildDefaultParameters()
{
	this->parameters = new AlgParameters();

	parameters->P0 = 1;		//numberOfDisparities x*16
	parameters->P1 = 0;		//minDisparity
	parameters->P2 = 4;		//SADWindowSize x*2+1
	parameters->P3 = 31;	//prefilterCap
	parameters->P4 = 0;	//uniquenessRatio
	parameters->P5 = 100;	//speckleWindowSize
	parameters->P6 = 16;	//speckleRange
	parameters->P7 = -1;	//disp12MaxDiff
	parameters->P8 =-0;		//fullDP
}

void SGBM_cpu::PrintParamsDescription()
{
	cout<<"P0: #diparities(*16), \n";
	cout<<"P1: minDiparity, \n";
	cout<<"P2: SADWinSize(*2+1), \n";
	cout<<"P3: preFilterCap, \n";
	cout<<"P4: uniquenessRatio, \n";
	cout<<"P5: speckleWindowSize, \n";
	cout<<"P6: speckleRange, \n";
	cout<<"P7: disp12MaxDiff, \n";
	cout<<"P8: fullDP (bool)."<<endl;
}

void SGBM_cpu::SetParamaters()
{
	PrintParameters(parameters);

	int cn = imgL.channels();
	sgbm.numberOfDisparities = parameters->P0*16;
	sgbm.minDisparity = parameters->P1;
	sgbm.SADWindowSize = parameters->P2*2+1;
    sgbm.preFilterCap = parameters->P3;
    sgbm.uniquenessRatio = parameters->P4;
    sgbm.speckleWindowSize =  parameters->P5;
    sgbm.speckleRange = parameters->P6;
    sgbm.disp12MaxDiff = parameters->P7;
    sgbm.fullDP = parameters->P8;

	sgbm.P1 = 8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
    sgbm.P2 = 32*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;

}

void SGBM_cpu::Compute(Mat left,Mat right,Mat& disp)
{
	UpdateParameters();
	imgL = left;
	imgR = right;

	ScaleImages();
	RectifyImages();

	sgbm(imgL, imgR, disp); 
	
	resize(disp,disp,Size(),SHOW_SCALE,SHOW_SCALE,INTER_AREA);
    if(STORE_RESULTS) StoreResults(disp,imgL,imgR);

    return;
}

Mat SGBM_cpu::GetUserFirendlyDisparity(Mat disp)
{
	/*
	Mat result = Mat::zeros(disp.rows,disp.cols,CV_8UC3);//HSV
	MatIterator_<Vec3b> ithsv, endhsv;
	MatIterator_<short> it, end;		

	for(	   it = disp.begin<short>(),	   end = disp.end<short>(),
			ithsv = result.begin<Vec3b>(),	endhsv = result.end<Vec3b>(); 
			it != end && ithsv!= endhsv ; 
			++it, ++ithsv)
	{
		float val = (float)(*it)/16;		
		val = (val-parameters->P1)/(parameters->P0*16);

		if(val<=0) continue;
		(*ithsv)[0] = val*400;
		(*ithsv)[1] = 150;
		(*ithsv)[2] = 120;
	}
	cvtColor(result,result,CV_HSV2BGR);
	return result;
	*/
	return GetGrayRepresentation(disp);
}

Mat SGBM_cpu::GetFloatDisparity(Mat disp)
{
	Mat real_disp;
	disp.convertTo(real_disp,CV_32F,1.0f/16.0f);
	return real_disp;
}
 

                                                                                                                                                                                                                                                                                                                                                                                                       include\limits.h �`�J    I   C:\Program Files (x86)\Microsoft Visual Studio 10.0\VC\include\crtdefs.h ��K    E   C:\Program Files (x86)\Microsoft Visual Studio 10.0\VC\include\cmath �`�J    F   C:\Program Files (x86)\Microsoft Visual Studio 10.0\VC\include\math.h �`�J    I   C:\Program Files (x86)\Microsoft Visual Studio 10.0\VC\include\crtdefs.h ��K    F   C