#include "cpu_BM_stereo.h"

void BM_cpu::Initialize(Size s,string calibration_path)
{
	name = "cpu_BM";
	Load(s,calibration_path);
}

void BM_cpu::BuildDefaultParameters()
{
	//good 4 textured {4,0,8,10,50,19,9,35,1,2}
	this->parameters = new AlgParameters();

	parameters->P0 = 4;		// x*16
	parameters->P1 = 0;		//min disp	
	parameters->P2 = 8;		//SADWindowSize x*2+1
	parameters->P3 = 10;	//prefilterCap
	parameters->P4 = 50;	//texture threshold
	parameters->P5 = 19;	//uniquenessRatio
	parameters->P6 = 9;		//speckleWindowSize
	parameters->P7 = 35;	//speckleRange
	parameters->P8 = 1;		//disp12MaxDiff
	parameters->P9 = 2;		//preFilterSize (2...n)
}

void BM_cpu::PrintParamsDescription()
{
	cout<<"P0: number of diparities (*16), \n";
	cout<<"P1: min diparity, \n";
	cout<<"P2: SAD window size (*2+1), \n";
	cout<<"P3: pre-filter cap, \n";
	cout<<"P4: texture threshold, \n";
	cout<<"P5: uniqueness ratio, \n";
	cout<<"P6: speckle window size, \n";
	cout<<"P7: speckle range, \n";
	cout<<"P8: disparity max difference, \n";
	cout<<"P9: preFilter size (*2+1)."<<endl;	
}

void BM_cpu::SetParamaters()
{
	PrintParameters(parameters);
	
	bm.state->preFilterType = CV_STEREO_BM_XSOBEL;
	bm.state->trySmallerWindows =	0;

	bm.state->numberOfDisparities = Clamp(parameters->P0,1,100)*16;
	bm.state->minDisparity =		parameters->P1;
	bm.state->SADWindowSize =		Clamp(parameters->P2,2,100)*2+1;
    bm.state->preFilterCap =		Clamp(parameters->P3,1,63);	
	bm.state->textureThreshold =	Clamp(parameters->P4,0,10000);
    bm.state->uniquenessRatio =		Clamp(parameters->P5,0,10000);
    bm.state->speckleWindowSize =	parameters->P6;
    bm.state->speckleRange =		parameters->P7;
    bm.state->disp12MaxDiff =		parameters->P8;
	bm.state->preFilterSize =		Clamp(parameters->P9,2,100)*2+1;
}

void BM_cpu::Compute(Mat left,Mat right,Mat& disp)
{
	UpdateParameters();

	if(left.channels()==1)
	{
		imgL = left;
		imgR = right;
	}
	else
	{	
		cvtColor(left,imgL,CV_BGR2GRAY);
		cvtColor(right,imgR,CV_BGR2GRAY);
	}

	ScaleImages();
	RectifyImages();

	bm(imgL, imgR, disp,CV_16S);  

	resize(disp,disp,Size(),SHOW_SCALE,SHOW_SCALE,INTER_AREA);

    if(STORE_RESULTS) StoreResults(disp,imgL,imgR);

    return;
}

Mat  BM_cpu::GetUserFirendlyDisparity(Mat disp)
{
	/*
	Mat result = Mat::zeros(disp.rows,disp.cols,CV_8UC3);//HSV
	MatIterator_<Vec3b> ithsv, endhsv;
	MatIterator_<short> it, end;		

	for(	it = disp.begin<short>(),	   end = disp.end<short>(),
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

Mat  BM_cpu::GetFloatDisparity(Mat disp)
{
	Mat real_disp;
	disp.convertTo(real_disp,CV_32F,1.0f/16.0f);
	return real_disp;
}


 

                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                kedCompareExchange  �HeapSetInformation  � EncodePointer �TerminateProcess  �GetCurrentProcess �UnhandledExceptionFilter  �SetUnhandledExceptionFilter  IsDebuggerPresent � DecodePointer �QueryPerformanceCounter �GetCurrentProcessId yGetSystemTimeAsFileTime IsProcessorFeaturePresent                                                                                                                                                                                                                           