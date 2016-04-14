#include  "gpu_BM_stereo.h"

void BM_gpu::Initialize(Size s,string calibration_path)
{
	name = "gpu_BM";
	setDevice(0);
	bm = new StereoBM_GPU(bm->PREFILTER_XSOBEL);
	Load(s,calibration_path);
	// Prepare disparity map of specified type
    gpu_disp.create(img_size, outputType);

}

void BM_gpu::SetParamaters()
{
	PrintParameters(parameters);

	bm->ndisp = Clamp(parameters->P0,-100,19)*16;
	bm->winSize = Clamp(parameters->P1,2,25)*2+1;	
	bm->avergeTexThreshold = parameters->P2;
}

void BM_gpu::Compute(Mat left,Mat right,Mat& disp)
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

    gpu_left.upload(imgL);
    gpu_right.upload(imgR);

	bm->operator()(gpu_left, gpu_right, gpu_disp);       
	
	resize(gpu_disp,gpu_aux,Size(),SHOW_SCALE,SHOW_SCALE,INTER_AREA);
	gpu_aux.download(disp);

    if(STORE_RESULTS) StoreResults(disp,imgL,imgR);

    return;
}

void BM_gpu::BuildDefaultParameters()
{
	//good 4 textured {4,8,10}
	this->parameters = new AlgParameters();
	parameters->P0 = 4.0f;
	parameters->P1 = 8;		//winSize x*2+1
	parameters->P2 = 10;	
}

void BM_gpu::PrintParamsDescription()
{
	cout<<"P0: number of diparities (*16),\nP1: window size (*2+1),\nP2: texture threshold."<<endl;
}

Mat BM_gpu::GetUserFirendlyDisparity(Mat disp)
{
	/*
	Mat result = Mat::zeros(disp.rows,disp.cols,CV_8UC3);//HSV
	MatIterator_<Vec3b> ithsv, endhsv;
	MatIterator_<uchar> it, end;		

	for(	   it = disp.begin<uchar>(),	   end = disp.end<uchar>(),
			ithsv = result.begin<Vec3b>(),	endhsv = result.end<Vec3b>(); 
			it != end && ithsv!= endhsv ; 
			++it, ++ithsv)
	{
		float val = (float)(*it);		
		val = (val)/(parameters->P0*16);

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

Mat BM_gpu::GetFloatDisparity(Mat disp)
{
	Mat real_disp;
	disp.convertTo(real_disp,CV_32F);
	return real_disp;
}