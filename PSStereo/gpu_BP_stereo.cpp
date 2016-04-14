#include  "gpu_BP_stereo.h"

void BP_gpu::Initialize(Size s, string calibration_path)
{
	name = "gpu_BP";
	setDevice(0);
	bp = new StereoBeliefPropagation();
	Load(s,calibration_path);
	// Prepare disparity map of specified type
    gpu_disp.create(img_size, outputType);

}

void BP_gpu::BuildDefaultParameters()
{
	//good 4 textured {4,10,4,15,0.04,35,0.5,5}
	this->parameters = new AlgParameters();
	/*
	int ndisp;
	int levels;
	int iters;
	StereoBeliefPropagation::estimateRecommendedParams(img_size.width,img_size.height,ndisp,iters,levels);
	*/
	parameters->P0 = 4;
	parameters->P1 = 10;		
	parameters->P2 = 4;
	parameters->P3 = 15.0f;
	parameters->P4 = 0.04f;		
	parameters->P5 = 35.0;
	parameters->P6 = 0.5f;	
}

void BP_gpu::SetParamaters()
{
	PrintParameters(parameters);

	bp->ndisp = Clamp(parameters->P0,1,20)*16;
	bp->iters = Clamp(parameters->P1,1,100);
	bp->levels = Clamp(parameters->P2,1,7);
	bp->max_data_term = Clamp(parameters->P3,-50,50);
	bp->data_weight = Clamp(parameters->P4,-50,50);
	bp->max_disc_term = Clamp(parameters->P5,-50,50);
	bp->disc_single_jump = Clamp(parameters->P6,-50,50);
	bp->msg_type = CV_16SC1;
}

void BP_gpu::PrintParamsDescription()
{
	cout<<"P0: number of diparities (*16), \n";
	cout<<"P1: iterations, \n";
	cout<<"P2: levels, \n";
	cout<<"P3: max data term, \n";
	cout<<"P4: data weight, \n";
	cout<<"P5: max disc. term, \n";
	cout<<"P6: disc. single jump \n"<<endl;		
}

void BP_gpu::Compute(Mat left,Mat right,Mat& disp)
{    
	UpdateParameters();
	imgL = left;
	imgR = right;
	ScaleImages();
	RectifyImages();

    gpu_left.upload(imgL);
    gpu_right.upload(imgR);

	bp->operator()(gpu_left, gpu_right, gpu_disp);       
	
	resize(gpu_disp,gpu_aux,Size(),SHOW_SCALE,SHOW_SCALE,INTER_AREA);
	gpu_aux.download(disp); 

    if(STORE_RESULTS) StoreResults(disp,imgL,imgR);

    return;
}

Mat BP_gpu::GetUserFirendlyDisparity(Mat disp)
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

Mat BP_gpu::GetFloatDisparity(Mat disp)
{
	Mat real_disp;
	disp.convertTo(real_disp,CV_32F);
	return real_disp;
}

