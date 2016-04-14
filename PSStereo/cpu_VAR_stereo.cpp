#include "cpu_VAR_stereo.h"

void VAR_cpu::Initialize(Size s,string calibration_path)
{
	name = "cpu_VAR";
	Load(s,calibration_path);
}

void VAR_cpu::BuildDefaultParameters()
{
	this->parameters = new AlgParameters();

	parameters->P0 = 0;		//maxDisp
	parameters->P1 = -2;	//minDisp
	parameters->P2 = 3;		//levels
	parameters->P3 = 0.5f;	//pyrScale
	parameters->P4 = 25;	//nIt
	parameters->P5 = 3;		//poly_n
	parameters->P6 = 0.0f;	//poly_sigma
	parameters->P7 = 15.0f;	//fi
	parameters->P8 = 0.03f;	//lambda
	parameters->P9 = 0;		//penalization
}

void VAR_cpu::PrintParamsDescription()
{
	cout<<"P0: maxDisp(*16), ";
	cout<<"P1: minDisp, ";
	cout<<"P2: levels, ";
	cout<<"P3: pyrScale, ";
	cout<<"P4: nIt, ";
	cout<<"P5: poly_n, ";
	cout<<"P6: poly_sigma, ";
	cout<<"P7: fi, ";
	cout<<"P8: lambda";
	cout<<"P9: penalization [0,1,2]."<<endl;
}

void VAR_cpu::SetParamaters()
{
	PrintParameters(parameters);

	var.maxDisp = parameters->P0*16;
	var.minDisp = parameters->P1*16;
	var.levels =  parameters->P2;     
    var.pyrScale = parameters->P3;    
    var.nIt = parameters->P4;
    var.poly_n = parameters->P5;
    var.poly_sigma = parameters->P6;
    var.fi = parameters->P7;
    var.lambda = parameters->P8;
    var.penalization = parameters->P9;
    var.cycle = var.CYCLE_V;          
	var.flags = var.USE_SMART_ID | var.USE_AUTO_PARAMS | var.USE_INITIAL_DISPARITY | var.USE_MEDIAN_FILTERING ;

}

void VAR_cpu::Compute(Mat left,Mat right,Mat& disp)
{
	UpdateParameters();
	imgL = left;
	imgR = right;
	ScaleImages();
	RectifyImages();

	Mat disp_aux;
	var(imgL, imgR, disp_aux);    

    disp_aux.convertTo(disp, CV_8U);

	resize(disp,disp,Size(),SHOW_SCALE,SHOW_SCALE,INTER_AREA);
    if(STORE_RESULTS) StoreResults(disp,imgL,imgR);

    return;
}

Mat VAR_cpu::GetUserFirendlyDisparity(Mat disp)
{
	/*
	Mat result = Mat::zeros(disp.rows,disp.cols,CV_8UC3);//HSV
	MatIterator_<Vec3b> ithsv, endhsv;
	MatIterator_<char> it, end;		

	for(	   it = disp.begin<char>(),	   end = disp.end<char>(),
			ithsv = result.begin<Vec3b>(),	endhsv = result.end<Vec3b>(); 
			it != end && ithsv!= endhsv ; 
			++it, ++ithsv)
	{
		float val = (float)(*it);		
		val = (val-parameters->P1*16)/(parameters->P0*16);

		(*ithsv)[0] = val*400;
		(*ithsv)[1] = 150;
		(*ithsv)[2] = 120;
	}
	cvtColor(result,result,CV_HSV2BGR);
	return result;
	*/
	return GetGrayRepresentation(disp);
}

Mat VAR_cpu::GetFloatDisparity(Mat disp)
{
	Mat real_disp;
	disp.convertTo(real_disp,CV_32F);
	return real_disp;
}
 

