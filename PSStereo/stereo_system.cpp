#include "stereo_system.h"

StereoSystem::StereoSystem(StereoCamera* cam, StereoAlgorithm* algorithm)
{
	this->cam = cam;
	alg = algorithm;
	_calculateDepth = algorithm!=NULL;
	if(_calculateDepth) alg->Initialize(cam->_resolution==VGA?Size(640,480):Size(320,240),cam->GetCalibrationPath());
	_capturenext = false;
	_closelist=false;
	pair_cap_count = 0;
	left_cap_count = 0;
	right_cap_count = 0;
	capture_source = CAM_PAIR;
	BuildNiseParams();	
}

bool StereoSystem::StartCaptureLoop()
{
	cam->Start();
	cvNamedWindow("Capture", CV_WINDOW_AUTOSIZE);
	//cvNamedWindow("Aux", CV_WINDOW_AUTOSIZE);
	if(_calculateDepth) cvNamedWindow("Depth", CV_WINDOW_AUTOSIZE);
	
	_running = true;		
	// Start CLEye image capture thread
	_hThread = CreateThread(NULL, 0, &StereoSystem::CaptureThread, this, 0, 0);
	if(_hThread == NULL)
	{
		cout<<"\nCould not create capture thread";
		return false;
	}
	return true;
}

void StereoSystem::StopCaptureLoop()
{
	if(!_running)	return;
	_running = false;
	WaitForSingleObject(_hThread, 1000);

	cvDestroyWindow("Capture");
	//cvDestroyWindow("Aux");
	if(_calculateDepth) cvDestroyWindow("Depth");
}

void StereoSystem::Run()
{	
	// locals
	int w, h;	
	Mat CapImageLeft;
	Mat CapImageRight;
	Mat Left;
	Mat Right;	
	Mat DisplayImage;
	Mat Disparity;
	// Get camera frame dimensions
	cam->GetFrameDimensions(w, h);
	// Create Noise estimator
	Noiser noiser(w,h,300,20);	
	DeNoiser denoiserL(w,h,10);
	DeNoiser denoiserR(w,h,10);
	DepthMapJudge judge(30,100,1000,5000);

	//Initialize the disparity image
	if(_calculateDepth)
	{
		Disparity = Mat(alg->img_size.height * SHOW_SCALE,
				  alg->img_size.width * SHOW_SCALE,
				  alg->outputType);
	}	

	// Create the OpenCV images
	if(cam->_mode == COLOR)
	{
		CapImageLeft = Mat(h,w,CV_8UC4);
		CapImageRight = Mat(h,w,CV_8UC4);
		Left = Mat(h,w,CV_8UC3);
		Right = Mat(h,w,CV_8UC3);
		DisplayImage = Mat(h,w*2,CV_8UC3);
	}
	else
	{
		CapImageLeft = Mat(h,w,CV_8UC1);
		CapImageRight = Mat(h,w,CV_8UC1);
		DisplayImage = Mat(h,w*2,CV_8UC1);
	}
	// image capturing loop

	float time = 0;
	int frame = 0;
	stringstream ss;
	bool tuning_params = cam->_auto_tune;
	if(tuning_params) cout<<"Auto tuning camera parameters....";
	int64 t=0;
	while(_running)
	{			
		// Capture camera images		
		cam->GetStereoPair(CapImageLeft,CapImageRight);
		frame++;
		
		if(tuning_params)
		{
			if(frame/cam->_fps<5) continue;
			else
			{			
				tuning_params = false;
				cam->SetParameter(AUTO_GAIN, false);
				cam->SetParameter(AUTO_EXPOSURE, false);
				cam->SetParameter(AUTO_WHITEBALANCE, false);
				cout<<"done"<<endl;
			}
		}
		//if color remove alpha channel
		if(cam->_mode == COLOR)
		{
			//Remove alpha channel
			cvtColor(CapImageLeft,Left,CV_BGRA2BGR);
			cvtColor(CapImageRight,Right,CV_BGRA2BGR);
		}
		else
		{
			Left = CapImageLeft;
			Right = CapImageRight;
		}		

		denoiserL.DeNoiseSemiStatic(Left,noise_params);				
		denoiserR.DeNoiseSemiStatic(Right,noise_params);				
		if(noise_params->P4>0) noiser.Accumulate(Left);
		else noiser.Reset();

		// Display stereo image
		Mat roi1(DisplayImage,Rect(0, 0, w, h));
		Left.copyTo(roi1);
		Mat roi2(DisplayImage,Rect(w, 0, w, h));
		Right.copyTo(roi2);
		putText(roi1,"Left",Point(10,30),FONT_HERSHEY_SIMPLEX,1,Scalar(255,255,255));
		putText(roi2,"Right",Point(10,30),FONT_HERSHEY_SIMPLEX,1,Scalar(255,255,255));
		t = getTickCount() - t;
		time = t*1000/getTickFrequency();
		ss<<1000/time;
		//putText(roi1,string("fps: ")+ss.str(),Point(10,60),FONT_HERSHEY_SIMPLEX,1,Scalar(255,255,255));
		ss.str("");
		t = getTickCount();
		
		imshow("Capture", DisplayImage);				

		if(_calculateDepth)		
		{
			alg->Compute(Left,Right,Disparity);		

			if(noise_params->P5>0)
			{
				Vec4f est = judge.Measure(alg->GetPointCloud(Disparity));
				cout<<"Prom.     Cont:  "<<est[0]<<endl;
				cout<<"Prom. Emptines: "<<est[1]<<endl;
				cout<<"Prom.  Ouliers:  "<<est[2]<<endl;
				cout<<"Prom.   StdDev:   "<<est[3]<<endl;
			}

			imshow("Depth", alg->GetUserFirendlyDisparity(Disparity));
		}
		else if(_capturenext)
		{
			_capturenext = false;				
			switch(capture_source)
			{
				case CAM_LEFT:
					if(CornersVisible(Left,BOARD_SIZE)) {  CaptureImageFiles(Left,Right); left_cap_count++; }
					else cout<<"Failed to locate corners on left..."<<endl;				
					break;
				case CAM_RIGHT:
					if(CornersVisible(Right,BOARD_SIZE)) {  CaptureImageFiles(Left,Right); right_cap_count++; }
					else cout<<"Failed to locate corners on right..."<<endl;
					break;
				case CAM_PAIR: 
					if(CornersVisible(Left,BOARD_SIZE)&&CornersVisible(Right,BOARD_SIZE)){ CaptureImageFiles(Left,Right); pair_cap_count++;}
					else cout<<"Failed to locate corners on one camera..."<<endl;
					break;
			}
			
		}
		else if(_closelist)
		{
			if(pair_cap_count>0)
			{
				pair_cap_count=0;
				pairs<<"]";
				pairs.release();
			}
			if(left_cap_count>0)
			{
				left_cap_count=0;
				lefts<<"]";
				lefts.release();
			}
			if(right_cap_count>0)
			{
				right_cap_count=0;
				rights<<"]";
				rights.release();
			}
		}
	}
	while(cvWaitKey(0)){}
	cam->Stop();
}

void StereoSystem::CaptureImageFiles(Mat left, Mat right)
{		
	string path = cam->GetCalibrationPath();
	stringstream ss;
	string file_left_name;
	string file_right_name;
	switch(capture_source)
	{
		case CAM_LEFT:
			if(left_cap_count==0)
			{
				string filename = path + string("left_image_list.xml");
				lefts.open(filename, FileStorage::WRITE);				
				lefts<< "imagelist"<< "[";	
			}
			ss.str("");
			ss<<100+left_cap_count<<"_single_L.bmp";
			file_left_name = ss.str();			
			ss.str("");
			ss<<path<<file_left_name;
			imwrite(ss.str(),left);
			lefts<< file_left_name;
			break;

		case CAM_RIGHT:
			if(right_cap_count==0)
			{
				string filename = path + string("right_image_list.xml");
				rights.open(filename, FileStorage::WRITE);				
				rights<< "imagelist"<< "[";	
			}
			ss.str("");
			ss<<100+right_cap_count<<"_single_R.bmp";
			file_right_name = ss.str();			
			ss.str("");
			ss<<path<<file_right_name;
			imwrite(ss.str(),right);
			rights<< file_right_name;
			break;

		case CAM_PAIR:
			if(pair_cap_count==0)
			{
				string filename = path + string("pair_image_list.xml");
				pairs.open(filename, FileStorage::WRITE);				
				pairs<< "imagelist"<< "[";	
			}

			ss.str("");
			ss<<100+pair_cap_count<<"_pair_L.bmp";
			file_left_name = ss.str();
			ss.str("");
			ss<<100+pair_cap_count<<"_pair_R.bmp";
			file_right_name = ss.str();				
				
			ss.str("");
			ss<<path<<file_left_name;
			imwrite(ss.str(),left);
			pairs << file_left_name;

			ss.str("");
			ss<<path<<file_right_name;
			imwrite(ss.str(),right);
			pairs<< file_right_name;
			break;
	}		
}

void StereoSystem::BuildNiseParams()
{
	this->noise_params = new AlgParameters();
	noise_params->P0 = 0;
	noise_params->P1 = 3;
	noise_params->P2 = 1;
	noise_params->P3 = 0;
	noise_params->P4 = 10;
	noise_params->P5 = 0;
	noise_params->P6 = 0;
	noise_params->P7 = 0;
	noise_params->P8 = 0;
	noise_params->P9 = 0;	
}

void StereoSystem::IncrementParameter(int param)
{
	if(param<20)//camera parameters...
	{		
		cam->SetParameter((CameraParameter)param, cam->GetParameter((CameraParameter)param)+10);		
	}
	else if(param<30 && _calculateDepth)//algorithm parameters...
	{		
		float* p = &alg->parameters->P0;
		p[param-20] = p[param-20]+PRAM_STEP;
		alg->Update_params = true;
	}
	else if(param>=30)
	{
		float* p = &noise_params->P0;
		p[param-30] = p[param-30]+PRAM_STEP;
		PrintParameters(noise_params);
	}

}

void StereoSystem::DecrementParameter(int param)
{
	if(param<20)//camera parameters...
	{
		cam->SetParameter((CameraParameter)param, cam->GetParameter((CameraParameter)param)-10);
	}
	else if(param<30 && _calculateDepth)//algorithm parameters...
	{
		float* p = &alg->parameters->P0;
		p[param-20] = p[param-20]-PRAM_STEP ;
		alg->Update_params = true;
	}
	else if(param>=30)
	{
		float* p = &noise_params->P0;
		p[param-30] = p[param-30]-PRAM_STEP;
		PrintParameters(noise_params);
	}
}

void StereoSystem::CaptureNextFrame()
{
	_capturenext = true;
}

void StereoSystem::CloseImageList()
{
	_closelist = true;
}

DWORD WINAPI StereoSystem::CaptureThread(LPVOID instance)
{
	// seed the RNG with current tick count and thread id
	srand(GetTickCount() + GetCurrentThreadId());
	// forward thread to Capture function
	StereoSystem *pThis = (StereoSystem *)instance;
	pThis->Run();
	return 0;
}

void MenuInteraction(StereoSystem *ssys)
{
	cout<<"--------------------------------MENU--------------------------------\n";
	printf("The <ESC> key will exit the program\n");
	printf("Use the following keys to change parameters:\n"
		"\t'g'   - select camera gain parameter\n"
		"\t'e'   - select camera exposure parameter\n"
		"\t'0-9' - select algorithm parameter\n"
		"\ta,s,d,f,z,x - select noise parameter\n"
		"\t'+'   - increment selected parameter\n"
		"\t'-'   - decrement selected parameter\n"      
		"\t'i'   - select Left camera\n"
		"\t'o'   - select Right camera\n"
		"\t'p'   - select Stereo pair\n"		
		"\t'c'   - capture selected source\n"
		"\t'l'   - close all stereo calibration image lists\n");
	cout<<"--------------------------------------------------------------------\n";

	int param = -1, key;
	while((key = cvWaitKey(0)) != 0x1b)
	{
		switch(key)
		{
			case 'g':	case 'G':	printf("Camera parameter Gain\n");			param = GAIN;				break;
			case 'e':	case 'E':	printf("Camera parameter Exposure\n");		param = EXPOSURE;			break;

			case '0':	case '1': case '2': case '3': case '4': 
			case '5':	case '6': case '7': case '8': case '9': 
				cout<< "Algorithm parameter " << (char)key << endl;				param = key-28;					break;

			case 'A': case 'a':
				cout<< "Filter parameter " << "0(a)" << endl;					param = 30;						break;
			case 'S': case 's':
				cout<< "Filter parameter " << "1(s)" << endl;					param = 31;						break;
			case 'D': case 'd':
				cout<< "Filter parameter " << "2(d)" << endl;					param = 32;						break;
			case 'F': case 'f':
				cout<< "Filter parameter " << "3(f)" << endl;					param = 33;						break;
			case 'Z': case 'z':
				cout<< "Filter parameter " << "4(z)" << endl;					param = 34;						break;
			case 'X': case 'x':
				cout<< "Filter parameter " << "5(x)" << endl;					param = 35;						break;

			case '+':	if(ssys)		ssys->IncrementParameter(param);												break;
			case '-':	if(ssys)		ssys->DecrementParameter(param);												break;

			case 'i': case 'I':	if(ssys)		ssys->capture_source = CAM_LEFT;  cout<<"Source: left cam"<<endl;		break;
			case 'o': case 'O':	if(ssys)		ssys->capture_source = CAM_RIGHT;  cout<<"Source: right cam"<<endl;	break;
			case 'p': case 'P':	if(ssys)		ssys->capture_source = CAM_PAIR;  cout<<"Source: pair"<<endl;		break;

			case 'c': case 'C':	
				if(ssys)	ssys->CaptureNextFrame(); 
				switch(ssys->capture_source)
				{
					case CAM_LEFT: cout << ssys->left_cap_count+1 << " left camera images captured."<<endl;		break;
					case CAM_RIGHT: cout << ssys->right_cap_count+1 << " rigth camera images captured."<<endl;	break;
					case CAM_PAIR: cout << ssys->pair_cap_count+1 << " pair images captured."<<endl;				break;
				}
				break;

			case 'l': case 'L':	if(ssys)		ssys->CloseImageList(); cout << "Image lists closed." << endl;	break;
		}
	}
}