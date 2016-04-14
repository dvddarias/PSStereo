#include "main.h"

int WorkWithPictures()
{
	//posible richard pairs: tsukuba, venus, cones, teddy, hci1, hci2
	string pair = "cones";

    Mat imgL, imgR, disp;
	if(!LoadImages(pair.c_str(),"png",imgL,imgR)) {return -1;}
	
	StereoAlgorithm* alg = NULL;

	//----------LOCAL---------
	//alg = new BM_cpu();	
	//alg = new BM_gpu();
	alg = new NewStereo();

	//---------GLOBAL---------	
	//alg = new BP_gpu();
	//alg = new CSBP_gpu();

	//-------NOT USED---------
	//alg = new SGBM_cpu();
	//alg = new VAR_cpu();	
	
	alg->Initialize(Size(imgL.cols,imgL.rows),
					string("D:\\Tutorials & Teaching\\!!TESIS\\Projects\\VSStereo\\PSStereo\\Bin\\CalibrationData\\PSEye\\"));

	DepthMapJudge judge(30,100,500,2500);
	Noiser noiser(imgL.cols,imgL.rows,60,0);	
	DeNoiser denoiser(imgL.cols,imgL.rows,2);
	//denoiser.Denoise(imgL,imgR,NULL);

	ShowImages(imgL,imgR,"Capture");

	printf("The <ESC> key will exit the program\n");
	printf("Use the following keys to change parameters:\n"
	"\t'0-9' - select algorithm parameter\n"
	"\t'+'   - increment selected parameter\n"
	"\t'-'   - decrement selected parameter\n");

	int param = 20, key = -1;	
	float* p;
	double t;
	Vec4f est;
	do
	{		
		switch(key)
		{
			case '0':	case '1': case '2': case '3': case '4': 
			case '5':	case '6': case '7': case '8': case '9': 
				cout<< "Algorithm parameter " << (char)key << endl;				param = key-28;					break;

			case '+':	
				p = &alg->parameters->P0;
				p[param-20] = p[param-20]+PRAM_STEP;
				alg->Update_params = true;
				PrintParameters(alg->parameters);
				break;
			case '-':	
				p = &alg->parameters->P0;
				p[param-20] = p[param-20]-PRAM_STEP;
				alg->Update_params = true;
				PrintParameters(alg->parameters);
				break;
			case 'c':
				t = (double)getTickCount();				
				alg->Compute(imgL,imgR,disp);
				t = ((double)getTickCount() - t)*1000./getTickFrequency();
				printf("          Time: %fms\n",t);	
				printf("           fps: %f\n",1000./t);	

				//est = Vec4f::all(0);
				//est = judge.Measure(alg->GetPointCloud(disp));
				//cout<< "Prom.     Cont: "<<est[0]<<endl;
				//cout<< "Prom. Emptines: "<<est[1]<<endl;
				//cout<< "Prom.  Ouliers: "<<est[2]<<endl;
				//cout<< "Prom.   StdDev: "<<est[3]<<endl;

				imshow("Depth", alg->GetUserFirendlyDisparity(disp));
				break;
			case 'n':
				string dir = string(GetExecutablePath()) + string("..\\..\\Pics\\")+pair+string("\\");
				stringstream ss;
				for(int i =0;i<60;i++)
				{
					cout<<i<<" "; 
					if(i==59)cout<<endl;
					ss.str("");	
					ss<<pair<<"_L"<<1000+i<<".bmp";
					Mat Left = imread(dir+ss.str(),CV_LOAD_IMAGE_GRAYSCALE);
					ss.str("");	
					ss<<pair<<"_R"<<1000+i<<".bmp";
					Mat Right = imread(dir+ss.str(),CV_LOAD_IMAGE_GRAYSCALE);
					//denoiser.Denoise(Left,Right,NULL);
					alg->Compute(Left,Right,disp);
					noiser.Accumulate(alg->GetFloatDisparity(disp));
				}
				break;
		}
	}while((key = cvWaitKey(0)) != 0x1b);

	return 0;
}

int WorkWithCameras()
{
	if(!PSEye::CameraAviable()){ return -1;}
	StereoCamera* cam = new PSEye(CAMERA_MODE, CAMERA_COLOR, CAMERA_SPEED, AUTO_TUNE);		
	//if(!Minoru::CameraAviable()){ return -1;}
	//StereoCamera* cam = new Minoru(CAMERA_MODE, CAMERA_COLOR, CAMERA_SPEED, false);			
	//para calibrar las camaras (tirando o no las fotos)	
	//para calibrar selecciona bien el el tamaño del tablero (esquinas internas) y el SQUARE_SIZE en los parámetros globales
	//si tienes que tirar las fotos primero seleccionas cada camara y el par y tiras las 20 fotos	
	//cierras las 3 listas de fotos (se cierran todas las que tienen más de una foto) y aprietas esc.
	//Calibrate(false,false,BOARD_SIZE,new StereoSystem(cam,NULL));
	
	StereoAlgorithm* alg = NULL;

	//----------LOCAL---------
	alg = new BM_cpu();	
	//alg = new BM_gpu();

	//---------GLOBAL---------	
	//alg = new BP_gpu();
	//alg = new CSBP_gpu();

	//-------NOT USED---------
	//alg = new SGBM_cpu();
	//alg = new VAR_cpu();	
		
	StereoSystem *ssys = new StereoSystem(cam, alg);		

	ssys->StartCaptureLoop();
	MenuInteraction(ssys);
	ssys->StopCaptureLoop();
	cam->Stop();

	delete ssys;	
	return 0;
}

int WorkWithGlasses()
{
	if(!PSEye::CameraAviable()){ return -1;}
	StereoCamera* cam = new PSEye(VGA,COLOR,60,true);

	//if(!Minoru::CameraAviable()){ return -1;}
	//StereoCamera* cam = new Minoru(VGA,COLOR,30,false);

	StereoAlgorithm* alg = new BM_gpu();
	alg->Initialize(Size(640,480),cam->GetCalibrationPath());
	Mat bgra_left(480,640,CV_8UC4);
	Mat bgra_right(480,640,CV_8UC4);
	Mat bgr_anaglyph(480,640,CV_8UC3);
	Mat final_anaglyph;
	Mat left_aux;
	Mat right_aux;
	int disp = 11;
	int up = 0;
	int key = -1;
	Mat M = Mat::zeros(2,3,CV_32F);
	M.at<float>(0,0) = 1;
	M.at<float>(1,1) = 1;
	cam->Start();
	do
	{
		//left, right, up, down keys
		if(key==2555904)disp++;
		if(key==2424832)disp--;
		if(key==2490368)up++;
		if(key==2621440)up--;
		//Capture
		cam->GetStereoPair(bgra_left,bgra_right);
		//Rectification
		alg->imgL = bgra_left;
		alg->imgR = bgra_right;
		alg->RectifyImages();
		bgra_left = alg->imgL;
		bgra_right = alg->imgR;
		//Left & Right Transformation
		M.at<float>(0,2) = disp;
		M.at<float>(1,2) = up;
		warpAffine(bgra_right,right_aux,M,bgra_right.size());
		M.at<float>(0,2) = -disp;
		M.at<float>(1,2) = -up;
		warpAffine(bgra_left,left_aux,M,bgra_left.size());
		//Channel Mix
		int from_to_left[] = { 0,0, 1,1 };
		mixChannels(&right_aux,1,&bgr_anaglyph,1,from_to_left,2);
		int from_to_right[] = { 2,2 };
		mixChannels(&left_aux,1,&bgr_anaglyph,1,from_to_right,1);
		//Select only channels overlap		
		Mat roi(bgr_anaglyph,Rect((disp<0?disp*-1:disp),0,bgr_anaglyph.cols-2*(disp<0?disp*-1:disp),bgr_anaglyph.rows));
		//Resize
		resize(roi,final_anaglyph,Size(roi.cols*2,roi.rows*2));
		//Show
		imshow("Left",left_aux);
		imshow("Right",right_aux);
		imshow("Anaglyph",final_anaglyph);
	}
	while((key = cvWaitKey(1)) != 0x1b);
	cam->Stop();
	delete cam;
	return 0;
}

int main(int argc, char *argv[])
{	
	return WorkWithPictures();
	//return WorkWithCameras();
	//return WorkWithGlasses();
}