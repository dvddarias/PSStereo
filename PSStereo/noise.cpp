#include "noise.h"

//----------------------------------------Noiser----------------------------------------------------

Noiser::Noiser(int width, int height, int testsize, int skip_amount)
{
	images = new Mat[testsize];
	total = testsize;
	w = width;
	h = height;
	this->skip_amount = skip_amount;
	actual = 0;
	this->ready = true;
}

void Noiser::Accumulate(Mat img)
{
	if(actual<skip_amount||(actual-skip_amount)>=total){actual++; return;}

	Mat eq, aux;
	eq = img;
	//normalize(img,eq);
	//equalizeHist(img,eq);
	eq.convertTo(aux,CV_32F);
	images[actual-skip_amount] = aux.clone();
	actual++;
	if((actual-skip_amount)==total)
	{
		ComputeCleanImage();
		EstimateNoise();
	}
}

void Noiser::ComputeCleanImage()
{
	cleanImage = Mat(h,w,CV_32F,Scalar(0));
	for(int i =0;i<total;i++)
	{
		cleanImage+=images[i];
	}
	cleanImage/=total;
}

void Noiser::EstimateNoise()
{
	cout<<"Estimating noise..."<<endl;
	Mat desv_estandar(h,w,CV_32F,Scalar(0));	
	for(int i =0;i<total;i++)
	{
		//desviacion estandar		
		Mat aux(h,w,CV_32F,Scalar(0));
		pow(cleanImage-images[i],2,aux);
		desv_estandar += aux;
	}
	desv_estandar/=total;
	sqrt(desv_estandar,desv_estandar);
	
	double ruido_promedio =  mean(desv_estandar).val[0];
	double ruido_peor;
	minMaxLoc(desv_estandar,NULL,&ruido_peor);

	Mat aux;
	meanStdDev(cleanImage,noArray(),aux);
	double signal = aux.at<double>(0);

	meanStdDev(desv_estandar,noArray(),aux);
	double noise = aux.at<double>(0);

	double SNR = 10*log10(signal/noise);

	cout<<"Ruido Promedio: "<<ruido_promedio<<endl;
	cout<<"Ruido Peor: "<<ruido_peor<<endl;
	cout<<"SNR: "<<SNR<<" db"<<endl;
	this->ready = false;
	

}

void Noiser::Reset()
{
	if(ready) return;
	images = new Mat[total];
	actual = 0;
	this->ready = true;
}

//----------------------------------------DeNoiser----------------------------------------------------

DeNoiser::DeNoiser(int width, int height, int history_size)
{
	images = new Mat*[history_size];	
	w= width;
	h= height;

	for(int i=0;i<history_size;++i){ images[i] = new Mat(h,w,CV_8U,Scalar::all(0)); }
	Accum = Mat(h,w,CV_32FC1,Scalar(0));
	image_count = 0;
	first_time = true;
	hist_size= history_size;
}

void  DeNoiser::DeNoiseSemiStatic(Mat &image, AlgParameters* params) 
{
	if(params==NULL)
	{
		params = new AlgParameters();
		params->P0 = 1;
		params->P1 = 3;
		params->P2 = 1;
		params->P3 = 0;
		params->P4 = 10;
		params->P5 = 0;
		params->P6 = 0;
		params->P7 = 0;
		params->P8 = 0;
		params->P9 = 0;	
	}
	long t = getTickCount();
	if(params->P0>0)
	{		
		for(int i = 0;i<params->P1;i++)	medianBlur(image,image,params->P2*2+1);
	}
	if(params->P3 >0)
	{
		if(image_count<hist_size-1)
		{ 
			image.copyTo(*images[(hist_size-1)-image_count]);		
			image_count++;
		}
		else
		{
			//guardar el actual en el inicio del historial
			image.copyTo(*images[0]);	
			//poner el acumulador igual que la imagen
			image.convertTo(Accum,Accum.type());		
		
			int i,j,img,count;
			Mat* actual;
			uchar value;
			uchar base;
			float accum;
			for( i = 0; i < h; ++i)
			{
				for ( j = 0; j < w; ++j)
				{
					base = image.at<uchar>(i,j);
					accum = base;
					count = 1;
					for(img = 1; img<hist_size; ++img)
					{
						actual = images[img];
						value = actual->at<uchar>(i,j);
						if(abs(value-base)>params->P4) break;
						count++;
						accum+=value;
					}
					Accum.at<float>(i,j) = accum/count;
				}
			}

			//poner el resultado en la imagen
			Accum.convertTo(image,image.type());
		
			//correr las imagenes
			Mat* last = images[hist_size-1];
			for(int i =hist_size-1;i>0;i--) images[i] = images[i-1];
			images[0] = last;
		}
	}

	t = getTickCount() - t;
	float time = t*1000/getTickFrequency();
	if(first_time)
	{
		printf("Noise time: %fms, Fps: %f\n",time,1000/time);
		first_time = false;
	}
}

void DeNoiser::DeNoiseStatic(Mat &image, AlgParameters* params)
{
	if(params==NULL)
	{
		params = new AlgParameters();
		params->P0 = 1;
		params->P1 = 3;
		params->P2 = 1;
		params->P3 = 0;
		params->P4 = 10;
		params->P5 = 0;
		params->P6 = 0;
		params->P7 = 0;
		params->P8 = 0;
		params->P9 = 0;	
	}
	long t = getTickCount();
	if(params->P0>0)
	{		
		for(int i = 0;i<params->P1;i++)	medianBlur(image,image,params->P2*2+1);
	}
	if(params->P3 >0)
	{
		if(image_count<hist_size-1)
		{
			accumulate(image,Accum);
			image.copyTo(*images[(hist_size-1)-image_count]);		
			image_count++;
		}
		else
		{
			//acumular el frame actual
			accumulate(image,Accum);
			//guardar el actual en el inicio del historial
			image.copyTo(*images[0]);	
			//poner como resultado el prom de los history;
			Accum.convertTo(image,image.type(),1.0/hist_size);
			//en este punto la imágen esta lista, falta correr el historial	
			subtract(Accum,*images[hist_size-1],Accum,noArray(),CV_32F);
			//correr las imagenes
			Mat* last = images[hist_size-1];
			for(int i =hist_size-1;i>0;i--) images[i] = images[i-1];
			images[0] = last;
		}
	}

	t = getTickCount() - t;
	float time = t*1000/getTickFrequency();
	if(first_time)
	{
		printf("Noise time: %fms, Fps: %f\n",time,1000/time);
		first_time = false;
	}
}

//----------------------------------------DepthMapJudge----------------------------------------------------

DepthMapJudge::DepthMapJudge(int total, float radius, float minZ,float maxZ)
{
	this->total = total;
	this->actual = 0;	
	this->accumulated = Vec4f::all(0);
	this->radius = radius;
	this->minZ = minZ;
	this->maxZ = maxZ;
	this->ws = 5;
	this->points= Mat(1,pow((float)2*ws+1,2),CV_32F,Scalar::all(0));
	
}

void DepthMapJudge::Judge(Mat xyz)
{
	if(actual>=total) return;
	actual++;
	accumulated+=Measure(xyz);
	if(actual==total)
	{
		cout<<"In "<<total<<"depth maps:"<<endl;
		cout<<"Prom. Cont:  "<<accumulated[0]/total<<endl;
		cout<<"Prom. Emptines: "<<accumulated[1]/total<<endl;
		cout<<"Prom. Outliers: "<<accumulated[2]/total<<endl;
		cout<<"Prom. DevStd:   "<<accumulated[3]/total<<endl;
	}	
}

Vec4f DepthMapJudge::Measure(Mat xyz)
{
	float density = 0;
	float emptines = 0;
	float outliers = 0;
	float stdev = 0;
	int amount = 0;
	const double max_z = 10000;

    for(int y = 0; y < xyz.rows; y++)
    {
        for(int x = 0; x < xyz.cols; x++)
        {			
			Vec3f point = xyz.at<Vec3f>(y, x);
            if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z || point[2]<=0 ){ emptines++; continue; }
			if(point[2]<=minZ||point[2]>=maxZ){ outliers++; }
			Vec2f aux = CalculateNeighbours(point,x,y,xyz);
			density+= aux[0]/pow((float)2*ws+1,2);
			stdev+=aux[1];
			amount++;
        }
    } 
	return Vec4f(density/amount,emptines/(xyz.cols*xyz.rows),outliers/(xyz.cols*xyz.rows),stdev/(amount*radius));
}

Vec2f DepthMapJudge::CalculateNeighbours(Vec3f center, int cx, int cy, Mat xyz)
{
	int count = 0;
	const double max_z = 10000;
	for(int y = max(0,cy-ws); y < xyz.rows && y<=cy+ws; y++)
    {
        for(int x = max(0,cx-ws); x < xyz.cols && x<=cx+ws; x++)
        {			
			Vec3f point = xyz.at<Vec3f>(y, x);
            if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue; 
			if(norm(point-center)<=radius)
			{
				points.at<float>(0,count) = point[2];
				count++;
			}
        }
    } 
	//create a mask that selects the non zero vlues in points
	Mat mask(points.rows,points.cols,CV_8U,Scalar::all(0));
	for(int i=0;i<count;i++){ mask.at<uchar>(0,i)=255;	}
	//calculate standard deviation
	Mat aux;
	meanStdDev(points,noArray(),aux,mask);
	double stdev = aux.at<double>(0);
	//clean points
	bitwise_and(points,0,points);

	return Vec2f(count,stdev);
}
