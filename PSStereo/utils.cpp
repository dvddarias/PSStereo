#include "utils.h"

//----------------------------------------Functions----------------------------------------------------

string GetExecutablePath()
{
	char strPathName[_MAX_PATH];
	GetModuleFileNameA(NULL,LPSTR(strPathName), _MAX_PATH);	
	*(strrchr(strPathName, '\\') + 1) = '\0';
	return string(strPathName);
}

void ShowImages(Mat& left, Mat& right,const char* label)
{
	Mat final(left.rows,left.cols*2,CV_8UC3);
    Mat roi1(final, Rect(0, 0, left.cols, left.rows ));
    left.copyTo(roi1);
    Mat roi2(final, Rect(left.cols, 0, left.cols, left.rows ));
    right.copyTo(roi2);
    imshow( label, final );
}

int LoadImages(const char* pair,const char* ext,Mat& left,Mat& right)
{
	string path = string(GetExecutablePath()) + string("..\\..\\Pics\\")+string(pair)+string("\\");
    left = imread(path+string("imL.")+string(ext), CV_LOAD_IMAGE_COLOR);
    right = imread(path+string("imR.")+string(ext), CV_LOAD_IMAGE_COLOR);
    if(!left.data||!right.data)
    {
        cout << "Could not open or find the images" << endl;
        return 0;
    }
    return 1;
}

int LoadImage(const char* pair,const char* name,Mat& img)
{
	string path = string(GetExecutablePath()) + string("..\\..\\Pics\\")+string(pair)+string("\\");
    img = imread(path+string(name), CV_LOAD_IMAGE_GRAYSCALE);
    if(!img.data)
    {
        cout << "Could not open or find the image" << endl;
        return 0;
    }
    return 1;
}

float ReadDisparity(int row, int col, Mat& img)
{
	return img.at<uchar>(Point(col,row))/4;
}

void showHistogram(Mat& img)
{
	int bins = 256; // number of bins
	int nc = img.channels(); // number of channels
	vector<Mat> hist(nc); // histogram arrays

	// Initalize histogram arrays
	for (int i = 0; i < hist.size(); i++)
		hist[i] = Mat::zeros(1, bins, CV_32SC1);

	// Calculate the histogram of the image
	for (int i = 0; i < img.rows; i++)
	{
		for (int j = 0; j < img.cols; j++)
		{
			for (int k = 0; k < nc; k++)
			{
				uchar val = nc == 1 ? img.at<uchar>(i,j) : img.at<Vec3b>(i,j)[k];
				hist[k].at<int>(val) += 1;
			}
		}
	}
	// For each histogram arrays, obtain the maximum (peak) value
	// Needed to normalize the display later
	int hmax[3] = {0,0,0};
	for (int i = 0; i < nc; i++)
	{
		for (int j = 0; j < bins-1; j++)
			hmax[i] = hist[i].at<int>(j) > hmax[i] ? hist[i].at<int>(j) : hmax[i];
	}

	const char* wname[3] = { "blue", "green", "red" };
	Scalar colors[3] = { Scalar(255,0,0), Scalar(0,255,0), Scalar(0,0,255) };

	vector<Mat> canvas(nc);

	// Display each histogram in a canvas
	for (int i = 0; i < nc; i++)
	{
		canvas[i] = Mat::ones(125, bins, CV_8UC3);
		for (int j = 0, rows = canvas[i].rows; j < bins-1; j++)
		{
			line(	canvas[i],
					Point(j, rows),
					Point(j, rows - (hist[i].at<int>(j) * rows/hmax[i])),
					nc == 1 ? Scalar(200,200,200) : colors[i],
					1, 8, 0
				);
		}
		imshow(nc == 1 ? "Aux" : wname[i], canvas[i]);
	}
}

void AmplifyColor(Mat& img_disp,Mat& result,int maxdisp)
{
    Mat lookUpTable(1, 256, CV_8U);
    uchar* p = lookUpTable.data;
    for( int i = 0; i <= maxdisp; ++i)
        p[i] = (i*255)/maxdisp;

    LUT(img_disp, lookUpTable, result);
}

double CosAngle(Vec3b v1,Vec3b v2)
{
    return ((double)(v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2]))/(norm(v1)*norm(v2));
}

double Dist(Vec3b v1,Vec3b v2)
{
	return ((double)sqrt(pow((double)v1[0]-v2[0],2)+pow((double)v1[1]-v2[1],2)+pow((double)v1[2]-v2[2],2)));
}

Mat GetDFT(Mat& I)
{ 
    Mat padded;                            //expand input image to optimal size
    int m = getOptimalDFTSize( I.rows );
    int n = getOptimalDFTSize( I.cols ); // on the border add zero values
    copyMakeBorder(I, padded, 0, m - I.rows, 0, n - I.cols, BORDER_CONSTANT, Scalar::all(0));

    Mat planes[] = {Mat_<float>(padded), Mat::zeros(padded.size(), CV_32F)};
    Mat complexI;
    merge(planes, 2, complexI);         // Add to the expanded another plane with zeros

    dft(complexI, complexI);            // this way the result may fit in the source matrix

    // compute the magnitude and switch to logarithmic scale
    // => log(1 + sqrt(Re(DFT(I))^2 + Im(DFT(I))^2))
    split(complexI, planes);                   // planes[0] = Re(DFT(I), planes[1] = Im(DFT(I))
    magnitude(planes[0], planes[1], planes[0]);// planes[0] = magnitude
    Mat magI = planes[0];

    magI += Scalar::all(1);                    // switch to logarithmic scale
    log(magI, magI);

    // crop the spectrum, if it has an odd number of rows or columns
    magI = magI(Rect(0, 0, magI.cols & -2, magI.rows & -2));

    // rearrange the quadrants of Fourier image  so that the origin is at the image center
    int cx = magI.cols/2;
    int cy = magI.rows/2;

    Mat q0(magI, Rect(0, 0, cx, cy));   // Top-Left - Create a ROI per quadrant
    Mat q1(magI, Rect(cx, 0, cx, cy));  // Top-Right
    Mat q2(magI, Rect(0, cy, cx, cy));  // Bottom-Left
    Mat q3(magI, Rect(cx, cy, cx, cy)); // Bottom-Right

    Mat tmp;                           // swap quadrants (Top-Left with Bottom-Right)
    q0.copyTo(tmp);
    q3.copyTo(q0);
    tmp.copyTo(q3);

    q1.copyTo(tmp);                    // swap quadrant (Top-Right with Bottom-Left)
    q2.copyTo(q1);
    tmp.copyTo(q2);

    normalize(magI, magI, 0, 1, CV_MINMAX); // Transform the matrix with float values into a
                                            // viewable image form (float between values 0 and 1).
    
    return magI;    
}

void Wiener2(Mat& src, Mat& dst, int szWindowX, int szWindowY )
{
	int nRows = szWindowY;
	int nCols = szWindowX;
    Mat srcStub;
	Scalar noise_power;

	Mat p_kernel( Size(nCols,nRows), CV_32F, Scalar( 1.0 / (double) (nRows * nCols)) );

	//Now create a temporary holding matrix
	Mat p_tmpMat1(src.rows, src.cols, src.type());
	Mat p_tmpMat2(src.rows, src.cols, src.type());
	Mat p_tmpMat3(src.rows, src.cols, src.type());
	Mat p_tmpMat4(src.rows, src.cols, src.type());
	
	//Local mean of input
	filter2D(src, p_tmpMat1,-1, p_kernel); //localMean

	//Local variance of input
	multiply(src, src, p_tmpMat2);	//in^2
	filter2D(p_tmpMat2, p_tmpMat3,-1, p_kernel);

	//Subtract off local_mean^2 from local variance
	multiply( p_tmpMat1, p_tmpMat1, p_tmpMat4 ); //localMean^2
	subtract( p_tmpMat3, p_tmpMat4, p_tmpMat3 ); //filter(in^2) - localMean^2 ==> localVariance

	//Estimate noise power	
	noise_power = mean(p_tmpMat3);

	// result = local_mean  + ( max(0, localVar - noise) ./ max(localVar, noise)) .* (in - local_mean)

	subtract(src, p_tmpMat1, dst);		     //in - local_mean
	max( p_tmpMat3, noise_power, p_tmpMat2 ); //max(localVar, noise)

	add( p_tmpMat3, -noise_power, p_tmpMat3 ); //localVar - noise
	max( p_tmpMat3, 0, p_tmpMat3 ); // max(0, localVar - noise)

	divide(p_tmpMat3, p_tmpMat2, p_tmpMat3 );  //max(0, localVar-noise) / max(localVar, noise)

	multiply( p_tmpMat3, dst, dst);
	add(dst, p_tmpMat1, dst);
}

Mat Sharper(Mat& img)
{
	Mat blurred; double sigma = 1, threshold = 5, amount = 1;
	GaussianBlur(img, blurred, Size(), sigma, sigma);
	Mat lowContrastMask = abs(img - blurred) < threshold;
	Mat sharpened = img*(1+amount) + blurred*(-amount);
	img.copyTo(sharpened, lowContrastMask);
	return sharpened;
}

void PrintParameters(AlgParameters* params)
{
	cout<< "P0: " << params->P0;
	cout<< ",P1: " << params->P1;
	cout<< ",P2: " << params->P2;
	cout<< ",P3: " << params->P3;
	cout<< ",P4: " << params->P4;
	cout<< ",P5: " << params->P5;
	cout<< ",P6: " << params->P6;
	cout<< ",P7: " << params->P7;
	cout<< ",P8: " << params->P8;
	cout<< ",P9: " << params->P9<<endl;
}

bool CornersVisible(Mat &img, Size boardSize)
{
    const int maxScale = 2;
    vector<Point2f> corners;
    vector<Point3f> objectPoints;
    Size imageSize = img.size();
	bool found = false;

    if(img.empty()) return false;

    for( int scale = 1; scale <= maxScale; scale++ )
    {
        Mat timg;
        if( scale == 1 )
            timg = img;
        else
            resize(img, timg, Size(), scale, scale);
		if(cv::findChessboardCorners(timg, boardSize, corners, CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_NORMALIZE_IMAGE + CV_CALIB_CB_FILTER_QUADS)) return true;
    }
	return false;
}

float Clamp(float param,float minval,float maxval)
{
	if(param>=maxval)return maxval;
	else if(param<=minval)return minval;
	return param;
}

Mat GetGrayRepresentation(Mat disparity)
{
	Mat disp;
	disparity.convertTo(disp,CV_16S);
	Mat result = Mat::zeros(disp.rows,disp.cols,CV_8U);//Gray
	Mat _stddev;
	Mat _mean;
	Mat mask;
	disp.convertTo(mask,CV_8U);
	meanStdDev(disp,_mean,_stddev, mask);
	double stddev = _stddev.at<double>(0);
	double mean = _mean.at<double>(0);
	double range = stddev*2.5;

	MatIterator_<uchar> itgray, endgray;
	MatIterator_<short> it, end;	
	for(	it = disp.begin<short>(),	   end = disp.end<short>(),
			itgray = result.begin<uchar>(),	endgray = result.end<uchar>(); 
			it != end && itgray!= endgray ; 
			++it, ++itgray)
	{

		short val = (*it);		
		if(val<=0) continue;
		if(val>mean+range) (*itgray)=250;
		else if(val<mean-range) (*itgray)=30;
		else
		{
			double value = (double)(val-(mean-range))/(range*2);
			(*itgray)=30+value*220;
		}

	}
	return result;
}