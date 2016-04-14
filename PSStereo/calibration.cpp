#include "calibration.h"

bool Calibrate(bool getpictures,bool use_pairs_only,Size boardsize, StereoSystem* ssys)
{
	SetCurrentDirectoryA((ssys->cam->GetCalibrationPath()).c_str());
	if(getpictures)
	{	
		ssys->StartCaptureLoop();
		MenuInteraction(ssys);
		ssys->StopCaptureLoop();
		delete ssys;
	}

	string left_images_listfn = "left_image_list.xml";
	string right_images_listfn = "right_image_list.xml";
	string pair_images_listfn = "pair_image_list.xml";
    vector<string> leftimagelist;
	vector<string> rightimagelist;
	vector<string> pairimagelist;
	if(	!readStringList(left_images_listfn, leftimagelist)||
		!readStringList(right_images_listfn, rightimagelist)||
		!readStringList(pair_images_listfn, pairimagelist)||
		leftimagelist.empty()||
		rightimagelist.empty()||
		pairimagelist.empty())
    {
        cout << "can not open images list or the some of the string lists are empty" << endl;
        return 0;
    }	

	if(use_pairs_only) StereoCalib(pairimagelist, boardsize,0, true, true);
	else
	{
		Mat left_matrix;
		Mat left_coeff;
		CalibrateSingleCamera(leftimagelist,boardsize,left_matrix,left_coeff);
		Mat right_matrix;
		Mat right_coeff;
		CalibrateSingleCamera(rightimagelist,boardsize,right_matrix,right_coeff);
		Mat calibMatrix[4];
		calibMatrix[0]= left_matrix;
		calibMatrix[1]= left_coeff;
		calibMatrix[2]= right_matrix;
		calibMatrix[3]= right_coeff;
		StereoCalib(pairimagelist, boardsize, calibMatrix, true, true);
	}

	cout<<"Calibration finished...";
    return 1;
}

void CalibrateSingleCamera(const vector<string>& imagelist, Size boardSize, Mat& cameraMatrix, Mat& distCoeffs)
{
    bool displayCorners = true;
    const int maxScale = 2;
    const float squareSize = 1.0f; 
    vector< vector<Point2f> > imagePoints;
    vector< vector<Point3f> > objectPoints;
    Size imageSize;

    int i, j, k, nimages = (int)imagelist.size();

    imagePoints.resize(nimages);
    vector<string> goodImageList;

    for( i = j = 0; i < nimages; i++ )
    {               
            const string& filename = imagelist[i];
            Mat img = imread(filename, 0);
            if(img.empty())
                continue;
            if( imageSize == Size() )
                imageSize = img.size();
            else if( img.size() != imageSize )
            {
                cout << "The image " << filename << " has the size different from the previous image size. Skipping the image\n";
                continue;
            }
            bool found = false;
            vector<Point2f>& corners = imagePoints[j];
            for( int scale = 1; scale <= maxScale; scale++ )
            {
                Mat timg;
                if( scale == 1 )
                    timg = img;
                else
                    resize(img, timg, Size(), scale, scale);
				found = cv::findChessboardCorners(timg, boardSize, corners, CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_NORMALIZE_IMAGE + CV_CALIB_CB_FILTER_QUADS);
                if(found)
                {
                    if( scale > 1 )
                    {
                        Mat cornersMat(corners);
                        cornersMat *= 1./scale;
                    }
                    break;
                }
            }
			if(found) cornerSubPix(img, corners, Size(11,11), Size(-1,-1),
								   TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,
                                   60, 0.001));
            if( displayCorners )
            {
                cout << filename << endl;
                Mat cimg, cimg1;
                cvtColor(img, cimg, CV_GRAY2BGR);
                drawChessboardCorners(cimg, boardSize, corners, found);
                double sf = 640./MAX(img.rows, img.cols);
                resize(cimg, cimg1, Size(), sf, sf);
                imshow("corners", cimg1);
                char c = (char)waitKey(1000);
                if( c == 27 || c == 'q' || c == 'Q' ) //Allow ESC to quit
                    exit(-1);
            }
            else putchar('.');
            if(!found) continue;			
            goodImageList.push_back(imagelist[i]);
			j++;
    }
    cout << j << " images have been successfully detected.\n";
    nimages = j;
    if( nimages < 2 )
    {
        cout << "Error: too little images to run the calibration\n";
        return;
    }
    imagePoints.resize(nimages);
    objectPoints.resize(nimages);

    for( i = 0; i < nimages; i++ )
    {
        for( j = 0; j < boardSize.height; j++ )
            for( k = 0; k < boardSize.width; k++ )
                objectPoints[i].push_back(Point3f(float(j*squareSize),float( k*squareSize), 0));
    }

    cout << "Running calibration ...\n";
	
	vector<Mat> rvecs, tvecs;
    cameraMatrix = Mat::eye(3, 3, CV_64F);
	distCoeffs;

	double repr_err = calibrateCamera(objectPoints,imagePoints,imageSize,cameraMatrix,distCoeffs,rvecs,tvecs,CV_CALIB_FIX_ASPECT_RATIO+CV_CALIB_FIX_K3);
	
    cout << "done with RMS error=" << repr_err << endl;

	const string& example = imagelist[0];
    Mat img = imread(example, 0);
	Mat view;
	undistort(img, view, cameraMatrix, distCoeffs);
	imshow("Undisorted",view);
	cvWaitKey();
}

void StereoCalib(const vector<string>& imagelist, Size boardSize, Mat calibmatix[]=0, bool useCalibrated=true, bool showRectified=true)
{
    if( imagelist.size() % 2 != 0 )
    {
        cout << "Error: the image list contains odd (non-even) number of elements\n";
        return;
    }

    bool displayCorners = true;
    const int maxScale = 2;
    const float squareSize = 1.0f;  // Set this to your actual square size
    // ARRAY AND VECTOR STORAGE:

    vector< vector<Point2f> > imagePoints[2];
    vector< vector<Point3f> > objectPoints;
    Size imageSize;

    int i, j, k, nimages = (int)imagelist.size()/2;

    imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);
    vector<string> goodImageList;

    for( i = j = 0; i < nimages; i++ )
    {
        for( k = 0; k < 2; k++ )
        {
            const string& filename = imagelist[i*2+k];
            Mat img = imread(filename, 0);
            if(img.empty())
                break;
            if( imageSize == Size() )
                imageSize = img.size();
            else if( img.size() != imageSize )
            {
                cout << "The image " << filename << " has the size different from the first image size. Skipping the pair\n";
                break;
            }
            bool found = false;
            vector<Point2f>& corners = imagePoints[k][j];
            for( int scale = 1; scale <= maxScale; scale++ )
            {
                Mat timg;
                if( scale == 1 )
                    timg = img;
                else
                    resize(img, timg, Size(), scale, scale);
				found = cv::findChessboardCorners(timg, boardSize, corners, CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_NORMALIZE_IMAGE + CV_CALIB_CB_FILTER_QUADS);
                if(found)
                {
                    if( scale > 1 )
                    {
                        Mat cornersMat(corners);
                        cornersMat *= 1./scale;
                    }
                    break;
                }
            }
			if(found) cornerSubPix(img, corners, Size(11,11), Size(-1,-1),
								   TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,
                                   60, 0.001));
            if( displayCorners )
            {
                cout << filename << endl;
                Mat cimg, cimg1;
                cvtColor(img, cimg, CV_GRAY2BGR);
                drawChessboardCorners(cimg, boardSize, corners, found);
                double sf = 640./MAX(img.rows, img.cols);
                resize(cimg, cimg1, Size(), sf, sf);
                imshow("corners", cimg1);
                char c = (char)waitKey(1000);
                if( c == 27 || c == 'q' || c == 'Q' ) //Allow ESC to quit
                    exit(-1);
            }
            else putchar('.');
            if(!found) break;			
        }
        if( k == 2 )
        {
            goodImageList.push_back(imagelist[i*2]);
            goodImageList.push_back(imagelist[i*2+1]);
            j++;
        }
    }
    cout << j << " pairs have been successfully detected.\n";
    nimages = j;
    if( nimages < 2 )
    {
        cout << "Error: too little pairs to run the calibration\n";
        return;
    }

    imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);
    objectPoints.resize(nimages);

    for( i = 0; i < nimages; i++ )
    {
        for( j = 0; j < boardSize.height; j++ )
            for( k = 0; k < boardSize.width; k++ )
                objectPoints[i].push_back(Point3f(float(j*squareSize),float( k*squareSize), 0));
    }

    cout << "Running stereo calibration ...\n";

	int flags;
    Mat cameraMatrix[2];	
	Mat distCoeffs[2];
	cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
    cameraMatrix[1] = Mat::eye(3, 3, CV_64F);	
	flags = CV_CALIB_FIX_ASPECT_RATIO + 			
			CV_CALIB_ZERO_TANGENT_DIST + 
			CV_CALIB_SAME_FOCAL_LENGTH +
			CV_CALIB_FIX_K3;

	if(calibmatix!=0)//if cameras were calibrated independently
	{    
		flags = CV_CALIB_FIX_INTRINSIC;
		cameraMatrix[0] = calibmatix[0];
		cameraMatrix[1] = calibmatix[2];
		distCoeffs[0] = calibmatix[1];
		distCoeffs[1] = calibmatix[3];
	}
    Mat R, T, E, F;

    double rms = stereoCalibrate(objectPoints,
					imagePoints[0], imagePoints[1],
                    cameraMatrix[0], distCoeffs[0],
                    cameraMatrix[1], distCoeffs[1],
                    imageSize, R, T, E, F,
                    TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
					flags);
    cout << "done with RMS error=" << rms << endl;

// CALIBRATION QUALITY CHECK
// because the output fundamental matrix implicitly
// includes all the output information,
// we can check the quality of calibration using the
// epipolar geometry constraint: m2^t*F*m1=0
    double err = 0;
    int npoints = 0;
    vector<Vec3f> lines[2];
    for( i = 0; i < nimages; i++ )
    {
        int npt = (int)imagePoints[0][i].size();
        Mat imgpt[2];
        for( k = 0; k < 2; k++ )
        {
            imgpt[k] = Mat(imagePoints[k][i]);
            undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
            computeCorrespondEpilines(imgpt[k], k+1, F, lines[k]);
        }
        for( j = 0; j < npt; j++ )
        {
            double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
                                imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
                           fabs(imagePoints[1][i][j].x*lines[0][j][0] +
                                imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
            err += errij;
        }
        npoints += npt;
    }
    cout << "average reprojection err = " <<  err/npoints << endl;

    // save intrinsic parameters
    FileStorage fs("intrinsics.yml", CV_STORAGE_WRITE);
    if( fs.isOpened() )
    {
        fs << "M1" << cameraMatrix[0];
		fs << "D1" << distCoeffs[0];
		fs << "M2" << cameraMatrix[1];
		fs << "D2" << distCoeffs[1];
        fs.release();
    }
    else
        cout << "Error: can not save the intrinsic parameters\n";

    Mat R1, R2, P1, P2, Q;
    Rect validRoi[2];

    stereoRectify(cameraMatrix[0], distCoeffs[0],
                  cameraMatrix[1], distCoeffs[1],
                  imageSize, R, T, R1, R2, P1, P2, Q,
                  CALIB_ZERO_DISPARITY, -1, imageSize, &validRoi[0], &validRoi[1]);

    fs.open("extrinsics.yml", CV_STORAGE_WRITE);
    if( fs.isOpened() )
    {
        fs << "R" << R;
		fs << "T" << T;
		fs << "R1" << R1;
		fs << "R2" << R2;
		fs << "P1" << P1;
		fs << "P2" << P2;
		fs << "Q" << Q;
        fs.release();
    }
    else
        cout << "Error: can not save the intrinsic parameters\n";

    // OpenCV can handle left-right
    // or up-down camera arrangements
    bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));

// COMPUTE AND DISPLAY RECTIFICATION
    if( !showRectified )
        return;

    Mat rmap[2][2];
// IF BY CALIBRATED (BOUGUET'S METHOD)
    if( useCalibrated )
    {
        // we already computed everything
    }
// OR ELSE HARTLEY'S METHOD
    else
 // use intrinsic parameters of each camera, but
 // compute the rectification transformation directly
 // from the fundamental matrix
    {
        vector<Point2f> allimgpt[2];
        for( k = 0; k < 2; k++ )
        {
            for( i = 0; i < nimages; i++ )
                std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));
        }
        F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
        Mat H1, H2;
        stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, imageSize, H1, H2, 3);

        R1 = cameraMatrix[0].inv()*H1*cameraMatrix[0];
        R2 = cameraMatrix[1].inv()*H2*cameraMatrix[1];
        P1 = cameraMatrix[0];
        P2 = cameraMatrix[1];
    }

    //Precompute maps for cv::remap()
    initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
    initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

    Mat canvas;
    double sf;
    int w, h;
    if( !isVerticalStereo )
    {
        sf = 600./MAX(imageSize.width, imageSize.height);
        w = cvRound(imageSize.width*sf);
        h = cvRound(imageSize.height*sf);
        canvas.create(h, w*2, CV_8UC3);
    }
    else
    {
        sf = 300./MAX(imageSize.width, imageSize.height);
        w = cvRound(imageSize.width*sf);
        h = cvRound(imageSize.height*sf);
        canvas.create(h* 2, w, CV_8UC3);
    }

    for( i = 0; i < nimages; i++ )
    {
        for( k = 0; k < 2; k++ )
        {
            Mat img = imread(goodImageList[i*2+k], 0), rimg, cimg;
            remap(img, rimg, rmap[k][0], rmap[k][1], CV_INTER_LINEAR);
            cvtColor(rimg, cimg, CV_GRAY2BGR);
            Mat canvasPart = !isVerticalStereo ? canvas(Rect(w*k, 0, w, h)) : canvas(Rect(0, h*k, w, h));
            resize(cimg, canvasPart, canvasPart.size(), 0, 0, CV_INTER_AREA);
            if( useCalibrated )
            {
                Rect vroi(cvRound(validRoi[k].x*sf), cvRound(validRoi[k].y*sf),
                          cvRound(validRoi[k].width*sf), cvRound(validRoi[k].height*sf));
                rectangle(canvasPart, vroi, Scalar(0,0,255), 3, 8);
            }
        }

        if( !isVerticalStereo)
            for( j = 0; j < canvas.rows; j += 16 )
                line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
        else
            for( j = 0; j < canvas.cols; j += 16 )
                line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);
		if(showRectified) imshow("rectified", canvas);
        char c = (char)waitKey();
        if( c == 27 || c == 'q' || c == 'Q' )
            break;
    }
}

bool readStringList( const string& filename, vector<string>& l )
{	
    l.resize(0);
    FileStorage fs(filename, FileStorage::READ);
    if( !fs.isOpened() )
        return false;
    FileNode n = fs.getFirstTopLevelNode();
    if( n.type() != FileNode::SEQ )
        return false;
    FileNodeIterator it = n.begin(), it_end = n.end();
    for( ; it != it_end; ++it )
        l.push_back((string)*it);
    return true;
}