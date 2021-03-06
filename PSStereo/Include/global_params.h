#ifndef GLOBAL_PARAMS_H
#define GLOBAL_PARAMS_H

#include "stereo_camera.h"

#define USE_CALIBRATION false
#define STORE_RESULTS	false
#define SQUARE_SIZE		53.0f
#define BOARD_SIZE		Size(13,10)

#define WORK_SCALE		1.0f
#define SHOW_SCALE		1.0f

#define CAMERA_SPEED	25
#define AUTO_TUNE		true
#define CAMERA_MODE		VGA
#define CAMERA_COLOR	MONO

#define PRAM_STEP		1.0f

#endif                                                                                                                   left,Mat right,Mat& disp);
	void Initialize(Size s,string calibration_path);
	void SetParamaters();
	void PrintParamsDescription();
	void BuildDefaultParameters();

	Mat GetUserFirendlyDisparity(Mat disp);
	Mat GetFloatDisparity(Mat disp);
};

#endif                                                                                                                                                                                                                                                                              _MBCS                  _CPPUNWIND                              =                  Q                   b   	   Release\               ?   C:\Program Files (x86)\Microsoft Visual Studio 10.0\VC\include               F   C:\Program Files (x86)\Microsoft Visual Studio 10.0\VC\atlmfc\include               <   C:\Program Files (x86)\Microsoft SDKs\Windows\v7.0A\include               =   C:\Program Files (x86)\Microsoft SDKs\Windows\v7.0A\\include            <   B   C:\Program Files (x86)\Microsoft Visual Studio 10.0\VC\atlmfc\lib            <   ;   C:\Program