#include "PSEyeDevice.h"

//camera parameters
#define EYE_GRAY (CAMERA_COLOR == CLEYE_MONO_RAW||CAMERA_COLOR == CLEYE_MONO_PROCESSED)
#define EYE_RESOLUTION_X (CAMERA_MODE == CLEYE_QVGA?320:640)
#define EYE_RESOLUTION_Y (CAMERA_MODE == CLEYE_QVGA?240:480)
#define EYE_DEVICE_URI "StereoEye"

#define EYE_COLOR_HORIZONTAL_FOV	(62.0f)
#define EYE_DEPTH_HORIZONTAL_FOV	(58.5f)
#define EYE_COLOR_VERTICAL_FOV		(48.6f)
#define EYE_DEPTH_VERTICAL_FOV		(45.6f)

static const XnInt MAX_SHIFT_VAL = 2047;
static const XnInt PARAM_COEFF_VAL = 4;
static const XnInt SHIFT_SCALE_VAL = 10;
static const XnInt GAIN_VAL = 42;
static const XnInt ZPD_VAL = 120;
static const XnInt CONST_SHIFT_VAL = 200;
static const XnInt DEVICE_MAX_DEPTH_VAL = 10000;
static const XnDouble ZPPS_VAL = 0.10520000010728836;
static const XnDouble LDDIS_VAL = 7.5;

typedef struct  
{
	int refCount;
} EyeStreamFrameCookie;

class EyeStream : public oni::driver::StreamBase
{
public:
	~EyeStream()
	{
		stop();
	}

	OniStatus start()
	{
		xnOSCreateThread(threadFunc, this, &m_threadHandle);

		return ONI_STATUS_OK;
	}

	void stop()
	{
		m_running = false;
	}

	virtual OniStatus SetVideoMode(OniVideoMode*) = 0;
	virtual OniStatus GetVideoMode(OniVideoMode* pVideoMode) = 0;

	OniBool isPropertySupported(int propertyId)
	{
		OniBool status = FALSE;
		switch (propertyId)
		{
		case ONI_STREAM_PROPERTY_CROPPING:
		case ONI_STREAM_PROPERTY_HORIZONTAL_FOV:
		case ONI_STREAM_PROPERTY_VERTICAL_FOV:
		case ONI_STREAM_PROPERTY_VIDEO_MODE:
			status = TRUE;
			break;
		default:
			status = FALSE;
			break;
		}
		return status;
	}

	OniStatus getProperty(int propertyId, void* data, int* pDataSize)
	{
		OniStatus status = ONI_STATUS_NOT_SUPPORTED;
		switch (propertyId)
		{
			case ONI_STREAM_PROPERTY_CROPPING:
				if (*pDataSize != sizeof(OniCropping))
				{
					printf("Unexpected size: %d != %d\n", *pDataSize, sizeof(OniCropping));
					status = ONI_STATUS_ERROR;
				}
				else
				{
					((OniCropping*)data)->enabled = false;
					status = ONI_STATUS_OK;
				}
				break;
			case ONI_STREAM_PROPERTY_HORIZONTAL_FOV:
				{
					float* val = (float*)data;
					XnDouble tmp;
					if (EYE_RESOLUTION_X == 640)
						tmp =  EYE_COLOR_HORIZONTAL_FOV * xnl::Math::DTR;
					else
						tmp = EYE_DEPTH_HORIZONTAL_FOV * xnl::Math::DTR;
					*val = (float)tmp;
					status = ONI_STATUS_OK;
					break;
				}		
			case ONI_STREAM_PROPERTY_VERTICAL_FOV:
				{
					float* val = (float*)data;
					XnDouble tmp;
					if (EYE_RESOLUTION_Y == 480)
						tmp =  EYE_COLOR_VERTICAL_FOV * xnl::Math::DTR;
					else
						tmp = EYE_DEPTH_VERTICAL_FOV * xnl::Math::DTR;
					*val = (float)tmp;
					status = ONI_STATUS_OK;
					break;
				}
			case ONI_STREAM_PROPERTY_VIDEO_MODE:
				{
					if (*pDataSize != sizeof(OniVideoMode))
					{
						printf("Unexpected size: %d != %d\n", *pDataSize, sizeof(OniVideoMode));
						status = ONI_STATUS_ERROR;
					}
					else
					{
						status = GetVideoMode((OniVideoMode*)data);
					}			
					break;
				}		
			default:
				status = ONI_STATUS_NOT_SUPPORTED;
				break;
		}
		return status;
	}

	OniStatus setProperty(int propertyId, const void* data, int dataSize)
	{
		if (propertyId == ONI_STREAM_PROPERTY_VIDEO_MODE)
		{
			if (dataSize != sizeof(OniVideoMode))
			{
				printf("Unexpected size: %d != %d\n", dataSize, (int)sizeof(OniVideoMode));
				return ONI_STATUS_ERROR;
			}
			return SetVideoMode((OniVideoMode*)data);
		}
		return ONI_STATUS_NOT_IMPLEMENTED;
	}

	OniDriverFrame* AcquireFrame(int dataSize)
	{
		OniDriverFrame* pFrame = (OniDriverFrame*)xnOSCalloc(1, sizeof(OniDriverFrame));
		if (pFrame == NULL)
		{
			XN_ASSERT(FALSE);
			return NULL;
		}

		pFrame->frame.data = xnOSMallocAligned(dataSize, XN_DEFAULT_MEM_ALIGN);
		if (pFrame->frame.data == NULL)
		{
			XN_ASSERT(FALSE);
			return NULL;
		}

		pFrame->pDriverCookie = xnOSMalloc(sizeof(EyeStreamFrameCookie));
		((EyeStreamFrameCookie*)pFrame->pDriverCookie)->refCount = 1;

		pFrame->frame.dataSize = dataSize;
		return pFrame;
	}

	void addRefToFrame(OniDriverFrame* pFrame)
	{
		++((EyeStreamFrameCookie*)pFrame->pDriverCookie)->refCount;
	}

	void releaseFrame(OniDriverFrame* pFrame)
	{
		if (0 == --((EyeStreamFrameCookie*)pFrame->pDriverCookie)->refCount)
		{
			xnOSFreeAligned(pFrame->frame.data);
			xnOSFree(pFrame);
		}
	}

	virtual void Mainloop() = 0;
	
	StereoCamera* cam;

protected:
	// Thread
	static XN_THREAD_PROC threadFunc(XN_THREAD_PARAM pThreadParam)
	{
		EyeStream* pStream = (EyeStream*)pThreadParam;
		pStream->m_running = true;
		pStream->Mainloop();

		XN_THREAD_PROC_RETURN(XN_STATUS_OK);
	}
	
	int singleRes(int x, int y) {return y*EYE_RESOLUTION_X+x;}

	bool m_running;

	XN_THREAD_HANDLE m_threadHandle;

};

class EyeDepthStream : public EyeStream
{
public:
	StereoAlgorithm* stereo_alg;

	OniStatus SetVideoMode(OniVideoMode*) {return ONI_STATUS_NOT_IMPLEMENTED;}
	OniStatus GetVideoMode(OniVideoMode* pVideoMode)
	{
		pVideoMode->pixelFormat = ONI_PIXEL_FORMAT_DEPTH_1_MM;
		pVideoMode->fps = cam->_fps;
		pVideoMode->resolutionX = EYE_RESOLUTION_X;
		pVideoMode->resolutionY = EYE_RESOLUTION_Y;
		return ONI_STATUS_OK;
	}
	
	OniStatus getProperty(int propertyId, void* data, int* pDataSize)
	{
		OniStatus status = ONI_STATUS_NOT_SUPPORTED;
		switch (propertyId)
		{
			case XN_STREAM_PROPERTY_CLOSE_RANGE:
			case XN_STREAM_PROPERTY_INPUT_FORMAT:
			case XN_STREAM_PROPERTY_CROPPING_MODE:
			case XN_STREAM_PROPERTY_PIXEL_REGISTRATION:
			case XN_STREAM_PROPERTY_WHITE_BALANCE_ENABLED:
			case XN_STREAM_PROPERTY_HOLE_FILTER:
			case XN_STREAM_PROPERTY_REGISTRATION_TYPE:
			case XN_STREAM_PROPERTY_AGC_BIN:
			case XN_STREAM_PROPERTY_PIXEL_SIZE_FACTOR:
			case XN_STREAM_PROPERTY_DCMOS_RCMOS_DISTANCE:
				return ONI_STATUS_NOT_SUPPORTED;
			case ONI_STREAM_PROPERTY_MAX_VALUE:
				{
					XnInt * val = (XnInt *)data;
					*val = DEVICE_MAX_DEPTH_VAL;
					status = ONI_STATUS_OK;
					break;
				}
			case ONI_STREAM_PROPERTY_MIRRORING:
				{
					XnBool * val = (XnBool *)data;
					*val = TRUE;
					status = ONI_STATUS_OK;
					break;
				}			
			case XN_STREAM_PROPERTY_GAIN:
				{
					XnInt* val = (XnInt*)data;
					*val = GAIN_VAL;
					status = ONI_STATUS_OK;
					break;
				}			
			case XN_STREAM_PROPERTY_CONST_SHIFT:
				{
					XnInt* val = (XnInt*)data;
					*val = CONST_SHIFT_VAL;
					status = ONI_STATUS_OK;
					break;
				}
			case XN_STREAM_PROPERTY_MAX_SHIFT:
				{
					XnInt* val = (XnInt*)data;
					*val = MAX_SHIFT_VAL;
					status = ONI_STATUS_OK;
					break;
				}
			case XN_STREAM_PROPERTY_PARAM_COEFF:
				{
					XnInt* val = (XnInt*)data;
					*val = PARAM_COEFF_VAL;
					status = ONI_STATUS_OK;
					break;
				}
			case XN_STREAM_PROPERTY_SHIFT_SCALE:
				{
					XnInt* val = (XnInt*)data;
					*val = SHIFT_SCALE_VAL;
					status = ONI_STATUS_OK;
					break;
				}
			case XN_STREAM_PROPERTY_ZERO_PLANE_DISTANCE:
				{
					XnInt* val = (XnInt*)data;
					*val = ZPD_VAL;
					status = ONI_STATUS_OK;
					break;
				}
			case XN_STREAM_PROPERTY_ZERO_PLANE_PIXEL_SIZE:
				{
					XnDouble* val = (XnDouble*)data;
					*val = ZPPS_VAL;
					status = ONI_STATUS_OK;
					break;
				}
			case XN_STREAM_PROPERTY_EMITTER_DCMOS_DISTANCE:
				{
					XnDouble* val = (XnDouble*)data;
					*val = LDDIS_VAL;
					status = ONI_STATUS_OK;
					break;
				}
			case XN_STREAM_PROPERTY_S2D_TABLE:
				{
					*pDataSize = sizeof(S2D);
					xnOSMemCopy(data, S2D, sizeof(S2D));
					status = ONI_STATUS_OK;
					break;
					
				}
			case XN_STREAM_PROPERTY_D2S_TABLE:
				{					
					*pDataSize = sizeof(D2S);
					xnOSMemCopy(data, D2S, sizeof(D2S));
					status = ONI_STATUS_OK;					
					break;
				}
			default:
				status = EyeStream::getProperty(propertyId, data, pDataSize);
				break;
		}

		return status;
	}
	
	OniBool isPropertySupported(int propertyId)
	{
		OniBool status = FALSE;
		switch (propertyId)
		{
			case ONI_STREAM_PROPERTY_MAX_VALUE:
			case ONI_STREAM_PROPERTY_MIRRORING:
			case XN_STREAM_PROPERTY_GAIN:
			case XN_STREAM_PROPERTY_CONST_SHIFT:
			case XN_STREAM_PROPERTY_MAX_SHIFT:
			case XN_STREAM_PROPERTY_PARAM_COEFF:
			case XN_STREAM_PROPERTY_SHIFT_SCALE:
			case XN_STREAM_PROPERTY_ZERO_PLANE_DISTANCE:
			case XN_STREAM_PROPERTY_ZERO_PLANE_PIXEL_SIZE:
			case XN_STREAM_PROPERTY_EMITTER_DCMOS_DISTANCE:
			case XN_STREAM_PROPERTY_S2D_TABLE:
			case XN_STREAM_PROPERTY_D2S_TABLE:
				status = FALSE;
			default:
				status = EyeStream::isPropertySupported(propertyId);
				break;
		}
		return status;
	}
	
private:

	void Mainloop()
	{
		int frameId = 1;
		int w, h;
		int channels = EYE_GRAY?1:4;
		cam->GetFrameDimensions(w, h);

		Mat CapImageLeft;
		Mat CapImageRight;
		Mat Left;
		Mat Right;

		Mat distance(h,w,CV_32F);
		Mat xyz;
		Mat disp;

		// Create the OpenCV images
		if(!EYE_GRAY)
		{
			CapImageLeft = Mat(h,w,CV_8UC4);
			CapImageRight = Mat(h,w,CV_8UC4);
			Left = Mat(h,w,CV_8UC3);
			Right = Mat(h,w,CV_8UC3);
		}
		else
		{
			CapImageLeft = Mat(h,w,CV_8UC1);
			CapImageRight = Mat(h,w,CV_8UC1);
		}

		while (m_running)
		{
			OniDriverFrame* pDriverFrame = AcquireFrame(w * h * sizeof(OniDepthPixel));
			OniFrame* pFrame = &pDriverFrame->frame;

			if (pFrame == NULL) {printf("Didn't get frame...\n"); continue;}

			// Capture camera images
			cam->GetStereoPair(CapImageLeft,CapImageRight);
			//if color remove alpha channel
			if(!EYE_GRAY)
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
			// Fill openni frame with 0
			xnOSMemSet(pFrame->data, 0, pFrame->dataSize);
			//Create openCV image wrapper to openNI image
			Mat ONI2DepthMap = Mat(h,w,CV_16U,pFrame->data);
			//compute disparity			
			stereo_alg->Compute(Left,Right,disp);	
			xyz = stereo_alg->GetPointCloud(disp);
			int from_to[] = { 2,0 };
			mixChannels(&xyz,1,&distance,1,from_to,1);
			//move from disparity image to wrapper
			int i,j;
			ushort* p;
			for( i = 0; i < h; ++i)
			{
				p = ONI2DepthMap.ptr<ushort>(i);
				for ( j = 0; j < w; ++j)
				{
					float val = distance.at<float>(Point(j,i));					
					if(val>=4000||val<=400) p[j] = 0;
					else p[j] = (ushort)val;
				}
			}			
			// Fill metadata
			pFrame->frameIndex = frameId;

			pFrame->videoMode.pixelFormat = ONI_PIXEL_FORMAT_DEPTH_1_MM;
			pFrame->videoMode.resolutionX = EYE_RESOLUTION_X;
			pFrame->videoMode.resolutionY = EYE_RESOLUTION_Y;
			pFrame->videoMode.fps = cam->_fps;

			pFrame->width = EYE_RESOLUTION_X;
			pFrame->height = EYE_RESOLUTION_Y;

			pFrame->cropOriginX = pFrame->cropOriginY = 0;
			pFrame->croppingEnabled = FALSE;

			pFrame->sensorType = ONI_SENSOR_DEPTH;
			pFrame->stride = EYE_RESOLUTION_X*sizeof(OniDepthPixel);
			pFrame->timestamp = frameId*33000;

			raiseNewFrame(pDriverFrame);

			frameId++;

			
		}
	}
};

class EyeImageStream : public EyeStream
{
public:

	OniStatus SetVideoMode(OniVideoMode*) {return ONI_STATUS_NOT_IMPLEMENTED;}
	OniStatus GetVideoMode(OniVideoMode* pVideoMode)
	{
		pVideoMode->pixelFormat = ONI_PIXEL_FORMAT_RGB888;
		pVideoMode->fps = CAMERA_SPEED;
		pVideoMode->resolutionX = EYE_RESOLUTION_X;
		pVideoMode->resolutionY = EYE_RESOLUTION_Y;
		return ONI_STATUS_OK;
	}

private:

	void Mainloop()
	{		
		int frameId = 1;
		int w, h;
		int channels = EYE_GRAY?1:4;
		// Get camera frame dimensions
		cam->GetFrameDimensions(w, h);
		Mat cameraImage = Mat(h,w,EYE_GRAY?CV_8UC1:CV_8UC4);
		while (m_running)
		{
			OniDriverFrame* pDriverFrame = AcquireFrame(EYE_RESOLUTION_X*EYE_RESOLUTION_Y*sizeof(OniRGB888Pixel));
			OniFrame* pFrame = &pDriverFrame->frame;

			if (pFrame == NULL) {printf("Didn't get frame...\n"); continue;}

			// Capture camera images				
			cam->GetLeftFrame(cameraImage);

			Mat ONI2ColorImage = Mat(h,w,CV_8UC3,pFrame->data);

			if(EYE_GRAY) cvtColor(cameraImage,ONI2ColorImage,CV_GRAY2RGB);
			else cvtColor(cameraImage,ONI2ColorImage,CV_BGRA2RGB);

			// Fill metadata
			pFrame->frameIndex = frameId;

			pFrame->videoMode.pixelFormat = ONI_PIXEL_FORMAT_RGB888;
			pFrame->videoMode.resolutionX = EYE_RESOLUTION_X;
			pFrame->videoMode.resolutionY = EYE_RESOLUTION_Y;
			pFrame->videoMode.fps = cam->_fps;

			pFrame->width = EYE_RESOLUTION_X;
			pFrame->height = EYE_RESOLUTION_Y;

			pFrame->cropOriginX = pFrame->cropOriginY = 0;
			pFrame->croppingEnabled = FALSE;

			pFrame->sensorType = ONI_SENSOR_COLOR;
			pFrame->stride = EYE_RESOLUTION_X*3;
			pFrame->timestamp = frameId*33000;

			raiseNewFrame(pDriverFrame);

			frameId++;
		}
	}
};

class EyeDevice : public oni::driver::DeviceBase
{
public:
	EyeDevice(OniDeviceInfo* pInfo, oni::driver::DriverServices& driverServices) : m_pInfo(pInfo), m_driverServices(driverServices)
	{
		m_numSensors = 2;
		//Depth sensor
		m_sensors[0].pSupportedVideoModes = XN_NEW_ARR(OniVideoMode, 1);
		m_sensors[0].sensorType = ONI_SENSOR_DEPTH;
		m_sensors[0].numSupportedVideoModes = 1;
		m_sensors[0].pSupportedVideoModes[0].pixelFormat = ONI_PIXEL_FORMAT_DEPTH_1_MM;
		m_sensors[0].pSupportedVideoModes[0].fps = CAMERA_SPEED;
		m_sensors[0].pSupportedVideoModes[0].resolutionX = EYE_RESOLUTION_X;
		m_sensors[0].pSupportedVideoModes[0].resolutionY = EYE_RESOLUTION_Y;
		//Depth color
		m_sensors[1].pSupportedVideoModes = XN_NEW_ARR(OniVideoMode, 1);
		m_sensors[1].sensorType = ONI_SENSOR_COLOR;
		m_sensors[1].numSupportedVideoModes = 1;
		m_sensors[1].pSupportedVideoModes[0].pixelFormat =  ONI_PIXEL_FORMAT_RGB888;
		m_sensors[1].pSupportedVideoModes[0].fps = CAMERA_SPEED;
		m_sensors[1].pSupportedVideoModes[0].resolutionX = EYE_RESOLUTION_X;
		m_sensors[1].pSupportedVideoModes[0].resolutionY = EYE_RESOLUTION_Y;

	}

	OniDeviceInfo* GetInfo()
	{
		return m_pInfo;
	}

	OniStatus getSensorInfoList(OniSensorInfo** pSensors, int* numSensors)
	{
		*numSensors = m_numSensors;
		*pSensors = m_sensors;

		return ONI_STATUS_OK;
	}

	oni::driver::StreamBase* createStream(OniSensorType sensorType)
	{
		if (sensorType == ONI_SENSOR_DEPTH)
		{
			EyeDepthStream* pDepth = XN_NEW(EyeDepthStream);

			pDepth->cam = cam;
			pDepth->stereo_alg = stereo_alg;
				
			return pDepth;
		}
		if (sensorType == ONI_SENSOR_COLOR)
		{
			EyeImageStream* pImage = XN_NEW(EyeImageStream);

			pImage->cam = cam;

			return pImage;
		}

		m_driverServices.errorLoggerAppend("EyeDevice: Can't create a stream of type %d", sensorType);
		return NULL;
	}

	void destroyStream(oni::driver::StreamBase* pStream)
	{ 
		XN_DELETE(pStream); 
	}

	OniStatus  getProperty(int propertyId, void* data, int* pDataSize)
	{
		OniStatus rc = ONI_STATUS_OK;

		switch (propertyId)
		{
		case ONI_DEVICE_PROPERTY_DRIVER_VERSION:
			{
				if (*pDataSize == sizeof(OniVersion))
				{
					OniVersion* version = (OniVersion*)data;
					version->major = version->minor = version->maintenance = version->build = 2;
				}
				else
				{
					m_driverServices.errorLoggerAppend("Unexpected size: %d != %d\n", *pDataSize, sizeof(OniVersion));
					rc = ONI_STATUS_ERROR;
				}
			}
			break;
		default:
			m_driverServices.errorLoggerAppend("Unknown property: %d\n", propertyId);
			rc = ONI_STATUS_ERROR;
		}
		return rc;
	}

	StereoCamera* cam;
	StereoAlgorithm* stereo_alg;

private:

	EyeDevice(const EyeDevice&);
	void operator=(const EyeDevice&);

	OniDeviceInfo* m_pInfo;
	int m_numSensors;
	OniSensorInfo m_sensors[10];
	oni::driver::DriverServices& m_driverServices;
};

class EyeDriver : public oni::driver::DriverBase
{
public:

	EyeDriver(OniDriverServices* pDriverServices) : DriverBase(pDriverServices)
	{}

	virtual oni::driver::DeviceBase* deviceOpen(const char* uri)
	{
		for (xnl::Hash<OniDeviceInfo*, oni::driver::DeviceBase*>::Iterator iter = m_devices.Begin(); iter != m_devices.End(); ++iter)
		{
			if (xnOSStrCmp(iter->Key()->uri, uri) == 0)
			{
				// Found
				if (iter->Value() != NULL)
				{
					// already using
					return iter->Value();
				}

				EyeDevice* pDevice = XN_NEW(EyeDevice, iter->Key(), getServices());
				iter->Value() = pDevice;				
				cam = new PSEye(CAMERA_MODE, CAMERA_COLOR, CAMERA_SPEED, true);
				cam->Start();				
				pDevice->cam = cam;

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

				alg->Initialize(Size(EYE_RESOLUTION_X,EYE_RESOLUTION_Y),cam->GetCalibrationPath());

				pDevice->stereo_alg = alg;

				return pDevice;
			}
		}

		getServices().errorLoggerAppend("Looking for '%s'", uri);
		return NULL;
	}

	virtual void deviceClose(oni::driver::DeviceBase* pDevice)
	{
		for (xnl::Hash<OniDeviceInfo*, oni::driver::DeviceBase*>::Iterator iter = m_devices.Begin(); iter != m_devices.End(); ++iter)
		{
			if (iter->Value() == pDevice)
			{
				iter->Value() = NULL;

				XN_DELETE(pDevice);
				return;
			}
		}

		// not our device?!
		XN_ASSERT(FALSE);
	}

	virtual OniStatus tryDevice(const char* uri)
	{
		if (xnOSStrCmp(uri, EYE_DEVICE_URI) != 0||CLEyeGetCameraCount()!=2)
		{
			return ONI_STATUS_ERROR;
		}

		OniDeviceInfo* pInfo = XN_NEW(OniDeviceInfo);
		xnOSStrCopy(pInfo->uri, uri, ONI_MAX_STR);
		xnOSStrCopy(pInfo->vendor, "Sony", ONI_MAX_STR);
		xnOSStrCopy(pInfo->name, "Stereo PlayStation 3 Eye", ONI_MAX_STR);

		m_devices[pInfo] = NULL;

		deviceConnected(pInfo);

		return ONI_STATUS_OK;
	}

	void shutdown() 
	{		
		cam->Stop();
		delete cam;
	}

	StereoCamera* cam;

protected:

	XN_THREAD_HANDLE m_threadHandle;

	xnl::Hash<OniDeviceInfo*, oni::driver::DeviceBase*> m_devices;
};

ONI_EXPORT_DRIVER(EyeDriver);
