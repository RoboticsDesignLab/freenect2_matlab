#include "freenect2.h"

int freenect2::countDevices(libfreenect2::Freenect2 *context) {
	return context->enumerateDevices();
}

freenect2::DeviceStruct* freenect2::getDevice(libfreenect2::Freenect2 *context, const char *serial, PacketPipeline pipeline) {
	freenect2::DeviceStruct *d = new freenect2::DeviceStruct;

	std::string deviceSerial = (serial == nullptr) ? "" : std::string(serial);

	if (deviceSerial == "")
	{
		deviceSerial = context->getDefaultDeviceSerialNumber();
	}

	switch (pipeline)
	{
	case freenect2::OPENGL:
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
		d->pipeline = new libfreenect2::OpenGLPacketPipeline();
		break;
#else
		return nullptr;
#endif
	case freenect2::CUDA:
#ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
		d->pipeline = new libfreenect2::CudaPacketPipeline();
		break;
#else
		return nullptr;
#endif
	case freenect2::OPENCL:
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
		d->pipeline = new libfreenect2::OpenCLPacketPipeline();
		break;
#else
		return nullptr;
#endif
	case freenect2::CPU:
	default:
		d->pipeline = new libfreenect2::CpuPacketPipeline();
		break;
	}

	d->kinect = context->openDevice(deviceSerial, d->pipeline);

	if (d->kinect == 0) {
		return nullptr;
	}

	d->frames = new freenect2::FrameStruct();

	return d;
}

void freenect2::initializeDevice(DeviceStruct * d, int listener_flags)
{
	d->listener = new libfreenect2::SyncMultiFrameListener(listener_flags);
	d->kinect->setColorFrameListener(d->listener);
	d->kinect->setIrAndDepthFrameListener(d->listener);

	bool success = d->kinect->start();

	if (!success){
		return;
	}

	d->registration = new libfreenect2::Registration(d->kinect->getIrCameraParams(), d->kinect->getColorCameraParams());
	d->frames->undistorted = new libfreenect2::Frame(512, 424, 4);
	d->frames->undistorted->format = libfreenect2::Frame::Format::Float;

	d->frames->registered = new libfreenect2::Frame(512, 424, 4);
	d->frames->registered->format = libfreenect2::Frame::Format::BGRX;

	d->frames->bigdepth = new libfreenect2::Frame(1920, 1082, 4);
	d->frames->bigdepth->format = libfreenect2::Frame::Format::Float;
}

void freenect2::deleteDevice(DeviceStruct * device)
{
	bool result = 0;

	result = device->kinect->stop();

	result = device->kinect->close();

	if (device->frameMap.size() > 0) {
		cleanupFrame(device);
	}

	delete device->frames->undistorted;
	device->frames->undistorted = nullptr;

	delete device->frames->registered;
	device->frames->registered = nullptr;

	delete device->frames->bigdepth;
	device->frames->bigdepth = nullptr;

	delete device->frames;
	device->frames = nullptr;

	delete device->registration;
	device->registration = nullptr;

	delete device->listener;
	device->listener = nullptr;

	delete device->kinect;
	device->kinect = nullptr;

	delete device;
}

freenect2::FrameStruct * freenect2::processFrame(DeviceStruct * d, int timeout)
{
	if (!d->listener->hasNewFrame()) {
		return nullptr;
	}

	if (timeout > 0) {
		if (!d->listener->waitForNewFrame(d->frameMap, timeout)) {
			return nullptr;
		}
	}
	else {
		d->listener->waitForNewFrame(d->frameMap);
	}

	d->frames->rgb = d->frameMap[libfreenect2::Frame::Color];
	d->frames->ir = d->frameMap[libfreenect2::Frame::Ir];
	d->frames->depth = d->frameMap[libfreenect2::Frame::Depth];

	//d->registration->apply(d->frames->rgb, d->frames->depth, d->frames->undistorted, d->frames->registered, true, d->frames->bigdepth);
	//d->registration->apply(d->frames->rgb, d->frames->depth, d->frames->undistorted, d->frames->registered);

	return d->frames;
}

mxArray * freenect2::getFrame(DeviceStruct * device, FrameType type)
{

	libfreenect2::Frame *frame = nullptr;
	size_t dims = 2;
	size_t size[3];
	mxClassID classId = mxUINT8_CLASS;

	switch (type)
	{
	case freenect2::COLOR:
		frame = device->frames->rgb;
		break;
	case freenect2::IR:
		frame = device->frames->ir;
		break;
	case freenect2::DEPTH:
		frame = device->frames->depth;
		break;
	case freenect2::UNDISTORTED:
		frame = device->frames->undistorted;
		break;
	case freenect2::REGISTERED:
		frame = device->frames->registered;
		break;
	case freenect2::BIGDEPTH:
		frame = device->frames->bigdepth;
		break;
	default:
		break;
	}

	switch (frame->format)
	{
	case libfreenect2::Frame::Format::Raw:
		return nullptr;
	case libfreenect2::Frame::Format::Float:
		classId = mxSINGLE_CLASS;
		size[0] = 1;
		break;
	case libfreenect2::Frame::Format::BGRX:
	case libfreenect2::Frame::Format::RGBX:
		classId = mxUINT8_CLASS;
		dims = 3;
		size[0] = 4;
		break;
	case libfreenect2::Frame::Format::Gray:
		classId = mxUINT8_CLASS;
		size[0] = frame->bytes_per_pixel;
		break;
	default:
		return nullptr;
	}

	size[1] = frame->width;
	size[2] = frame->height;

	mxArray *result = mxCreateUninitNumericArray((size[0] > 0) ? 3 : 2, (size[0] > 0) ? size : &size[1], classId, mxREAL);
	unsigned char * out = (unsigned char*)mxGetData(result);
	unsigned char * in = frame->data;

	memcpy(out, in, frame->width * frame->height * frame->bytes_per_pixel);

	return result;

}

void freenect2::cleanupFrame(DeviceStruct * d) {
	d->listener->release(d->frameMap);
}

bool test_protonect_code() {

	/// [context]
	libfreenect2::Freenect2 freenect2;
	libfreenect2::Freenect2Device *dev = 0;
	libfreenect2::PacketPipeline *pipeline = 0;
	/// [context]

	std::string serial = "";

	bool viewer_enabled = true;
	bool enable_rgb = true;
	bool enable_depth = true;
	int deviceId = -1;
	size_t framemax = -1;

	/// [discovery]
	if (freenect2.enumerateDevices() == 0)
	{
		return false;
	}

	serial = freenect2.getDefaultDeviceSerialNumber();



	dev = freenect2.openDevice(serial);

	if (dev == 0)
	{
		return false;
	}

	/// [listeners]
	int types = 0;
	if (enable_rgb)
		types |= libfreenect2::Frame::Color;
	if (enable_depth)
		types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
	libfreenect2::SyncMultiFrameListener listener(types);
	libfreenect2::FrameMap frames;

	dev->setColorFrameListener(&listener);
	dev->setIrAndDepthFrameListener(&listener);
	/// [listeners]

	/// [start]
	if (!dev->start())
		return false;

	/// [start]

	/// [registration setup]
	libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
	libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
	/// [registration setup]

	size_t framecount = 0;

	/// [loop start]
	while (true)
	{
		if (!listener.waitForNewFrame(frames, 10 * 1000)) // 10 sconds
		{
			return false;
		}
		libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
		libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
		libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
		/// [loop start]

		if (enable_rgb && enable_depth)
		{
			/// [registration]
			registration->apply(rgb, depth, &undistorted, &registered);
			/// [registration]
		}

		/// [loop end]
		listener.release(frames);
	}
	/// [loop end]

	/// [stop]
	dev->stop();
	dev->close();
	/// [stop]

	delete registration;

	return 0;
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	// Get the command string
	char cmd[64];
	char err[256];
	
	if (nrhs < 1 || mxGetString(prhs[0], cmd, sizeof(cmd)))
		mexErrMsgTxt("First input should be a command string less than 64 characters long.");

	if (!strcmp("test", cmd)) {
		test_protonect_code();
		return;
	}

	if (!strcmp("logStdio", cmd)) {
		mycout_redirect.redirect();
		return;
	}

	// New
	if (!strcmp("new", cmd)) {
		// Check parameters
		if (nlhs != 1)
			mexErrMsgTxt("New: One output expected.");
		// Return a handle to a new C++ instance
		plhs[0] = convertPtr2Mat<libfreenect2::Freenect2>(new libfreenect2::Freenect2());
		return;
	}

	//// Get the class instance pointer from the second input
	libfreenect2::Freenect2 *context = convertMat2Ptr<libfreenect2::Freenect2>(prhs[1]);

	// Check there is a second input, which should be the class instance handle
	if (nrhs < 2) {
		mexErrMsgTxt("Second input should be a libfreenect context.");
	}

	if (!strcmp("enumerateDevices", cmd)) {
		if (nlhs != 1) {
			sprintf_s(err, "enumerateDevices: One output expected. Got %d.", nlhs);
			mexErrMsgTxt(err);
		}
		plhs[0] = mxCreateDoubleScalar(freenect2::countDevices(context));
		return;
	}

	if (!strcmp("openDevice", cmd)) {
		// Check parameters
		if (nrhs < 4) {
			mexErrMsgTxt("openDevice: Unexpected arguments.");
		}
		if (nlhs != 1) {
			mexErrMsgTxt("openDevice: One output expected.");
		}

		char * serial = "";
		if (nrhs >= 3) {
			serial = mxArrayToString(prhs[2]);
		}

		freenect2::PacketPipeline pipeline = freenect2::PacketPipeline::CPU;
		if (nrhs == 4) {
			uint8_t * ptr = (uint8_t *)mxGetData(prhs[3]);

			pipeline = (freenect2::PacketPipeline)*ptr;
		}
		
		plhs[0] = convertPtr2Mat<freenect2::DeviceStruct>(freenect2::getDevice(context, serial, pipeline));
		return;
	}

	// Check there is a second input, which should be the class instance handle
	if (nrhs < 3) {
		mexErrMsgTxt("Third input should be a device definition structure.");
	}

	//// Get the class instance pointer from the second input
	freenect2::DeviceStruct *device = convertMat2Ptr<freenect2::DeviceStruct>(prhs[2]);

	if (!strcmp("initializeDevice", cmd)) {
		// Check parameters
		if (nrhs < 3) {
			mexErrMsgTxt("initializeDevice: Unexpected arguments.");
		}
		if (nlhs != 0) {
			mexErrMsgTxt("initializeDevice: No output expected.");
		}
		freenect2::initializeDevice(device);
		return;
	}

	if (!strcmp("processFrame", cmd)) {
		// Check parameters
		if (nrhs < 2) {
			mexErrMsgTxt("getFrame: Unexpected arguments.");
		}
		if (nlhs != 1) {
			mexErrMsgTxt("getFrame: One output expected.");
		}

		bool result = !(freenect2::processFrame(device) == nullptr);

		plhs[0] = mxCreateLogicalScalar(result);
		return;
	}

	if (!strcmp("getFrame", cmd)) {

		// Check parameters
		if (nrhs < 3) {
			mexErrMsgTxt("getFrame: Expected arguments.");
		}
		if (nlhs != 1) {
			mexErrMsgTxt("getFrame: One output expected.");
		}

		freenect2::FrameType type = (freenect2::FrameType)*(uint8_t *)mxGetData(prhs[3]);

		plhs[0] = freenect2::getFrame(device, type);
		return;
	}

	if (!strcmp("cleanupFrame", cmd)) {
		// Check parameters
		if (nrhs < 2) {
			mexErrMsgTxt("cleanupFrame: Unexpected arguments.");
		}
		if (nlhs != 0) {
			mexErrMsgTxt("cleanupFrame: Unexpected output.");
		}

		freenect2::cleanupFrame(device);
		return;
	}

	// Delete
	if (!strcmp("delete", cmd)) {
		// Destroy the C++ object

		if (device != nullptr) {
			freenect2::deleteDevice(device);
		}

		destroyObject<libfreenect2::Freenect2>(prhs[1]);
		// Warn if other commands were ignored
		if (nlhs != 0 || nrhs != 3)
			mexWarnMsgTxt("Delete: Unexpected arguments ignored.");
		return;
	}

	// Got here, so command not recognized
	mexErrMsgTxt("Command not recognized.");
}
