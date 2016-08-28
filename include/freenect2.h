#pragma once

#include <iostream>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <mex.h>

#define DEFAULT_FRAME_TYPES = libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth

namespace freenect2 {

	enum FrameType {
		COLOR = 1,
		IR = 2,
		DEPTH = 3,
		UNDISTORTED = 4,
		REGISTERED = 5,
		BIGDEPTH = 6
	};

	enum PacketPipeline {
		CPU = 0,
		OPENGL = 1,
		CUDA = 2,
		OPENCL = 3,
	};

	struct FrameStruct {
		libfreenect2::Frame *undistorted;
		libfreenect2::Frame *registered;
		libfreenect2::Frame *rgb;
		libfreenect2::Frame *depth;
		libfreenect2::Frame *ir;
		libfreenect2::Frame *bigdepth;
	};

	struct DeviceStruct {
		DeviceStruct() : frameMap(libfreenect2::FrameMap()) {};

		libfreenect2::Freenect2Device *kinect;
		libfreenect2::PacketPipeline *pipeline;
		libfreenect2::SyncMultiFrameListener *listener;
		libfreenect2::Registration *registration;
		libfreenect2::FrameMap frameMap;
		FrameStruct *frames;
	};

	int countDevices(libfreenect2::Freenect2 *context);

	DeviceStruct* getDevice(
		libfreenect2::Freenect2 *context, 
		const char *serial = nullptr, 
		PacketPipeline pipeline = PacketPipeline::CPU
	);

	void initializeDevice(
		DeviceStruct *device,
		int listener_flags =
			libfreenect2::Frame::Color |
			libfreenect2::Frame::Ir |
			libfreenect2::Frame::Depth
	);

	void deleteDevice(DeviceStruct *device);

	FrameStruct* processFrame(DeviceStruct *device, int timeout = 1000);
	mxArray* getFrame(DeviceStruct *device, FrameType type);
	void cleanupFrame(DeviceStruct * d);
}

class mystream : public std::streambuf
{
protected:
	virtual std::streamsize xsputn(const char *s, std::streamsize n) { mexPrintf("%.*s", n, s); return n; }
	virtual int overflow(int c = EOF) { if (c != EOF) { mexPrintf("%.1s", &c); } return 1; }
};
class scoped_redirect_cout
{
public:
	scoped_redirect_cout() {};
	~scoped_redirect_cout() { std::cout.rdbuf(old_buf); }
	void redirect() { old_buf = std::cout.rdbuf(); std::cout.rdbuf(&mout); }
private:
	mystream mout;
	std::streambuf *old_buf;
};

static scoped_redirect_cout mycout_redirect;

#define CLASS_HANDLE_SIGNATURE 0xFF00F0A5
template<class base> class class_handle
{
public:
	class_handle(base *ptr) : ptr_m(ptr), name_m(typeid(base).name()) { signature_m = CLASS_HANDLE_SIGNATURE; }
	~class_handle() { signature_m = 0; delete ptr_m; }
	bool isValid() { return ((signature_m == CLASS_HANDLE_SIGNATURE) && !strcmp(name_m.c_str(), typeid(base).name())); }
	base *ptr() { return ptr_m; }

private:
	uint32_t signature_m;
	std::string name_m;
	base *ptr_m;
};

template<class base> inline mxArray *convertPtr2Mat(base *ptr)
{
	mexLock();
	mxArray *out = mxCreateNumericMatrix(1, 1, mxUINT64_CLASS, mxREAL);
	*((uint64_t *)mxGetData(out)) = reinterpret_cast<uint64_t>(new class_handle<base>(ptr));
	return out;
}

template<class base> inline class_handle<base> *convertMat2HandlePtr(const mxArray *in)
{
	if (mxGetNumberOfElements(in) != 1 || mxGetClassID(in) != mxUINT64_CLASS || mxIsComplex(in))
		mexErrMsgTxt("Input must be a real uint64 scalar.");
	class_handle<base> *ptr = reinterpret_cast<class_handle<base> *>(*((uint64_t *)mxGetData(in)));
	if (!ptr->isValid())
		mexErrMsgTxt("Handle not valid.");
	return ptr;
}

template<class base> inline base *convertMat2Ptr(const mxArray *in)
{
	return convertMat2HandlePtr<base>(in)->ptr();
}

template<class base> inline void destroyObject(const mxArray *in)
{
	delete convertMat2HandlePtr<base>(in);
	mexUnlock();
}
