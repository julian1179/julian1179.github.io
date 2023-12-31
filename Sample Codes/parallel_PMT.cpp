#include <stdio.h>
#include <stdlib.h>
#include <Windows.h>
#include <SetupAPI.h>
#include <INITGUID.H>
#include <strsafe.h>
#include <WinIoCtl.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
//#include <QApplication>
//#include <QLabel>
#include <vector>
#include <omp.h>

#define _USE_MATH_DEFINES
#include <cmath>

#ifndef M_PI
	#define M_PI 3.14159265358979311599796346854
#endif



#include "xdma_public.h"

#pragma comment(lib, "setupapi.lib")

#define ONE_MB (1024UL * 1024UL)
#define US_PER_S (1000UL * 1000UL)
#define Width (4096UL)
#define Height (2048UL)
#define memAddr_rd (0x80000000); // MSb doesn't do anything
#define memAddr_wr (0x00000000); // MSb doesn't do anything
LARGE_INTEGER addr_rd;
LARGE_INTEGER addr_wr;

using namespace cv;

/* helper struct to remember the Xdma device names */
typedef struct {
	TCHAR base_path[MAX_PATH + 1]; /* path to first found Xdma device */
	TCHAR c2h0_path[MAX_PATH + 1];	/* card to host DMA 0 */
	TCHAR h2c0_path[MAX_PATH + 1];	/* host to card DMA 0 */
	PBYTE buffer; /* pointer to the allocated buffer */
	DWORD buffer_size; /* size of the buffer in bytes */
	HANDLE c2h0;
	HANDLE h2c0;
} xdma_device;

typedef struct {
	LARGE_INTEGER start;
	LARGE_INTEGER end;
	LARGE_INTEGER freq;
} Timer;

typedef struct {
	int y;
	int x;
} Coords;

__inline static void timer_start(Timer *timer) {
	QueryPerformanceFrequency(&timer->freq);
	QueryPerformanceCounter(&timer->start);
}

__inline static LONGLONG timer_elapsed_us(Timer *timer) {
	QueryPerformanceCounter(&timer->end);
	return (((timer->end.QuadPart - timer->start.QuadPart) * US_PER_S) / timer->freq.QuadPart);
}

int find_devices(GUID guid, TCHAR *devpath, size_t len_devpath) {
	
	HDEVINFO dev_info = SetupDiGetClassDevs((LPGUID)&guid, NULL, NULL, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
	if (dev_info == INVALID_HANDLE_VALUE) {
		fprintf(stderr, "GetDevices INVALID_HANDLE_VALUE\n");
		exit(-1);
	}

	SP_DEVICE_INTERFACE_DATA dev_interface;
	dev_interface.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);

	// enumerate through devices
	DWORD index;
	for (index = 0; SetupDiEnumDeviceInterfaces(dev_info, NULL, &guid, index, &dev_interface); ++index) {

		// get required buffer size
		ULONG detail_size = 0;
		if (!SetupDiGetDeviceInterfaceDetail(dev_info, &dev_interface, NULL, 0, &detail_size, NULL) && GetLastError() != ERROR_INSUFFICIENT_BUFFER) {
			fprintf(stderr, "SetupDiGetDeviceInterfaceDetail - get length failed\n");
			break;
		}

		// allocate space for device interface detail
		PSP_DEVICE_INTERFACE_DETAIL_DATA dev_detail = (PSP_DEVICE_INTERFACE_DETAIL_DATA)HeapAlloc(GetProcessHeap(), HEAP_ZERO_MEMORY, detail_size);
		if (!dev_detail) {
			fprintf(stderr, "HeapAlloc failed\n");
			break;
		}
		dev_detail->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA);

		// get device interface detail
		if (!SetupDiGetDeviceInterfaceDetail(dev_info, &dev_interface, dev_detail, detail_size, NULL, NULL)) {
			fprintf(stderr, "SetupDiGetDeviceInterfaceDetail - get detail failed\n");
			HeapFree(GetProcessHeap(), 0, dev_detail);
			break;
		}

		StringCchCopy(devpath, len_devpath, dev_detail->DevicePath);
		HeapFree(GetProcessHeap(), 0, dev_detail);
	}

	SetupDiDestroyDeviceInfoList(dev_info);

	return index;
}

xdma_device open_XDMA_devices(int write_to_card) {
	DWORD num_devices;
	xdma_device xdma;

	/* find a Xdma device */
	num_devices = find_devices(GUID_DEVINTERFACE_XDMA, &xdma.base_path[0], sizeof(xdma.base_path));
	printf("Found %d Xdma device%s.\n", num_devices, num_devices == 1 ? "" : "s");
	if (num_devices < 1) {
		exit(-1);
	}

	strcpy_s(xdma.c2h0_path, sizeof xdma.c2h0_path, &xdma.base_path[0]);
	strcat_s(xdma.c2h0_path, sizeof xdma.c2h0_path, "\\c2h_0");
	if (write_to_card) {
		strcpy_s(xdma.h2c0_path, sizeof xdma.h2c0_path, &xdma.base_path[0]);
		strcat_s(xdma.h2c0_path, sizeof xdma.h2c0_path, "\\h2c_0");
	}

	/* open XDMA Card-to-Host 0 device */
	xdma.c2h0 = CreateFile(xdma.c2h0_path, GENERIC_READ | GENERIC_WRITE,
		0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
	if (xdma.c2h0 == INVALID_HANDLE_VALUE) {
		fprintf(stderr, "Error opening device, win32 error code: %d\n", GetLastError());
		if (xdma.c2h0) CloseHandle(xdma.c2h0);
		if (write_to_card) {
			if (xdma.h2c0) CloseHandle(xdma.h2c0);
		}
		return xdma;
	}
	if (INVALID_SET_FILE_POINTER == SetFilePointerEx(xdma.c2h0, addr_rd, NULL, FILE_BEGIN)) {
		fprintf(stderr, "Error setting file pointer for C2H, win32 error code: %ld\n", GetLastError());
		if (xdma.c2h0) CloseHandle(xdma.c2h0);
		if (write_to_card) {
			if (xdma.h2c0) CloseHandle(xdma.h2c0);
		}
		return xdma;
	}
	printf("	Read Addr: 0x%4X %4X\n", addr_rd.HighPart, addr_rd.LowPart);

	if (write_to_card) {
		/* open XDMA Host-to-Card 0 device */
		xdma.h2c0 = CreateFile(xdma.h2c0_path, GENERIC_READ | GENERIC_WRITE,
			0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
		if (xdma.c2h0 == INVALID_HANDLE_VALUE) {
			fprintf(stderr, "Error opening device, win32 error code: %d\n", GetLastError());
			if (xdma.c2h0) CloseHandle(xdma.c2h0);
			if (xdma.h2c0) CloseHandle(xdma.h2c0);
			return xdma;
		}
		if (INVALID_SET_FILE_POINTER == SetFilePointerEx(xdma.h2c0, addr_wr, NULL, FILE_BEGIN)) {
			fprintf(stderr, "Error setting file pointer for H2C, win32 error code: %ld\n", GetLastError());
			if (xdma.c2h0) CloseHandle(xdma.c2h0);
			if (xdma.h2c0) CloseHandle(xdma.h2c0);
			return xdma;
		}
		printf("	Write Addr: 0x%4X %4X\n", addr_wr.HighPart, addr_wr.LowPart);
	}

	return xdma;

}

xdma_device read_FPGA(xdma_device *xdmaDev, Timer *t) {
	xdma_device xdma = *xdmaDev;
	Timer timer = *t;
	LONGLONG elapsed;
	DWORD num_bytes_read;

	/* transfer data from FPGA Card to Host PC using SGDMA engine */
	printf("Starting C2H read %u MB.\n", xdma.buffer_size / ONE_MB);
	timer_start(&timer);
	if (!ReadFile(xdma.c2h0, (LPVOID)xdma.buffer, xdma.buffer_size, (LPDWORD)&num_bytes_read, NULL)) {
		fprintf(stderr, "	ReadFile from device %s failed: %d\n", xdma.c2h0_path, GetLastError());
	}
	elapsed = timer_elapsed_us(&timer);
	printf("	XDMA C2H finished after %lld us.\n", elapsed);

	return xdma;
}

xdma_device write_FPGA(xdma_device* xdmaDev, Timer* t) {
	xdma_device xdma = *xdmaDev;
	Timer timer = *t;
	LONGLONG elapsed;
	DWORD num_bytes_read;

	/* transfer data from Host PC to FPGA Card using SGDMA engine */
	printf("Starting H2C write %u MB.\n", xdma.buffer_size / ONE_MB);
	timer_start(&timer);
	if (!WriteFile(xdma.h2c0, (LPVOID)xdma.buffer, xdma.buffer_size, (LPDWORD)&num_bytes_read, NULL)) {
		fprintf(stderr, "	WriteFile from device %s failed: %d\n", xdma.h2c0_path, GetLastError());
	}
	elapsed = timer_elapsed_us(&timer);
	printf("	XDMA H2C finished after %lld us.\n", elapsed);

	return xdma;
}

std::pair<int, int>* pmtCoords(const std::pair<int, int>& sz, double r0) {
	//const std::pair<int, int> sz = size;

	//double sF = static_cast<double>(sz.first) / sz.second;
	int xC = static_cast<int>(round(sz.second / 2));
	int yC = static_cast<int>(round(sz.first / 2));

	std::vector<int> x(sz.second);
	for (int i = 0; i < sz.second; ++i) {
		x[i] = i - xC;
	}
	std::vector<int> y(sz.first);
	for (int i = 0; i < sz.first; ++i) {
		y[i] = i - yC;
	}

	double th_step = 2 * M_PI / sz.first;
	std::vector<double> th(sz.first);
	for (int i = 0; i < sz.first; ++i) {
		th[i] = i * th_step;
	}

	double r_max = sqrt(xC * xC + yC * yC);
	double p_max = log10(r_max / r0);
	double p_min = 0;
	std::vector<double> p(sz.second);
	double p_step = (p_max - p_min) / (sz.second - 1);
	for (int i = 0; i < sz.second; ++i) {
		p[i] = p_min + i * p_step;
	}

	std::pair<int, int>* G = new std::pair<int, int>[sz.first * sz.second];
	for (int i = 0; i < sz.second; ++i) {
		for (int j = 0; j < sz.first; ++j) {
			int xt = xC + static_cast<int>(round(r0 * pow(10, p[i]) * cos(th[j])));
			int yt = yC + static_cast<int>(round(r0 * pow(10, p[i]) * sin(th[j])));
			if (xt > 0 && xt <= sz.second && yt > 0 && yt <= sz.first) {
				G[sz.second * j + i] = { yt,xt };
				//G[sz.second * j + i].y = yt;
			}
			else {
				G[sz.second * j + i] = { 1,1 };
				//G[sz.second * j + i].y = 1;
			}
		}
	}

	return G;
}


void lpt(const PBYTE input_buffer, const PBYTE output_buffer, std::pair<int, int>* G, const std::pair<int, int>& sz) {

	const auto sz_second = sz.second;
	const auto sz_first = sz.first;

	// Remember to enable OpenMP (in Visual Studio: in Properties -> C/C++ -> Language -> OpenMP)
	// https://stackoverflow.com/questions/4515276/openmp-is-not-creating-threads-in-visual-studio
	#pragma omp parallel for num_threads(24)
	for (int y = 0; y < sz_first; ++y) {
		for (int x = 0; x < sz_second; ++x) {			
			//output_buffer[sz_second * y + x] = input_buffer[sz_second * G[sz_second * y + x].first + G[sz_second * y + x].second];
			PBYTE out_ptr = output_buffer + sz_second * y + x;
			PBYTE in_ptr = input_buffer + sz_second * G[sz_second * y + x].first + G[sz_second * y + x].second;
			*out_ptr = *in_ptr;
		}
	}
	return;
}

void combineToBGR(const PBYTE gray_im1, const PBYTE gray_im2, const PBYTE gray_im3, PBYTE color_im, int total_pixels) {
	for (int i = 0; i < total_pixels; ++i) {
		color_im[3 * i] = gray_im1[i];      // Blue channel
		color_im[3 * i + 1] = gray_im2[i];  // Green channel
		color_im[3 * i + 2] = gray_im3[i];  // Red channel
	}
}

int __cdecl main(int argc, char* argv[]) {

	LONG status = 0;
	Timer timer;
	LONGLONG elapsed;

	UNREFERENCED_PARAMETER(argc);
	UNREFERENCED_PARAMETER(argv);

	int im_Size;
	
	//int imNum = 7;
	String im_dir1 = "c:\\users\\lapt\\documents\\fpga\\pcie driver projects\\xdma_driver_CPU_LPT\\build\\x64\\bin\\testimage" + std::to_string(5) + ".bmp";
	String im_dir2 = "c:\\users\\lapt\\documents\\fpga\\pcie driver projects\\xdma_driver_CPU_LPT\\build\\x64\\bin\\testimage" + std::to_string(6) + ".bmp";
	String im_dir3 = "c:\\users\\lapt\\documents\\fpga\\pcie driver projects\\xdma_driver_CPU_LPT\\build\\x64\\bin\\testimage" + std::to_string(7) + ".bmp";
	Mat im_input1 = imread(im_dir1, IMREAD_GRAYSCALE);
	Mat im_input2 = imread(im_dir2, IMREAD_GRAYSCALE);
	Mat im_input3 = imread(im_dir3, IMREAD_GRAYSCALE);
	//PBYTE input_buffer = im_input1.data; // Backup of the pointer to the original image data.
	Size s = im_input1.size();
	int im_rows = s.height;
	int im_cols = s.width;
	im_Size = im_rows * im_cols;
	std::pair<int, int> size_pair;
	size_pair.first = im_rows;  // y
	size_pair.second = im_cols; //x

	Mat image_in_copy(im_rows, im_cols, CV_8UC1, im_input1.data);
	Mat image_out1(im_rows, im_cols, CV_8UC1, im_input1.data);
	Mat image_out2(im_rows, im_cols, CV_8UC1, im_input2.data);
	Mat image_out3(im_rows, im_cols, CV_8UC1, im_input3.data);
	Mat image_color(im_rows, im_cols, CV_8UC3);
	printf("	Saving original bytes to Bitmap\n");
	imwrite("c:\\users\\lapt\\documents\\fpga\\pcie driver projects\\xdma_driver_CPU_LPT\\build\\x64\\bin\\Output Files\\Original" + std::to_string(5) + "-copy.bmp", image_in_copy);

	std::pair<int, int>* pmt_coords = pmtCoords(size_pair, 0.1);
	PBYTE pmt_im_buffer1 = (PBYTE)_aligned_malloc(size_pair.first * size_pair.second, 128);
	if (!pmt_im_buffer1) {
		// Handle memory allocation failure
		printf("!!!	FAILED TO ALLOCATE MEMORY FOR pmt_im_buffer1\n");
		status = 1;
		return status;
	}
	PBYTE pmt_im_buffer2 = (PBYTE)_aligned_malloc(size_pair.first * size_pair.second, 128);
	if (!pmt_im_buffer2) {
		// Handle memory allocation failure
		printf("!!!	FAILED TO ALLOCATE MEMORY FOR pmt_im_buffer2\n");
		status = 1;
		return status;
	}
	PBYTE pmt_im_buffer3 = (PBYTE)_aligned_malloc(size_pair.first * size_pair.second, 128);
	if (!pmt_im_buffer3) {
		// Handle memory allocation failure
		printf("!!!	FAILED TO ALLOCATE MEMORY FOR pmt_im_buffer3\n");
		status = 1;
		return status;
	}

	namedWindow("color window", WINDOW_AUTOSIZE);
	int numIter = 0;
	elapsed = 0;
	int runtime_limit = (int)60e6;
	cv::Mat bgr_channels[3];
	while (elapsed <= runtime_limit) {
		++numIter;
		timer_start(&timer);
		lpt(im_input1.data, pmt_im_buffer1, pmt_coords, size_pair);
		image_out1.data = pmt_im_buffer1;
		bgr_channels[0] = image_out1;

		lpt(im_input2.data, pmt_im_buffer2, pmt_coords, size_pair);
		image_out2.data = pmt_im_buffer2;
		bgr_channels[1] = image_out2;

		lpt(im_input3.data, pmt_im_buffer3, pmt_coords, size_pair);
		image_out3.data = pmt_im_buffer3;
		bgr_channels[2] = image_out3;
		
		//combineToBGR(image_out1.data, image_out2.data, image_out3.data, image_color.data, im_rows*im_cols);
		merge(bgr_channels,3, image_color);
		elapsed += timer_elapsed_us(&timer);
		imshow("color window", image_color);
		waitKey(1);
		
	}
	double runtime = float(elapsed) / numIter;
	printf("~~~	%d evaluations: imShow took an average of %.3f ms, totaling %.3f per image or %.2f fps.\n", numIter, float(runtime)/1000, float(runtime)/3000, 3000000/float(runtime));

	
	printf("	Saving PMT bytes to Bitmap\n");
	imwrite("c:\\users\\lapt\\documents\\fpga\\pcie driver projects\\xdma_driver_CPU_LPT\\build\\x64\\bin\\Output Files\\PMT" + std::to_string(5) + "_im.bmp", image_out1);


	//_aligned_free(pmt_im_buffer);
	return status;
}

