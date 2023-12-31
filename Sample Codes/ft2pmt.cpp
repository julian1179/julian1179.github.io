#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/mat.hpp>
#include <vector>

#define _USE_MATH_DEFINES
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979311599796346854
#endif

using namespace cv;

typedef struct {
	int x;
	int y;
} Coords;


std::vector<std::vector<Coords>> pmtCoords(const std::pair<int, int>& sz, double r0) {
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

	std::vector<std::vector<Coords>> G(sz.first, std::vector<Coords>(sz.second));
	for (int i = 0; i < sz.second; ++i) {
		for (int j = 0; j < sz.first; ++j) {
			int xt = xC + static_cast<int>(round(r0 * pow(10, p[i]) * cos(th[j])));
			int yt = yC + static_cast<int>(round(r0 * pow(10, p[i]) * sin(th[j])));
			if (xt > 0 && xt <= sz.second && yt > 0 && yt <= sz.first) {
				G[j][i].x = xt;
				G[j][i].y = yt;
			}
			else {
				G[j][i].x = 1;
				G[j][i].y = 1;
			}
		}
	}

	return G;
}


int __cdecl main(int argc, char* argv[]) {

	Mat im_input;
	int im_Size;

	String im_dir = "c:\\users\\lapt\\documents\\fpga\\pcie driver projects\\xdma_driver_CPU_LPT\\build\\x64\\bin\\testimage" + std::to_string(5) + ".bmp";
	im_input = imread(im_dir, IMREAD_GRAYSCALE);
	Size s = im_input.size();
	int im_rows = s.height;
	int im_cols = s.width;
	im_Size = im_rows * im_cols;

	Mat image_out0(im_rows, im_cols, CV_8UC1, im_input.data);
	printf("	Saving bytes to Bitmap\n");
	imwrite("c:\\users\\lapt\\documents\\fpga\\pcie driver projects\\xdma_driver_CPU_LPT\\build\\x64\\bin\\Output Files\\Original-copy.bmp", image_out0);



}