#pragma once
#pragma once
#include <utility>
#include <k4a/k4a.hpp>

#include "Pixel.h"


#include<iostream>
#include<vector>
#include<array>
#include <fstream>
#include <chrono>
#include <string>
#include <math.h>
#include <sstream>

#include<k4a/k4a.hpp>	
#include<k4a/k4atypes.h>	

#include <opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>

#include<pcl/io/pcd_io.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/visualization/pcl_visualizer.h>

#include "Pixel.h"
#include "DepthPixelColorizer.h"
#include "StaticImageProperties.h"
#include "k4a_grabber.h"

#include <conio.h>

#pragma comment(lib, "User32.lib")
#pragma comment(lib, "gdi32.lib")

using namespace std;
using namespace cv;
using namespace sen;

//宏
//方便控制是否 std::cout 信息
#define DEBUG_std_cout 0

namespace Show
{
	void ShowDep_Color();
	void create_xy_table(const k4a_calibration_t* calibration, k4a_image_t xy_table);
	void generate_point_cloud(const k4a::image depth_image, const k4a_image_t xy_table, k4a_image_t point_cloud, int* point_count);
	void write_point_cloud(const char* file_name, const k4a_image_t point_cloud, int point_count);
}
