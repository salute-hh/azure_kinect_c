#include <k4a/k4a.hpp>
#include <k4a/k4a.h>
#include <k4arecord/record.h>
#include <k4arecord/playback.h>
#include <windows.h>

#include <iostream>
#include <string>
#include <vector>
#include <array>
#include <atomic>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <thread>
#include <mutex>

#include "Pixel.h"
#include "DepthPixelColorizer.h"
#include "StaticImageProperties.h"
#include "turbojpeg.h"
using namespace std;
using namespace cv;
using namespace sen;
k4a_wait_result_t result = K4A_WAIT_RESULT_TIMEOUT;
k4a_record_t recording;
k4a_image_t color_image;
k4a_image_t color_image1;
cv::Mat color_frame;//传感器 SDK 能够以 BGRA 像素格式提供彩色图像。

k4a_device_configuration_t config;
k4a_device_t device;
k4a_capture_t capture;
k4a_capture_t capture1;
atomic<bool> exiting(false);

int32_t timeout_ms;

k4a_image_t initTexture;
k4a_image_t colorImageBuffer;
std::array<k4a_image_t, 30> colorBuffer;
k4a_image_t outColorFrame;
int bufferIndex = 0;
uint8_t* colorTextureBuffer = NULL;
uint8_t* colorTextureBuffer1 = NULL;
int flag = 0;
k4a::image colorImage;


int main(int argc, char* argv[])
{
	string root_path, path_input, save_name;
	string color_dir, ir_dir, depth_dir, skeleton_dir;
	string path1, path2, path3, path4, path5;
	root_path = "D:\\situp0\\";    // 根文件夹目录
	cout << "Please Input Name: ";
	cin >> path_input;//输入样本名
	save_name = path_input + ".mkv";
	path_input = root_path + path_input + "\\";


	path1 = "md " + path_input;


	system(path1.c_str());       // 创建文件夹


	const uint32_t deviceCount = k4a::device::get_installed_count();
	if (deviceCount == 0)
	{
		cout << "no azure kinect devices detected!" << endl;
	}

	config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	config.camera_fps = K4A_FRAMES_PER_SECOND_30;
	config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	config.color_format = K4A_IMAGE_FORMAT_COLOR_MJPG;//K4A_IMAGE_FORMAT_COLOR_MJPG
	config.color_resolution = K4A_COLOR_RESOLUTION_720P;
	config.synchronized_images_only = true;
	

	cout << "Started opening K4A device..." << endl;
	k4a_device_open(0, &device);
	k4a_device_start_cameras(device, &config);
	cout << "Finished opening K4A device." << endl;
	
	string file_name_recording = path_input + save_name;
	k4a_record_create(file_name_recording.c_str(), device, config, &recording);
	k4a_record_write_header(recording);

	int32_t timeout_sec_for_first_capture = 60;
	if (config.wired_sync_mode == K4A_WIRED_SYNC_MODE_SUBORDINATE)
	{
		timeout_sec_for_first_capture = 360;
		std::cout << "[subordinate mode] Waiting for signal from master" << std::endl;
	}
	clock_t first_capture_start = clock();


	while (!exiting && (clock() - first_capture_start) < (CLOCKS_PER_SEC * timeout_sec_for_first_capture))
	{
		result = k4a_device_get_capture(device, &capture, 100);
		if (result == K4A_WAIT_RESULT_SUCCEEDED)
		{
			k4a_capture_release(capture);
			break;
		}
		else if (result == K4A_WAIT_RESULT_FAILED)
		{
			std::cerr << "Runtime error: k4a_device_get_capture() returned error: " << result << std::endl;
			return 1;
		}
	}

	if (exiting)
	{
		k4a_device_close(device);
		return 0;
	}
	else if (result == K4A_WAIT_RESULT_TIMEOUT)
	{
		std::cerr << "Timed out waiting for first capture." << std::endl;
		return 1;
	}

	std::cout << "Started recording" << std::endl;

	int recording_length = 3600;//one hour max
	if (recording_length <= 0)
	{
		std::cout << "Press Ctrl-C to stop recording." << std::endl;
	}

	uint32_t camera_fps = 30;

	if (camera_fps <= 0 || (config.color_resolution == K4A_COLOR_RESOLUTION_OFF &&
		config.depth_mode == K4A_DEPTH_MODE_OFF))
	{
		std::cerr << "Either the color or depth modes must be enabled to record." << std::endl;
		return 1;
	}
	clock_t recording_start = clock();
	timeout_ms = 1000 / camera_fps;

	do
	{

		result = k4a_device_get_capture(device, &capture, timeout_ms);

		if (result == K4A_WAIT_RESULT_TIMEOUT)
		{
			continue;
		}
		else if (result != K4A_WAIT_RESULT_SUCCEEDED)
		{
			std::cerr << "Runtime error: k4a_device_get_capture() returned " << result << std::endl;
			break;
		}
		color_image = k4a_capture_get_color_image(capture);
		int color_width, color_height;
		color_width = k4a_image_get_width_pixels(color_image);
		color_height = k4a_image_get_height_pixels(color_image);

		if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
			color_width,
			color_height,
			color_width * 4 * (int)sizeof(uint8_t),
			&color_image1))
		{
			std::cout << "Failed to create image buffer\n";
			return -1;
		}

		tjhandle tjHandle;
		tjHandle = tjInitDecompress();
		if (tjDecompress2(tjHandle,
			k4a_image_get_buffer(color_image),
			static_cast<unsigned long>(k4a_image_get_size(color_image)),
			k4a_image_get_buffer(color_image1),
			color_width,
			0, // pitch
			color_height,
			TJPF_BGRA,
			2048 | 256) != 0)
		{
			std::cout << "Failed to decompress color frame\n";
			if (tjDestroy(tjHandle))
			{
				std::cout << "Failed to destroy turboJPEG handle\n";
			}
			return -1;
		}
		if (tjDestroy(tjHandle))
		{
			std::cout << "Failed to destroy turboJPEG handle\n";
			return -1;
		}
		if (color_image1 != NULL)
		{
			color_frame = cv::Mat(k4a_image_get_height_pixels(color_image1), k4a_image_get_width_pixels(color_image1), CV_8UC4, k4a_image_get_buffer(color_image1), cv::Mat::AUTO_STEP);
			cv::namedWindow("color", CV_WINDOW_NORMAL);
			cv::imshow("color", color_frame);
		}

		if (waitKey(10) == 27)//ESC
		{
			flag = 1;
			std::cout << "Done" << std::endl;

			break;
			return 0;
		}
		k4a_image_release(color_image);
		k4a_image_release(color_image1);
		
		k4a_record_write_capture(recording, capture);
		k4a_capture_release(capture);
	} while (!flag && (!exiting && result != K4A_WAIT_RESULT_FAILED &&
		(recording_length < 0 || (clock() - recording_start < recording_length * CLOCKS_PER_SEC))));

	if (!exiting)
	{
		exiting = true;
		std::cout << "Stopping recording..." << std::endl;
	}

	k4a_device_stop_cameras(device);

	std::cout << "Saving recording..." << std::endl;
	k4a_record_flush(recording);
	k4a_record_close(recording);

	std::cout << "Done" << std::endl;
	k4a_device_close(device);


	return 0;
}