#include <fstream>
#include <memory.h>
#include <iostream>
#include "Avs_Radar.h"
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

void AVS_alloc_mem_tab(AVS_MEM_TAB *mem_tab, int num);

void get_radar_point(	AVS_RADAR_PREPROCESS_IN *radar_param, 
						vector<string> &txts);

void get_image_info(AVS_RADAR_PREPROCESS_IN *filter, 
					Mat &image_resize);

void get_infrared_image_info(	AVS_RADAR_PREPROCESS_IN *filter, 
								Mat &image_infrared_resize);

void verify_radar_infrared_interpolationprocess(AVS_RADAR_PREPROCESS_OUT *infrared_output);

void verify_radar_preprocess(AVS_RADAR_PREPROCESS_IN *input);

void get_classification_info(	AVS_RADAR_POSTPROCESS_IN *post_input, 
								string &classification);

void get_regression_info(	AVS_RADAR_POSTPROCESS_IN *post_input, 
							string &classification);

void verify_radar_undistortprocess(AVS_RADAR_PREPROCESS_OUT *undistort_output);

void verify_radar_interpolationprocess(AVS_RADAR_PREPROCESS_OUT *resize_output);

void verify_radar_postprocess(	cv::Mat					  *image_resize,
								AVS_RADAR_POSTPROCESS_OUT *post_output);
