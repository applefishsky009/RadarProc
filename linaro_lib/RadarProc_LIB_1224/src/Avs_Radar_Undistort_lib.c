#include "Avs_Radar_Undistort_lib.h"


void undistort_im_nearest(	AVS_RADAR_PREPROCESS_IN *inbuf, 
							AVS_RADAR_FILTER *avs_filter) {
	int i = 0, j = 0, index = 0, index_new = 0;
	int width	= inbuf->frame_info.width;
	int height	= inbuf->frame_info.height;
	int offset = width * height;
	float *data_R = inbuf->frame_info.data;
	float *data_G = inbuf->frame_info.data + offset;
	float *data_B = inbuf->frame_info.data + 2 * offset;
	// 内参
	double fx = inbuf->cam_info.fu;
	double fy = inbuf->cam_info.fv;
	double cx = inbuf->cam_info.cu;
	double cy = inbuf->cam_info.cv;
	double k1 = inbuf->cam_info.k1;
	double k2 = inbuf->cam_info.k2;
	double k3 = inbuf->cam_info.k3;
	double p1 = inbuf->cam_info.p1;
	double p2 = inbuf->cam_info.p2;
	// 中间变量
	double x1 = 0.0, y1 = 0.0, x2 = 0.0, y2 = 0.0, r2 = 0.0;
	int u_distorted = 0, v_distorted = 0;
	for (j = 0; j < height; ++j) {
		for (i = 0; i < width; ++i) {	// (i, j) = (x, y)
			index_new = j * width + i;
			x1 = (i - cx) / fx;
			y1 = (j - cy) / fy;
			r2 = x1 * x1 + y1 * y1;
			x2 = x1 * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2)
				+ 2 * p1 * x1 * y1 + p2 * (r2 + 2 * x1 * x1);
			y2 = y1 * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2)
				+ p1 * (r2 + 2 * y1 * y1) + 2 * p2 * x1 * y1;
			u_distorted = fx * x2 + cx + 0.5;
			v_distorted = fy * y2 + cy + 0.5;	// 对应畸变图像的坐标点
			index = v_distorted * width + u_distorted;
			if (u_distorted >= 0 && v_distorted >= 0 && u_distorted < width && v_distorted < height) {
				// 最近邻插值
				avs_filter->undistort_map.data[index_new]				= inbuf->frame_info.data[index];
				avs_filter->undistort_map.data[index_new + offset]		= inbuf->frame_info.data[index + offset];
				avs_filter->undistort_map.data[index_new + 2 * offset]	= inbuf->frame_info.data[index + 2 * offset];
			} else {
				avs_filter->undistort_map.data[index_new]				= 0.0;
				avs_filter->undistort_map.data[index_new + offset]		= 0.0;
				avs_filter->undistort_map.data[index_new + 2 * offset]	= 0.0;
			}
		}
	}
}

void undistort_im_bi(	AVS_RADAR_PREPROCESS_IN *inbuf,
						AVS_RADAR_FILTER		*avs_filter) {
	int i = 0, j = 0, index = 0, index_new = 0;
	int width = inbuf->frame_info.width;
	int height = inbuf->frame_info.height;
	int offset = width * height;
	float *data_R = inbuf->frame_info.data;
	float *data_G = inbuf->frame_info.data + offset;
	float *data_B = inbuf->frame_info.data + 2 * offset;
	// 内参
	double fx = inbuf->cam_info.fu;
	double fy = inbuf->cam_info.fv;
	double cx = inbuf->cam_info.cu;
	double cy = inbuf->cam_info.cv;
	double k1 = inbuf->cam_info.k1;
	double k2 = inbuf->cam_info.k2;
	double k3 = inbuf->cam_info.k3;
	double p1 = inbuf->cam_info.p1;
	double p2 = inbuf->cam_info.p2;
	// 中间变量
	double x1 = 0.0, y1 = 0.0, x2 = 0.0, y2 = 0.0, r2 = 0.0;
	float u_distorted = 0.f, v_distorted = 0.f;
	for (j = 0; j < height; ++j) {
		for (i = 0; i < width; ++i) {	// (i, j) = (x, y)
			index_new = j * width + i;
			x1 = (i - cx) / fx;
			y1 = (j - cy) / fy;
			r2 = x1 * x1 + y1 * y1;
			x2 = x1 * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2)
				+ 2 * p1 * x1 * y1 + p2 * (r2 + 2 * x1 * x1);
			y2 = y1 * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2)
				+ p1 * (r2 + 2 * y1 * y1) + 2 * p2 * x1 * y1;
			u_distorted = fx * x2 + cx;
			v_distorted = fy * y2 + cy;	// 对应畸变图像的坐标点
			// 双线性插值
			if (u_distorted >= 0 && v_distorted >= 0 && u_distorted < width && v_distorted < height) {
				avs_filter->undistort_map.data[index_new]				= get_scale_value(data_R, u_distorted, v_distorted, width, height);
				avs_filter->undistort_map.data[index_new + offset]		= get_scale_value(data_G, u_distorted, v_distorted, width, height);
				avs_filter->undistort_map.data[index_new + 2 * offset]	= get_scale_value(data_B, u_distorted, v_distorted, width, height);
			}
			else {
				avs_filter->undistort_map.data[index_new]				= 0.0;
				avs_filter->undistort_map.data[index_new + offset]		= 0.0;
				avs_filter->undistort_map.data[index_new + 2 * offset]	= 0.0;
			}
		}
	}
}

void undistort_output_info(	AVS_RADAR_PREPROCESS_IN		*inbuf,
							AVS_RADAR_FILTER			*avs_filter,
							AVS_RADAR_PREPROCESS_OUT	*outbuf) {
	avs_filter->undistort_map.width = inbuf->frame_info.width;
	avs_filter->undistort_map.height = inbuf->frame_info.height;
	avs_filter->undistort_map.channels = inbuf->frame_info.channel;
	outbuf->width	 = inbuf->frame_info.width;
	outbuf->height	 = inbuf->frame_info.height;
	outbuf->channels = AVS_RADAR_INPUT_CHANNEL;
	outbuf->data	 = avs_filter->undistort_map.data;
}