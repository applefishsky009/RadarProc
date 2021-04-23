#include "Avs_Radar_lib.h"

#if RADAR_DEBUG_INFO_V2
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
using namespace std;
#endif

// 雷达数据预处理，计算距离属性，方位角，速度直径补偿
void cal_radar_distri(	AVS_RADAR_PREPROCESS_IN *inbuf, 
						AVS_RADAR_FILTER		*avs_filter) {
	int i = 0, j = 0;
	// 预处理雷达数据
	avs_filter->radar_info_cache_num = inbuf->radar_info_cache_num;
	for (i = 0; i < inbuf->radar_info_cache_num; ++i) {
		avs_filter->radar_pre_info[i].radar_point_num = inbuf->radar_info[i].radar_point_num;
		// 预处理
		for (j = 0; j < inbuf->radar_info[i].radar_point_num; ++j) {
			// 距离
			float x_cor = inbuf->radar_info[i].radar_point_info[j].radar_point.x;
			float y_cor = inbuf->radar_info[i].radar_point_info[j].radar_point.y;
			avs_filter->radar_pre_info[i].radar_cal_info[j].dis = (float)sqrt((double)(x_cor * x_cor + y_cor * y_cor));
			// printf("%f \n", avs_filter->radar_pre_info[i].radar_cal_info[j].dis);
			// 速度
			float vx_cor = inbuf->radar_info[i].radar_point_info[j].radar_v.x;
			float vy_cor = inbuf->radar_info[i].radar_point_info[j].radar_v.y;
			avs_filter->radar_pre_info[i].radar_cal_info[j].vel = (float)sqrt((double)(vx_cor * vx_cor + vy_cor * vy_cor));
			// 角度
			avs_filter->radar_pre_info[i].radar_cal_info[j].arm = (float)atan2((double)y_cor, (double)x_cor);
			// 相对速度
			avs_filter->radar_pre_info[i].radar_cal_info[j].vrad_comp =
				inbuf->radar_info[i].radar_point_info[j].radar_v_comp.x * (x_cor / avs_filter->radar_pre_info[i].radar_cal_info[j].dis) +
				inbuf->radar_info[i].radar_point_info[j].radar_v_comp.y * (y_cor / avs_filter->radar_pre_info[i].radar_cal_info[j].dis);

			// 计算两点坐标用于映射，从雷达坐标系映射到世界坐标系再添加z轴信息
			avs_filter->radar_pre_info[i].radar_cal_info[j].radar_bottom.x = x_cor;
			avs_filter->radar_pre_info[i].radar_cal_info[j].radar_bottom.y = y_cor;
			//avs_filter->radar_pre_info[i].radar_cal_info[j].radar_bottom.z = 0 - inbuf->radar_calib.radar_height / 1000.f;
			avs_filter->radar_pre_info[i].radar_cal_info[j].radar_top.x = x_cor;
			avs_filter->radar_pre_info[i].radar_cal_info[j].radar_top.y = y_cor;

			//avs_filter->radar_pre_info[i].radar_cal_info[j].radar_top.z = (inbuf->radar_calib.radar_top - inbuf->radar_calib.radar_height) / 1000.f;
			// rcs
			//avs_filter->radar_pre_info[i].radar_cal_info[j].rcs = inbuf->radar_info[i].radar_point_info[j].rcs;
		}
		//int kkk = 0;
	}

	//// debug-info
	//ofstream out("sel.txt", ios::out);
	//for (i = 0; i < inbuf->radar_info_cache_num; ++i) {
	//	avs_filter->radar_pre_info[i].radar_point_num = inbuf->radar_info[i].radar_point_num;
	//	//ofstream out(to_string(i + 1) + ".txt", ios::out);
	//	out.precision(18);
	//	//out.flags(ios::left | ios::fixed);
	//	out.flags(ios::left | ios::scientific);
	//	out.fill('0');
	//	//out.width(8);
	//	for (j = 0; j < inbuf->radar_info[i].radar_point_num; ++j) {
	//		out <<	float(inbuf->radar_info[i].radar_point_info[j].radar_point.x) << "\t" << 
	//				float(inbuf->radar_info[i].radar_point_info[j].radar_point.y) << "\t" <<
	//				float(inbuf->radar_info[i].radar_point_info[j].radar_point.z) << "\t" <<
	//				float(inbuf->radar_info[i].radar_point_info[j].rcs) << "\t" <<
	//				float(avs_filter->radar_pre_info[i].radar_cal_info[j].dis) << "\t" <<
	//				float(avs_filter->radar_pre_info[i].radar_cal_info[j].arm) << "\t" <<
	//				float(avs_filter->radar_pre_info[i].radar_cal_info[j].vrad_comp) << "\t" <<
	//		endl;
	//	}
	//}
	//out.close();
	//int kkk = 0;
}
#if RADAR_DEBUG_INFO_V3
// 雷达数据预处理，计算距离属性，方位角，速度直径补偿
void cal_radar_distri_v3(	AVS_RADAR_PREPROCESS_IN *inbuf,
							AVS_RADAR_FILTER		*avs_filter) {
	int i = 0, j = 0;

	inbuf->radar_info_cache_num = AVS_DEBUG_CACHE;
	avs_filter->radar_info_cache_num = inbuf->radar_info_cache_num;
	// 将雷达点读入内存
	char buf[AVS_MAX_BUF_LEN * 10];
	char *token;
	for (i = 0; i < inbuf->radar_info_cache_num; ++i) {
		inbuf->radar_info[i].radar_point_num = AVS_DEBUG_POINT_NUM;
		FILE *fp = inbuf->debug_info.fp[i];
		for (j = 0; j < inbuf->radar_info[i].radar_point_num; ++j) {
			memset(buf, 0, AVS_MAX_BUF_LEN * 10);
			fgets(buf, AVS_MAX_BUF_LEN * 10, fp);
			token = strtok(buf, " ");	// 0
			inbuf->radar_info[i].radar_point_info[j].radar_point.x = atof(token);
			token = strtok(NULL, " ");	// 1
			inbuf->radar_info[i].radar_point_info[j].radar_point.y = atof(token);
			token = strtok(NULL, " ");	// 2
			inbuf->radar_info[i].radar_point_info[j].radar_point.z = atof(token);
			token = strtok(NULL, " ");	// 3
			inbuf->radar_info[i].radar_point_info[j].dyn_prop = atof(token);
			token = strtok(NULL, " ");	// 4
			inbuf->radar_info[i].radar_point_info[j].id = atof(token);
			token = strtok(NULL, " ");	// 5
			inbuf->radar_info[i].radar_point_info[j].rcs = atof(token);
			token = strtok(NULL, " ");	// 6
			inbuf->radar_info[i].radar_point_info[j].radar_v.x = atof(token);
			token = strtok(NULL, " ");	// 7
			inbuf->radar_info[i].radar_point_info[j].radar_v.y = atof(token);
			token = strtok(NULL, " ");	// 8
			inbuf->radar_info[i].radar_point_info[j].radar_v_comp.x = atof(token);
			token = strtok(NULL, " ");	// 9
			inbuf->radar_info[i].radar_point_info[j].radar_v_comp.y = atof(token);
			token = strtok(NULL, " ");	// 10
			inbuf->radar_info[i].radar_point_info[j].is_quality_valid = atof(token);
			token = strtok(NULL, " ");	// 11
			inbuf->radar_info[i].radar_point_info[j].ambig_state = atof(token);
			token = strtok(NULL, " ");	// 12
			inbuf->radar_info[i].radar_point_info[j].radar_rms.x = atof(token);
			token = strtok(NULL, " ");	// 13
			inbuf->radar_info[i].radar_point_info[j].radar_rms.y = atof(token);
			token = strtok(NULL, " ");	// 14
			inbuf->radar_info[i].radar_point_info[j].invalid_state = atof(token);
			token = strtok(NULL, " ");	// 15
			inbuf->radar_info[i].radar_point_info[j].phd0 = atof(token);
			token = strtok(NULL, " ");	// 16
			inbuf->radar_info[i].radar_point_info[j].radar_rms.x = atof(token);
			token = strtok(NULL, " ");	// 17
			inbuf->radar_info[i].radar_point_info[j].radar_rms.y = atof(token);
		}
	}

	// 预处理雷达数据
	for (i = 0; i < inbuf->radar_info_cache_num; ++i) {
		avs_filter->radar_pre_info[i].radar_point_num = inbuf->radar_info[i].radar_point_num;
		// 预处理
		for (j = 0; j < inbuf->radar_info[i].radar_point_num; ++j) {
			// 距离
			float x_cor = inbuf->radar_info[i].radar_point_info[j].radar_point.x;
			float y_cor = inbuf->radar_info[i].radar_point_info[j].radar_point.y;
			avs_filter->radar_pre_info[i].radar_cal_info[j].dis = (float)sqrt((double)(x_cor * x_cor + y_cor * y_cor));
			// printf("%f \n", avs_filter->radar_pre_info[i].radar_cal_info[j].dis);
			// 速度
			float vx_cor = inbuf->radar_info[i].radar_point_info[j].radar_v.x;
			float vy_cor = inbuf->radar_info[i].radar_point_info[j].radar_v.y;
			avs_filter->radar_pre_info[i].radar_cal_info[j].vel = (float)sqrt((double)(vx_cor * vx_cor + vy_cor * vy_cor));
			// 角度
			avs_filter->radar_pre_info[i].radar_cal_info[j].arm = (float)atan2((double)y_cor, (double)x_cor);
			// 相对速度
			avs_filter->radar_pre_info[i].radar_cal_info[j].vrad_comp =
				inbuf->radar_info[i].radar_point_info[j].radar_v_comp.x * (x_cor / avs_filter->radar_pre_info[i].radar_cal_info[j].dis) +
				inbuf->radar_info[i].radar_point_info[j].radar_v_comp.y * (y_cor / avs_filter->radar_pre_info[i].radar_cal_info[j].dis);

			// 计算两点坐标用于映射，从雷达坐标系映射到世界坐标系再添加z轴信息
			avs_filter->radar_pre_info[i].radar_cal_info[j].radar_bottom.x = x_cor;
			avs_filter->radar_pre_info[i].radar_cal_info[j].radar_bottom.y = y_cor;
			//avs_filter->radar_pre_info[i].radar_cal_info[j].radar_bottom.z = 0 - inbuf->radar_calib.radar_height / 1000.f;
			avs_filter->radar_pre_info[i].radar_cal_info[j].radar_top.x = x_cor;
			avs_filter->radar_pre_info[i].radar_cal_info[j].radar_top.y = y_cor;

			//avs_filter->radar_pre_info[i].radar_cal_info[j].radar_top.z = (inbuf->radar_calib.radar_top - inbuf->radar_calib.radar_height) / 1000.f;
			// rcs
			//avs_filter->radar_pre_info[i].radar_cal_info[j].rcs = inbuf->radar_info[i].radar_point_info[j].rcs;
		}
		//int kkk = 0;
	}
}
#endif

// 计算图像平面的x值
float cal_x(	float fu, float fv, float cu, float cv,
				float c1, float c2, float s1, float s2,
				float x, float y, float z) {
	float top = (fu * c2 + cu * c1 * s2) * x + (cu * c1 * c2 - s2 * fu) * y - (cu * s1) * z;
	float down = c1 * s2 * x + c1 * c2 * y - s1 * z;
	return top / down;
}

// 计算图像平面的y值
float cal_y(float fu, float fv, float cu, float cv,
			float c1, float c2, float s1, float s2,
			float x, float y, float z) {
	float top = s2 * (cv * c1 - fv * s1) * x + c2 * (cv * c1 - fv * s1) * y - (fv * c1 + cv * s1) * z;
	float down = c1 * s2 * x + c1 * c2 * y - s1 * z;
	return top / down;
}

// Bresenham直线生成算法
void avs_radar_line_set(int x0, int y0, int x1, int y1,
						float *radar_rcs_map, float *radar_dis_map,
						int dst_width, int dst_height,
						AVS_RADAR_POINT_PRE_INFO *radar_point,
						AVS_RADAR_POINT_INFO *radar_input) {

	int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
	int dy = abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
	int err = (dx > dy ? dx : -dy) / 2, e2;
	int flag = 0;

	for (;;) {
		// setPixel(x0, y0);
		if (!(x0 < 0 || x0 >= dst_width || y0 < 0 || y0 >= dst_height)) {	// 有效点
			*(radar_rcs_map + y0 * dst_width + x0) = (float)radar_input->rcs;	// 更新map
			*(radar_dis_map + y0 * dst_width + x0) = (float)radar_point->dis;
			if (flag == 0) {	// 第一个有效点即初始点
				radar_point->radar_top_image.x = x0;
				radar_point->radar_top_image.y = y0;
				flag = 1;
			} else { // 动态更新结尾点
				radar_point->radar_bottom_image.x = x0;
				radar_point->radar_bottom_image.y = y0;
			}
		}
		if (x0 == x1 && y0 == y1) break;
		e2 = err;
		if (e2 > -dx) { err -= dy; x0 += sx; }
		if (e2 < dy) { err += dx; y0 += sy; }
	}
}

// 根据相机内参将点投影到像素平面
void cal_radarInImage(	AVS_RADAR_PREPROCESS_IN *inbuf, 
						AVS_RADAR_FILTER *avs_filter) {
	int i = 0, j = 0, radar_point_num = 0;
	// 读入雷达参数
	float theta = inbuf->radar_calib.radar_radian;
	float radarInCamera_X = inbuf->radar_calib.radarInCamera_X;
	float radarInCamera_Y = inbuf->radar_calib.radarInCamera_Y;
	// 读入相机参数
	float pitch_a = inbuf->cam_info.pitch;
	float yaw_b = inbuf->cam_info.yaw;
	float height = inbuf->cam_info.height;
	float fu = inbuf->cam_info.fu;
	float fv = inbuf->cam_info.fv;
	float cu = inbuf->cam_info.cu;
	float cv = inbuf->cam_info.cv;
	float c1 = (float)cos((double)pitch_a), c2 = (float)cos((double)yaw_b), s1 = (float)sin((double)pitch_a), s2 = (float)sin((double)yaw_b);
	// 读入图像参数
	float dst_width = AVS_PROC_WIDTH;
	float dst_height = AVS_PROC_HEIGHT;
	float src_width = inbuf->frame_info.width;
	float src_height = inbuf->frame_info.height;
	// 生成雷达的数据map
	int offset1 = dst_width * dst_height * 3 * sizeof(float);
	int offset2 = offset1 + dst_width * dst_height * sizeof(float);
	float *src_data = (float *)avs_filter->resize_map.data;
	float *radar_rcs_map = (float *)((QWORD)avs_filter->resize_map.data + offset1);
	memset((unsigned char *)radar_rcs_map, 0, dst_width * dst_height * sizeof(float));	// 雷达图清空
	float *radar_dis_map = (float *)((QWORD)avs_filter->resize_map.data + offset2);
	memset((unsigned char *)radar_dis_map, 0, dst_width * dst_height * sizeof(float));	// 雷达图清空

	// 坐标系转换
	for (i = 0; i < inbuf->radar_info_cache_num; ++i) {
		radar_point_num = avs_filter->radar_pre_info[i].radar_point_num;
		for (j = 0; j < inbuf->radar_info[i].radar_point_num; ++j) {
			// 雷达点滤波
			avs_filter->radar_pre_info[i].radar_cal_info[j].valid = 0;
			AVS_CHECK_CONTINUE(inbuf->radar_info[i].radar_point_info[j].invalid_state != AVS_RADAR_INVALID_STATE_VALID);
			AVS_CHECK_CONTINUE(inbuf->radar_info[i].radar_point_info[j].dyn_prop < AVS_RADAR_DYNPROP_STATE_MOVING);
			AVS_CHECK_CONTINUE(inbuf->radar_info[i].radar_point_info[j].dyn_prop > AVS_RADAR_DYNPROP_STATE_CROSSING_MOVING);
			AVS_CHECK_CONTINUE(inbuf->radar_info[i].radar_point_info[j].ambig_state != AVS_RADAR_AMBIG_STATE_UNAMBIGUOUS);
			avs_filter->radar_pre_info[i].radar_cal_info[j].valid = 1;
			// 转换到世界坐标系
			float radar_x = avs_filter->radar_pre_info[i].radar_cal_info[j].radar_top.x;
			float radar_y = avs_filter->radar_pre_info[i].radar_cal_info[j].radar_top.y;
			float posInCamera_x = cos((double)theta) * radar_x - sin((double)theta) * radar_y + radarInCamera_X;
			float posInCamera_y = sin((double)theta) * radar_x + cos((double)theta) * radar_y + radarInCamera_Y;
			// 二维点云根据先验信息增加高度信息
			avs_filter->radar_pre_info[i].radar_cal_info[j].radar_top_world.x = posInCamera_x;
			avs_filter->radar_pre_info[i].radar_cal_info[j].radar_top_world.y = posInCamera_y;
			avs_filter->radar_pre_info[i].radar_cal_info[j].radar_top_world.z = (inbuf->radar_calib.radar_top - inbuf->radar_calib.radar_height) / 1000.f;
			avs_filter->radar_pre_info[i].radar_cal_info[j].radar_bottom_world.x = posInCamera_x;
			avs_filter->radar_pre_info[i].radar_cal_info[j].radar_bottom_world.y = posInCamera_y;
			avs_filter->radar_pre_info[i].radar_cal_info[j].radar_bottom_world.z = 0 - inbuf->radar_calib.radar_height / 1000.f;
			// 将世界坐标系转换为像素平面
			float x1 = posInCamera_x;
			float y1 = posInCamera_y;
			float z1 = avs_filter->radar_pre_info[i].radar_cal_info[j].radar_top_world.z;
			float x2 = posInCamera_x;
			float y2 = posInCamera_y;
			float z2 = avs_filter->radar_pre_info[i].radar_cal_info[j].radar_bottom_world.z;
			float d_x1 = cal_x(fu, fv, cu, cv, c1, c2, s1, s2, x1, y1, z1);
			float d_y1 = cal_y(fu, fv, cu, cv, c1, c2, s1, s2, x1, y1, z1);
			float d_x2 = cal_x(fu, fv, cu, cv, c1, c2, s1, s2, x2, y2, z2);
			float d_y2 = cal_y(fu, fv, cu, cv, c1, c2, s1, s2, x2, y2, z2);
			// resize插值采样
			d_x1 = dst_width / src_width * d_x1;
			d_y1 = dst_height / src_height * d_y1;
			d_x2 = dst_width / src_width * d_x2;
			d_y2 = dst_height / src_height * d_y2;
			// Bresenham算法 - 直线生成
			avs_radar_line_set(d_x1, d_y1, d_x2, d_y2, 
				radar_rcs_map, radar_dis_map, 
				dst_width, dst_height, 
				&avs_filter->radar_pre_info[i].radar_cal_info[j],
				&inbuf->radar_info[i].radar_point_info[j]);
			//printf("rcs: %d dis: %f\n", inbuf->radar_info[i].radar_point_info[j].rcs, 
			//	avs_filter->radar_pre_info[i].radar_cal_info[j].dis);
		}
	}
}

#if RADAR_DEBUG_INFO_V2
// debug信息
// 行信息转为float
void string2float_in(string &radar_point, vector<float> &point_info) {
	radar_point.push_back(' ');
	size_t pos = radar_point.find_first_of(' ');
	while (pos != string::npos) {
		string string_part = radar_point.substr(0, pos);
		float float_part = atof(string_part.c_str());
		point_info.push_back(float_part);
		radar_point = radar_point.substr(pos + 1);
		pos = radar_point.find_first_of(' ');
	}
}

// map写为txt
void write_map(float *src_data, ofstream &out, int width, int height) {
	int i = 0, j = 0, index = 0;
	out.precision(18);
	out.flags(ios::left | ios::scientific);
	out.fill('0');
	for (i = 0; i < height; ++i) {
		for (j = 0; j < width; ++j) {
			index = i * width + j;
			out << *(src_data + index) << " ";
		}
		out << endl;
	}
}

void cal_radarinimage_v2(	AVS_RADAR_PREPROCESS_IN *inbuf, 
							AVS_RADAR_FILTER *avs_filter) {
	int i = 0, j = 0, radar_point_num = 0;
	// 读入图像参数
	float dst_width = avs_filter->resize_map.width;
	float dst_height = avs_filter->resize_map.height;
	// 生成雷达的数据map
	int offset1 = dst_width * dst_height * 3 * sizeof(float);
	int offset2 = offset1 + dst_width * dst_height * sizeof(float);
	float *src_data = (float *)avs_filter->resize_map.data;
	float *radar_rcs_map = (float *)((QWORD)avs_filter->resize_map.data + offset1);
	memset((unsigned char *)radar_rcs_map, 0, dst_width * dst_height * sizeof(float));	// 雷达图清空
	float *radar_dis_map = (float *)((QWORD)avs_filter->resize_map.data + offset2);
	memset((unsigned char *)radar_dis_map, 0, dst_width * dst_height * sizeof(float));	// 雷达图清空

	// 读入数据
	ifstream fin("D:/visual/RadarProc/RadarProc/radar_points.txt");
	ifstream fin2("D:/visual/RadarProc/RadarProc/radar_xyz_endpoint.txt");
	for (i = 0; i < inbuf->radar_info_cache_num; ++i) {
		radar_point_num = avs_filter->radar_pre_info[i].radar_point_num;
		for (j = 0; j < inbuf->radar_info[i].radar_point_num; ++j) {
			string radar_point, radar_point2;
			vector<float> point_info, point_info2;
			getline(fin, radar_point);
			getline(fin2, radar_point2);
			string2float_in(radar_point, point_info);
			string2float_in(radar_point2, point_info2);
			avs_filter->radar_pre_info[i].radar_cal_info[j].radar_top_image_debug.x = point_info[0];
			avs_filter->radar_pre_info[i].radar_cal_info[j].radar_top_image_debug.y = point_info[1];
			avs_filter->radar_pre_info[i].radar_cal_info[j].radar_bottom_image_debug.x = point_info2[0];
			avs_filter->radar_pre_info[i].radar_cal_info[j].radar_bottom_image_debug.y = point_info2[1];
			inbuf->radar_info[i].radar_point_info[j].rcs = point_info[5];
			avs_filter->radar_pre_info[i].radar_cal_info[j].dis;
			//int kkk = 0;
		}
	}
	fin.close();
	fin2.close();

	// 坐标系转换
	for (i = 0; i < inbuf->radar_info_cache_num; ++i) {
		radar_point_num = avs_filter->radar_pre_info[i].radar_point_num;
		for (j = 0; j < inbuf->radar_info[i].radar_point_num; ++j) {
			// 坐标点
			float d_x1 = avs_filter->radar_pre_info[i].radar_cal_info[j].radar_top_image_debug.x;
			float d_y1 = avs_filter->radar_pre_info[i].radar_cal_info[j].radar_top_image_debug.y;
			float d_x2 = avs_filter->radar_pre_info[i].radar_cal_info[j].radar_bottom_image_debug.x;
			float d_y2 = avs_filter->radar_pre_info[i].radar_cal_info[j].radar_bottom_image_debug.y;
			// bresenham算法 - 直线生成
			avs_radar_line_set(d_x1, d_y1, d_x2, d_y2,
				radar_rcs_map, radar_dis_map,
				dst_width, dst_height,
				&avs_filter->radar_pre_info[i].radar_cal_info[j],
				&inbuf->radar_info[i].radar_point_info[j]);
		}
	}
	//// map输出
	//int offset_base = dst_width * dst_height * sizeof(float);
	//ofstream out1("R.txt", ios::out);
	//write_map(src_data, out1, dst_width, dst_height);
	//out1.close();
	//ofstream out2("G.txt", ios::out);
	//write_map((float *)((QWORD)src_data + offset_base), out2, dst_width, dst_height);
	//out2.close();
	//ofstream out3("B.txt", ios::out);
	//write_map((float *)((QWORD)src_data + offset_base * 2), out3, dst_width, dst_height);
	//out3.close();
	//ofstream out4("RCS.txt", ios::out);
	//write_map((float *)((QWORD)src_data + offset_base * 3), out4, dst_width, dst_height);
	//out4.close();
	//ofstream out5("DIS.txt", ios::out);
	//write_map((float *)((QWORD)src_data + offset_base * 4), out5, dst_width, dst_height);
	//out5.close();

	{	
		cv::Mat resave_img(dst_height, dst_width, CV_8UC3, cv::Scalar(0, 0, 0));
		for (int j = 0; j < dst_height; ++j) {
			for (int i = 0; i < dst_width; ++i) {
				int index = j * dst_width + i;
				uchar k = src_data[index];
				(unsigned char)resave_img.data[3 * index + 0] = 128;	//r
				//(unsigned char)resave_img.data[3 * index + 2] = (unsigned char)(src_data[index] * 255);	//r
				(unsigned char)resave_img.data[3 * index + 1] = (unsigned char)(radar_rcs_map[index] + 128); //g
				(unsigned char)resave_img.data[3 * index + 2] = (unsigned char)(radar_dis_map[index] + 128); //b
			}
		}
		cv::imwrite("d:/visual/radarproc/data_pre_v2/resize_save_d2.png", resave_img);
		resave_img.release();
	}
	int kkk = 0;
}
#endif

#if RADAR_DEBUG_INFO_V3

void cal_radarinimage_v3(	AVS_RADAR_PREPROCESS_IN *inbuf, 
							AVS_RADAR_FILTER *avs_filter) {
	int i = 0, j = 0, radar_point_num = 0;
	// 读入图像参数
	float dst_width = avs_filter->resize_map.width;
	float dst_height = avs_filter->resize_map.height;
	// 生成雷达的数据map
	int offset1 = dst_width * dst_height * 3 * sizeof(float);
	int offset3 = dst_width * dst_height * sizeof(float);
	int offset2 = offset1 + offset3;
	float *src_data = (float *)avs_filter->resize_map.data;
	float *src_data_G = (float *)((QWORD)avs_filter->resize_map.data + offset3);
	float *src_data_B = (float *)((QWORD)avs_filter->resize_map.data + 2 * offset3);
	float *radar_rcs_map = (float *)((QWORD)avs_filter->resize_map.data + offset1);
	memset((unsigned char *)radar_rcs_map, 0, dst_width * dst_height * sizeof(float));	// 雷达图清空
	float *radar_dis_map = (float *)((QWORD)avs_filter->resize_map.data + offset2);
	memset((unsigned char *)radar_dis_map, 0, dst_width * dst_height * sizeof(float));	// 雷达图清空

	// 读入数据
	FILE *fp_point		= inbuf->debug_info.fp_point;
	FILE *fp_point_xyz	= inbuf->debug_info.fp_point_xyz;
	char *token = NULL;
	char buf_point[AVS_MAX_BUF_LEN];
	char buf_point_xyz[AVS_MAX_BUF_LEN];
	for (i = 0; i < inbuf->radar_info_cache_num; ++i) {
		radar_point_num = avs_filter->radar_pre_info[i].radar_point_num;
		for (j = 0; j < inbuf->radar_info[i].radar_point_num; ++j) {
			fgets(buf_point, AVS_MAX_BUF_LEN, fp_point);
			fgets(buf_point_xyz, AVS_MAX_BUF_LEN, fp_point_xyz);
			//printf("buf_point: %s \n", buf_point);
			token = strtok(buf_point, " ");
			float x1 = atof(token);
			token = strtok(NULL, " ");
			float y1 = atof(token);

			//printf("buf_point_xyz: %s \n", buf_point_xyz);
			token = strtok(buf_point_xyz, " ");
			float x2 = atof(buf_point_xyz);
			token = strtok(NULL, " ");
			float y2 = atof(token);
			avs_filter->radar_pre_info[i].radar_cal_info[j].radar_top_image_debug.x = x1;
			avs_filter->radar_pre_info[i].radar_cal_info[j].radar_top_image_debug.y = y1;
			avs_filter->radar_pre_info[i].radar_cal_info[j].radar_bottom_image_debug.x = x2;
			avs_filter->radar_pre_info[i].radar_cal_info[j].radar_bottom_image_debug.y = y2;
		}
	}

	// 坐标系转换
	for (i = 0; i < inbuf->radar_info_cache_num; ++i) {
		radar_point_num = avs_filter->radar_pre_info[i].radar_point_num;
		for (j = 0; j < inbuf->radar_info[i].radar_point_num; ++j) {
			// 坐标点
			float d_x1 = avs_filter->radar_pre_info[i].radar_cal_info[j].radar_top_image_debug.x;
			float d_y1 = avs_filter->radar_pre_info[i].radar_cal_info[j].radar_top_image_debug.y;
			float d_x2 = avs_filter->radar_pre_info[i].radar_cal_info[j].radar_bottom_image_debug.x;
			float d_y2 = avs_filter->radar_pre_info[i].radar_cal_info[j].radar_bottom_image_debug.y;
			// bresenham算法 - 直线生成
			avs_radar_line_set(d_x1, d_y1, d_x2, d_y2,
				radar_rcs_map, radar_dis_map,
				dst_width, dst_height,
				&avs_filter->radar_pre_info[i].radar_cal_info[j],
				&inbuf->radar_info[i].radar_point_info[j]);
			int kkk = 0;
		}
	}

	FILE *out_map = inbuf->debug_info.out_map;

	for (j = 0; j < dst_height; ++j) {
		for (i = 0; i < dst_width; ++i) {
			int index = j * dst_width + i;
			int r_tmp = 128;
			//int r_tmp = (unsigned char)(src_data[index] * 255);
			int g_tmp = (unsigned char)(src_data_G[index] * 255);
			int b_tmp = (unsigned char)(src_data_B[index] * 255);
			int rcs_tmp = (unsigned char)(radar_rcs_map[index] + 128);
			int dis_tmp = (unsigned char)(radar_dis_map[index] + 128);
			fwrite(&dis_tmp, 1, 1, out_map);
			fwrite(&rcs_tmp, 1, 1, out_map);
			fwrite(&r_tmp, 1, 1, out_map);
		}
	}
}
#endif

void cal_output_info(	AVS_RADAR_PREPROCESS_IN		*inbuf,
						AVS_RADAR_FILTER			*avs_filter,
						AVS_RADAR_PREPROCESS_OUT	*outbuf) {
	outbuf->width		= avs_filter->resize_map.width;
	outbuf->height		= avs_filter->resize_map.height;
	outbuf->channels	= AVS_RADAR_OUTPUT_CHANNEL;
	outbuf->data		= avs_filter->resize_map.data;
}

void normalize_input(AVS_RADAR_FILTER *avs_filter) {
	int i = 0;
	int width = avs_filter->resize_map.width;
	int height= avs_filter->resize_map.height;
	float *data = avs_filter->resize_map.data;
	for (i = 0; i < width * height * 3; ++i) {
		//printf("%f\n", *(data + i));
		*(data + i) /= 255.f;
		//printf("%f\n", *(data + i));
	}
	//int kkk = 0;
}
