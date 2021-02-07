#include "test.hpp"

void *alloc_memory(unsigned int size, unsigned int aligment) {
	void *data;
	data = (char *)_aligned_malloc(size, aligment);
	return data;
}

void AVS_alloc_mem_tab(AVS_MEM_TAB *mem_tab, int num) {
	for (int i = 0; i < num; ++i) {
		if (mem_tab[i].size > 0) {
			mem_tab[i].base = alloc_memory(mem_tab[i].size, mem_tab[i].alignment);
		}
		else {
			mem_tab[i].base = NULL;
		}
		printf("tab %d memsize: %f M\n", i, mem_tab[i].size / 1024.0 / 1024.0);
	}
}

// 行信息转为float
void string2float(string &radar_point, vector<float> &point_info) {
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

// 从文件读入雷达信息
void get_radar_point(	AVS_RADAR_PREPROCESS_IN *radar_param, 
						vector<string>			&txts) {
	int i = 0, j = 0, point_cnt = 0;
	float tmp = 0.f;
	radar_param->radar_info_cache_num = txts.size();
	for (j = 0; j < radar_param->radar_info_cache_num; ++j) {
		ifstream fin(txts[j]);
		string radar_point;
		while (getline(fin, radar_point)) {
			vector<float> point_info;
			string2float(radar_point, point_info);	// 将一行字符串转换为vector
													// 将vector数据写入lib输入格式
			i = radar_param->radar_info[j].radar_point_num;
			radar_param->radar_info[j].radar_point_info[i].radar_point.x = point_info[0];
			radar_param->radar_info[j].radar_point_info[i].radar_point.y = point_info[1];
			radar_param->radar_info[j].radar_point_info[i].radar_point.z = point_info[2];
			radar_param->radar_info[j].radar_point_info[i].dyn_prop = point_info[3];
			radar_param->radar_info[j].radar_point_info[i].id = point_info[4];
			radar_param->radar_info[j].radar_point_info[i].rcs = point_info[5];
			radar_param->radar_info[j].radar_point_info[i].radar_v.x = point_info[6];
			radar_param->radar_info[j].radar_point_info[i].radar_v.y = point_info[7];
			radar_param->radar_info[j].radar_point_info[i].radar_v_comp.x = point_info[8];
			radar_param->radar_info[j].radar_point_info[i].radar_v_comp.y = point_info[9];
			radar_param->radar_info[j].radar_point_info[i].is_quality_valid = point_info[10];
			radar_param->radar_info[j].radar_point_info[i].ambig_state = point_info[11];
			radar_param->radar_info[j].radar_point_info[i].radar_rms.x = point_info[12];
			radar_param->radar_info[j].radar_point_info[i].radar_rms.y = point_info[13];
			radar_param->radar_info[j].radar_point_info[i].invalid_state = point_info[14];
			radar_param->radar_info[j].radar_point_info[i].phd0 = point_info[15];
			radar_param->radar_info[j].radar_point_info[i].radar_rms.x = point_info[16];
			radar_param->radar_info[j].radar_point_info[i].radar_rms.y = point_info[17];
			++radar_param->radar_info[j].radar_point_num;
		}
	}
}

// 图像Mat写入连续内存
void get_image_info(AVS_RADAR_PREPROCESS_IN *filter, Mat &image_resize) {
	int width = image_resize.cols;
	int height = image_resize.rows;
	// 遍历图像像素
	for (int j = 0; j < height; ++j) {
		for (int i = 0; i < width; ++i) {
			int index = j * width + i;
			unsigned char b1 = (unsigned char)image_resize.data[3 * index + 0];
			unsigned char g1 = (unsigned char)image_resize.data[3 * index + 1];
			unsigned char r1 = (unsigned char)image_resize.data[3 * index + 2];
			filter->frame_info.data[index] = (float)r1;
			filter->frame_info.data[width * height + index] = (float)g1;
			filter->frame_info.data[width * height * 2 + index] = (float)b1;
		}
	}
	// verify
	//cv::imwrite("D:/visual/RadarProc/data_pre_v2/resize.png", image_resize);
	//{
	//	cv::Mat resave_img = image_resize.clone();
	//	for (int j = 0; j < height; ++j) {
	//		for (int i = 0; i < width; ++i) {
	//			int index = j * width + i;
	//			(unsigned char)resave_img.data[3 * index + 2] = filter->frame_info.data[index];	//r
	//			(unsigned char)resave_img.data[3 * index + 1] = filter->frame_info.data[width * height + index]; //g
	//			(unsigned char)resave_img.data[3 * index + 0] = filter->frame_info.data[width * height * 2 + index]; //b
	//		}
	//	}
	//	cv::imwrite("D:/visual/RadarProc/nir_data/resize_save.png", resave_img);
	//	resave_img.release();
	//}
	int kkk = 0;
}

// 图像Mat写入连续内存
void get_infrared_image_info(AVS_RADAR_PREPROCESS_IN *filter, Mat &image_infrared_resize) {
	int width = image_infrared_resize.cols;
	int height = image_infrared_resize.rows;
	// 遍历图像像素
	for (int j = 0; j < height; ++j) {
		for (int i = 0; i < width; ++i) {
			int index = j * width + i;
			unsigned char pixel = (unsigned char)image_infrared_resize.data[3 * index + 0];
			filter->infrared_frame_info.data[index] = (float)pixel;
		}
	}
	// verify
	//cv::imwrite("D:/visual/RadarProc/nir_data/resize_nir.png", image_infrared_resize);
	//{
	//	cv::Mat resave_img = image_infrared_resize.clone();
	//	for (int j = 0; j < height; ++j) {
	//		for (int i = 0; i < width; ++i) {
	//			int index = j * width + i;
	//			(unsigned char)resave_img.data[index] = filter->infrared_frame_info.data[index];	//r
	//		}
	//	}
	//	cv::imwrite("D:/visual/RadarProc/nir_data/resize_nir_2.png", resave_img);
	//	resave_img.release();
	//}
}

// 雷达数据预处理验证
void verify_radar_preprocess(AVS_RADAR_PREPROCESS_IN *inbuf) {
	int dst_width = inbuf->frame_info.width;
	int dst_height = inbuf->frame_info.height;
	cv::Mat resave_img(dst_height, dst_width, CV_8UC3, cv::Scalar(0, 0, 0));
	int offset1 = dst_width * dst_height * 3 * sizeof(float);
	int offset2 = offset1 + dst_width * dst_height * sizeof(float);
	float *src_data = (float *)inbuf->frame_info.data;
	float *radar_rcs_map = (float *)((QWORD)inbuf->frame_info.data + offset1);
	float *radar_dis_map = (float *)((QWORD)inbuf->frame_info.data + offset2);
	for (int j = 0; j < dst_height; ++j) {
		for (int i = 0; i < dst_width; ++i) {
			int index = j * dst_width + i;
			(unsigned char)resave_img.data[3 * index + 0] = 128;	//r
			(unsigned char)resave_img.data[3 * index + 1] = (unsigned char)(radar_rcs_map[index] + 128); //g
			(unsigned char)resave_img.data[3 * index + 2] = (unsigned char)(radar_dis_map[index] + 128); //b
		}
	}
	cv::imwrite("../data_pre_v2/verify_kkk.png", resave_img);
	resave_img.release();
}

void get_classification_info(AVS_RADAR_POSTPROCESS_IN *post_input, string &classification) {
	int i = 0;
	ifstream fin(classification);
	string radar_point;
	while (getline(fin, radar_point)) {
		vector<float> point_info;
		string2float(radar_point, point_info);	// 将一行字符串转换为vector
		post_input->classification[i]->conf[0] = point_info[0];
		post_input->classification[i]->conf[1] = point_info[1];
		post_input->classification[i]->conf[2] = point_info[2];
		post_input->classification[i]->conf[3] = point_info[3];
		post_input->classification[i]->conf[4] = point_info[4];
		post_input->classification[i]->conf[5] = point_info[5];
		post_input->classification[i]->conf[6] = point_info[6];
		post_input->classification[i]->conf[7] = point_info[7];
		++i;
	}
	fin.close();
}

void get_regression_info(AVS_RADAR_POSTPROCESS_IN *post_input, string &classification) {
	int i = 0;
	ifstream fin(classification);
	string radar_point;
	while (getline(fin, radar_point)) {
		vector<float> point_info;
		string2float(radar_point, point_info);	// 将一行字符串转换为vector
		post_input->regression[i]->x1 = point_info[0];
		post_input->regression[i]->y1 = point_info[1];
		post_input->regression[i]->x2 = point_info[2];
		post_input->regression[i]->y2 = point_info[3];
		++i;
	}
	fin.close();
}

void verify_radar_undistortprocess(AVS_RADAR_PREPROCESS_OUT *undistort_output) {
	int dst_width = undistort_output->width;
	int dst_height = undistort_output->height;
	cv::Mat resave_img(dst_height, dst_width, CV_8UC3, cv::Scalar(0, 0, 0));
	int offset = dst_width * dst_height * sizeof(float);
	float *R_map = (float *)undistort_output->data;
	float *G_map = (float *)((QWORD)undistort_output->data + offset);
	float *B_map = (float *)((QWORD)undistort_output->data + offset * 2);
	for (int j = 0; j < dst_height; ++j) {
		for (int i = 0; i < dst_width; ++i) {
			int index = j * dst_width + i;
			(unsigned char)resave_img.data[3 * index + 2] = R_map[index]; //r
			(unsigned char)resave_img.data[3 * index + 1] = G_map[index]; //g
			(unsigned char)resave_img.data[3 * index + 0] = B_map[index]; //b
		}
	}
	cv::imwrite("../nir_data/verify_undistort.png", resave_img);
	resave_img.release();
	int kkk = 0;
}

void verify_radar_infrared_interpolationprocess(AVS_RADAR_PREPROCESS_OUT *infrared_output) {
	int dst_width = infrared_output->width;
	int dst_height = infrared_output->height;
	cv::Mat resave_img(dst_height, dst_width, CV_8UC1, cv::Scalar(0, 0, 0));
	float *dst_map = (float *)infrared_output->data;
	for (int j = 0; j < dst_height; ++j) {
		for (int i = 0; i < dst_width; ++i) {
			int index = j * dst_width + i;
			(unsigned char)resave_img.data[index] = dst_map[index];
		}
	}
	cv::imwrite("../nir_data/verify_nir_resize_1.png", resave_img);
	resave_img.release();
	int kkk = 0;
}

void verify_radar_interpolationprocess(AVS_RADAR_PREPROCESS_OUT *resize_output) {
	int dst_width = resize_output->width;
	int dst_height = resize_output->height;
	cv::Mat resave_img(dst_height, dst_width, CV_8UC3, cv::Scalar(0, 0, 0));
	int offset = dst_width * dst_height * sizeof(float);
	float *R_map = (float *)resize_output->data;
	float *G_map = (float *)((QWORD)resize_output->data + offset);
	float *B_map = (float *)((QWORD)resize_output->data + offset * 2);
	for (int j = 0; j < dst_height; ++j) {
		for (int i = 0; i < dst_width; ++i) {
			int index = j * dst_width + i;
			(unsigned char)resave_img.data[3 * index + 2] = R_map[index]; //r
			(unsigned char)resave_img.data[3 * index + 1] = G_map[index]; //g
			(unsigned char)resave_img.data[3 * index + 0] = B_map[index]; //b
		}
	}
	cv::imwrite("../nir_data/verify_resize_1.png", resave_img);
	resave_img.release();
	int kkk = 0;
}

void verify_radar_postprocess(	cv::Mat						*image_resize,
								AVS_RADAR_POSTPROCESS_OUT	*post_output) {
	int i = 0;
	for (i = 0; i < post_output->obj_num; ++i) {
		if (post_output->obj_info[i]->confidence < 0.5)
			break;
		cv::Point pt1, pt2;
		pt1.x = post_output->obj_info[i]->x1;
		pt1.y = post_output->obj_info[i]->y1;
		pt2.x = post_output->obj_info[i]->x2;
		pt2.y = post_output->obj_info[i]->y2;
		int cls = post_output->obj_info[i]->cls;
		if (cls == AVS_RADAR_OBJ_HUMAN) {
			cv::rectangle(*image_resize, pt1, pt2, cv::Scalar(255, 0, 0), 4, 8);
		} else if (cls == AVS_RADAR_OBJ_BIBYCLE){
			cv::rectangle(*image_resize, pt1, pt2, cv::Scalar(0, 255, 0), 4, 8);
		} else if (cls == AVS_RADAR_OBJ_BUS) {
			cv::rectangle(*image_resize, pt1, pt2, cv::Scalar(0, 0, 255), 4, 8);
		} else if (cls == AVS_RADAR_OBJ_CAR) {
			cv::rectangle(*image_resize, pt1, pt2, cv::Scalar(255, 0, 255), 4, 8);
		} else if (cls == AVS_RADAR_OBJ_MOTORCYCLE) {
			cv::rectangle(*image_resize, pt1, pt2, cv::Scalar(255, 255, 0), 4, 8);
		} else if (cls == AVS_RADAR_OBJ_TRAILER) {
			cv::rectangle(*image_resize, pt1, pt2, cv::Scalar(0, 255, 255), 4, 8);
		} else if (cls == AVS_RADAR_OBJ_TRUCK) {
			cv::rectangle(*image_resize, pt1, pt2, cv::Scalar(128, 255, 128), 4, 8);
		}
	}
	//cv::imwrite("jpgs/verify_resize_final_RT.png", *image_resize);
 	int kkk = 0;
}

void show_obj(AVS_RADAR_POSTPROCESS_OUT *post_output) {
	int i = 0;
	for (i = 0; i < post_output->obj_num; ++i) {
		printf("%d \t cor: %d %d %d %d \t cls: %d \t vel: %f \t \
 dis: %f \t conf:%f \t temp_pixel:%f \t temp:%f \n",
			i,
			post_output->obj_info[i]->x1,
			post_output->obj_info[i]->y1,
			post_output->obj_info[i]->x2,
			post_output->obj_info[i]->y2,
			post_output->obj_info[i]->cls,
			post_output->obj_info[i]->velocity,
			post_output->obj_info[i]->distance,
			post_output->obj_info[i]->confidence,
			post_output->obj_info[i]->temp_pixel,
			post_output->obj_info[i]->temp);
	}
}
