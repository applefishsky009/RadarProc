#include "test.hpp"

int main() {
	// 变量初始化
	int i = 0;
	double t = 0.f;

	// 预处理输入数据
	// string jpg = "../data_pre_v2/001_3.5cm.png";
	//string jpg = "../pre_post_data/n015-2018-07-24-11-22-45+0800__CAM_FRONT__1532402929162460.jpg";
	//string infrared_jpg = "../pre_post_data/n015-2018-07-24-11-22-45+0800__CAM_FRONT__1532402929162460.jpg";
	string jpg = "../nir_data/2_rgb_20210106_110350.jpg";
	string infrared_jpg = "../nir_data/1_nir_20210106_110330.jpg";
	vector<string> txts;
	txts.push_back("../pre_post_data/3_sweep_radar_1.txt");
	txts.push_back("../pre_post_data/3_sweep_radar_2.txt");
	txts.push_back("../pre_post_data/3_sweep_radar_3.txt");
	txts.push_back("../pre_post_data/3_sweep_radar_4.txt");
	txts.push_back("../pre_post_data/3_sweep_radar_5.txt");
	txts.push_back("../pre_post_data/3_sweep_radar_6.txt");
	txts.push_back("../pre_post_data/3_sweep_radar_7.txt");
	txts.push_back("../pre_post_data/3_sweep_radar_8.txt");
	txts.push_back("../pre_post_data/3_sweep_radar_9.txt");
	txts.push_back("../pre_post_data/3_sweep_radar_10.txt");
	txts.push_back("../pre_post_data/3_sweep_radar_11.txt");
	txts.push_back("../pre_post_data/3_sweep_radar_12.txt");
	txts.push_back("../pre_post_data/3_sweep_radar_13.txt");
	Mat image_source = imread(jpg);
	Mat image_infrared = imread(infrared_jpg);
	Mat image_resize, image_infrared_resize;
	resize(image_source, image_resize, cv::Size(1920, 1080));
	resize(image_infrared, image_infrared_resize, cv::Size(1280, 960));
	cv::imwrite("D:/visual/RadarProc/nir_data/resize.png", image_resize);

	// 输入验证
	//namedWindow("TrackBar", CV_WINDOW_AUTOSIZE);
	//imshow("TrackBar", image_resize);
	//int nWaitTime = 0;
	//while (true) {
	//	char chKey = cv::waitKey(nWaitTime);
	//	if (chKey == 27)
	//		break;
	//	if (chKey == ' ')
	//		nWaitTime = !nWaitTime;
	//}

	// 后处理输入数据
	string classification = "../pre_post_data/3_classification.txt";
	string regression = "../pre_post_data/3_regression.txt";

	// 初始化输入句柄 包含预处理句柄和后处理句柄
	AVS_RADAR_PROCESS_IN input;
	memset(&input, 0, sizeof(AVS_RADAR_PROCESS_IN));	// 句柄初始化

	AVS_RADAR_PREPROCESS_IN *pre_input = &input.pre_input;
	pre_input->frame_info.width = image_resize.cols;
	pre_input->frame_info.height = image_resize.rows;
	pre_input->frame_info.channel = 3;
	pre_input->infrared_frame_info.width = image_infrared_resize.cols;
	pre_input->infrared_frame_info.height = image_infrared_resize.rows;
	pre_input->infrared_frame_info.channel = 1;
	pre_input->infrared_calib_info.m11 = 0.989852860539301;
	pre_input->infrared_calib_info.m12 = -0.1461011482095614;
	pre_input->infrared_calib_info.m13 = 355.6865876136517;
	pre_input->infrared_calib_info.m21 = 0.03590646316358077;
	pre_input->infrared_calib_info.m22 = 0.8554565306626785;
	pre_input->infrared_calib_info.m23 = 89.58020698233526;
	pre_input->infrared_calib_info.m31 = 2.360413366804856e-05;
	pre_input->infrared_calib_info.m32 = -8.960393897429334e-05;
	pre_input->infrared_calib_info.m33 = 1;

	// 内存计算
	AVS_MEM_TAB mem_tab[AVS_RADAR_MEM_TAB];
	AVS_GetMemSize(&input, mem_tab);

	// 内存申请
	AVS_alloc_mem_tab(mem_tab, AVS_RADAR_MEM_TAB);

	// 内存分配
	void *filter = nullptr;
	AVS_CreatMemSize(&input, mem_tab, &filter);

	// 图像数据输入 rgb数据
	get_image_info(pre_input, image_resize);
	// 图像数据输入 红外数据
	get_infrared_image_info(pre_input, image_infrared_resize);
	// 相机参数设置
	pre_input->cam_info.fu = 1007.84;
	//pre_input->cam_info.fu = 483;
	pre_input->cam_info.fv = 1014.16;
	//pre_input->cam_info.fv = 480;
	pre_input->cam_info.cu = 649.10;
	//pre_input->cam_info.cu = 320;
	pre_input->cam_info.cv = 360.32;
	//pre_input->cam_info.cv = 192;
	pre_input->cam_info.height = 1750;	// mm 相机离地面高度
	pre_input->cam_info.pitch = 1.4 * (AVS_PI * 1.0 / 180);
	pre_input->cam_info.yaw = 0 * (AVS_PI * 1.0 / 180);
	pre_input->cam_info.k1 = -0.3923900430945697;
	pre_input->cam_info.k2 = 0.1413877908438226;
	pre_input->cam_info.k3 = 0.002443674780940085;
	pre_input->cam_info.p1 = -0.003599841241478862;
	pre_input->cam_info.p2 = -0.01032105435135168;

	// 雷达数据输入 x, y, rcs, invalid_state, dyn_prop, ambig_state, radar_v_comp一定需要
	//get_radar_point(pre_input, txts);

	// 雷达参数设置
	pre_input->radar_calib.radar_height		= 500;	// 雷达架设高度
	pre_input->radar_calib.radar_top		= 3000;	// 目标估计高度，固定为3000
	pre_input->radar_calib.radar_radian		= 0 * (AVS_PI * 1.0 / 180);
	pre_input->radar_calib.radarInCamera_X	= 0;	// 假设没有水平距离
	pre_input->radar_calib.radarInCamera_Y	= 0;	// 假设没有前后距离

	// 畸变校正参数设置
	pre_input->image_config.undistort_avalible = 0;	// 开启畸变校正

	// 图像畸变矫正接口
	AVS_RADAR_PREPROCESS_OUT undistort_output;
	t = (double)cv::getTickCount();
	AVS_UndistortProcess(filter, pre_input, sizeof(AVS_RADAR_PREPROCESS_IN), &undistort_output, sizeof(AVS_RADAR_PREPROCESS_OUT));
	t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
	cout << "undistortprocess cost time " << t << "s" << endl;

	// 畸变校正功能验证
	verify_radar_undistortprocess(&undistort_output);

	// 降采样参数配置
	pre_input->image_config.infrared_align = 1;	// 开启红外对齐，warpPerspective功能

	// 图像降采样接口 目前固定到640 * 384, 如果有红外输入，先对齐再降采样
	AVS_RADAR_INTERPOLATIONPROCESS_OUT interpolation_output;
	t = (double)cv::getTickCount();
	AVS_InterpolationProcess(filter, pre_input, sizeof(AVS_RADAR_PREPROCESS_IN), &interpolation_output, sizeof(AVS_RADAR_INTERPOLATIONPROCESS_OUT));
	t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
	cout << "interpolationprocess cost time " << t << "s" << endl;

	// 图像降采样功能验证
	verify_radar_interpolationprocess(&interpolation_output.rgb_info);
	// 红外图像降采样功能验证
	verify_radar_infrared_interpolationprocess(&interpolation_output.infrared_info);

	// debug信息输入, 如果不是debug库不用输入
	pre_input->debug_info.fp_point		= fopen("../pre_post_data/3_radar_points.txt", "r");
	pre_input->debug_info.fp_point_xyz	= fopen("../pre_post_data/3_radar_xyz_endpoint.txt", "r");
	pre_input->debug_info.out_map		= fopen("../pre_post_data/radar.rgb", "wb");
	pre_input->debug_info.fp[0]			= fopen("../pre_post_data/3_sweep_radar_1.txt", "r");
	pre_input->debug_info.fp[1]			= fopen("../pre_post_data/3_sweep_radar_2.txt", "r");
	pre_input->debug_info.fp[2]			= fopen("../pre_post_data/3_sweep_radar_3.txt", "r");
	pre_input->debug_info.fp[3]			= fopen("../pre_post_data/3_sweep_radar_4.txt", "r");
	pre_input->debug_info.fp[4]			= fopen("../pre_post_data/3_sweep_radar_5.txt", "r");
	pre_input->debug_info.fp[5]			= fopen("../pre_post_data/3_sweep_radar_6.txt", "r");
	pre_input->debug_info.fp[6]			= fopen("../pre_post_data/3_sweep_radar_7.txt", "r");
	pre_input->debug_info.fp[7]			= fopen("../pre_post_data/3_sweep_radar_8.txt", "r");
	pre_input->debug_info.fp[8]			= fopen("../pre_post_data/3_sweep_radar_9.txt", "r");
	pre_input->debug_info.fp[9]			= fopen("../pre_post_data/3_sweep_radar_10.txt", "r");
	pre_input->debug_info.fp[10]		= fopen("../pre_post_data/3_sweep_radar_11.txt", "r");
	pre_input->debug_info.fp[11]		= fopen("../pre_post_data/3_sweep_radar_12.txt", "r");
	pre_input->debug_info.fp[12]		= fopen("../pre_post_data/3_sweep_radar_13.txt", "r");

	// 雷达数据预处理
	AVS_RADAR_PREPROCESS_OUT output;
	t = (double)cv::getTickCount();
	AVS_PreProcess(filter, pre_input, sizeof(AVS_RADAR_PREPROCESS_IN), &output, sizeof(AVS_RADAR_PREPROCESS_OUT));
	t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
	cout << "preprocess cost time " << t << "s" << endl;

	fclose(pre_input->debug_info.fp_point);
	fclose(pre_input->debug_info.fp_point_xyz);
	fclose(pre_input->debug_info.out_map);
	for (i = 0; i < 13; i++) {
		fclose(pre_input->debug_info.fp[i]);
	}

	// 雷达数据预处理验证
	verify_radar_preprocess(pre_input);

	// 雷达数据后处理句柄
	AVS_RADAR_POSTPROCESS_IN *post_input = &input.post_input;

	// 读入分类信息
	get_classification_info(post_input, classification);

	// 读入回归信息
	get_regression_info(post_input, regression);

	// 后处理调试信息
	//post_input->post_debug_info.classification	= fopen("../pre_post_data/3_classification.txt", "r");
	//post_input->post_debug_info.regression		= fopen("../pre_post_data/3_regression.txt", "r");
	//post_input->post_debug_info.classification	= fopen("../pre_post_data/caffe_cls_01.05_2.txt", "r");
	//post_input->post_debug_info.regression		= fopen("../pre_post_data/caffe_reg_01.05_2.txt", "r");
	post_input->post_debug_info.classification	= fopen("../pre_post_data/DALL_cls_F.txt", "r");
	post_input->post_debug_info.regression		= fopen("../pre_post_data/DALL_reg_F.txt", "r");

	// 雷达数据后处理
	AVS_RADAR_POSTPROCESS_OUT post_output;
	t = (double)cv::getTickCount();
	AVS_PostProcess(filter, post_input, sizeof(AVS_RADAR_POSTPROCESS_IN), &post_output, sizeof(AVS_RADAR_POSTPROCESS_OUT));
	t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
	cout << "postprocess cost time " << t << "s" << endl;

	fclose(post_input->post_debug_info.classification);
	fclose(post_input->post_debug_info.regression);

	for (i = 0; i < post_output.obj_num; ++i) {
		printf("%d \t cor: %d %d %d %d \t cls: %d \t vel: %f \t \
 dis: %f \t conf:%f \t temp_pixel:%f \t temp:%f \n", 
			i,
			post_output.obj_info[i]->x1, 
			post_output.obj_info[i]->y1,
			post_output.obj_info[i]->x2,
			post_output.obj_info[i]->y2,
			post_output.obj_info[i]->cls,
			post_output.obj_info[i]->velocity,
			post_output.obj_info[i]->distance,
			post_output.obj_info[i]->confidence,
			post_output.obj_info[i]->temp_pixel,
			post_output.obj_info[i]->temp);
	}
	verify_radar_postprocess(&image_resize, &post_output);
	return 0;
}