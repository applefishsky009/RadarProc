#include "test.hpp"

#define IS_VIDEO 0

#if IS_VIDEO
int main(int argc, char* argv[]) {
	// ������ʼ��
	int i = 0;
	double t = 0.f;

	// Ԥ������������
	// string jpg = "../data_pre_v2/001_3.5cm.png";
	//string jpg = "../pre_post_data/n015-2018-07-24-11-22-45+0800__CAM_FRONT__1532402929162460.jpg";
	//string infrared_jpg = "../pre_post_data/n015-2018-07-24-11-22-45+0800__CAM_FRONT__1532402929162460.jpg";
	//string jpg = "../nir_data/2_rgb_20210106_110350.jpg";
	//string infrared_jpg = "../nir_data/1_nir_20210106_110330.jpg";
	//string jpg = "../nir_data/nir_test.jpg";
	//string infrared_jpg = "../nir_data/1_nir_20210106_110330.jpg";
	string jpg = "../nir_data/2_test.jpg";
	int kkk = argc;
	//string jpg = argv[1];
	string infrared_jpg = "../nir_data/20210120_151743_nir.jpg";
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
	resize(image_source, image_resize, cv::Size(640, 384));
	resize(image_infrared, image_infrared_resize, cv::Size(1280, 960));
	//cv::imwrite("D:/visual/RadarProc/nir_data/resize.png", image_resize);

	// ������֤
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

	// ������������
	string classification = "../pre_post_data/3_classification.txt";
	string regression = "../pre_post_data/3_regression.txt";

	// ��ʼ�������� ����Ԥ�������ͺ�����
	AVS_RADAR_PROCESS_IN input;
	memset(&input, 0, sizeof(AVS_RADAR_PROCESS_IN));	// �����ʼ��

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

	// �ڴ����
	AVS_MEM_TAB mem_tab[AVS_RADAR_MEM_TAB];
	AVS_GetMemSize(&input, mem_tab);

	// �ڴ�����
	AVS_alloc_mem_tab(mem_tab, AVS_RADAR_MEM_TAB);

	// �ڴ����
	void *filter = nullptr;
	AVS_CreatMemSize(&input, mem_tab, &filter);

	// ͼ���������� rgb����
	get_image_info(pre_input, image_resize);
	// ͼ���������� ��������
	get_infrared_image_info(pre_input, image_infrared_resize);
	// �����������
	pre_input->cam_info.fu = 1007.84;
	//pre_input->cam_info.fu = 483;
	pre_input->cam_info.fv = 1014.16;
	//pre_input->cam_info.fv = 480;
	pre_input->cam_info.cu = 649.10;
	//pre_input->cam_info.cu = 320;
	pre_input->cam_info.cv = 360.32;
	//pre_input->cam_info.cv = 192;
	pre_input->cam_info.height = 1750;	// mm ��������߶�
	pre_input->cam_info.pitch = 1.4 * (AVS_PI * 1.0 / 180);
	pre_input->cam_info.yaw = 0 * (AVS_PI * 1.0 / 180);
	pre_input->cam_info.k1 = -0.3923900430945697;
	pre_input->cam_info.k2 = 0.1413877908438226;
	pre_input->cam_info.k3 = 0.002443674780940085;
	pre_input->cam_info.p1 = -0.003599841241478862;
	pre_input->cam_info.p2 = -0.01032105435135168;

	// �״��������� x, y, rcs, invalid_state, dyn_prop, ambig_state, radar_v_compһ����Ҫ
	//get_radar_point(pre_input, txts);

	// �״��������
	pre_input->radar_calib.radar_height		= 500;	// �״����߶�
	pre_input->radar_calib.radar_top		= 3000;	// Ŀ����Ƹ߶ȣ��̶�Ϊ3000
	pre_input->radar_calib.radar_radian		= 0 * (AVS_PI * 1.0 / 180);
	pre_input->radar_calib.radarInCamera_X	= 0;	// ����û��ˮƽ����
	pre_input->radar_calib.radarInCamera_Y	= 0;	// ����û��ǰ�����

	// ����У����������
	pre_input->image_config.undistort_avalible = 0;	// �رջ���У��

	// ͼ���������ӿ�
	AVS_RADAR_PREPROCESS_OUT undistort_output;
	t = (double)cv::getTickCount();
	AVS_UndistortProcess(filter, pre_input, sizeof(AVS_RADAR_PREPROCESS_IN), &undistort_output, sizeof(AVS_RADAR_PREPROCESS_OUT));
	t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
	cout << "undistortprocess cost time " << t << "s" << endl;

	// ����У��������֤
	verify_radar_undistortprocess(&undistort_output);

	// ��������������
	pre_input->image_config.infrared_align = 0;	// �رպ�����룬warpPerspective����

	// ͼ�񽵲����ӿ� Ŀǰ�̶���640 * 384, ����к������룬�ȶ����ٽ�����
	AVS_RADAR_INTERPOLATIONPROCESS_OUT interpolation_output;
	t = (double)cv::getTickCount();
	AVS_InterpolationProcess(filter, pre_input, sizeof(AVS_RADAR_PREPROCESS_IN), &interpolation_output, sizeof(AVS_RADAR_INTERPOLATIONPROCESS_OUT));
	t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
	cout << "interpolationprocess cost time " << t << "s" << endl;

	// ͼ�񽵲���������֤
	verify_radar_interpolationprocess(&interpolation_output.rgb_info);
	// ����ͼ�񽵲���������֤
	if (pre_input->image_config.infrared_align) {
		verify_radar_infrared_interpolationprocess(&interpolation_output.infrared_info);
	}

	// debug��Ϣ����, �������debug�ⲻ������
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

	// �״�����Ԥ����
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

	// �״�����Ԥ������֤
	verify_radar_preprocess(pre_input);

	// caffe
	// �ⴴ��
	// �ڴ����
	AVS_CAFFE_INFO caffe_info;
	AVS_MEM_TAB caffe_mem_tab[AVS_RADAR_MEM_TAB];
	AVS_Caffe_GetMemSize(&caffe_info, mem_tab);
	// �ڴ�����
	AVS_alloc_mem_tab(mem_tab, AVS_RADAR_MEM_TAB);
	// �ڴ����
	void *filter_caffe = nullptr;
	AVS_Caffe_CreatMemSize(&caffe_info, mem_tab, &filter_caffe);
	// ��ʼ��
	char *proto = "../CaffeForward/2021-01-14-07-16-39_crf_net.prototxt";
	char *model = "../CaffeForward/2021-01-14-07-16-39_crf_netn.caffemodel";
	caffe_info.proto = proto;
	caffe_info.model = model;
	AVS_Caffe_Init(filter_caffe, &caffe_info, sizeof(AVS_CAFFE_INFO));
	// ǰ��
	memcpy(&caffe_info.caffe_input, &output, sizeof(AVS_RADAR_PREPROCESS_OUT));
	AVS_Caffe_Forward(filter_caffe, &caffe_info, sizeof(AVS_CAFFE_INFO), &input.post_input, sizeof(AVS_RADAR_POSTPROCESS_IN));

	// �״����ݺ�����
	AVS_RADAR_POSTPROCESS_IN *post_input = &input.post_input;

	// ����caffe C++�������ö��������Ϣ
	// ���������Ϣ
	// get_classification_info(post_input, classification);
	// ����ع���Ϣ
	// get_regression_info(post_input, regression);

	// ���������Ϣ
	//post_input->post_debug_info.classification	= fopen("../pre_post_data/3_classification.txt", "r");
	//post_input->post_debug_info.regression		= fopen("../pre_post_data/3_regression.txt", "r");
	//post_input->post_debug_info.classification	= fopen("../pre_post_data/caffe_cls_01.05_2.txt", "r");
	//post_input->post_debug_info.regression		= fopen("../pre_post_data/caffe_reg_01.05_2.txt", "r");
	//post_input->post_debug_info.classification	= fopen("../pre_post_data/DALL_cls_F.txt", "r");
	//post_input->post_debug_info.regression		= fopen("../pre_post_data/DALL_reg_F.txt", "r");
	post_input->post_debug_info.classification	= fopen(argv[2], "r");
	post_input->post_debug_info.regression		= fopen(argv[3], "r");

	// �״����ݺ���
	AVS_RADAR_POSTPROCESS_OUT post_output;
	t = (double)cv::getTickCount();
	AVS_PostProcess(filter, post_input, sizeof(AVS_RADAR_POSTPROCESS_IN), &post_output, sizeof(AVS_RADAR_POSTPROCESS_OUT));
	t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
	cout << "postprocess cost time " << t << "s" << endl;

	fclose(post_input->post_debug_info.classification);
	fclose(post_input->post_debug_info.regression);

	show_obj(&post_output);
	verify_radar_postprocess(&image_resize, &post_output);

	// Ϊ�����Ϣƥ�������Ϣ - Ŀǰ���״�ӿڲ�����
	// AVS_SBProcess(filter, &post_output, sizeof(AVS_RADAR_POSTPROCESS_OUT), &post_output, sizeof(AVS_RADAR_POSTPROCESS_OUT));
	// show_obj(&post_output);

	return 0;
}
#else 
int main(int argc, char* argv[]) {
	// ������ʼ��
	int i = 0;
	double t = 0.f;
	// ��Ƶ������Ϣ����
	string video = "../nir_data/test.mp4";
	VideoCapture cap;
	cap.open(video);
	AVS_CHECK_RETURN(!cap.isOpened(), 1);
	int width		= cap.get(CV_CAP_PROP_FRAME_WIDTH);		// ��ȡ֡���
	int height		= cap.get(CV_CAP_PROP_FRAME_HEIGHT);	// ��ȡ֡�߶�
	int frameRate	= cap.get(CV_CAP_PROP_FPS);				// ��ȡ֡��
	int totalFrames = cap.get(CV_CAP_PROP_FRAME_COUNT);		// ��֡��
	Size s(640, 384);
	VideoWriter writer = VideoWriter("../nir_data/myvideo.avi",
		CV_FOURCC('M', 'J', 'P', 'G'), 25, s);

	// �㷨���ʼ��
	// ��ʼ�������� ����Ԥ�������ͺ�����
	AVS_RADAR_PROCESS_IN input;
	memset(&input, 0, sizeof(AVS_RADAR_PROCESS_IN));	// �����ʼ��
	// Ԥ��������ʼ��
	AVS_RADAR_PREPROCESS_IN *pre_input = &input.pre_input;
	pre_input->frame_info.width = width;
	pre_input->frame_info.height = height;
	pre_input->frame_info.channel = 3;
	pre_input->infrared_frame_info.width = width;
	pre_input->infrared_frame_info.height = height;
	pre_input->infrared_frame_info.channel = 1;
	pre_input->infrared_calib_info.m11 = 0.989852860539301;	// surf�ķ���任����
	pre_input->infrared_calib_info.m12 = -0.1461011482095614;
	pre_input->infrared_calib_info.m13 = 355.6865876136517;
	pre_input->infrared_calib_info.m21 = 0.03590646316358077;
	pre_input->infrared_calib_info.m22 = 0.8554565306626785;
	pre_input->infrared_calib_info.m23 = 89.58020698233526;
	pre_input->infrared_calib_info.m31 = 2.360413366804856e-05;
	pre_input->infrared_calib_info.m32 = -8.960393897429334e-05;
	pre_input->infrared_calib_info.m33 = 1;
	// �ڴ����
	AVS_MEM_TAB mem_tab[AVS_RADAR_MEM_TAB];
	AVS_GetMemSize(&input, mem_tab);
	// �ڴ�����
	AVS_alloc_mem_tab(mem_tab, AVS_RADAR_MEM_TAB);
	// �ڴ����
	void *filter = nullptr;
	AVS_CreatMemSize(&input, mem_tab, &filter);
	// �����������
	pre_input->cam_info.fu = 1007.84;
	pre_input->cam_info.fv = 1014.16;
	pre_input->cam_info.cu = 649.10;
	pre_input->cam_info.cv = 360.32;
	pre_input->cam_info.height = 1750;	// mm ��������߶�
	pre_input->cam_info.pitch = 1.4 * (AVS_PI * 1.0 / 180);
	pre_input->cam_info.yaw = 0 * (AVS_PI * 1.0 / 180);
	pre_input->cam_info.k1 = -0.3923900430945697;
	pre_input->cam_info.k2 = 0.1413877908438226;
	pre_input->cam_info.k3 = 0.002443674780940085;
	pre_input->cam_info.p1 = -0.003599841241478862;
	pre_input->cam_info.p2 = -0.01032105435135168;
	// �״��������
	pre_input->radar_calib.radar_height = 500;	// �״����߶�
	pre_input->radar_calib.radar_top = 3000;	// Ŀ����Ƹ߶ȣ��̶�Ϊ3000
	pre_input->radar_calib.radar_radian = 0 * (AVS_PI * 1.0 / 180);
	pre_input->radar_calib.radarInCamera_X = 0;	// ����û��ˮƽ����
	pre_input->radar_calib.radarInCamera_Y = 0;	// ����û��ǰ�����
	// ����У����������
	pre_input->image_config.undistort_avalible = 0;	// �رջ���У��
	// ��������������
	pre_input->image_config.infrared_align = 0;	// �رպ�����룬warpPerspective����
	// caffe���ʼ��
	AVS_CAFFE_INFO caffe_info;
	AVS_MEM_TAB caffe_mem_tab[AVS_RADAR_MEM_TAB];
	AVS_Caffe_GetMemSize(&caffe_info, caffe_mem_tab);
	// �ڴ�����
	AVS_alloc_mem_tab(caffe_mem_tab, AVS_RADAR_MEM_TAB);
	// �ڴ����
	void *filter_caffe = nullptr;
	AVS_Caffe_CreatMemSize(&caffe_info, caffe_mem_tab, &filter_caffe);
	// ��ʼ��
	char *proto = "../CaffeForward/2021-01-14-07-16-39_crf_net.prototxt";
	char *model = "../CaffeForward/2021-01-14-07-16-39_crf_netn.caffemodel";
	caffe_info.proto = proto;
	caffe_info.model = model;
	AVS_Caffe_Init(filter_caffe, &caffe_info, sizeof(AVS_CAFFE_INFO));

	// ����
	Mat frame;
	int cnt = 0;
	while (1) {
		cap >> frame;	// ��֡
		cnt++;
		cout << "********** frame: "<<  cnt <<  "**********" << endl;
		AVS_CHECK_BREAK(frame.empty());
		Mat image_resize;
		resize(frame, image_resize, cv::Size(640, 384));
		///*
		get_image_info(pre_input, frame);
		// ����У���ӿ�
		AVS_RADAR_PREPROCESS_OUT undistort_output;
		t = (double)cv::getTickCount();
		AVS_UndistortProcess(filter, pre_input, sizeof(AVS_RADAR_PREPROCESS_IN), &undistort_output, sizeof(AVS_RADAR_PREPROCESS_OUT));
		t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
		cout << "undistortprocess cost time " << t << "s" << endl;
		// ����У��������֤
		// verify_radar_undistortprocess(&undistort_output);
		// �������ӿ� Ŀǰ�̶���640 * 384, ����к������룬�ȶ����ٽ�����
		AVS_RADAR_INTERPOLATIONPROCESS_OUT interpolation_output;
		t = (double)cv::getTickCount();
		AVS_InterpolationProcess(filter, pre_input, sizeof(AVS_RADAR_PREPROCESS_IN), &interpolation_output, sizeof(AVS_RADAR_INTERPOLATIONPROCESS_OUT));
		t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
		cout << "interpolationprocess cost time " << t << "s" << endl;
		// ͼ�񽵲���������֤
		// verify_radar_interpolationprocess(&interpolation_output.rgb_info);
		// �״�����Ԥ����
		AVS_RADAR_PREPROCESS_OUT output;
		t = (double)cv::getTickCount();
		AVS_PreProcess(filter, pre_input, sizeof(AVS_RADAR_PREPROCESS_IN), &output, sizeof(AVS_RADAR_PREPROCESS_OUT));
		t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
		cout << "preprocess cost time " << t << "s" << endl;
		// caffeǰ��
		memcpy(&caffe_info.caffe_input, &output, sizeof(AVS_RADAR_PREPROCESS_OUT));
		t = (double)cv::getTickCount();
		AVS_Caffe_Forward(filter_caffe, &caffe_info, sizeof(AVS_CAFFE_INFO), &input.post_input, sizeof(AVS_RADAR_POSTPROCESS_IN));
		t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
		cout << "caffe forward cost time " << t << "s" << endl;
		// �״����ݺ�����
		AVS_RADAR_POSTPROCESS_IN *post_input = &input.post_input;
		// �״����ݺ���
		AVS_RADAR_POSTPROCESS_OUT post_output;
		t = (double)cv::getTickCount();
		AVS_PostProcess(filter, post_input, sizeof(AVS_RADAR_POSTPROCESS_IN), &post_output, sizeof(AVS_RADAR_POSTPROCESS_OUT));
		t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
		cout << "postprocess cost time " << t << "s" << endl;
		//show_obj(&post_output);
		verify_radar_postprocess(&image_resize, &post_output);
		// imshow("video", image_resize);
		//*/
		writer << image_resize;
		int kkk = 0;
	}
	cap.release();
	return 0;
}
#endif // IS_VIDEO