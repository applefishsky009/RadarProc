#ifndef _AVS_CAFFE_LIB_H_
#define _AVS_CAFFE_LIB_H_

#include "Avs_Radar.h"
#include "Avs_Caffe.h"
#include "caffe/caffe.hpp"
#include "caffe/caffe_reg.hpp"
using namespace caffe;

typedef struct _AVS_CAFFE_FILTER_ {
	boost::shared_ptr<Net<float>> net;
}AVS_CAFFE_FILTER;

unsigned int get_blob_index(boost::shared_ptr<Net<float>> &net, char *query_blob_name);

unsigned int get_layer_index(boost::shared_ptr<Net<float>> &net, char *query_layer_name);

void caffe_forward(boost::shared_ptr<Net<float>> &net, float *data_ptr);

#endif