#include "rknn_process.h"
#include <QDebug>
#include <QPainter>
#include <QLineEdit>
#include <QDateTime>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <rknn_api.h>

using namespace std;
using namespace cv;

RknnProcess::RknnProcess(QString model_name) :
m_model_name(model_name) {
    Init();
}

RknnProcess::~RknnProcess() {
    Deinit();
}

static void dump_tensor_attr(rknn_tensor_attr* attr)
{
  printf("  index=%d, name=%s, n_dims=%d, dims=[%d, %d, %d, %d], n_elems=%d, size=%d, fmt=%s, type=%s, qnt_type=%s, "
         "zp=%d, scale=%f\n",
         attr->index, attr->name, attr->n_dims, attr->dims[0], attr->dims[1], attr->dims[2], attr->dims[3],
         attr->n_elems, attr->size, get_format_string(attr->fmt), get_type_string(attr->type),
         get_qnt_type_string(attr->qnt_type), attr->zp, attr->scale);
}

bool RknnProcess::Init() {
    FILE* fp = fopen(m_model_name.toStdString().c_str(), "rb");
    if (fp == nullptr) {
      printf("fopen %s fail!\n", m_model_name.toStdString().c_str());
      return NULL;
    }
    fseek(fp, 0, SEEK_END);
    int            model_len = ftell(fp);
    model     = (unsigned char*)malloc(model_len);
    fseek(fp, 0, SEEK_SET);
    if (model_len != fread(model, 1, model_len, fp)) {
      printf("fread %s fail!\n", m_model_name.toStdString().c_str());
      free(model);
      return NULL;
    }
    model_size = model_len;
    if (fp) {
      fclose(fp);
    }

      int            ret       = rknn_init(&ctx, model, model_size, 0, NULL);
      if (ret < 0) {
        printf("rknn_init fail! ret=%d\n", ret);
        return -1;
      }

      // Get sdk and driver version
      rknn_sdk_version sdk_ver;
      ret = rknn_query(ctx, RKNN_QUERY_SDK_VERSION, &sdk_ver, sizeof(sdk_ver));
      if (ret != RKNN_SUCC) {
        printf("rknn_query fail! ret=%d\n", ret);
        return -1;
      }

      printf("rknn_api/rknnrt version: %s, driver version: %s\n", sdk_ver.api_version, sdk_ver.drv_version);

      // Get Model Input Output Info
      rknn_input_output_num io_num;
      ret = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
      if (ret != RKNN_SUCC) {
        printf("rknn_query fail! ret=%d\n", ret);
        return -1;
      }
      printf("model input num: %d, output num: %d\n", io_num.n_input, io_num.n_output);

      printf("input tensors:\n");
      memset(&input_attrs, 0, sizeof(rknn_tensor_attr));
      ret = rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &(input_attrs), sizeof(rknn_tensor_attr));
      if (ret < 0) {
        printf("rknn_init error! ret=%d\n", ret);
        return -1;
      }
      dump_tensor_attr(&input_attrs);

      printf("output tensors:\n");
      memset(&output_attrs, 0,sizeof(rknn_tensor_attr));
      ret = rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &(output_attrs), sizeof(rknn_tensor_attr));
      if (ret != RKNN_SUCC) {
        printf("rknn_query fail! ret=%d\n", ret);
        return -1;
      }
      dump_tensor_attr(&output_attrs);

      // Get custom string
      rknn_custom_string custom_string;
      ret = rknn_query(ctx, RKNN_QUERY_CUSTOM_STRING, &custom_string, sizeof(custom_string));
      if (ret != RKNN_SUCC) {
        printf("rknn_query fail! ret=%d\n", ret);
        return -1;
      }
      printf("custom string: %s\n", custom_string.string);

    input_buffer = reinterpret_cast<unsigned char *>(malloc(input_attrs.dims[1] * input_attrs.dims[2] * input_attrs.dims[3]));
    output_buffer = reinterpret_cast<int8_t *>(malloc(output_attrs.dims[1] * output_attrs.dims[2] * output_attrs.dims[3] * sizeof(int8_t)));

    // Create input tensor memory
    input_mem = rknn_create_mem(ctx, input_attrs.size_with_stride);
    int output_size = output_attrs.n_elems * sizeof(int8_t);
    output_mem  = rknn_create_mem(ctx, output_size);

    // Set input tensor memory
    ret = rknn_set_io_mem(ctx, input_mem, &input_attrs);
    if (ret < 0) {
      printf("rknn_set_io_mem fail! ret=%d\n", ret);
      return -1;
    }

    // Set output tensor memory
    output_attrs.type = RKNN_TENSOR_INT8;
    // set output memory and attribute
    ret = rknn_set_io_mem(ctx, output_mem, &output_attrs);
    if (ret < 0) {
        printf("rknn_set_io_mem fail! ret=%d\n", ret);
        return -1;
    }

    return true;
}

bool RknnProcess::Deinit() {

    rknn_destroy_mem(ctx, input_mem);
    rknn_destroy_mem(ctx, output_mem);

    // destroy
    rknn_destroy(ctx);

    if (model != nullptr) {
      free(model);
    }

    if (input_buffer)
        free(input_buffer);
    if (output_buffer)
     free(output_buffer);

    return true;
}

#include <sys/time.h>
static inline int64_t getCurrentTimeUs()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_sec * 1000000 + tv.tv_usec;
}

int RknnProcess::Run() {
    int64_t start_us  = getCurrentTimeUs();
    int ret               = rknn_run(ctx, NULL);
    int64_t elapse_us = getCurrentTimeUs() - start_us;
    if (ret < 0) {
      printf("rknn run error %d\n", ret);
      return -1;
    }
    printf("Elapse Time = %.2fms, FPS = %.2f\n", elapse_us / 1000.f, 1000.f * 1000.f / elapse_us);
    return ret;
}



