#include "pixel_convert.h"

#include <cstring>
#include <iostream>

#include <opencv2/imgproc.hpp>

#ifdef HAVE_RGA
#include "im2d.h"
#include "rga.h"
#endif

using namespace std;

bool BgrToNv12Converter::convert(const cv::Mat& bgr,
                                 int dst_width,
                                 int dst_height,
                                 int dst_hor_stride,
                                 int dst_ver_stride,
                                 vector<uint8_t>& nv12)
{
    if (bgr.empty() || bgr.type() != CV_8UC3) {
        cerr << "BGR to NV12 failed: input frame must be CV_8UC3" << endl;
        return false;
    }
    if (bgr.cols != dst_width || bgr.rows != dst_height) {
        cerr << "BGR to NV12 failed: input frame size " << bgr.cols << "x" << bgr.rows
             << " does not match encoder size " << dst_width << "x" << dst_height << endl;
        return false;
    }
    if ((dst_width & 1) || (dst_height & 1) || dst_hor_stride < dst_width || dst_ver_stride < dst_height) {
        cerr << "BGR to NV12 failed: invalid destination geometry" << endl;
        return false;
    }

    size_t y_size = static_cast<size_t>(dst_hor_stride) * static_cast<size_t>(dst_ver_stride);
    size_t uv_size = y_size / 2;
    nv12.resize(y_size + uv_size);

#ifdef HAVE_RGA
    if (bgr.isContinuous()) {
        rga_buffer_t src = wrapbuffer_virtualaddr(
            const_cast<unsigned char*>(bgr.data),
            dst_width,
            dst_height,
            RK_FORMAT_BGR_888);
        rga_buffer_t dst = wrapbuffer_virtualaddr(
            nv12.data(),
            dst_width,
            dst_height,
            RK_FORMAT_YCbCr_420_SP,
            dst_hor_stride,
            dst_ver_stride);

        IM_STATUS status = imcvtcolor(src, dst, RK_FORMAT_BGR_888, RK_FORMAT_YCbCr_420_SP);
        if (status == IM_STATUS_SUCCESS || status == IM_STATUS_NOERROR) {
            return true;
        }
        if (!rga_failure_logged_) {
            cerr << "RGA BGR->NV12 failed: " << imStrError(status)
                 << ". Falling back to OpenCV CPU conversion." << endl;
            rga_failure_logged_ = true;
        }
    }
#endif

    if (!cpu_fallback_logged_) {
        cerr << "RGA BGR->NV12 is unavailable; using OpenCV CPU conversion fallback." << endl;
        cpu_fallback_logged_ = true;
    }
    return convertCpu(bgr, dst_width, dst_height, dst_hor_stride, dst_ver_stride, nv12);
}

bool BgrToNv12Converter::convertCpu(const cv::Mat& bgr,
                                    int dst_width,
                                    int dst_height,
                                    int dst_hor_stride,
                                    int dst_ver_stride,
                                    vector<uint8_t>& nv12)
{
    i420_.create(dst_height * 3 / 2, dst_width, CV_8UC1);
    cv::cvtColor(bgr, i420_, cv::COLOR_BGR2YUV_I420);

    uint8_t* y_dst = nv12.data();
    uint8_t* uv_dst = nv12.data() + static_cast<size_t>(dst_hor_stride) * static_cast<size_t>(dst_ver_stride);
    size_t y_plane_size = static_cast<size_t>(dst_hor_stride) * static_cast<size_t>(dst_ver_stride);
    size_t uv_plane_size = y_plane_size / 2;

    memset(y_dst, 16, y_plane_size);
    memset(uv_dst, 128, uv_plane_size);

    const uint8_t* y_src = i420_.ptr<uint8_t>(0);
    const uint8_t* u_src = y_src + static_cast<size_t>(dst_width) * static_cast<size_t>(dst_height);
    const uint8_t* v_src = u_src + static_cast<size_t>(dst_width / 2) * static_cast<size_t>(dst_height / 2);

    for (int row = 0; row < dst_height; ++row) {
        memcpy(y_dst + static_cast<size_t>(row) * dst_hor_stride,
               y_src + static_cast<size_t>(row) * dst_width,
               dst_width);
    }

    for (int row = 0; row < dst_height / 2; ++row) {
        uint8_t* uv_row = uv_dst + static_cast<size_t>(row) * dst_hor_stride;
        const uint8_t* u_row = u_src + static_cast<size_t>(row) * (dst_width / 2);
        const uint8_t* v_row = v_src + static_cast<size_t>(row) * (dst_width / 2);
        for (int col = 0; col < dst_width / 2; ++col) {
            uv_row[col * 2] = u_row[col];
            uv_row[col * 2 + 1] = v_row[col];
        }
    }

    return true;
}
