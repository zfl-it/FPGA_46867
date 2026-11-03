#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

class BgrToNv12Converter {
public:
    bool convert(const cv::Mat& bgr,
                 int dst_width,
                 int dst_height,
                 int dst_hor_stride,
                 int dst_ver_stride,
                 std::vector<uint8_t>& nv12);

private:
    bool convertCpu(const cv::Mat& bgr,
                    int dst_width,
                    int dst_height,
                    int dst_hor_stride,
                    int dst_ver_stride,
                    std::vector<uint8_t>& nv12);

    cv::Mat i420_;
    bool cpu_fallback_logged_ = false;
    bool rga_failure_logged_ = false;
};
