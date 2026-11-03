#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

struct MppH264Config {
    int width = 1280;
    int height = 720;
    int fps = 25;
    int bitrate = 3000000;
    int gop = 25;
    std::string profile = "baseline";
};

struct H264Packet {
    std::vector<uint8_t> data;
    uint64_t pts_ms = 0;
    uint64_t dts_ms = 0;
    bool keyframe = false;
};

class MppH264Encoder {
public:
    MppH264Encoder() = default;
    ~MppH264Encoder();

    bool init(const MppH264Config& cfg);
    bool encodeNv12(const uint8_t* nv12, size_t size, uint64_t pts_ms, std::vector<H264Packet>& packets);
    void release();
    bool isReady() const { return ready_; }

    int width() const { return cfg_.width; }
    int height() const { return cfg_.height; }
    int horStride() const { return hor_stride_; }
    int verStride() const { return ver_stride_; }
    size_t inputSize() const { return input_size_; }

private:
    bool setupEncoderConfig();

    MppH264Config cfg_;
    int hor_stride_ = 0;
    int ver_stride_ = 0;
    size_t input_size_ = 0;
    size_t frame_buffer_size_ = 0;
    size_t mdinfo_size_ = 0;

    void* ctx_ = nullptr;
    void* mpi_ = nullptr;
    void* enc_cfg_ = nullptr;
    void* buffer_group_ = nullptr;
    void* frame_buffer_ = nullptr;
    void* mdinfo_buffer_ = nullptr;
    bool input_log_printed_ = false;
    bool ready_ = false;
};
