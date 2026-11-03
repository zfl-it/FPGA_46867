#pragma once

#include <cstdint>
#include <string>

struct ZlmConfig {
    std::string sdk_dir = "./ZLMediaKit";
    std::string config_ini = "./ZLMediaKit/config.ini";
    std::string vhost = "__defaultVhost__";
    std::string app = "live";
    std::string stream = "plate";

    int width = 1280;
    int height = 720;
    int fps = 25;
    int bitrate = 3000000;

    int http_port = 8081;
    int rtsp_port = 8554;
    int rtmp_port = 1935;
    int rtc_port = 8000;
};

struct ZlmResolvedPaths {
    std::string include_dir;
    std::string lib_path;
};

class ZlmStreamer {
public:
    ZlmStreamer() = default;
    ~ZlmStreamer();

    bool init(const ZlmConfig& cfg);
    bool inputH264(const uint8_t* data, int len, uint64_t pts_ms, uint64_t dts_ms);
    void release();
    bool isReady() const { return ready_; }

    std::string webrtcUrl() const;
    std::string httpFlvUrl() const;
    std::string rtspUrl() const;
    const std::string& libPath() const { return paths_.lib_path; }

    static bool resolveSdkPaths(const std::string& sdk_dir, ZlmResolvedPaths& paths, std::string& error);

private:
    ZlmConfig cfg_;
    ZlmResolvedPaths paths_;
    void* media_ = nullptr;
    bool servers_started_ = false;
    bool ready_ = false;
};
