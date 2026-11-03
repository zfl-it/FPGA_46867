#include "zlm_streamer.h"

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <sstream>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

#include "mk_mediakit.h"

using namespace std;

namespace {

bool readableFile(const string& path)
{
    return access(path.c_str(), R_OK) == 0;
}

bool readableDir(const string& path)
{
    struct stat st {};
    return stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

string joinPath(const string& a, const string& b)
{
    if (a.empty()) return b;
    if (a[a.size() - 1] == '/') return a + b;
    return a + "/" + b;
}

string jsonEscape(const char* text)
{
    string s = text ? text : "";
    string out;
    out.reserve(s.size() + 16);
    for (unsigned char ch : s) {
        switch (ch) {
        case '\\': out += "\\\\"; break;
        case '"': out += "\\\""; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        default:
            if (ch < 0x20) {
                char buf[8];
                snprintf(buf, sizeof(buf), "\\u%04x", ch);
                out += buf;
            } else {
                out += static_cast<char>(ch);
            }
        }
    }
    return out;
}

void API_CALL onWebrtcAnswer(void* user_data, const char* answer, const char* err)
{
    const char* response_header[] = {
        "Content-Type", "application/json",
        "Access-Control-Allow-Origin", "*",
        "Access-Control-Allow-Headers", "*",
        "Access-Control-Allow-Methods", "POST, GET, OPTIONS",
        nullptr
    };

    string body;
    if (answer) {
        body = "{\"sdp\":\"" + jsonEscape(answer) + "\",\"type\":\"answer\",\"code\":0}";
    } else {
        body = "{\"msg\":\"" + jsonEscape(err ? err : "mk_webrtc_get_answer_sdp failed") + "\",\"code\":-1}";
    }

    mk_http_response_invoker_do_string(
        static_cast<mk_http_response_invoker>(user_data),
        200,
        response_header,
        body.c_str());
    mk_http_response_invoker_clone_release(static_cast<mk_http_response_invoker>(user_data));
}

void API_CALL onZlmHttpRequest(const mk_parser parser,
                               const mk_http_response_invoker invoker,
                               int* consumed,
                               const mk_sock_info sender)
{
    (void)sender;

    const char* url = mk_parser_get_url(parser);
    const char* method = mk_parser_get_method(parser);
    if (!url || strcmp(url, "/index/api/webrtc") != 0) {
        *consumed = 0;
        return;
    }

    *consumed = 1;

    const char* response_header[] = {
        "Content-Type", "application/json",
        "Access-Control-Allow-Origin", "*",
        "Access-Control-Allow-Headers", "*",
        "Access-Control-Allow-Methods", "POST, GET, OPTIONS",
        nullptr
    };

    if (method && strcmp(method, "OPTIONS") == 0) {
        mk_http_response_invoker_do_string(invoker, 200, response_header, "{\"code\":0}");
        return;
    }

    const char* host = mk_parser_get_header(parser, "Host");
    const char* app = mk_parser_get_url_param(parser, "app");
    const char* stream = mk_parser_get_url_param(parser, "stream");
    const char* params = mk_parser_get_url_params(parser);
    const char* type = mk_parser_get_url_param(parser, "type");
    const char* offer = mk_parser_get_content(parser, nullptr);

    if (!host || !app || !stream || !offer) {
        mk_http_response_invoker_do_string(
            invoker,
            400,
            response_header,
            "{\"msg\":\"missing Host/app/stream or WebRTC offer body\",\"code\":-1}");
        return;
    }

    char rtc_url[1024];
    snprintf(rtc_url,
             sizeof(rtc_url),
             "rtc://%s/%s/%s?%s",
             host,
             app,
             stream,
             params ? params : "");

    mk_webrtc_get_answer_sdp(
        mk_http_response_invoker_clone(invoker),
        onWebrtcAnswer,
        (type && type[0]) ? type : "play",
        offer,
        rtc_url);
}

} // namespace

ZlmStreamer::~ZlmStreamer()
{
    release();
}

bool ZlmStreamer::resolveSdkPaths(const string& sdk_dir, ZlmResolvedPaths& paths, string& error)
{
    vector<string> include_candidates = {
        joinPath(sdk_dir, "api/include"),
        joinPath(sdk_dir, "include"),
        joinPath(sdk_dir, "release/linux/Debug/include"),
        joinPath(sdk_dir, "release/linux/Release/include"),
    };

    vector<string> lib_candidates = {
        joinPath(sdk_dir, "release/linux/Debug/libmk_api.so"),
        joinPath(sdk_dir, "release/linux/Release/libmk_api.so"),
        joinPath(sdk_dir, "lib/libmk_api.so"),
        joinPath(sdk_dir, "libmk_api.so"),
    };

    paths = ZlmResolvedPaths();
    for (const string& dir : include_candidates) {
        if (readableDir(dir) && readableFile(joinPath(dir, "mk_mediakit.h"))) {
            paths.include_dir = dir;
            break;
        }
    }

    for (const string& lib : lib_candidates) {
        if (readableFile(lib)) {
            paths.lib_path = lib;
            break;
        }
    }

    if (paths.include_dir.empty()) {
        error = "ZLM header mk_mediakit.h not found under " + sdk_dir;
        return false;
    }
    if (paths.lib_path.empty()) {
        error = "ZLM libmk_api.so not found under " + sdk_dir;
        return false;
    }
    return true;
}

bool ZlmStreamer::init(const ZlmConfig& cfg)
{
    release();
    cfg_ = cfg;

    string error;
    if (!resolveSdkPaths(cfg_.sdk_dir, paths_, error)) {
        cerr << error << endl;
        return false;
    }

    if (!readableFile(cfg_.config_ini)) {
        cerr << "ZLM config.ini not found: " << cfg_.config_ini << endl;
        return false;
    }

    cout << "Stream backend: ZLMediaKit" << endl;
    cout << "ZLMediaKit SDK dir: " << cfg_.sdk_dir << endl;
    cout << "ZLMediaKit config.ini path: " << cfg_.config_ini << endl;
    cout << "libmk_api.so path: " << paths_.lib_path << endl;
    cout << "ZLM vhost/app/stream: " << cfg_.vhost << "/" << cfg_.app << "/" << cfg_.stream << endl;
    cout << "ZLM HTTP port: " << cfg_.http_port << endl;
    cout << "ZLM RTSP port: " << cfg_.rtsp_port << endl;
    cout << "ZLM RTMP port: " << cfg_.rtmp_port << endl;
    cout << "ZLM RTC port: " << cfg_.rtc_port << endl;

    mk_config mk_cfg {};
    mk_cfg.thread_num = 0;
    mk_cfg.log_level = 0;
    mk_cfg.log_mask = LOG_CONSOLE;
    mk_cfg.log_file_path = nullptr;
    mk_cfg.log_file_days = 0;
    mk_cfg.ini_is_path = 1;
    mk_cfg.ini = cfg_.config_ini.c_str();
    mk_cfg.ssl_is_path = 1;
    mk_cfg.ssl = nullptr;
    mk_cfg.ssl_pwd = nullptr;

    // This SDK's mk_env_init returns void. Startup failures are detected from
    // server/media creation calls below, which return explicit failure values.
    mk_env_init(&mk_cfg);

    mk_events events {};
    events.on_mk_http_request = onZlmHttpRequest;
    mk_events_listen(&events);

    if (!mk_http_server_start(static_cast<uint16_t>(cfg_.http_port), 0)) {
        cerr << "mk_http_server_start failed on port " << cfg_.http_port << endl;
        release();
        return false;
    }
    servers_started_ = true;
    if (!mk_rtsp_server_start(static_cast<uint16_t>(cfg_.rtsp_port), 0)) {
        cerr << "mk_rtsp_server_start failed on port " << cfg_.rtsp_port << endl;
        release();
        return false;
    }
    if (!mk_rtmp_server_start(static_cast<uint16_t>(cfg_.rtmp_port), 0)) {
        cerr << "mk_rtmp_server_start failed on port " << cfg_.rtmp_port << endl;
        release();
        return false;
    }
    if (!mk_rtc_server_start(static_cast<uint16_t>(cfg_.rtc_port))) {
        cerr << "mk_rtc_server_start failed on port " << cfg_.rtc_port
             << " (WebRTC disabled, RTSP/HTTP-FLV still available)" << endl;
        // Non-fatal: WebRTC is optional, RTSP and HTTP-FLV still work.
    }

    mk_media media = mk_media_create(
        cfg_.vhost.c_str(),
        cfg_.app.c_str(),
        cfg_.stream.c_str(),
        0.0f,
        0,
        0);
    if (!media) {
        cerr << "mk_media_create failed for "
             << cfg_.vhost << "/" << cfg_.app << "/" << cfg_.stream << endl;
        release();
        return false;
    }

    if (!mk_media_init_video(media, MKCodecH264, cfg_.width, cfg_.height, static_cast<float>(cfg_.fps), cfg_.bitrate)) {
        cerr << "mk_media_init_video failed: codec=H264"
             << ", width=" << cfg_.width
             << ", height=" << cfg_.height
             << ", fps=" << cfg_.fps
             << ", bitrate=" << cfg_.bitrate << endl;
        mk_media_release(media);
        release();
        return false;
    }

    mk_media_init_complete(media);
    media_ = media;
    ready_ = true;

    cout << "WebRTC playback URL: " << webrtcUrl() << endl;
    cout << "HTTP-FLV playback URL: " << httpFlvUrl() << endl;
    cout << "RTSP playback URL: " << rtspUrl() << endl;
    return true;
}

bool ZlmStreamer::inputH264(const uint8_t* data, int len, uint64_t pts_ms, uint64_t dts_ms)
{
    if (!ready_ || !media_) {
        cerr << "mk_media_input_h264 failed: ZLM media is not ready" << endl;
        return false;
    }
    if (!data || len <= 0) {
        cerr << "mk_media_input_h264 failed: empty H.264 packet" << endl;
        return false;
    }

    int ok = mk_media_input_h264(
        static_cast<mk_media>(media_),
        data,
        len,
        dts_ms,
        pts_ms);
    if (!ok) {
        cerr << "mk_media_input_h264 failed: len=" << len
             << ", pts_ms=" << pts_ms
             << ", dts_ms=" << dts_ms << endl;
        return false;
    }
    return true;
}

void ZlmStreamer::release()
{
    if (media_) {
        mk_media_release(static_cast<mk_media>(media_));
        media_ = nullptr;
    }
    if (ready_) {
        ready_ = false;
    }
    if (servers_started_) {
        mk_stop_all_server();
        servers_started_ = false;
    }
}

string ZlmStreamer::webrtcUrl() const
{
    ostringstream oss;
    oss << "http://<RK3568-IP>:" << cfg_.http_port
        << "/index/api/webrtc?app=" << cfg_.app
        << "&stream=" << cfg_.stream
        << "&type=play";
    return oss.str();
}

string ZlmStreamer::httpFlvUrl() const
{
    ostringstream oss;
    oss << "http://<RK3568-IP>:" << cfg_.http_port
        << "/" << cfg_.app << "/" << cfg_.stream << ".live.flv";
    return oss.str();
}

string ZlmStreamer::rtspUrl() const
{
    ostringstream oss;
    oss << "rtsp://<RK3568-IP>:" << cfg_.rtsp_port
        << "/" << cfg_.app << "/" << cfg_.stream;
    return oss.str();
}
