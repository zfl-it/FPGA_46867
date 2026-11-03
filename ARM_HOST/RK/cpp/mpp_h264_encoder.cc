#include "mpp_h264_encoder.h"

#include <algorithm>
#include <cctype>
#include <cstring>
#include <iostream>
#include <utility>

#include "rockchip/mpp_buffer.h"
#include "rockchip/mpp_frame.h"
#include "rockchip/mpp_meta.h"
#include "rockchip/mpp_packet.h"
#include "rockchip/rk_mpi.h"
#include "rockchip/rk_venc_cfg.h"
#include "rockchip/rk_venc_cmd.h"
#include "rockchip/rk_venc_rc.h"

using namespace std;

namespace {

int alignTo(int value, int alignment)
{
    return (value + alignment - 1) & ~(alignment - 1);
}

bool hasAnnexBStartCode(const uint8_t* data, size_t size)
{
    if (!data || size < 4) return false;
    for (size_t i = 0; i + 4 <= size && i < 64; ++i) {
        if (i + 3 <= size && data[i] == 0 && data[i + 1] == 0 && data[i + 2] == 1) return true;
        if (data[i] == 0 && data[i + 1] == 0 && data[i + 2] == 0 && data[i + 3] == 1) return true;
    }
    return false;
}

bool packetHasIdr(const uint8_t* data, size_t size)
{
    if (!data || size < 5) return false;
    size_t i = 0;
    while (i + 4 < size) {
        size_t start = size;
        size_t prefix = 0;
        for (; i + 3 < size; ++i) {
            if (data[i] == 0 && data[i + 1] == 0 && data[i + 2] == 1) {
                start = i;
                prefix = 3;
                break;
            }
            if (i + 4 < size && data[i] == 0 && data[i + 1] == 0 && data[i + 2] == 0 && data[i + 3] == 1) {
                start = i;
                prefix = 4;
                break;
            }
        }
        if (start == size) break;
        size_t nal = start + prefix;
        if (nal < size && (data[nal] & 0x1f) == 5) return true;
        i = nal + 1;
    }
    return false;
}

const char* bufferTypeName(MppBufferType type)
{
    switch (type) {
    case MPP_BUFFER_TYPE_NORMAL: return "MPP_BUFFER_TYPE_NORMAL";
    case MPP_BUFFER_TYPE_ION: return "MPP_BUFFER_TYPE_ION";
    case MPP_BUFFER_TYPE_EXT_DMA: return "MPP_BUFFER_TYPE_EXT_DMA";
    case MPP_BUFFER_TYPE_DRM: return "MPP_BUFFER_TYPE_DRM";
#ifdef MPP_BUFFER_TYPE_DMA_HEAP
    case MPP_BUFFER_TYPE_DMA_HEAP: return "MPP_BUFFER_TYPE_DMA_HEAP";
#endif
    default: return "MPP_BUFFER_TYPE_UNKNOWN";
    }
}

bool mppBufferSyncAvailable()
{
#if defined(mpp_buffer_sync_begin) && defined(mpp_buffer_sync_end)
    return true;
#else
    return false;
#endif
}

bool mppBufferSyncBegin(MppBuffer buffer)
{
#if defined(mpp_buffer_sync_begin) && defined(mpp_buffer_sync_end)
    MPP_RET ret = mpp_buffer_sync_begin(buffer);
    if (ret != MPP_OK) {
        cerr << "MPP buffer sync begin failed: ret=" << ret << endl;
        return false;
    }
#else
    (void)buffer;
#endif
    return true;
}

bool mppBufferSyncEnd(MppBuffer buffer)
{
#if defined(mpp_buffer_sync_begin) && defined(mpp_buffer_sync_end)
    MPP_RET ret = mpp_buffer_sync_end(buffer);
    if (ret != MPP_OK) {
        cerr << "MPP buffer sync end failed: ret=" << ret << endl;
        return false;
    }
#else
    (void)buffer;
#endif
    return true;
}

const MppBufferType kFrameBufferType = MPP_BUFFER_TYPE_DRM;

} // namespace

MppH264Encoder::~MppH264Encoder()
{
    release();
}

bool MppH264Encoder::init(const MppH264Config& cfg)
{
    release();
    cfg_ = cfg;

    if (cfg_.width <= 0 || cfg_.height <= 0 || (cfg_.width & 1) || (cfg_.height & 1)) {
        cerr << "MPP encoder init failed: width/height must be positive even values, got "
             << cfg_.width << "x" << cfg_.height << endl;
        return false;
    }
    if (cfg_.fps <= 0 || cfg_.bitrate <= 0 || cfg_.gop <= 0) {
        cerr << "MPP encoder init failed: invalid fps/bitrate/gop, fps=" << cfg_.fps
             << ", bitrate=" << cfg_.bitrate
             << ", gop=" << cfg_.gop << endl;
        return false;
    }

    hor_stride_ = alignTo(cfg_.width, 16);
    ver_stride_ = alignTo(cfg_.height, 16);
    input_size_ = static_cast<size_t>(hor_stride_) * static_cast<size_t>(ver_stride_) * 3 / 2;
    frame_buffer_size_ = static_cast<size_t>(alignTo(hor_stride_, 64)) * static_cast<size_t>(alignTo(ver_stride_, 64)) * 3 / 2;
    mdinfo_size_ = static_cast<size_t>(alignTo(hor_stride_, 64) >> 6) * static_cast<size_t>(alignTo(ver_stride_, 16) >> 4) * 16;

    input_log_printed_ = false;

    MPP_RET ret = mpp_buffer_group_get_internal(reinterpret_cast<MppBufferGroup*>(&buffer_group_), kFrameBufferType);
    if (ret != MPP_OK) {
        cerr << "MPP encoder init failed: mpp_buffer_group_get_internal ret=" << ret << endl;
        release();
        return false;
    }

    ret = mpp_buffer_get(static_cast<MppBufferGroup>(buffer_group_),
                         reinterpret_cast<MppBuffer*>(&frame_buffer_),
                         frame_buffer_size_);
    if (ret != MPP_OK) {
        cerr << "MPP encoder init failed: mpp_buffer_get input ret=" << ret << endl;
        release();
        return false;
    }

    ret = mpp_buffer_get(static_cast<MppBufferGroup>(buffer_group_),
                         reinterpret_cast<MppBuffer*>(&mdinfo_buffer_),
                         max<size_t>(mdinfo_size_, 4096));
    if (ret != MPP_OK) {
        cerr << "MPP encoder init failed: mpp_buffer_get mdinfo ret=" << ret << endl;
        release();
        return false;
    }

    ret = mpp_create(reinterpret_cast<MppCtx*>(&ctx_), reinterpret_cast<MppApi**>(&mpi_));
    if (ret != MPP_OK) {
        cerr << "MPP encoder init failed: mpp_create ret=" << ret << endl;
        release();
        return false;
    }

    MppPollType timeout = MPP_POLL_BLOCK;
    ret = static_cast<MppApi*>(mpi_)->control(static_cast<MppCtx>(ctx_), MPP_SET_OUTPUT_TIMEOUT, &timeout);
    if (ret != MPP_OK) {
        cerr << "MPP encoder init failed: MPP_SET_OUTPUT_TIMEOUT ret=" << ret << endl;
        release();
        return false;
    }

    ret = mpp_init(static_cast<MppCtx>(ctx_), MPP_CTX_ENC, MPP_VIDEO_CodingAVC);
    if (ret != MPP_OK) {
        cerr << "MPP encoder init failed: mpp_init H.264 encoder ret=" << ret << endl;
        release();
        return false;
    }

    if (!setupEncoderConfig()) {
        release();
        return false;
    }

    MppBuffer frame_buf = static_cast<MppBuffer>(frame_buffer_);
    void* frame_ptr = mpp_buffer_get_ptr(frame_buf);
    if (frame_ptr) {
        if (!mppBufferSyncBegin(frame_buf)) {
            release();
            return false;
        }
        memset(frame_ptr, 0, frame_buffer_size_);
        if (!mppBufferSyncEnd(frame_buf)) {
            release();
            return false;
        }
    }

    ready_ = true;
    cout << "MPP encoder initialized: H.264"
         << ", width=" << cfg_.width
         << ", height=" << cfg_.height
         << ", hor_stride=" << hor_stride_
         << ", ver_stride=" << ver_stride_
         << ", input_size=" << input_size_
         << ", frame_buffer_size=" << frame_buffer_size_
         << ", buffer_type=" << bufferTypeName(kFrameBufferType)
         << ", fps=" << cfg_.fps
         << ", bitrate=" << cfg_.bitrate
         << ", gop=" << cfg_.gop
         << ", profile=" << cfg_.profile
         << ", format=MPP_FMT_YUV420SP/NV12"
         << ", buffer_sync=" << (mppBufferSyncAvailable() ? "enabled" : "not available") << endl;
    return true;
}

bool MppH264Encoder::setupEncoderConfig()
{
    MPP_RET ret = mpp_enc_cfg_init(reinterpret_cast<MppEncCfg*>(&enc_cfg_));
    if (ret != MPP_OK) {
        cerr << "MPP encoder init failed: mpp_enc_cfg_init ret=" << ret << endl;
        return false;
    }

    MppEncCfg cfg = static_cast<MppEncCfg>(enc_cfg_);
    mpp_enc_cfg_set_s32(cfg, "prep:width", cfg_.width);
    mpp_enc_cfg_set_s32(cfg, "prep:height", cfg_.height);
    mpp_enc_cfg_set_s32(cfg, "prep:hor_stride", hor_stride_);
    mpp_enc_cfg_set_s32(cfg, "prep:ver_stride", ver_stride_);
    mpp_enc_cfg_set_s32(cfg, "prep:format", MPP_FMT_YUV420SP);

    mpp_enc_cfg_set_s32(cfg, "rc:mode", MPP_ENC_RC_MODE_CBR);
    mpp_enc_cfg_set_s32(cfg, "rc:fps_in_flex", 0);
    mpp_enc_cfg_set_s32(cfg, "rc:fps_in_num", cfg_.fps);
    mpp_enc_cfg_set_s32(cfg, "rc:fps_in_denorm", 1);
    mpp_enc_cfg_set_s32(cfg, "rc:fps_out_flex", 0);
    mpp_enc_cfg_set_s32(cfg, "rc:fps_out_num", cfg_.fps);
    mpp_enc_cfg_set_s32(cfg, "rc:fps_out_denorm", 1);
    mpp_enc_cfg_set_s32(cfg, "rc:gop", cfg_.gop);
    mpp_enc_cfg_set_s32(cfg, "rc:bps_target", cfg_.bitrate);
    mpp_enc_cfg_set_s32(cfg, "rc:bps_max", cfg_.bitrate * 17 / 16);
    mpp_enc_cfg_set_s32(cfg, "rc:bps_min", cfg_.bitrate * 15 / 16);
    mpp_enc_cfg_set_s32(cfg, "rc:qp_init", -1);
    mpp_enc_cfg_set_s32(cfg, "rc:qp_max", 51);
    mpp_enc_cfg_set_s32(cfg, "rc:qp_min", 10);
    mpp_enc_cfg_set_s32(cfg, "rc:qp_max_i", 51);
    mpp_enc_cfg_set_s32(cfg, "rc:qp_min_i", 10);
    mpp_enc_cfg_set_s32(cfg, "rc:qp_ip", 2);
    mpp_enc_cfg_set_u32(cfg, "rc:drop_mode", MPP_ENC_RC_DROP_FRM_DISABLED);
    mpp_enc_cfg_set_u32(cfg, "rc:drop_thd", 20);
    mpp_enc_cfg_set_u32(cfg, "rc:drop_gap", 1);

    mpp_enc_cfg_set_s32(cfg, "codec:type", MPP_VIDEO_CodingAVC);

    string profile = cfg_.profile;
    transform(profile.begin(), profile.end(), profile.begin(), ::tolower);
    int profile_idc = 66;
    int cabac = 0;
    if (profile == "main") {
        profile_idc = 77;
        cabac = 1;
    } else if (profile == "high") {
        profile_idc = 100;
        cabac = 1;
    } else if (profile != "baseline") {
        cerr << "Unknown H.264 profile '" << cfg_.profile << "', using baseline" << endl;
    }

    mpp_enc_cfg_set_s32(cfg, "h264:profile", profile_idc);
    mpp_enc_cfg_set_s32(cfg, "h264:level", 40);
    mpp_enc_cfg_set_s32(cfg, "h264:cabac_en", cabac);
    mpp_enc_cfg_set_s32(cfg, "h264:cabac_idc", 0);
    mpp_enc_cfg_set_s32(cfg, "h264:trans8x8", 0);

    ret = static_cast<MppApi*>(mpi_)->control(static_cast<MppCtx>(ctx_), MPP_ENC_SET_CFG, cfg);
    if (ret != MPP_OK) {
        cerr << "MPP encoder init failed: MPP_ENC_SET_CFG ret=" << ret << endl;
        return false;
    }

    MppEncHeaderMode header_mode = MPP_ENC_HEADER_MODE_EACH_IDR;
    ret = static_cast<MppApi*>(mpi_)->control(static_cast<MppCtx>(ctx_), MPP_ENC_SET_HEADER_MODE, &header_mode);
    if (ret != MPP_OK) {
        cerr << "MPP encoder init failed: MPP_ENC_SET_HEADER_MODE ret=" << ret << endl;
        return false;
    }

    MppEncSeiMode sei_mode = MPP_ENC_SEI_MODE_DISABLE;
    ret = static_cast<MppApi*>(mpi_)->control(static_cast<MppCtx>(ctx_), MPP_ENC_SET_SEI_CFG, &sei_mode);
    if (ret != MPP_OK) {
        cerr << "MPP encoder warning: MPP_ENC_SET_SEI_CFG ret=" << ret << endl;
    }

    return true;
}

bool MppH264Encoder::encodeNv12(const uint8_t* nv12,
                                size_t size,
                                uint64_t pts_ms,
                                vector<H264Packet>& packets)
{
    packets.clear();

    if (!ready_ || !ctx_ || !mpi_) {
        cerr << "MPP encodeNv12 failed: encoder is not ready" << endl;
        return false;
    }
    if (!nv12 || size < input_size_) {
        cerr << "MPP encodeNv12 failed: input NV12 size " << size
             << " is smaller than required " << input_size_ << endl;
        return false;
    }

    if (!input_log_printed_) {
        cout << "MPP encodeNv12 input: input size=" << size
             << ", required input_size=" << input_size_
             << ", frame_buffer_size=" << frame_buffer_size_
             << ", buffer_sync=" << (mppBufferSyncAvailable() ? "enabled" : "not available")
             << endl;
        if (!mppBufferSyncAvailable()) {
            cerr << "MPP encodeNv12 warning: mpp_buffer_sync_begin/end not available in current MPP headers; "
                 << "using memset+memcpy with " << bufferTypeName(kFrameBufferType) << endl;
        }
        input_log_printed_ = true;
    }

    MppBuffer frame_buf = static_cast<MppBuffer>(frame_buffer_);
    void* frame_ptr = mpp_buffer_get_ptr(frame_buf);
    if (!frame_ptr) {
        cerr << "MPP encodeNv12 failed: input MppBuffer has no CPU pointer" << endl;
        return false;
    }
    if (!mppBufferSyncBegin(frame_buf)) {
        return false;
    }
    memset(frame_ptr, 0, frame_buffer_size_);
    memcpy(frame_ptr, nv12, input_size_);
    if (!mppBufferSyncEnd(frame_buf)) {
        return false;
    }

    MppFrame frame = nullptr;
    MPP_RET ret = mpp_frame_init(&frame);
    if (ret != MPP_OK || !frame) {
        cerr << "MPP encodeNv12 failed: mpp_frame_init ret=" << ret << endl;
        return false;
    }

    mpp_frame_set_width(frame, cfg_.width);
    mpp_frame_set_height(frame, cfg_.height);
    mpp_frame_set_hor_stride(frame, hor_stride_);
    mpp_frame_set_ver_stride(frame, ver_stride_);
    mpp_frame_set_fmt(frame, MPP_FMT_YUV420SP);
    mpp_frame_set_pts(frame, static_cast<RK_S64>(pts_ms));
    mpp_frame_set_dts(frame, static_cast<RK_S64>(pts_ms));
    mpp_frame_set_eos(frame, 0);
    mpp_frame_set_buffer(frame, frame_buf);

    MppPacket packet = nullptr;
    MppMeta frame_meta = mpp_frame_get_meta(frame);
    if (mdinfo_buffer_) {
        mpp_meta_set_buffer(frame_meta, KEY_MOTION_INFO, static_cast<MppBuffer>(mdinfo_buffer_));
    }

    ret = static_cast<MppApi*>(mpi_)->encode_put_frame(static_cast<MppCtx>(ctx_), frame);
    mpp_frame_deinit(&frame);
    if (ret != MPP_OK) {
        cerr << "MPP encodeNv12 failed: encode_put_frame ret=" << ret << endl;
        return false;
    }

    bool eoi = false;
    do {
        packet = nullptr;
        ret = static_cast<MppApi*>(mpi_)->encode_get_packet(static_cast<MppCtx>(ctx_), &packet);
        if (ret != MPP_OK) {
            cerr << "MPP encodeNv12 failed: encode_get_packet ret=" << ret << endl;
            if (packet) mpp_packet_deinit(&packet);
            packet = nullptr;
            return false;
        }

        if (!packet) {
            break;
        }

        uint8_t* ptr = static_cast<uint8_t*>(mpp_packet_get_pos(packet));
        size_t len = mpp_packet_get_length(packet);
        if (ptr && len > 0) {
            if (!hasAnnexBStartCode(ptr, len)) {
                cerr << "MPP encodeNv12 failed: H.264 packet is not Annex-B" << endl;
                mpp_packet_deinit(&packet);
                return false;
            }

            H264Packet out;
            out.data.assign(ptr, ptr + len);
            out.pts_ms = pts_ms;
            out.dts_ms = pts_ms;
            out.keyframe = packetHasIdr(ptr, len);
            if (mpp_packet_has_meta(packet)) {
                MppMeta meta = mpp_packet_get_meta(packet);
                RK_S32 intra = 0;
                if (mpp_meta_get_s32(meta, KEY_OUTPUT_INTRA, &intra) == MPP_OK && intra) {
                    out.keyframe = true;
                }
            }
            packets.push_back(std::move(out));
        }

        if (mpp_packet_is_partition(packet)) {
            eoi = mpp_packet_is_eoi(packet);
        } else {
            eoi = true;
        }
        mpp_packet_deinit(&packet);
        packet = nullptr;
    } while (!eoi);

    return !packets.empty();
}

void MppH264Encoder::release()
{
    ready_ = false;

    if (ctx_) {
        mpp_destroy(static_cast<MppCtx>(ctx_));
        ctx_ = nullptr;
        mpi_ = nullptr;
    }
    if (enc_cfg_) {
        mpp_enc_cfg_deinit(static_cast<MppEncCfg>(enc_cfg_));
        enc_cfg_ = nullptr;
    }
    if (frame_buffer_) {
        mpp_buffer_put(static_cast<MppBuffer>(frame_buffer_));
        frame_buffer_ = nullptr;
    }
    if (mdinfo_buffer_) {
        mpp_buffer_put(static_cast<MppBuffer>(mdinfo_buffer_));
        mdinfo_buffer_ = nullptr;
    }
    if (buffer_group_) {
        mpp_buffer_group_put(static_cast<MppBufferGroup>(buffer_group_));
        buffer_group_ = nullptr;
    }
    input_log_printed_ = false;
}
