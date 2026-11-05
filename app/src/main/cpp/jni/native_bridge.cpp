#include <GLES3/gl3.h>
#include <android/log.h>
#include <jni.h>

#include <cstring>
#include <memory>

#include "core/engine.h"
#include "gl/renderer.h"

#define LOG_TAG "ReconNative"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

namespace {

using recon::Engine;
using recon::GLRenderer;

struct NativeContext {
    std::unique_ptr<Engine> engine;
    GLRenderer renderer;
};

inline NativeContext* ctx(jlong handle) {
    return reinterpret_cast<NativeContext*>(handle);
}

jlong nativeCreate(JNIEnv*, jobject) {
    auto* context = new NativeContext();
    context->engine = std::make_unique<Engine>();
    return reinterpret_cast<jlong>(context);
}

void nativeDestroy(JNIEnv*, jobject, jlong handle) {
    auto* c = ctx(handle);
    if (!c) return;
    c->renderer.abandon();
    delete c;
}

jboolean nativeGlInit(JNIEnv*, jobject, jlong handle) {
    auto* c = ctx(handle);
    if (!c) return JNI_FALSE;
    return c->renderer.init() ? JNI_TRUE : JNI_FALSE;
}

void nativeGlShutdown(JNIEnv*, jobject, jlong handle) {
    auto* c = ctx(handle);
    if (c && c->renderer.initialized()) c->renderer.shutdown();
}

void copyPlane(const uint8_t* src, int rows, int cols, int rowStride, int pixelStride,
               std::vector<uint8_t>& dst) {
    dst.resize(static_cast<size_t>(rows) * cols);
    if (pixelStride == 1 && rowStride == cols) {
        std::memcpy(dst.data(), src, static_cast<size_t>(rows) * cols);
        return;
    }
    for (int r = 0; r < rows; ++r) {
        const uint8_t* row = src + static_cast<int64_t>(r) * rowStride;
        uint8_t* out = dst.data() + static_cast<size_t>(r) * cols;
        if (pixelStride == 1) {
            std::memcpy(out, row, cols);
        } else {
            for (int c = 0; c < cols; ++c) out[c] = row[static_cast<int64_t>(c) * pixelStride];
        }
    }
}

void nativeSubmitFrame(JNIEnv* env, jobject, jlong handle,
                       jobject yBuf, jint yRowStride, jint yPixelStride,
                       jobject uBuf, jobject vBuf, jint uvRowStride, jint uvPixelStride,
                       jint width, jint height, jlong timestampNs,
                       jfloat qx, jfloat qy, jfloat qz, jfloat qw,
                       jfloat tx, jfloat ty, jfloat tz,
                       jfloat fx, jfloat fy, jfloat cx, jfloat cy,
                       jboolean tracking,
                       jobject depthBuf, jint depthRowStride,
                       jint depthWidth, jint depthHeight, jlong depthTimestampNs,
                       jboolean depthApiActive,
                       jobject confidenceBuf, jint confidenceRowStride,
                       jint confidencePixelStride, jint confidenceWidth, jint confidenceHeight,
                       jfloatArray depthInfoArr) {
    auto* c = ctx(handle);
    if (!c || width <= 1 || height <= 1) return;

    const auto* yPtr = static_cast<const uint8_t*>(env->GetDirectBufferAddress(yBuf));
    const auto* uPtr = static_cast<const uint8_t*>(env->GetDirectBufferAddress(uBuf));
    const auto* vPtr = static_cast<const uint8_t*>(env->GetDirectBufferAddress(vBuf));
    if (!yPtr || !uPtr || !vPtr) return;

    // Walking a plane with a bogus stride reads past the buffer; reject the
    // frame instead of trusting the caller.
    const int yNeed = (width - 1) * yPixelStride + 1;
    const int uvNeed = (width / 2 - 1) * uvPixelStride + 1;
    if (yPixelStride <= 0 || uvPixelStride <= 0 || yRowStride < yNeed ||
        uvRowStride < uvNeed) {
        return;
    }

    recon::FrameInput frame;
    frame.width = width;
    frame.height = height;
    frame.timestampNs = timestampNs;
    copyPlane(yPtr, height, width, yRowStride, yPixelStride, frame.y);
    copyPlane(uPtr, height / 2, width / 2, uvRowStride, uvPixelStride, frame.u);
    copyPlane(vPtr, height / 2, width / 2, uvRowStride, uvPixelStride, frame.v);

    // ARCore depth image: DEPTH16, one 16-bit millimetre value per pixel.
    if (depthBuf && depthWidth > 1 && depthHeight > 1) {
        const auto* dPtr =
            static_cast<const uint8_t*>(env->GetDirectBufferAddress(depthBuf));
        const jlong capacity = env->GetDirectBufferCapacity(depthBuf);
        const jlong rowBytes = static_cast<jlong>(depthWidth) * 2;
        if (dPtr && depthRowStride >= rowBytes && capacity > 0) {
            frame.depthWidth = depthWidth;
            frame.depthHeight = depthHeight;
            frame.depthTimestampNs = depthTimestampNs;
            frame.depthMm.resize(static_cast<size_t>(depthWidth) * depthHeight);
            for (int r = 0; r < depthHeight; ++r) {
                const jlong offset = static_cast<jlong>(r) * depthRowStride;
                if (offset + rowBytes > capacity) break;
                std::memcpy(frame.depthMm.data() + static_cast<size_t>(r) * depthWidth,
                            dPtr + offset, static_cast<size_t>(rowBytes));
            }
        }
    }

    // Confidence grid: one byte per depth pixel. Only copied when it matches
    // the depth size, so a mismatched pair can never mask the wrong pixels.
    if (confidenceBuf && confidenceWidth == depthWidth && confidenceHeight == depthHeight &&
        confidenceWidth > 1 && confidenceHeight > 1 && confidencePixelStride > 0) {
        const int need = (confidenceWidth - 1) * confidencePixelStride + 1;
        if (confidenceRowStride >= need) {
            const auto* cPtr =
                static_cast<const uint8_t*>(env->GetDirectBufferAddress(confidenceBuf));
            const jlong capacity = env->GetDirectBufferCapacity(confidenceBuf);
            if (cPtr && capacity > 0) {
                frame.confidence.assign(
                    static_cast<size_t>(confidenceWidth) * confidenceHeight, 0);
                bool complete = true;
                for (int r = 0; r < confidenceHeight && complete; ++r) {
                    const jlong offset = static_cast<jlong>(r) * confidenceRowStride;
                    if (offset + need > capacity) {
                        complete = false;
                        break;
                    }
                    uint8_t* out = frame.confidence.data() +
                                   static_cast<size_t>(r) * confidenceWidth;
                    const uint8_t* row = cPtr + offset;
                    if (confidencePixelStride == 1) {
                        std::memcpy(out, row, static_cast<size_t>(confidenceWidth));
                    } else {
                        for (int c = 0; c < confidenceWidth; ++c) {
                            out[c] = row[static_cast<int64_t>(c) * confidencePixelStride];
                        }
                    }
                }
                if (complete) {
                    frame.confidenceWidth = confidenceWidth;
                    frame.confidenceHeight = confidenceHeight;
                } else {
                    frame.confidence.clear();
                }
            }
        }
    }

    frame.qx = qx;
    frame.qy = qy;
    frame.qz = qz;
    frame.qw = qw;
    frame.tx = tx;
    frame.ty = ty;
    frame.tz = tz;
    frame.K.fx = fx;
    frame.K.fy = fy;
    frame.K.cx = cx;
    frame.K.cy = cy;
    frame.K.width = width;
    frame.K.height = height;
    frame.tracking = tracking == JNI_TRUE;
    frame.depthApiActive = depthApiActive == JNI_TRUE;
    if (depthInfoArr && env->GetArrayLength(depthInfoArr) >= 10) {
        jfloat* info = env->GetFloatArrayElements(depthInfoArr, nullptr);
        if (info) {
            std::memcpy(frame.depthTransform, info, sizeof(frame.depthTransform));
            // Depth-grid intrinsics come from Camera.getTextureIntrinsics scaled
            // to the depth image; the depth map is aligned to the camera texture,
            // not to the CPU image.
            if (info[6] > 0.f && info[7] > 0.f) {
                frame.Kdepth.fx = info[6];
                frame.Kdepth.fy = info[7];
                frame.Kdepth.cx = info[8];
                frame.Kdepth.cy = info[9];
                frame.Kdepth.width = depthWidth;
                frame.Kdepth.height = depthHeight;
            }
            env->ReleaseFloatArrayElements(depthInfoArr, info, JNI_ABORT);
        }
    }

    c->engine->submitFrame(std::move(frame));
}

jboolean nativeRender(JNIEnv* env, jobject, jlong handle, jint cameraTexture,
                      jfloatArray viewArr, jfloatArray projArr, jfloatArray bgUvArr,
                      jint width, jint height, jint mode, jfloatArray anchorArr) {
    auto* c = ctx(handle);
    if (!c || !viewArr || !projArr || !bgUvArr) return JNI_FALSE;

    jfloat* view = env->GetFloatArrayElements(viewArr, nullptr);
    jfloat* proj = env->GetFloatArrayElements(projArr, nullptr);
    jfloat* bgUv = env->GetFloatArrayElements(bgUvArr, nullptr);
    int anchorCount = 0;
    jfloat* anchors = nullptr;
    if (anchorArr) {
        anchorCount = env->GetArrayLength(anchorArr) / 3;
        if (anchorCount > 0) anchors = env->GetFloatArrayElements(anchorArr, nullptr);
    }

    if (view && proj && bgUv) {
        c->renderer.render(view, proj, cameraTexture, bgUv, width, height, mode,
                           anchors, anchorCount, c->engine->mesh(),
                           c->engine->dynamicMesh());
    }

    if (anchors) env->ReleaseFloatArrayElements(anchorArr, anchors, JNI_ABORT);
    if (bgUv) env->ReleaseFloatArrayElements(bgUvArr, bgUv, JNI_ABORT);
    if (proj) env->ReleaseFloatArrayElements(projArr, proj, JNI_ABORT);
    if (view) env->ReleaseFloatArrayElements(viewArr, view, JNI_ABORT);
    return JNI_TRUE;
}

void nativeGetStats(JNIEnv* env, jobject, jlong handle, jfloatArray floatsArr,
                    jintArray intsArr) {
    auto* c = ctx(handle);
    if (!c) return;
    Engine& engine = *c->engine;

    if (floatsArr && env->GetArrayLength(floatsArr) >= 2) {
        const jfloat values[2] = {engine.processMs(), engine.fps()};
        env->SetFloatArrayRegion(floatsArr, 0, 2, values);
    }
    if (intsArr && env->GetArrayLength(intsArr) >= 7) {
        const jint values[7] = {engine.keyframeCount(), engine.triangleCount(),
                                engine.vertexCount(), engine.blockCount(),
                                engine.framesProcessed(), engine.depthSource(),
                                engine.phase()};
        env->SetIntArrayRegion(intsArr, 0, 7, values);
    }
}

void nativeSetScanning(JNIEnv*, jobject, jlong handle, jboolean scanning) {
    if (auto* c = ctx(handle)) c->engine->setScanning(scanning == JNI_TRUE);
}

jstring nativeDrainDiagnostics(JNIEnv* env, jobject, jlong handle) {
    auto* c = ctx(handle);
    if (!c) return env->NewStringUTF("");
    const std::string text = c->engine->drainDiagnostics();
    return env->NewStringUTF(text.c_str());
}

void nativeReset(JNIEnv*, jobject, jlong handle) {
    if (auto* c = ctx(handle)) c->engine->reset();
}

jboolean nativeExportMesh(JNIEnv* env, jobject, jlong handle, jstring path, jstring format) {
    auto* c = ctx(handle);
    if (!c || !path || !format) return JNI_FALSE;
    const char* pathChars = env->GetStringUTFChars(path, nullptr);
    const char* formatChars = env->GetStringUTFChars(format, nullptr);
    bool ok = false;
    if (pathChars && formatChars) {
        ok = c->engine->exportMesh(pathChars, formatChars);
    }
    if (formatChars) env->ReleaseStringUTFChars(format, formatChars);
    if (pathChars) env->ReleaseStringUTFChars(path, pathChars);
    return ok ? JNI_TRUE : JNI_FALSE;
}

void nativeDepthPreview(JNIEnv* env, jobject, jlong handle, jintArray depthArr,
                        jintArray confArr) {
    auto* c = ctx(handle);
    if (!c) return;
    constexpr int kCount = Engine::kPreviewW * Engine::kPreviewH;

    std::unique_ptr<uint32_t[]> depthPixels;
    std::unique_ptr<uint32_t[]> confPixels;
    if (depthArr && env->GetArrayLength(depthArr) >= kCount) {
        depthPixels = std::make_unique<uint32_t[]>(kCount);
    }
    if (confArr && env->GetArrayLength(confArr) >= kCount) {
        confPixels = std::make_unique<uint32_t[]>(kCount);
    }
    c->engine->preview(depthPixels.get(), confPixels.get());
    if (depthPixels) {
        env->SetIntArrayRegion(depthArr, 0, kCount,
                               reinterpret_cast<const jint*>(depthPixels.get()));
    }
    if (confPixels) {
        env->SetIntArrayRegion(confArr, 0, kCount,
                               reinterpret_cast<const jint*>(confPixels.get()));
    }
}

jboolean nativeHasMesh(JNIEnv*, jobject, jlong handle) {
    auto* c = ctx(handle);
    if (!c) return JNI_FALSE;
    const auto mesh = c->engine->mesh();
    return (mesh && !mesh->indices.empty()) ? JNI_TRUE : JNI_FALSE;
}

JNINativeMethod kMethods[] = {
    {"create", "()J", reinterpret_cast<void*>(nativeCreate)},
    {"destroy", "(J)V", reinterpret_cast<void*>(nativeDestroy)},
    {"glInit", "(J)Z", reinterpret_cast<void*>(nativeGlInit)},
    {"glShutdown", "(J)V", reinterpret_cast<void*>(nativeGlShutdown)},
    {"submitFrame", "(JLjava/nio/ByteBuffer;IILjava/nio/ByteBuffer;Ljava/nio/ByteBuffer;IIIIJFFFFFFFFFFFZLjava/nio/ByteBuffer;IIIJZLjava/nio/ByteBuffer;IIII[F)V",
     reinterpret_cast<void*>(nativeSubmitFrame)},
    {"render", "(JI[F[F[FIII[F)Z", reinterpret_cast<void*>(nativeRender)},
    {"getStats", "(J[F[I)V", reinterpret_cast<void*>(nativeGetStats)},
    {"setScanning", "(JZ)V", reinterpret_cast<void*>(nativeSetScanning)},
    {"getDiagnostics", "(J)Ljava/lang/String;",
     reinterpret_cast<void*>(nativeDrainDiagnostics)},
    {"reset", "(J)V", reinterpret_cast<void*>(nativeReset)},
    {"exportMesh", "(JLjava/lang/String;Ljava/lang/String;)Z",
     reinterpret_cast<void*>(nativeExportMesh)},
    {"depthPreview", "(J[I[I)V", reinterpret_cast<void*>(nativeDepthPreview)},
    {"hasMesh", "(J)Z", reinterpret_cast<void*>(nativeHasMesh)},
};

}  // namespace

extern "C" JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM* vm, void*) {
    JNIEnv* env = nullptr;
    if (vm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6) != JNI_OK) {
        return JNI_ERR;
    }
    jclass clazz = env->FindClass("com/example/recon/NativeRecon");
    if (!clazz) {
        LOGE("NativeRecon class not found");
        return JNI_ERR;
    }
    const jint count = sizeof(kMethods) / sizeof(kMethods[0]);
    if (env->RegisterNatives(clazz, kMethods, count) != JNI_OK) {
        LOGE("RegisterNatives failed");
        return JNI_ERR;
    }
    LOGI("registered %d natives", count);
    return JNI_VERSION_1_6;
}
