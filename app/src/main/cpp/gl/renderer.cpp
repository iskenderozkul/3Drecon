#include "gl/renderer.h"

#include <android/log.h>

#include <cmath>
#include <cstring>

#include "math/math.h"

#ifndef GL_TEXTURE_EXTERNAL_OES
#define GL_TEXTURE_EXTERNAL_OES 0x8D65
#endif

#define LOG_TAG "ReconGL"
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

namespace recon {
namespace {

// AR mode draws the fused surface over the camera image. Half opacity keeps
// the scan legible while the real object stays visible through it; modes 1-3
// stay fully opaque because they are diagnostics, not the live overlay.
constexpr float kOverlayAlpha = 0.5f;
constexpr float kDynamicAlpha = 0.85f;

const char* kBackgroundVertex = R"(#version 300 es
layout(location = 0) in vec2 aPosition;
layout(location = 1) in vec2 aTexCoord;
out vec2 vTexCoord;
void main() {
    gl_Position = vec4(aPosition, 0.0, 1.0);
    vTexCoord = aTexCoord;
}
)";

const char* kBackgroundFragment = R"(#version 300 es
#extension GL_OES_EGL_image_external_essl3 : require
precision highp float;
uniform samplerExternalOES uTexture;
in vec2 vTexCoord;
out vec4 fragColor;
void main() {
    fragColor = texture(uTexture, vTexCoord);
}
)";

const char* kMeshVertex = R"(#version 300 es
layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec4 aColor;
uniform mat4 uMvp;
out vec3 vWorld;
out vec3 vNormal;
out vec4 vColor;
void main() {
    gl_Position = uMvp * vec4(aPosition, 1.0);
    vWorld = aPosition;
    vNormal = aNormal;
    vColor = aColor;
}
)";

const char* kMeshFragment = R"(#version 300 es
precision highp float;
in vec3 vWorld;
in vec3 vNormal;
in vec4 vColor;
uniform int uMode;
uniform vec3 uCameraPos;
uniform float uAlpha;
out vec4 fragColor;

vec3 jet(float t) {
    t = clamp(t, 0.0, 1.0);
    return clamp(vec3(1.5 - abs(4.0 * t - 3.0),
                      1.5 - abs(4.0 * t - 2.0),
                      1.5 - abs(4.0 * t - 1.0)), 0.0, 1.0);
}

void main() {
    vec3 n = normalize(vNormal);
    vec3 toEye = normalize(uCameraPos - vWorld);
    // Two-sided: a flipped face still gets lit instead of turning black, which
    // is what made the overlay look like dark torn patches.
    float diffuse = abs(dot(n, toEye));

    if (uMode == 1) {
        fragColor = vec4(0.10, 0.92, 1.0, 1.0);
        return;
    }
    if (uMode == 2) {
        fragColor = vec4(n * 0.5 + 0.5, 1.0);
        return;
    }
    if (uMode == 3) {
        float dist = length(uCameraPos - vWorld);
        float t = clamp((dist - 0.3) / 3.5, 0.0, 1.0);
        fragColor = vec4(jet(t) * (0.45 + 0.55 * diffuse), 1.0);
        return;
    }
    // The surface sits over the live camera image, so it is drawn as a
    // depth-tested translucent layer: the nearest shell blends with the real
    // object behind it instead of hiding it like an opaque blanket.
    vec3 lit = vColor.rgb * (0.32 + 0.68 * diffuse);
    fragColor = vec4(lit, uAlpha);
}
)";

const char* kAnchorVertex = R"(#version 300 es
layout(location = 0) in vec3 aPosition;
uniform mat4 uMvp;
void main() {
    gl_Position = uMvp * vec4(aPosition, 1.0);
}
)";

const char* kAnchorFragment = R"(#version 300 es
precision mediump float;
uniform vec4 uColor;
out vec4 fragColor;
void main() {
    fragColor = uColor;
}
)";

Mat4 projectionTimesView(const float* view, const float* projection) {
    Mat4 v, p;
    std::memcpy(v.m, view, sizeof(float) * 16);
    std::memcpy(p.m, projection, sizeof(float) * 16);
    return p * v;
}

Mat4 translation(float x, float y, float z) {
    Mat4 t;
    t.m[12] = x;
    t.m[13] = y;
    t.m[14] = z;
    return t;
}

}  // namespace

void GLRenderer::resetHandles() {
    initialized_ = false;
    backgroundProgram_ = meshProgram_ = anchorProgram_ = 0;
    backgroundVao_ = backgroundVbo_ = backgroundUvVbo_ = 0;
    meshVao_ = meshLineVao_ = meshVbo_ = meshEbo_ = meshLineEbo_ = 0;
    dynamicVao_ = dynamicVbo_ = dynamicEbo_ = 0;
    anchorVao_ = anchorVbo_ = 0;
    uploadedMeshVersion_ = 0;
    uploadedLineVersion_ = 0;
    uploadedDynamicVersion_ = 0;
    indexCount_ = 0;
    lineIndexCount_ = 0;
    dynamicIndexCount_ = 0;
}

void GLRenderer::abandon() { resetHandles(); }

GLuint GLRenderer::compileShader(GLenum type, const char* source) {
    GLuint shader = glCreateShader(type);
    if (!shader) return 0;
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);
    GLint ok = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[1024];
        glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
        LOGE("shader compile failed: %s", log);
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

GLuint GLRenderer::linkProgram(const char* vertex, const char* fragment) {
    GLuint vs = compileShader(GL_VERTEX_SHADER, vertex);
    GLuint fs = compileShader(GL_FRAGMENT_SHADER, fragment);
    if (!vs || !fs) {
        if (vs) glDeleteShader(vs);
        if (fs) glDeleteShader(fs);
        return 0;
    }
    GLuint program = glCreateProgram();
    glAttachShader(program, vs);
    glAttachShader(program, fs);
    glLinkProgram(program);
    GLint ok = 0;
    glGetProgramiv(program, GL_LINK_STATUS, &ok);
    glDeleteShader(vs);
    glDeleteShader(fs);
    if (!ok) {
        char log[1024];
        glGetProgramInfoLog(program, sizeof(log), nullptr, log);
        LOGE("program link failed: %s", log);
        glDeleteProgram(program);
        return 0;
    }
    return program;
}

bool GLRenderer::init() {
    if (initialized_) return true;

    backgroundProgram_ = linkProgram(kBackgroundVertex, kBackgroundFragment);
    meshProgram_ = linkProgram(kMeshVertex, kMeshFragment);
    anchorProgram_ = linkProgram(kAnchorVertex, kAnchorFragment);
    if (!backgroundProgram_ || !meshProgram_ || !anchorProgram_) return false;

    bgPosLoc_ = 0;
    bgUvLoc_ = 1;
    bgTexLoc_ = glGetUniformLocation(backgroundProgram_, "uTexture");
    meshMvpLoc_ = glGetUniformLocation(meshProgram_, "uMvp");
    meshModeLoc_ = glGetUniformLocation(meshProgram_, "uMode");
    meshCamLoc_ = glGetUniformLocation(meshProgram_, "uCameraPos");
    meshAlphaLoc_ = glGetUniformLocation(meshProgram_, "uAlpha");
    anchorMvpLoc_ = glGetUniformLocation(anchorProgram_, "uMvp");
    anchorColorLoc_ = glGetUniformLocation(anchorProgram_, "uColor");

    // Full-screen quad (triangle strip).
    const float quad[8] = {-1.f, 1.f, -1.f, -1.f, 1.f, 1.f, 1.f, -1.f};
    glGenVertexArrays(1, &backgroundVao_);
    glBindVertexArray(backgroundVao_);
    glGenBuffers(1, &backgroundVbo_);
    glBindBuffer(GL_ARRAY_BUFFER, backgroundVbo_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quad), quad, GL_STATIC_DRAW);
    glEnableVertexAttribArray(bgPosLoc_);
    glVertexAttribPointer(bgPosLoc_, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
    glGenBuffers(1, &backgroundUvVbo_);
    glBindBuffer(GL_ARRAY_BUFFER, backgroundUvVbo_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 8, nullptr, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(bgUvLoc_);
    glVertexAttribPointer(bgUvLoc_, 2, GL_FLOAT, GL_FALSE, 0, nullptr);

    // Mesh buffers (persistent VBO + EBO, double VAO for lines).
    glGenBuffers(1, &meshVbo_);
    glGenBuffers(1, &meshEbo_);
    glGenBuffers(1, &meshLineEbo_);

    glGenVertexArrays(1, &meshVao_);
    glBindVertexArray(meshVao_);
    glBindBuffer(GL_ARRAY_BUFFER, meshVbo_);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(MeshVertex),
                          reinterpret_cast<void*>(offsetof(MeshVertex, px)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(MeshVertex),
                          reinterpret_cast<void*>(offsetof(MeshVertex, nx)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(MeshVertex),
                          reinterpret_cast<void*>(offsetof(MeshVertex, r)));
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, meshEbo_);

    glGenVertexArrays(1, &meshLineVao_);
    glBindVertexArray(meshLineVao_);
    glBindBuffer(GL_ARRAY_BUFFER, meshVbo_);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(MeshVertex),
                          reinterpret_cast<void*>(offsetof(MeshVertex, px)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(MeshVertex),
                          reinterpret_cast<void*>(offsetof(MeshVertex, nx)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(MeshVertex),
                          reinterpret_cast<void*>(offsetof(MeshVertex, r)));
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, meshLineEbo_);

    // Dynamic (moving-surface) buffers: same vertex layout, rebuilt per frame.
    glGenBuffers(1, &dynamicVbo_);
    glGenBuffers(1, &dynamicEbo_);
    glGenVertexArrays(1, &dynamicVao_);
    glBindVertexArray(dynamicVao_);
    glBindBuffer(GL_ARRAY_BUFFER, dynamicVbo_);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(MeshVertex),
                          reinterpret_cast<void*>(offsetof(MeshVertex, px)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(MeshVertex),
                          reinterpret_cast<void*>(offsetof(MeshVertex, nx)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(MeshVertex),
                          reinterpret_cast<void*>(offsetof(MeshVertex, r)));
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, dynamicEbo_);

    // Anchor diamond marker.
    const float s = 0.055f;
    const float yTop = 0.10f, yMid = 0.02f, yBottom = -0.07f;
    const float marker[] = {
        // top -> ring
        0.f, yTop, 0.f, s, yMid, 0.f,
        0.f, yTop, 0.f, 0.f, yMid, s,
        0.f, yTop, 0.f, -s, yMid, 0.f,
        0.f, yTop, 0.f, 0.f, yMid, -s,
        // ring
        s, yMid, 0.f, 0.f, yMid, s,
        0.f, yMid, s, -s, yMid, 0.f,
        -s, yMid, 0.f, 0.f, yMid, -s,
        0.f, yMid, -s, s, yMid, 0.f,
        // ring -> bottom
        s, yMid, 0.f, 0.f, yBottom, 0.f,
        0.f, yMid, s, 0.f, yBottom, 0.f,
        -s, yMid, 0.f, 0.f, yBottom, 0.f,
        0.f, yMid, -s, 0.f, yBottom, 0.f,
        // stem
        0.f, yBottom, 0.f, 0.f, -0.16f, 0.f};
    anchorVertexCount_ = static_cast<GLuint>(sizeof(marker) / sizeof(float) / 3);
    glGenVertexArrays(1, &anchorVao_);
    glBindVertexArray(anchorVao_);
    glGenBuffers(1, &anchorVbo_);
    glBindBuffer(GL_ARRAY_BUFFER, anchorVbo_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(marker), marker, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

    glBindVertexArray(0);
    glDisable(GL_CULL_FACE);
    initialized_ = true;
    return true;
}

void GLRenderer::shutdown() {
    if (!initialized_) return;
    if (backgroundProgram_) glDeleteProgram(backgroundProgram_);
    if (meshProgram_) glDeleteProgram(meshProgram_);
    if (anchorProgram_) glDeleteProgram(anchorProgram_);
    if (backgroundVao_) glDeleteVertexArrays(1, &backgroundVao_);
    if (backgroundVbo_) glDeleteBuffers(1, &backgroundVbo_);
    if (backgroundUvVbo_) glDeleteBuffers(1, &backgroundUvVbo_);
    if (meshVao_) glDeleteVertexArrays(1, &meshVao_);
    if (meshLineVao_) glDeleteVertexArrays(1, &meshLineVao_);
    if (meshVbo_) glDeleteBuffers(1, &meshVbo_);
    if (meshEbo_) glDeleteBuffers(1, &meshEbo_);
    if (meshLineEbo_) glDeleteBuffers(1, &meshLineEbo_);
    if (dynamicVao_) glDeleteVertexArrays(1, &dynamicVao_);
    if (dynamicVbo_) glDeleteBuffers(1, &dynamicVbo_);
    if (dynamicEbo_) glDeleteBuffers(1, &dynamicEbo_);
    if (anchorVao_) glDeleteVertexArrays(1, &anchorVao_);
    if (anchorVbo_) glDeleteBuffers(1, &anchorVbo_);
    resetHandles();
}

void GLRenderer::uploadMesh(const MeshData& mesh) {
    glBindBuffer(GL_ARRAY_BUFFER, meshVbo_);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(mesh.vertices.size() * sizeof(MeshVertex)),
                 mesh.vertices.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, meshEbo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(mesh.indices.size() * sizeof(uint32_t)),
                 mesh.indices.data(), GL_DYNAMIC_DRAW);
    indexCount_ = static_cast<GLsizei>(mesh.indices.size());
    uploadedMeshVersion_ = mesh.version;
    uploadedLineVersion_ = 0;
    lineIndexCount_ = 0;
}

void GLRenderer::uploadDynamicMesh(const MeshData& mesh) {
    glBindBuffer(GL_ARRAY_BUFFER, dynamicVbo_);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(mesh.vertices.size() * sizeof(MeshVertex)),
                 mesh.vertices.empty() ? nullptr : mesh.vertices.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, dynamicEbo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(mesh.indices.size() * sizeof(uint32_t)),
                 mesh.indices.empty() ? nullptr : mesh.indices.data(), GL_DYNAMIC_DRAW);
    dynamicIndexCount_ = static_cast<GLsizei>(mesh.indices.size());
    uploadedDynamicVersion_ = mesh.version;
}

void GLRenderer::ensureLineIndices(const MeshData& mesh) {
    if (uploadedLineVersion_ == mesh.version && lineIndexCount_ > 0) return;
    std::vector<uint32_t> lines;
    lines.reserve(mesh.indices.size() * 2);
    for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        const uint32_t a = mesh.indices[i];
        const uint32_t b = mesh.indices[i + 1];
        const uint32_t c = mesh.indices[i + 2];
        lines.push_back(a);
        lines.push_back(b);
        lines.push_back(b);
        lines.push_back(c);
        lines.push_back(c);
        lines.push_back(a);
    }
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, meshLineEbo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(lines.size() * sizeof(uint32_t)),
                 lines.empty() ? nullptr : lines.data(), GL_DYNAMIC_DRAW);
    lineIndexCount_ = static_cast<GLsizei>(lines.size());
    uploadedLineVersion_ = mesh.version;
}

void GLRenderer::drawBackground(int cameraTexture, const float* uv) {
    if (!cameraTexture) return;
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    glDisable(GL_BLEND);

    glUseProgram(backgroundProgram_);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_EXTERNAL_OES, static_cast<GLuint>(cameraTexture));
    glUniform1i(bgTexLoc_, 0);

    glBindVertexArray(backgroundVao_);
    glBindBuffer(GL_ARRAY_BUFFER, backgroundUvVbo_);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(float) * 8, uv);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);
}

void GLRenderer::drawMesh(const std::shared_ptr<const MeshData>& mesh, int mode,
                          const float* mvp, const float* cameraPos) {
    if (!mesh || mesh->indices.empty() || !meshProgram_) return;
    if (uploadedMeshVersion_ != mesh->version) uploadMesh(*mesh);

    glUseProgram(meshProgram_);
    glUniformMatrix4fv(meshMvpLoc_, 1, GL_FALSE, mvp);
    glUniform1i(meshModeLoc_, mode);
    glUniform1f(meshAlphaLoc_, mode == 0 ? kOverlayAlpha : 1.0f);
    glUniform3f(meshCamLoc_, cameraPos[0], cameraPos[1], cameraPos[2]);

    if (mode == 1) {
        ensureLineIndices(*mesh);
        if (lineIndexCount_ == 0) return;
        glBindVertexArray(meshLineVao_);
        glDrawElements(GL_LINES, lineIndexCount_, GL_UNSIGNED_INT, nullptr);
    } else {
        glBindVertexArray(meshVao_);
        glDrawElements(GL_TRIANGLES, indexCount_, GL_UNSIGNED_INT, nullptr);
    }
    glBindVertexArray(0);
}

void GLRenderer::drawDynamicMesh(const std::shared_ptr<const MeshData>& mesh,
                                 const float* mvp, const float* cameraPos) {
    if (!mesh || mesh->indices.empty() || !meshProgram_) return;
    if (uploadedDynamicVersion_ != mesh->version) uploadDynamicMesh(*mesh);
    glUseProgram(meshProgram_);
    glUniformMatrix4fv(meshMvpLoc_, 1, GL_FALSE, mvp);
    glUniform1i(meshModeLoc_, 0);
    glUniform1f(meshAlphaLoc_, kDynamicAlpha);
    glUniform3f(meshCamLoc_, cameraPos[0], cameraPos[1], cameraPos[2]);
    glBindVertexArray(dynamicVao_);
    glDrawElements(GL_TRIANGLES, dynamicIndexCount_, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
}

void GLRenderer::drawAnchors(const float* mvp, const float* positions, int count) {
    if (!anchorProgram_ || count <= 0) return;
    Mat4 base;
    std::memcpy(base.m, mvp, sizeof(float) * 16);
    glUseProgram(anchorProgram_);
    glUniform4f(anchorColorLoc_, 1.0f, 0.72f, 0.05f, 1.0f);
    glBindVertexArray(anchorVao_);
    for (int i = 0; i < count; ++i) {
        const Mat4 model = base * translation(positions[i * 3], positions[i * 3 + 1],
                                              positions[i * 3 + 2]);
        glUniformMatrix4fv(anchorMvpLoc_, 1, GL_FALSE, model.m);
        glDrawArrays(GL_LINES, 0, anchorVertexCount_);
    }
    glBindVertexArray(0);
}

void GLRenderer::render(const float* view, const float* projection, int cameraTexture,
                        const float* backgroundUv, int width, int height, int mode,
                        const float* anchorPositions, int anchorCount,
                        const std::shared_ptr<const MeshData>& mesh,
                        const std::shared_ptr<const MeshData>& dynamicMesh) {
    if (!initialized_ && !init()) return;

    glViewport(0, 0, width, height);
    glClearColor(0.02f, 0.03f, 0.05f, 1.f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    drawBackground(cameraTexture, backgroundUv);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glDepthMask(GL_TRUE);
    // Depth testing keeps a single shell (the nearest surface) on screen; the
    // AR overlay then blends that shell with the camera image instead of
    // covering it. Diagnostic modes draw opaque.
    if (mode == 0) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    } else {
        glDisable(GL_BLEND);
    }

    const Mat4 vp = projectionTimesView(view, projection);
    float cameraPos[3];
    cameraPos[0] = -(view[0] * view[12] + view[1] * view[13] + view[2] * view[14]);
    cameraPos[1] = -(view[4] * view[12] + view[5] * view[13] + view[6] * view[14]);
    cameraPos[2] = -(view[8] * view[12] + view[9] * view[13] + view[10] * view[14]);

    drawMesh(mesh, mode, vp.m, cameraPos);
    drawDynamicMesh(dynamicMesh, vp.m, cameraPos);
    drawAnchors(vp.m, anchorPositions, anchorCount);

    glBindVertexArray(0);
    glUseProgram(0);
}

}  // namespace recon
