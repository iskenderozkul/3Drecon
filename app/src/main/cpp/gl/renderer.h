#pragma once

#include <GLES3/gl3.h>

#include <cstdint>
#include <memory>
#include <vector>

#include "mesh/mesher.h"

namespace recon {

// OpenGL ES 3.0 renderer. Draws the live ARCore camera texture, the fused TSDF
// surface (persistent VBO/EBO), and world-locked AR anchors. All methods must
// be called on the GL thread.
class GLRenderer {
public:
    GLRenderer() = default;
    ~GLRenderer() = default;

    bool init();
    // Deletes GL objects; only valid on the GL thread.
    void shutdown();
    // Drops handles without issuing GL calls (context already gone).
    void abandon();
    bool initialized() const { return initialized_; }

    void render(const float* view, const float* projection, int cameraTexture,
                const float* backgroundUv, int width, int height, int mode,
                const float* anchorPositions, int anchorCount,
                const std::shared_ptr<const MeshData>& mesh,
                const std::shared_ptr<const MeshData>& dynamicMesh);

private:
    GLuint compileShader(GLenum type, const char* source);
    GLuint linkProgram(const char* vertex, const char* fragment);
    void resetHandles();
    void uploadMesh(const MeshData& mesh);
    void uploadDynamicMesh(const MeshData& mesh);
    void ensureLineIndices(const MeshData& mesh);
    void drawBackground(int cameraTexture, const float* uv);
    void drawMesh(const std::shared_ptr<const MeshData>& mesh, int mode, const float* mvp,
                  const float* cameraPos);
    void drawDynamicMesh(const std::shared_ptr<const MeshData>& mesh, const float* mvp,
                         const float* cameraPos);
    void drawAnchors(const float* mvp, const float* positions, int count);

    GLuint backgroundProgram_ = 0;
    GLuint meshProgram_ = 0;
    GLuint anchorProgram_ = 0;

    GLuint backgroundVao_ = 0;
    GLuint backgroundVbo_ = 0;
    GLuint backgroundUvVbo_ = 0;

    GLuint meshVao_ = 0;
    GLuint meshLineVao_ = 0;
    GLuint meshVbo_ = 0;
    GLuint meshEbo_ = 0;
    GLuint meshLineEbo_ = 0;

    // Moving surfaces: a small mesh rebuilt per frame, drawn on top of the
    // static surface in amber so it reads as "this part is alive".
    GLuint dynamicVao_ = 0;
    GLuint dynamicVbo_ = 0;
    GLuint dynamicEbo_ = 0;

    GLuint anchorVao_ = 0;
    GLuint anchorVbo_ = 0;
    GLuint anchorVertexCount_ = 0;

    GLint bgPosLoc_ = -1;
    GLint bgUvLoc_ = -1;
    GLint bgTexLoc_ = -1;

    GLint meshMvpLoc_ = -1;
    GLint meshModeLoc_ = -1;
    GLint meshCamLoc_ = -1;
    GLint meshAlphaLoc_ = -1;

    GLint anchorMvpLoc_ = -1;
    GLint anchorColorLoc_ = -1;

    uint32_t uploadedMeshVersion_ = 0;
    uint32_t uploadedLineVersion_ = 0;
    uint32_t uploadedDynamicVersion_ = 0;
    GLsizei indexCount_ = 0;
    GLsizei lineIndexCount_ = 0;
    GLsizei dynamicIndexCount_ = 0;

    bool initialized_ = false;
};

}  // namespace recon
