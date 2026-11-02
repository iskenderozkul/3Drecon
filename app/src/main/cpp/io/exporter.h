#pragma once

#include <string>

#include "mesh/mesher.h"

namespace recon {

// Writes the fused mesh to disk. "OBJ" produces a Wavefront file with inline
// vertex colours ("v x y z r g b"), "PLY" a Stanford ASCII polygon file with
// per-vertex colour properties. Returns true only when the whole file was
// written. The mesh is the live TSDF surface, so no data is synthesized.
bool exportMesh(const MeshData& mesh, const std::string& path, const std::string& format);

}  // namespace recon
