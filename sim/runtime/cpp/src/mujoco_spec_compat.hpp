#pragma once

#include <mujoco/mujoco.h>

namespace lingtu::sim::mujoco_compat {

inline void set_attach_conflict_error(mjSpec *spec) noexcept {
  // MuJoCo 3.3 exposes the explicit conflict policy. It was removed from the
  // 3.5 public mjsCompiler struct; current versions reject incompatible
  // authored globals during compilation instead. Keep this version seam local
  // so the Composer does not spread SDK-specific conditionals.
#if mjVERSION_HEADER >= 3003000 && mjVERSION_HEADER < 3005000
  spec->compiler.conflict = mjCONFLICT_ERROR;
#else
  (void)spec;
#endif
}

}  // namespace lingtu::sim::mujoco_compat
