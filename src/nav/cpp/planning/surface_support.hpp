#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <string_view>

namespace nav_kernel::support {

struct SurfaceQuery {
  double resolution, height, tolerance, maxSlope;
  int minZ, maxZ;
};

// The adapters provide measured occupied and ray-observed free cells. Both
// planners use the same first-surface, clearance, gap and slope rules.
template<class Occupied, class Free>
bool surfacePatch(const SurfaceQuery& q, double x, double y, double bodyZ,
                  Occupied occupied, Free free, double& height, const char** failure = nullptr) {
  const double r=q.resolution;
  const auto fail = [&](const char* reason) { if (failure) *failure=reason; return false; };
  const auto column = [&](double cx, double cy, double& h, const char** reason) {
    const int ix=int(std::floor(cx/r)), iy=int(std::floor(cy/r));
    const int top=std::min(int(std::floor(bodyZ/r)),q.maxZ);
    const int bottom=std::max(q.minZ,int(std::floor((bodyZ-q.height-q.tolerance-.5*r)/r)));
    for (int z=top; z>=bottom; --z) {
      if (!occupied(ix,iy,z)) continue;
      h=(z+.5)*r;
      const double depth=bodyZ-h;
      if (depth+.5*r <= q.height-q.tolerance+1e-7 || depth-.5*r > q.height+q.tolerance+1e-7) {
        if (reason) *reason="robot_ground_support_height_mismatch";
        return false;
      }
      if (!free(ix,iy,z+1)) {
        if (reason) *reason="robot_ground_support_clearance_unobserved";
        return false;
      }
      return true;
    }
    if (reason) *reason="robot_ground_support_unobserved";
    return false;
  };
  const char* reason="robot_ground_support_unconfirmed";
  if (!column(x,y,height,&reason)) {
    if (std::string_view(reason)!="robot_ground_support_unobserved") return fail(reason);
    // Interpolate only a small surface patch surrounded by measured returns.
    // Sampling gaps need not line up with four axis-aligned columns. Keep the
    // fit local (10 cm per axis), and never extrapolate across an edge.
    const int ix=int(std::floor(x/r)), iy=int(std::floor(y/r));
    const int radius=std::min(2,int(std::floor(.10/r+1e-9)));
    struct Sample { double x,y,h; };
    std::array<Sample,24> samples{};
    int count=0; unsigned quadrants=0;
    double sx=0,sy=0,sh=0;
    for (int dy=-radius;dy<=radius;++dy) for (int dx=-radius;dx<=radius;++dx) {
      if (dx==0 && dy==0) continue;
      const double cx=(ix+dx+.5)*r,cy=(iy+dy+.5)*r;
      const double px=cx-x,py=cy-y;
      if (std::abs(px)>.10+1e-9 || std::abs(py)>.10+1e-9) continue;
      double h=0;
      if (!column(cx,cy,h,nullptr)) continue;
      samples[count++]={px,py,h};sx+=px;sy+=py;sh+=h;
      if (std::abs(px)>1e-9 && std::abs(py)>1e-9)
        quadrants |= 1U << ((px>0?1:0)+(py>0?2:0));
    }
    if (count<4 || quadrants!=15U) return fail(reason);
    const double mx=sx/count,my=sy/count,mh=sh/count;
    double xx=0,yy=0,xy=0,xh=0,yh=0;
    for (int i=0;i<count;++i) {
      const auto& p=samples[i];const double dx=p.x-mx,dy=p.y-my;
      xx+=dx*dx;yy+=dy*dy;xy+=dx*dy;xh+=dx*(p.h-mh);yh+=dy*(p.h-mh);
    }
    const double determinant=xx*yy-xy*xy;
    if (determinant<=1e-9*r*r*r*r) return fail(reason);
    const double gx=(xh*yy-yh*xy)/determinant,gy=(yh*xx-xh*xy)/determinant;
    if (q.maxSlope>0 && std::hypot(gx,gy)>q.maxSlope+1e-9) return fail("robot_ground_support_slope");
    height=mh-gx*mx-gy*my;
    const double depth=bodyZ-height;
    if (depth+.5*r<=q.height-q.tolerance+1e-7 || depth-.5*r>q.height+q.tolerance+1e-7)
      return fail("robot_ground_support_height_mismatch");
    for (int i=0;i<count;++i)
      // Voxel-center heights on the same surface can differ by one cell.
      if (std::abs(samples[i].h-height-gx*samples[i].x-gy*samples[i].y)>r+1e-9)
        return fail("robot_ground_support_surface_discontinuous");
    const int iz=int(std::floor(height/r));
    if (iz<q.minZ || iz+1>q.maxZ || !free(ix,iy,iz+1)) return fail(reason);
    // A grazing ray can traverse the cell containing the fitted surface without
    // observing a hole. A ray below that cell or a measured lower surface is
    // contradictory evidence. Preserve both; never fill them in the map.
    for (int z=iz-1;z>=q.minZ;--z)
      if (occupied(ix,iy,z) || free(ix,iy,z)) return fail("robot_ground_support_observed_drop");
    return true;
  }
  const double d=2*r;
  // Include diagonal returns: a sampled plane need not land on the four
  // axis-aligned probe columns. This estimates slope around an observed
  // center; it does not turn an unknown center into a supported surface.
  const std::array<std::array<double,2>,8> offsets{{
      {d,0},{-d,0},{0,d},{0,-d},{r,r},{r,-r},{-r,r},{-r,-r}}};
  std::array<double,8> h{};
  std::array<bool,8> found{};
  double xx=0,yy=0,xy=0,xh=0,yh=0;
  for (std::size_t i=0;i<offsets.size();++i) {
    found[i]=column(x+offsets[i][0],y+offsets[i][1],h[i],nullptr);
    if (!found[i]) continue;
    xx+=offsets[i][0]*offsets[i][0]; yy+=offsets[i][1]*offsets[i][1];
    xy+=offsets[i][0]*offsets[i][1];
    xh+=offsets[i][0]*(h[i]-height); yh+=offsets[i][1]*(h[i]-height);
  }
  const double determinant=xx*yy-xy*xy;
  if (determinant <= 1e-9*r*r*r*r) return fail("robot_ground_support_patch_incomplete");
  const double gx=(xh*yy-yh*xy)/determinant,gy=(yh*xx-xh*xy)/determinant;
  if (q.maxSlope>0 && std::hypot(gx,gy)>q.maxSlope+1e-9) return fail("robot_ground_support_slope");
  for (std::size_t i=0;i<offsets.size();++i)
    if (found[i] && std::abs(h[i]-height-gx*offsets[i][0]-gy*offsets[i][1])>r+1e-9)
      return fail("robot_ground_support_surface_discontinuous");
  return true;
}

}  // namespace nav_kernel::support
