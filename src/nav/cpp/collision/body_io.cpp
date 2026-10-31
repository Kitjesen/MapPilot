#include "collision/body_io.hpp"

#include <cmath>
#include <sstream>
#include <stdexcept>

namespace lingtu::nav::collision {
std::shared_ptr<Body> readBody(std::istream& input) {
  std::string header;
  std::size_t count=0;
  if (!(input>>header)) throw std::runtime_error("missing body header");
  const bool typed=header=="BODY2";
  if (typed) input>>count;
  else {
    std::istringstream number(header);
    char extra;
    if (!(number>>count) || number>>extra) throw std::runtime_error("invalid body count");
  }
  if (!input || count==0) throw std::runtime_error("invalid body count");
  auto body=std::make_shared<Body>();
  for (std::size_t i=0;i<count;++i) {
    Part p;
    input>>p.link>>p.center.x>>p.center.y>>p.center.z;
    for (auto& value:p.rotation) input>>value;
    input>>p.half_size.x>>p.half_size.y>>p.half_size.z;
    if (typed) {
      std::string shape;
      bool reserved = false;
      input>>shape>>p.radius_m>>p.half_length_m>>p.support_contact>>reserved;
      if (shape=="box") p.shape=Shape::Box;
      else if (shape=="sphere") p.shape=Shape::Sphere;
      else if (shape=="cylinder") p.shape=Shape::Cylinder;
      else if (shape=="capsule") p.shape=Shape::Capsule;
      else throw std::runtime_error("unknown body shape");
    }
    if (!input || p.link.empty()) throw std::runtime_error("incomplete body part");
    for (double v:{p.center.x,p.center.y,p.center.z,p.half_size.x,p.half_size.y,
                   p.half_size.z,p.radius_m,p.half_length_m})
      if (!std::isfinite(v)) throw std::runtime_error("nonfinite body part");
    if (p.half_size.x<=0 || p.half_size.y<=0 || p.half_size.z<=0 ||
        p.radius_m<0 || p.half_length_m<0 ||
        (p.shape!=Shape::Box && p.radius_m<=0))
      throw std::runtime_error("invalid body geometry");
    for (int row=0;row<3;++row) for (int column=0;column<3;++column) {
      double value=0;
      for(int k=0;k<3;++k) value+=p.rotation[3*row+k]*p.rotation[3*column+k];
      if(!std::isfinite(value) || std::abs(value-(row==column?1.:0.))>1e-6)
        throw std::runtime_error("body rotation is not orthonormal");
    }
    body->parts.push_back(p);
  }
  std::string extra;
  if (input>>extra) throw std::runtime_error("unexpected body data");
  return body;
}
}  // namespace lingtu::nav::collision
