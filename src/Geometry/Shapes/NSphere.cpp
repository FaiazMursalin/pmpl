#include "NSphere.h"
#include <cmath>
#include "nonstd/container_ops.h"

#include "Utilities/PMPLExceptions.h"
#include "Utilities/MPUtils.h"


/*------------------------------ Construction --------------------------------*/

NSphere::
NSphere(const size_t _n, const double _r) : NShape('s'), m_center(_n, 0), 
  m_radius(_r) { }


NSphere::
NSphere(const std::vector<double>& _c, const double _r) : NShape('s'), 
  m_center(_c), m_radius(_r) { }

/*------------------------------- Accessors ----------------------------------*/

size_t
NSphere::
GetDimension() const noexcept {
  return m_center.size();
}


void
NSphere::
SetCenter(const std::vector<double>& _c) noexcept {
  const size_t maxIndex = std::min(_c.size(), GetDimension());
  for(size_t i = 0; i < maxIndex; ++i)
    m_center[i] = _c[i];
}


const std::vector<double>&
NSphere::
GetCenter() const noexcept {
  return m_center;
}


double
NSphere::
GetRadius() const noexcept {
  return m_radius;
}


void
NSphere::
SetRadius(const double _r) noexcept {
  m_radius = _r;
}


void
NSphere::
Translate(const std::vector<double>& _v) noexcept {
  const size_t maxIndex = std::min(_v.size(), GetDimension());
  for(size_t i = 0; i < maxIndex; ++i)
    m_center[i] += _v[i];
}

std::pair<Point3d, Point3d> 
NSphere::
AxisAlignedExtremePoints() {
  Point3d f(0,0,0), s(0,0,0);
  const size_t maxIndex = std::min(GetDimension(), (size_t) 3);
  for(size_t i = 0; i < maxIndex; ++i) {
    f[i] = m_center[i] - m_radius;
    s[i] = m_center[i] + m_radius;
  }
  return std::make_pair(f, s);
}

/*--------------------------------- Testing ----------------------------------*/

bool
NSphere::
Contains(const std::vector<double>& _p) const noexcept {
  return Clearance(_p) >= 0;
}

bool
NSphere::
Contains(const Vector3d& _p) const noexcept {
  return Clearance(_p) >= 0;
}


double
NSphere::
Clearance(const std::vector<double>& _p) const noexcept {
  const size_t maxIndex = std::min(_p.size(), GetDimension());
  double sum = 0;
  for(size_t i = 0; i < maxIndex; ++i)
    sum += std::pow((_p[i] - m_center[i]), 2);
  return m_radius - std::sqrt(sum);
}

double
NSphere::
Clearance(const Vector3d& _p) const noexcept {
  const size_t maxIndex = GetDimension();
  double clearance = 0;
  for(size_t i = 0; i < maxIndex; ++i)
    clearance += std::pow((_p[i] - m_center[i]), 2);
  clearance = m_radius - std::sqrt(clearance); 
  return clearance;
}

std::vector<double>
NSphere::
ClearancePoint(std::vector<double> _p) const noexcept {
  // Only consider dimensions that are in both _p and this.
  const size_t maxIndex = std::min(_p.size(), GetDimension());
  auto point = _p;
  point.resize(maxIndex, 0);

  // Transform point to the coordinate frame at the sphere's center.
  for(size_t i = 0; i < maxIndex; ++i)
    point[i] -= m_center[i];

  // Find the scaling factor that places point on the sphere's boundary.
  const double scale = m_radius / nonstd::magnitude<double>(point);

  // Scale the values in point and return to original coordinate frame.
  for(size_t i = 0; i < maxIndex; ++i) {
    point[i] *= scale;
    point[i] += m_center[i];
  }

  // Copy the modified values back to the original point.
  std::copy(point.begin(), point.begin() + maxIndex, _p.begin());

  return _p;
}

double
NSphere::
DirectedClearance(const std::vector<double>& _p, const std::vector<double>& _d) const noexcept {
  const auto& distances = LineSphereIntersection(_p, _d, m_center, m_radius);
  // Both defined and positive take the min
  if(distances.first >= 0  && distances.second >= 0)
    return (distances.first <= distances.second)? distances.first : distances.second;
  else // otherwise take max
    return (distances.first >= distances.second)? distances.first : distances.second;
}

double
NSphere::
DirectedClearance(const Vector3d& _p, const Vector3d& _d) const noexcept {
  const auto& distances = LineSphereIntersection(_p, _d, m_center, m_radius);
  if(distances.first >= 0  && distances.second >= 0)
    return (distances.first <= distances.second)? distances.first : distances.second;
  else
    return (distances.first >= distances.second)? distances.first : distances.second;
}


/*--------------------------------- Sampling ---------------------------------*/

std::vector<double>
NSphere::
Sample() const {
  // Generate a random value for each dimension in the range [-1,1] with
  // gaussian probability.
  std::vector<double> point(GetDimension());
  for(auto& value : point)
    value = GRand();

  // Scale and translate the point appropriately.
  const double scale = m_radius * DRand() / nonstd::magnitude<double>(point);
  for(size_t i = 0; i < point.size(); ++i) {
    point[i] *= scale;
    point[i] += m_center[i];
  }

  return point;
}

/*----------------------------------- I/O ------------------------------------*/

std::istream&
operator>>(std::istream& _is, NSphere& _sphere) {
  _sphere.m_center.clear();

  // Read opening bracket.
  char c;
  if(!(_is >> c && c == '['))
    throw ParseException(WHERE, "Failed reading NSphere bounds. Missing '['.");

  // Read the center point values.
  double temp;
  while(1) {
    // Eat white space.
    _is >> std::ws;

    // If the next character is not digit, we are done reading center point
    // values.
    if(!isdigit(_is.peek()))
      break;

    // Otherwise, read the next center point value.
    if(!(_is >> temp))
      throw ParseException(WHERE, "Failed reading center point value " +
          std::to_string(_sphere.m_center.size()) + ".");
    _sphere.m_center.push_back(temp);
  }

  // Read separator.
  if(!(_is >> c and c == ';'))
    throw ParseException(WHERE, "Failed reading NSphere bounds. Missing ';'.");

  // Read the radius.
  if(!(_is >> temp))
    throw ParseException(WHERE, "Failed reading NSphere radius.");
  _sphere.m_radius = temp;

  // Read the last separator.
  _is >> std::ws;
  if(!(_is >> c and c == ']'))
    throw ParseException(WHERE, "Failed reading NSphere bounds. Missing ']'.");

  return _is;
}


std::ostream&
operator<<(std::ostream& _os, const NSphere& _sphere) {
  _os << "[ ";
  for(size_t i = 0; i < _sphere.GetDimension() - 1; ++i)
    _os << _sphere.GetCenter()[i] << " ";
  return _os << "; " << _sphere.GetRadius() << " ]";
}

/*----------------------------------------------------------------------------*/
