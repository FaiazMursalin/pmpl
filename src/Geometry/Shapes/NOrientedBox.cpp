#include "NOrientedBox.h"

#include <cmath>

#include "Utilities/PMPLExceptions.h"


/*------------------------------ Construction --------------------------------*/

NOrientedBox::
NOrientedBox(const size_t _n, const std::vector<double>& _r,
  const EulerAngle& _e) : NBox(std::max(_n,_r.size())), m_center(_n, 0),
  m_orientation(_e) {
  for (size_t i = 0; i < _n && i < _r.size(); ++i) {
    NBox::SetRange(i, -_r[i], _r[i]);
    m_semiRanges.push_back(_r[i]);
  }
}


NOrientedBox::
NOrientedBox(const std::vector<double>& _center, const std::vector<double>& _r,
  const EulerAngle& _e) : NBox(std::max(_center.size(), _r.size())), m_center(_center),
	m_orientation(_e) {
  // assign the ranges as from -range to +range
  for (size_t i = 0; i < _r.size(); ++i){
    NBox::SetRange(i, -_r[i], _r[i]);
    m_semiRanges.push_back(_r[i]);
  }
}

/*------------------------------- Accessors ----------------------------------*/

size_t
NOrientedBox::
GetDimension() const noexcept {
  return NBox::GetDimension();
}


void
NOrientedBox::
SetCenter(const std::vector<double>& _c) noexcept {
  // Set center for the oriented box
  const size_t maxIndex = std::min(_c.size(), GetDimension());
  for(size_t i = 0; i < maxIndex; ++i) 
    m_center[i] = _c[i];
}


const std::vector<double>&
NOrientedBox::
GetCenter() const noexcept {
  return m_center;
}


const Range<double>&
NOrientedBox::
GetRange(const size_t _i) const noexcept {
  return NBox::GetRange(_i);
}


const std::vector<Range<double>>&
NOrientedBox::
GetRanges() const noexcept {
  return NBox::GetRanges();
}


void
NOrientedBox::
SetRange(const size_t _i, const Range<double>& _r) noexcept {
  m_center[_i] = _r.Center();
  NBox::SetRange(_i, _r.min - m_center[_i], _r.max + m_center[_i]);
  m_semiRanges[_i] = NBox::GetRange(_i).Length();
}


void
NOrientedBox::
SetRange(const size_t _i, Range<double>&& _r) noexcept {
  m_center[_i] = _r.Center();
  NBox::SetRange(_i, _r.min - m_center[_i], _r.max + m_center[_i]);
  m_semiRanges[_i] = NBox::GetRange(_i).Length();
}


void
NOrientedBox::
SetRange(const size_t _i, const double _min, const double _max) noexcept {
  m_center[_i] = 0.5 * (_min + _max);
  NBox::SetRange(_i, _min - m_center[_i], _max - m_center[_i]);
  m_semiRanges[_i] = 0.5 * (_max - _min); 
}

std::pair<Point3d, Point3d> 
NOrientedBox::
AxisAlignedExtremePoints() {
  Point3d f(0,0,0), s(0,0,0);
  const size_t maxIndex = std::min((size_t) 3, GetDimension());
  // Number of corner points
  size_t n = std::pow(2, maxIndex);
  for(size_t i = 0; i < n; ++i) {
    std::vector<double> p(maxIndex, 0);
    auto xRange = NBox::GetRange(0);
    p[0] = (i%2 == 0)? xRange.min: xRange.max;
    auto yRange = NBox::GetRange(1);
    p[1] = (i%4 < 2)? yRange.min: yRange.max;
    if(maxIndex == 3) {
      auto zRange = NBox::GetRange(2);
      p[2] = (i < 4)? zRange.min: zRange.max;
    }
    TransformPoint(p, false);
    for(size_t j = 0; j < maxIndex; ++j) {
      f[j] = (i>0)? std::min(f[j], p[j]): p[j];
      s[j] = (i>0)? std::max(s[j], p[j]): p[j];
    }
  }
  return std::make_pair(f, s);
}


/*------------------------------ Point Testing -------------------------------*/

bool
NOrientedBox::
Contains(const std::vector<double>& _p) const noexcept {
  std::vector<double> point( _p);
  TransformPoint(point);
  return NBox::Contains(point);
}

bool
NOrientedBox::
Contains(const Vector3d& _p) const noexcept {
  Vector3d point( _p);
  const size_t maxIndex = GetDimension();
  for(size_t i =0 ; i< maxIndex; ++i)
    point[i] -= m_center[i];
  point = -m_orientation * point;
  return NBox::Contains(point);
}

double
NOrientedBox::
Clearance(const std::vector<double>& _p) const noexcept {
  std::vector<double> p(_p);
  TransformPoint(p);
  return NBox::Clearance(p);
}

double
NOrientedBox::
Clearance(const Vector3d& _p) const noexcept {
  Vector3d point( _p);
  const size_t maxIndex = GetDimension();
  for(size_t i =0 ; i< maxIndex; ++i)
    point[i] -= m_center[i];
  point = -m_orientation * point;
  return NBox::Clearance(point);
}


std::vector<double>
NOrientedBox::
ClearancePoint(std::vector<double> _p) const noexcept {
  std::vector<double> point( _p);
  TransformPoint(point);
  point = NBox::ClearancePoint(point);
  TransformPoint(point, false);
  return point;
}

double
NOrientedBox::
DirectedClearance(const std::vector<double>& _p, const std::vector<double>& _d) const noexcept {
  const size_t dimension = GetDimension();
  std::vector<double> p(_p), d(_d);
  TransformPoint(p);
  // Transform the direction vector - rotation only
  RotatePoint(d);
  p.resize(dimension, 0);
  d.resize(dimension, 0);
  d = nonstd::unit(d);
  
  const auto& distances = LineBoxIntersection(p, d, m_semiRanges);
  // Both defined and positive take the min
  if(distances.first >= 0  && distances.second >= 0)
    return (distances.first <= distances.second)? distances.first : distances.second;
  else // otherwise take max
    return (distances.first >= distances.second)? distances.first : distances.second;
}

double
NOrientedBox::
DirectedClearance(const Vector3d& _p, const Vector3d& _d) const noexcept {
  const size_t dimension = GetDimension();
  Vector3d p(_p), d(_d);
  if(dimension < 3) {
    p[2] = d[2] = 0;
  }
  for(size_t i =0; i<dimension; ++i)
    p[i] -= m_center[i];
  d = -m_orientation * d;
  p = -m_orientation * p;
 
  if(dimension < 3) {
    p[2] = d[2] = 0;
  }
  d.selfNormalize();
  
  const auto& distances = LineBoxIntersection(p, d, m_semiRanges);
  // Both defined and positive take the min
  if(distances.first >= 0  && distances.second >= 0)
    return (distances.first <= distances.second)? distances.first : distances.second;
  else // otherwise take max
    return (distances.first >= distances.second)? distances.first : distances.second;
}


/*------------------------- Helper Functions ---------------------------------*/
void
NOrientedBox::
TransformPoint(std::vector<double>& _p, bool _f) const noexcept {
  const size_t maxIndex = std::min(_p.size(), GetDimension());
  // If forward transform - translate to center first
  if (_f)
    for (size_t i = 0; i < maxIndex; ++i)
      _p[i] -= m_center[i];

  // Rotation transformation - assuming only 3 degrees rotations
  RotatePoint(_p, _f);

  // If reverse transform - translate to center after rotation
  if (!_f)
    for (size_t i = 0; i < maxIndex; ++i)
      _p[i] += m_center[i];
}

void
NOrientedBox::
RotatePoint(std::vector<double>& _p, bool _f) const noexcept {
  // Rotation transformation - assuming only 3 degrees rotations
  Vector3d v(0,0,0);
  for(size_t i = 0; i < _p.size() && i < 3; ++i)
    v[i] = _p[i];
  if(_f)
    v = (-m_orientation) * v;
  else
    v = m_orientation * v;
  for(size_t i = 0; i < _p.size() && i < 3; ++i)
    _p[i] = v[i];
}


/*----------------------------------------------------------------------------*/
