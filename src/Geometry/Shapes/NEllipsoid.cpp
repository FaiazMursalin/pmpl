#include "NEllipsoid.h"
#include "nonstd/container_ops.h"
#include "Utilities/PMPLExceptions.h"
#include "Utilities/MPUtils.h"


/*------------------------------ Construction --------------------------------*/

NEllipsoid::
NEllipsoid(const size_t _n, const std::vector<double>& _r, const EulerAngle& _e)
  : NShape('e'), m_center(_n, 0), m_axes(_r), m_orientation(_e) {
  if(_r.empty())
    m_axes.assign(_n,std::numeric_limits<double>::max());
}


NEllipsoid::
NEllipsoid(const std::vector<double>& _c, const std::vector<double>& _r, 
  const EulerAngle& _e): NShape('e'), m_center(_c), m_axes(_r), m_orientation(_e) {
  if(_r.empty()) 
    m_axes.assign(_c.size(),std::numeric_limits<double>::max());
}

/*------------------------------- Accessors ----------------------------------*/

size_t
NEllipsoid::
GetDimension() const noexcept {
  return m_center.size();
}


void
NEllipsoid::
SetCenter(const std::vector<double>& _c) noexcept {
  const size_t maxIndex = std::min(_c.size(), GetDimension());
  for(size_t i = 0; i < maxIndex; ++i)
    m_center[i] = _c[i];
}


const std::vector<double>&
NEllipsoid::
GetCenter() const noexcept {
  return m_center;
}


double
NEllipsoid::
GetAxisSemiRange(size_t _i) const noexcept {
  return m_axes[_i];
}


void
NEllipsoid::
SetAxisSemiRange(size_t _i, const double _r) noexcept {
  // Check if the dimension is small than the axis dimension
  if(_i+1 > m_axes.size())	
    m_axes.resize(_i+1,0);
  if(_i+1 > GetDimension())	
    m_center.resize(_i+1,0);
	
  m_axes[_i] = _r;
}

Orientation 
NEllipsoid::
GetOrientation() const noexcept {
  return m_orientation;
}


void
NEllipsoid::
Translate(const std::vector<double>& _v) noexcept {
  const size_t maxIndex = std::min(_v.size(), GetDimension());
  for(size_t i = 0; i < maxIndex; ++i)
    m_center[i] += _v[i];
}

std::pair<Point3d, Point3d> 
NEllipsoid::
AxisAlignedExtremePoints() {
  Point3d f(0,0,0), s(0,0,0);
  const size_t maxIndex = std::min((size_t) 3, GetDimension());
  size_t n = std::pow(2, maxIndex);
  for(size_t i = 0; i < n; ++i) {
    std::vector<double> p(maxIndex, 0);
    p[0] = (i%2 == 0)?(-m_axes[0]): m_axes[0];
    p[1] = (i%4 < 2)?(-m_axes[1]): m_axes[1];
    if(maxIndex == 3) {
      p[2] = (i < 4)?(-m_axes[2]): m_axes[2];
    }
    TransformPoint(p, false);
    for(size_t j = 0; j < maxIndex; ++j) {
      f[j] = (i>0)? std::min(f[j], p[j]): p[j];
      s[j] = (i>0)? std::max(s[j], p[j]): p[j];
    }
  }
  return std::make_pair(f, s);
}

/*--------------------------------- Testing ----------------------------------*/

bool
NEllipsoid::
Contains(const std::vector<double>& _p) const noexcept {
  std::vector<double> p(_p);
  TransformPoint(p);
  return (ScaleFactor(p) <= 1);
}

bool
NEllipsoid::
Contains(const Vector3d& _p) const noexcept {
  const size_t maxIndex= GetDimension();
  Vector3d p(_p);
  for(size_t i =0; i< maxIndex; ++i)
    p[i] -= m_center[i];
  p = -m_orientation *p;

  double scale = 0;
  for(size_t i = 0; i < maxIndex; ++i)
    scale += std::pow(p[i]/m_axes[i], 2);
  scale = std::sqrt(scale);

  return (scale <= 1);
}


double
NEllipsoid::
ExactClearance(std::vector<double> _p) const noexcept {
  TransformPoint(_p);
  const double length = nonstd::magnitude<double>(_p);
  const double scale = ScaleFactor(_p);
  return (length*(1/scale - 1));
}

double
NEllipsoid::
BoxClearance(std::vector<double> _p) const noexcept {
  TransformPoint(_p);
  double clp = ScaleFactor(_p);
  if(clp == 1) return 0;
  const size_t maxIndex = std::min(_p.size(), m_axes.size());
  double clearance = std::numeric_limits<double>::max();
  for(size_t i = 0; i < maxIndex; ++i) 
    clearance = std::min((m_axes[i] - fabs(_p[i])), clearance);
  
  return clearance;
}

double
NEllipsoid::
Clearance(const std::vector<double>& _p) const noexcept {
  std::vector<double> p(_p);
  TransformPoint(p);
  const double length = nonstd::magnitude<double>(p);
  double scale = ScaleFactor(p);

  double clearance = length*(1/scale - 1);
  if(clearance == 0) return clearance;

  const size_t maxIndex = std::min(p.size(), m_axes.size());
  for(size_t i = 0; i < maxIndex; ++i)
    if(p[i] == 0) return clearance;
   
  for(size_t i = 0; i < maxIndex; ++i) {
    double sum = 0;
    for(size_t j = 0; j < maxIndex; ++j)
      if(j != i)
        sum += std::pow((p[j]/m_axes[j]),2);
    sum = std::sqrt(1-sum)*m_axes[i];
    clearance = std::min((sum - fabs(p[i])), clearance);
  }
  return clearance;
}

double
NEllipsoid::
Clearance(const Vector3d& _p) const noexcept {
  const size_t maxIndex= GetDimension();
  Vector3d p(_p);
  for(size_t i =0; i< maxIndex; ++i)
    p[i] -= m_center[i];
  p = -m_orientation *p;

  const double length = p.norm();
  double scale = 0;
  for(size_t i = 0; i < maxIndex; ++i)
    scale += std::pow(p[i]/m_axes[i], 2);
  scale = std::sqrt(scale);

  double clearance = length * (1/scale - 1);
  if(clearance == 0) return clearance;

  for(size_t i = 0; i < maxIndex; ++i)
    if(p[i] == 0) return clearance;
   
  for(size_t i = 0; i < maxIndex; ++i) {
    double sum = 0;
    for(size_t j = 0; j < maxIndex; ++j)
      if(j != i)
        sum += std::pow((p[j]/m_axes[j]),2);
    sum = std::sqrt(1-sum)*m_axes[i];
    clearance = std::min((sum - fabs(p[i])), clearance);
  }
  return clearance;
}


std::vector<double>
NEllipsoid::
ClearancePoint(std::vector<double> _p) const noexcept {
  // Only consider dimensions that are in both _p and this.
  const size_t maxIndex = std::min(_p.size(), GetDimension());
  auto point = _p;
  point.resize(maxIndex, 0);

  // Transform point to the coordinate frame at the ellipsoid's center.
  TransformPoint(point);

  // Find the scaling factor that places point on the ellipsoid's boundary.
  const double scale = ScaleFactor(point);

  // Scale the values in point
  for(size_t i = 0; i < maxIndex; ++i) {
    point[i] /= scale;
  }

  // Return to world coordinate frame.
  TransformPoint(point, false);
  // Copy the modified values back to the original point.
  std::copy(point.begin(), point.begin() + maxIndex, _p.begin());

  return _p;
}

double
NEllipsoid::
DirectedClearance(const std::vector<double>& _p, const std::vector<double>& _d) const noexcept {
  // Get the transformation of the direction vector - Only rotation
  const size_t dimension = GetDimension();
  std::vector<double> p(_p), d(_d);
  RotatePoint(d);
  TransformPoint(p);
  d = nonstd::unit(d);
  p.resize(dimension, 0);
  d.resize(dimension, 0);
  const auto& distances = LineEllipsoidIntersection(p, d, m_axes);
  // Both defined and positive take the min
  if(distances.first >= 0  && distances.second >= 0)
    return (distances.first <= distances.second)? distances.first : distances.second;
  else // otherwise take max
    return (distances.first >= distances.second)? distances.first : distances.second;
}

double
NEllipsoid::
DirectedClearance(const Vector3d& _p, const Vector3d& _d) const noexcept {
  // Get the transformation of the direction vector - Only rotation
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
  
  const auto& distances = LineEllipsoidIntersection(p, d, m_axes);
  // Both defined and positive take the min
  if(distances.first >= 0  && distances.second >= 0)
    return (distances.first <= distances.second)? distances.first : distances.second;
  else // otherwise take max
    return (distances.first >= distances.second)? distances.first : distances.second;
}


/*------------------------- Helper Functions ---------------------------------*/
void
NEllipsoid::
TransformPoint(std::vector<double>& _p, bool _f) const noexcept {
  const size_t maxIndex = std::min(_p.size(), GetDimension());
  // If forward transform - translate to center first
  if(_f)
    for(size_t i = 0; i < maxIndex; ++i)
      _p[i] -= m_center[i];	

  // Rotation transformation - assuming only 3 degrees rotations
  RotatePoint(_p, _f);

	// If reverse transform - translate to center after rotation
  if(!_f)
    for(size_t i = 0; i < maxIndex; ++i)
      _p[i] += m_center[i];	
}

void
NEllipsoid::
RotatePoint(std::vector<double>& _p, bool _f) const noexcept {
  // Rotation transformation - assuming only 3 degrees rotations
  Vector3d v(0, 0, 0);
  for (size_t i = 0; i < _p.size() && i < 3; ++i)
    v[i] = _p[i];
  if (_f)
    v = (-m_orientation) * v;
  else
    v = m_orientation * v;
  for (size_t i = 0; i < _p.size() && i < 3; ++i)
    _p[i] = v[i];
}

double 
NEllipsoid::
ScaleFactor(std::vector<double>& _p) const noexcept  {
  const size_t maxIndex = std::min(_p.size(), m_axes.size());
  double sum = 0;
  for(size_t i = 0; i < maxIndex; ++i)
    sum += pow(_p[i]/m_axes[i], 2);
  return sqrt(sum); 
}

/*----------------------------------------------------------------------------*/
