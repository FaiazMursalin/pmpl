#include "NCapsule.h"
#include <tuple>

#include "nonstd/container_ops.h"

#include "Utilities/PMPLExceptions.h"
#include "Utilities/MPUtils.h"


/*------------------------------ Construction --------------------------------*/

NCapsule::
NCapsule(const size_t _n, const double _r) : NShape('c'), 
  m_center(_n, 0), m_line(_n, 0),
  m_segment(LineSegmentType(std::vector<double>(_n, 0), std::vector<double>(_n, 0))), 
  m_radius(_r), m_length(0) { }


NCapsule::
NCapsule(const LineSegmentType&  _l,  const double _r) : NShape('c'), 
  m_segment(_l), m_radius(_r) {
  ResizeDimension();
  LineSegment();
}

NCapsule::
NCapsule(const std::vector<double>&  _p1, const std::vector<double>& _p2, 
	const double _r) : NShape('c'), m_segment(_p1, _p2), m_radius(_r) { 
  ResizeDimension();
  LineSegment();
}

/*------------------------------- Accessors ----------------------------------*/

size_t
NCapsule::
GetDimension() const noexcept {
  return m_center.size();
}


void
NCapsule::
SetCenter(const std::vector<double>& _c) noexcept {
  const size_t maxIndex = std::min(_c.size(), GetDimension());
  for (size_t i = 0; i < maxIndex; ++i) {
    const double offset = _c[i] - m_center[i];
    m_segment.min[i] += offset;
    m_segment.max[i] += offset;
  }
  LineSegment();
}


const std::vector<double>&
NCapsule::
GetCenter() const noexcept {
  return m_center;
}


double
NCapsule::
GetRadius() const noexcept {
  return m_radius;
}


void
NCapsule::
SetRadius(const double _r) noexcept {
  m_radius = _r;
}

const NCapsule::LineSegmentType& 
NCapsule::
GetLineSegment() const noexcept {
  return m_segment;
}

void 
NCapsule::
SetLineSegment(const LineSegmentType& _l) noexcept {
  m_segment = _l;
  ResizeDimension();
  LineSegment();
}

void 
NCapsule::
SetLineSegment(LineSegmentType&& _l) noexcept {
  m_segment = std::move(_l);
  ResizeDimension();
  LineSegment();
}

void 
NCapsule::
SetLineSegment(const size_t _i, std::vector<double>& _p1, 
  std::vector<double>& _p2) noexcept {
  m_segment.min = _p1;
  m_segment.max = _p2;
  ResizeDimension();
  LineSegment();
}

std::pair<Point3d, Point3d> 
NCapsule::
AxisAlignedExtremePoints() {
  Point3d f(0,0,0), s(0,0,0);
  const size_t maxIndex = std::min(GetDimension(), (size_t)3);
  for(size_t i = 0; i < maxIndex; ++i) {
    f[i] = std::min(m_segment.min[i], m_segment.max[i]) - m_radius;
    s[i] = std::max(m_segment.min[i], m_segment.max[i]) + m_radius;
  }
  return std::make_pair(f, s);
}


/*--------------------------------- Testing ----------------------------------*/

bool
NCapsule::
Contains(const std::vector<double>& _p) const noexcept {
  return Clearance(_p) >= 0;
}

bool
NCapsule::
Contains(const Vector3d& _p) const noexcept {
  return Clearance(_p) >= 0;
}


double
NCapsule::
Clearance(const std::vector<double>& _p) const noexcept {
  const size_t maxIndex = std::min(_p.size(), GetDimension());
  // Get the nearest point in line segment to _p
  auto nearPt = NearestPoint(_p);
  std::vector<double> p(_p);
  if(nearPt == 0) {
    for(size_t i = 0; i < maxIndex; ++i)
     p[i] -= m_segment.min[i];
  }
  else if(nearPt == 1){
    for(size_t i = 0; i < maxIndex; ++i)
     p[i] -= m_segment.max[i];
  }
  else{
   for(size_t i = 0; i < maxIndex; ++i)
    p[i] -= (m_segment.min[i] + nearPt * (m_segment.max[i] - m_segment.min[i]));
  }
  return m_radius - nonstd::magnitude<double>(p);
}

double
NCapsule::
Clearance(const Vector3d& _p) const noexcept {
  const size_t maxIndex = GetDimension();
  // Get the nearest point in line segment to _p
  double t = 0;
  for(size_t i = 0; i < maxIndex; ++i)
    t += (_p[i] - m_segment.min[i]) * m_line[i];
  double clearance = 0;
  if(t <= 0) {
    for(size_t i = 0; i < maxIndex; ++i)
      clearance += std::pow((_p[i] - m_segment.min[i]), 2);
  }
  else if(t >= m_length){
    for(size_t i = 0; i < maxIndex; ++i)
     clearance += std::pow((_p[i] - m_segment.max[i]), 2);
  }
  else{
    for(size_t i = 0; i < maxIndex; ++i)
      clearance += std::pow((_p[i] - (m_segment.min[i] + t * m_line[i])), 2); 
  }
  return m_radius - std::sqrt(clearance);
}


std::vector<double>
NCapsule::
ClearancePoint(std::vector<double> _p) const noexcept {
  // Only consider dimensions that are in both _p and this.
  const size_t maxIndex = std::min(_p.size(), GetDimension());
  auto point = _p;
  point.resize(maxIndex, 0);

  // Get the nearest point in line segment to _p
  std::vector<double> nearPt(m_segment.min);
  auto near = NearestPoint(_p);
  for(size_t i = 0; i < maxIndex; ++i)
     nearPt[i] += near * (m_segment.max[i] - m_segment.min[i]);

  // Transform point to the coordinate frame to the nearest point in line segment.
  for(size_t i = 0; i < maxIndex; ++i)
    point[i] -= nearPt[i];

  // Find the scaling factor that places point on the capsule's boundary.
  const double scale = m_radius / nonstd::magnitude<double>(point);

  // Scale the values in point and return to original coordinate frame.
  for(size_t i = 0; i < maxIndex; ++i) {
    point[i] *= scale;
    point[i] += nearPt[i];
  }

  // Copy the modified values back to the original point.
  std::copy(point.begin(), point.begin() + maxIndex, _p.begin());

  return _p;
}

double
NCapsule::
DirectedClearance(const std::vector<double>& _p, const std::vector<double>& _d) const noexcept {
  const size_t dimension = GetDimension();
    
  std::vector<double> p(_p);
  for (size_t j = 0; j < dimension; ++j)
    p[j] -= m_segment.min[j];
  double t1 = nonstd::dot<double>(m_line, p)/m_length;
  double t2 = nonstd::dot<double>(m_line, _d)/m_length;
  // Get the intersection with the infinite tube
  const auto& d = LineCylinderIntersection(_p, _d, m_segment.min, m_line, m_radius);

  double distance, undefined = -std::numeric_limits<double>::max();
  double dist[] = {undefined, undefined, undefined, undefined};
  // Both defined and positive take the min
  if(d.first >= 0  && d.second >= 0)
    distance = (d.first <= d.second)? d.first : d.second;
  else // otherwise take max
    distance = (d.first >= d.second)? d.first : d.second;
  double mind = -undefined, maxd = undefined ;
  // if the distance is defined
  if(distance > undefined) {
    // Check the parameter
    double t = t1 + distance * t2; 
    // if within the line
    if(t >= 0 && t <= 1) return distance;
    if(t < 0)
     std::tie(dist[0],dist[1]) = LineSphereIntersection(_p, _d, m_segment.min, m_radius);
    else
     std::tie(dist[2],dist[3]) = LineSphereIntersection(_p, _d, m_segment.max, m_radius);
  }
  else {
    std::tie(dist[0],dist[1]) = LineSphereIntersection(_p, _d, m_segment.min, m_radius);
    std::tie(dist[2],dist[3]) = LineSphereIntersection(_p, _d, m_segment.max, m_radius);
  }
  double t3;
  for(size_t i = 0; i < 4; ++i) {
    if(dist[i] > undefined){
      t3 = t1 + dist[i] * t2;
      if((i < 2 && t3 <= 0) || (i >=2 && t3 >= 1)){
        mind = std::min(mind, dist[i]);
        maxd = std::max(maxd, dist[i]);
      }
    }
  }
  if(mind >= 0 && maxd >= 0) 
    return mind;
  else
    return maxd;
}


double
NCapsule::
DirectedClearance(const Vector3d& _p, const Vector3d& _d) const noexcept {
  const size_t dimension = GetDimension();
    
  double t1 = 0, t2 = 0;
  for (size_t j = 0; j < dimension; ++j) {
    t1 += m_line[j]*(_p[j] - m_segment.min[j]);
    t2 += m_line[j] * _d[j];
  }
  t1 /= m_length;
  t2 /= m_length;
  // Get the intersection with the infinite tube
  const auto& d = LineCylinderIntersection(_p, _d, m_segment.min, m_line, m_radius);

  double distance, undefined = -std::numeric_limits<double>::max();
  // Both defined and positive take the min
  if(d.first >= 0  && d.second >= 0)
    distance = (d.first <= d.second)? d.first : d.second;
  else // otherwise take max
    distance = (d.first >= d.second)? d.first : d.second;

  double mind = -undefined, maxd = undefined;
  double dist[] = {undefined, undefined, undefined, undefined};
  // if the distance is defined
  if(distance > undefined) {
    // Check the parameter
    double t = t1 + distance * t2; 
    // if within the line
    if(t >= 0 && t <= 1) return distance;
    // if beyond first end-point
    if(t < 0)
     std::tie(dist[0],dist[1]) = LineSphereIntersection(_p, _d, m_segment.min, m_radius);
    else // beyond second end-point
     std::tie(dist[2],dist[3]) = LineSphereIntersection(_p, _d, m_segment.max, m_radius);
  }
  else { // Get intersection points with both spheres
    std::tie(dist[0],dist[1]) = LineSphereIntersection(_p, _d, m_segment.min, m_radius);
    std::tie(dist[2],dist[3]) = LineSphereIntersection(_p, _d, m_segment.max, m_radius);
  }
  double t3;
  // check whether the intersection dist is defined and beyond the cylinder area
  for(size_t i = 0; i < 4; ++i) {
    if(dist[i] > undefined){
      t3 = t1 + dist[i] * t2;
      if((i < 2 && t3 <= 0) || (i > 1 && t3 >= 1)){
        mind = std::min(mind, dist[i]);
        maxd = std::max(maxd, dist[i]);
      }
    }
  }
  // all the distances where positive
  if(mind >= 0 && maxd >= 0) 
    return mind;
  else
    return maxd;
}


/*------------------------------- Accessors ----------------------------------*/
void 
NCapsule::
ResizeDimension() {
  const size_t maxIndex = std::max(m_segment.min.size(), m_segment.max.size());
  if (m_segment.min.size() < m_segment.max.size())
    m_segment.min.resize(maxIndex, 0);
  else if(m_segment.min.size() > m_segment.max.size())
    m_segment.max.resize(maxIndex, 0);
  m_center.resize(maxIndex, 0);

  for (size_t i = 0; i < maxIndex; ++i)
    m_center[i] = 0.5*(m_segment.min[i] + m_segment.max[i]);
}

void
NCapsule::
LineSegment() noexcept {
  // Only consider dimension of the capsule 
  const size_t maxIndex = GetDimension();
  
  // vector of the line segment 
  m_line.resize(maxIndex, 0);
  for (size_t i = 0; i < maxIndex; ++i) {
    m_line[i] = m_segment.max[i] - m_segment.min[i];
  }
  m_length = nonstd::magnitude<double>(m_line);
  if(m_length > 0)
   // normalize the normal
   m_line = nonstd::unit(m_line);
}


double 
NCapsule::
NearestPoint(const std::vector<double>& _p) const noexcept {
  // vector of the line segment
  std::vector<double> ap(_p);
  const size_t maxIndex = std::min(ap.size(),GetDimension());
  for (size_t i = 0; i < maxIndex; ++i){
    ap[i] -= m_segment.min[i]; 
  }
  // parameter value of the point
  double t = nonstd::dot<double>(m_line, ap)/m_length;
  
  // If the projection is beyond start point
  if (t <= 0) 
    return 0;
  // if the pojection is beyond end point
  else if (t >= 1) 
    return 1;
  else {
    return t;
  }
}
