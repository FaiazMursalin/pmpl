#include <cstddef>
#include <iostream>
#include <vector>
#include <utility>
#include <cmath>
#include <limits>
#include "LineIntersection.h"


inline std::pair<double, double> 
GetRootsQuadratic(double& _a, double& _b, double& _c) {
  // Output distances - initialized to infinite for starting
  std::pair<double,double> distances = 
      std::make_pair(-std::numeric_limits<double>::max(), 
      -std::numeric_limits<double>::max());
  if(fabs(_a) == 0) 
    return distances;
  // Find the numerator of the second term in finding roots of quadratic 
  // equation b^2 - ac 
  double sqrNum = (_b*_b) - (_a*_c);
  // Atleast 1 intersection
  if(sqrNum >= 0) {
    distances.first = (-_b)/_a;
    // 2 intersections
    if(sqrNum > 0) {
      double secondTerm = sqrt(sqrNum)/_a;
      distances.second = distances.first - secondTerm;
      distances.first += secondTerm;
    }
  }
  return distances;
}

/// Compute the distance of point from a line
double PointLineDistance(std::vector<double>& _p, std::vector<double>& _d,
                         std::vector<double>& _p0) noexcept{
  auto pp = PointLineProjection(_p, _d, _p0);
  for(size_t i = 0; i < pp.size(); ++i)
    pp[i] -= _p0[i];
  return nonstd::magnitude<double>(pp);
}
/// Compute the projected point on a line
std::vector<double>
PointLineProjection(std::vector<double>& _p, std::vector<double>& _d,
                    const std::vector<double>& _p0) noexcept {
  // Generate the vector connecting point to the point on the line 
  std::vector<double> v = _p0;
  for(size_t i = 0; i < v.size(); ++i)
    v[i] -= _p[i];
  
  // Get the base projection of the point on the line 
  double base = nonstd::dot<double>(v, _d);
  std::vector<double> out = _p;
  for(size_t i = 0; i < out.size(); ++i)
    out[i] += base*_d[i];
  return out;
}

/// Compute the signed distance of the surface from the start point
double
LinePlaneIntersection(std::vector<double>& _p, std::vector<double>& _d,
    std::vector<double>& _n, std::vector<double>& _pl) noexcept {
  size_t dimension = _n.size();
  // Check whether the line is parallel to plane
  double denom = nonstd::dot<double>(_n, _d);
  if (denom == 0)
    return std::numeric_limits<double>::max();
  // Return the d value
  double num = 0;
  for (size_t i = 0; i < dimension; ++i)
    num += (_pl[i] - _p[i])*_n[i];
  return num / denom;
}

/// Compute the signed distance of the sphere's surface from the start point
std::pair<double,double>
LineSphereIntersection(const std::vector<double>& _p, const std::vector<double>& _d,
                       const std::vector<double>& _c, const double _r) noexcept {
  size_t dimension = _c.size();
  
  // Find the co-efficients of the quadratic equation
  double a = 1,
         b = 0, 
         c = -(_r * _r);
  double pc;
  for(size_t i = 0; i < dimension; ++i) {
    pc = _p[i] - _c[i];  // Vector connecting point to center
    b += (pc * _d[i]);
    c += (pc * pc);
  }
  return GetRootsQuadratic(a,b,c);
}

/// Compute the signed distance of the ellipsoid's surface from the start point
std::pair<double,double>
LineEllipsoidIntersection(const std::vector<double>& _p, const std::vector<double>& _d,
                          const std::vector<double>& _r) noexcept {
  size_t dimension = _r.size();
  
  // Find the squared product of the ranges
  double rangeProduct = 1;
  for(size_t i = 0; i < dimension; ++i)
    rangeProduct *= _r[i];
  rangeProduct *= rangeProduct;
  // Find the vector of the squared product of other ranges
  std::vector<double> ranges(dimension);
  for(size_t i = 0; i < dimension; ++i)
    ranges[i] = rangeProduct/(_r[i]*_r[i]);
  // Find the co-efficients of the quadratic equation
  double a = 0, b = 0, c = -rangeProduct;
  for(size_t i = 0; i < dimension; i++) {
    a += (_d[i]*_d[i]*ranges[i]);
    b += (ranges[i]* _p[i] * _d[i]);
    c += (ranges[i]*_p[i]*_p[i]);
  }
  return GetRootsQuadratic(a,b,c);
}

/// Compute the signed distance of the box's surface from the start point
std::pair<double, double>
LineBoxIntersection(const std::vector<double>& _p, const std::vector<double>& _d,
  const std::vector<double>& _r) noexcept {
  size_t dimension = _r.size();
  double undefined = -std::numeric_limits<double>::max();
  // Output distances - initialized to infinite for starting
  std::pair<double, double> distances = std::make_pair(undefined,
    undefined);

  // Get the plane intersection for each dimension
  for (size_t i = 0; i < dimension; ++i) {
    // Check whether the planes are parallel to the line
    // normal.direction == 0
    if (std::fabs(_d[i]) < std::numeric_limits<double>::epsilon()) continue;
    
    for (size_t k = 0; k < 2; k++) {
      // get the distance
      double lambda = (((k%2== 0)?_r[i]: -_r[i]) - _p[i]) / _d[i];
      bool withinRange = true;
      // Check whether the intersection point is within range
      for (size_t j = 0; j < dimension; ++j) {
        if (i == j) continue;
        double pi = _p[j] + lambda * _d[j];
        if (pi < -_r[j] || pi > _r[j]) {
          withinRange = false;
          break;
        }
      }
      // if within range store the distance
      if (withinRange) {
        if (distances.first == undefined)
          distances.first = lambda;
        else if (std::fabs(lambda - distances.first) > std::numeric_limits<double>::epsilon()
          && distances.second == undefined)
          distances.second = lambda;
      }
    }
  }
  return distances;
}

/// Compute the signed distance of the cylinder's surface from the start point
std::pair<double, double>
LineCylinderIntersection(const std::vector<double>& _p, const std::vector<double>& _d,
   const std::vector<double>& _a, const std::vector<double>& _l, const double _r) noexcept {
  size_t dimension = _a.size();
  
  // Get the direction vector of the cylinder's line
  // and the vector connecting the point in cylinder's line 
  // and point on the query line
  
  // find the projection of the direction vector on the line
  // and point on the line
  double dl = 0,
         pl = 0;
  // Find the co-efficients of the quadratic equation
  double a = 1,
         b = 0,
         c = -(_r*_r);
  double pc;
  for (size_t i = 0; i < dimension; ++i) {
    pc = _p[i] -  _a[i];
    dl += (_l[i]*_d[i]);
    pl += (_l[i] * pc);
    b += (pc * _d[i]);
    c += (pc * pc);
  }
  
  // Find the co-efficients of the quadratic equation
  a -= (dl * dl);
  b -= (pl * dl);
  c -= (pl * pl);
  return GetRootsQuadratic(a,b,c);
}

///////////////////Vector3d///////////////////////////////
/// Compute the signed distance of the sphere's surface from the start point
std::pair<double, double>
LineSphereIntersection(const Vector3d& _p, const Vector3d& _d,
	const std::vector<double>& _c, const double _r) noexcept {
	size_t dimension = _c.size();

	// Find the co-efficients of the quadratic equation
	double a = 1,
		b = 0,
		c = -(_r * _r);
	double pc;
	for (size_t i = 0; i < dimension; ++i) {
		pc = _p[i] - _c[i];  // Vector connecting point to center
		b += (pc * _d[i]);
		c += (pc * pc);
	}
	return GetRootsQuadratic(a, b, c);
}

/// Compute the signed distance of the ellipsoid's surface from the start point
std::pair<double, double>
LineEllipsoidIntersection(const Vector3d& _p, const Vector3d& _d,
	const std::vector<double>& _r) noexcept {
	size_t dimension = _r.size();

	// Find the squared product of the ranges
	double rangeProduct = 1;
	for (size_t i = 0; i < dimension; ++i)
		rangeProduct *= _r[i];
	rangeProduct *= rangeProduct;
	// Find the vector of the squared product of other ranges
	std::vector<double> ranges(dimension);
	for (size_t i = 0; i < dimension; ++i)
		ranges[i] = rangeProduct / (_r[i] * _r[i]);
	// Find the co-efficients of the quadratic equation
	double a = 0, b = 0, c = -rangeProduct;
	for (size_t i = 0; i < dimension; i++) {
		a += (_d[i] * _d[i] * ranges[i]);
		b += (ranges[i] * _p[i] * _d[i]);
		c += (ranges[i] * _p[i] * _p[i]);
	}
	return GetRootsQuadratic(a, b, c);
}

/// Compute the signed distance of the box's surface from the start point
std::pair<double, double>
LineBoxIntersection(const Vector3d& _p, const Vector3d& _d,
	const std::vector<double>& _r) noexcept {
	size_t dimension = _r.size();
	double undefined = -std::numeric_limits<double>::max();
	// Output distances - initialized to infinite for starting
	std::pair<double, double> distances = std::make_pair(undefined,
		undefined);

	// Get the plane intersection for each dimension
	for (size_t i = 0; i < dimension; ++i) {
		// Check whether the planes are parallel to the line
		// normal.direction == 0
		if (std::fabs(_d[i]) < std::numeric_limits<double>::epsilon()) continue;

		for (size_t k = 0; k < 2; k++) {
			// get the distance
			double lambda = (((k % 2 == 0) ? _r[i] : -_r[i]) - _p[i]) / _d[i];
			bool withinRange = true;
			// Check whether the intersection point is within range
			for (size_t j = 0; j < dimension; ++j) {
				if (i == j) continue;
				double pi = _p[j] + lambda * _d[j];
				if (pi < -_r[j] || pi > _r[j]) {
					withinRange = false;
					break;
				}
			}
			// if within range store the distance
			if (withinRange) {
				if (distances.first == undefined)
					distances.first = lambda;
				else if (std::fabs(lambda - distances.first) > std::numeric_limits<double>::epsilon()
					&& distances.second == undefined)
					distances.second = lambda;
			}
		}
	}
	return distances;
}

/// Compute the signed distance of the cylinder's surface from the start point
std::pair<double, double>
LineCylinderIntersection(const Vector3d& _p, const Vector3d& _d,
	const std::vector<double>& _a, const std::vector<double>& _l, const double _r) noexcept {
	size_t dimension = _a.size();

	// Get the direction vector of the cylinder's line
	// and the vector connecting the point in cylinder's line 
	// and point on the query line

	// find the projection of the direction vector on the line
	// and point on the line
	double dl = 0,
		pl = 0;
	// Find the co-efficients of the quadratic equation
	double a = 1,
		b = 0,
		c = -(_r*_r);
	double pc;
	for (size_t i = 0; i < dimension; ++i) {
		pc = _p[i] - _a[i];
		dl += (_l[i] * _d[i]);
		pl += (_l[i] * pc);
		b += (pc * _d[i]);
		c += (pc * pc);
	}

	// Find the co-efficients of the quadratic equation
	a -= (dl * dl);
	b -= (pl * dl);
	c -= (pl * pl);
	return GetRootsQuadratic(a, b, c);
}
