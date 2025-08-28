#ifndef LINE_INTERSECTION_H_
#define LINE_INTERSECTION_H_

#include <cstddef>
#include <iostream>
#include <vector>
#include <utility>
#include <cmath>
#include "nonstd/container_ops.h"
#include "Vector.h"
#include "Geometry/Boundaries/Range.h"

using namespace mathtool;
////////////////////////////////////////////////////////////////////////////////
/// Utilities to find intersections of line with the N-dimensional shape
/// The utilities are defined outside the class of the shapes to be used 
/// without creating an object for the shape as required by shapes like 
/// cylinder and capsules that uses sphere utilities
////////////////////////////////////////////////////////////////////////////////

inline std::pair<double, double> GetRootsQuadratic(double& _a, double& _b, double& _c);

/// Compute the distance of point from a line
/// @param _p Point of the line
/// @param _d Unit direction vector of the line
/// @param _p0 Point to project
double PointLineDistance(std::vector<double>& _p, std::vector<double>& _d,
                         std::vector<double>& _p0) noexcept;
/// Compute the projected point on a line
/// @param _p Point of the line
/// @param _d Unit direction vector of the line
/// @param _p0 Point to project
std::vector<double>
PointLineProjection(std::vector<double>& _p, std::vector<double>& _d,
                    const std::vector<double>& _p0) noexcept;

/// Compute the signed distance of the surface from the start point
/// @param _p Point of the line
/// @param _d Direction vector of the line
/// @param _n normal of the plane
/// @param _pl Point on the plane 
double
LinePlaneIntersection(std::vector<double>& _p, std::vector<double>& _d,
	std::vector<double>& _n, std::vector<double>& _pl) noexcept;

/// Compute the signed distance of the sphere's surface from the start point
/// @param _p Point of the line
/// @param _d Direction vector of the line
/// @param _c center of the sphere
/// @param _r radius of the sphere 
std::pair<double,double>
LineSphereIntersection(const std::vector<double>& _p, const std::vector<double>& _d,
                       const std::vector<double>& _c, const double _r) noexcept;

/// Compute the signed distance of the ellipsoid's surface from the start point
/// Assumption: Ellipsoid centered at origin
/// @param _p Point of the line
/// @param _d Direction vector of the line
/// @param _r Semi-ranges of the ellipsoid 
std::pair<double,double>
LineEllipsoidIntersection(const std::vector<double>& _p, const std::vector<double>& _d,
                          const std::vector<double>& _r) noexcept;

/// Compute the signed distance of the box's surface from the start point
/// Assumption: Box centered at origin
/// @param _p Point of the line
/// @param _d Direction vector of the line
/// @param _r Semi-ranges of the box 
std::pair<double, double>
LineBoxIntersection(const std::vector<double>& _p, const std::vector<double>& _d,
	const std::vector<double>& _r) noexcept;

/// Compute the signed distance of the cylinder's surface from the start point
/// Assumption: Infinite cylinder
/// @param _p Point of the line
/// @param _d Direction vector of the line
/// @param _a,_b End-points of the underlying line of the cylinder
std::pair<double, double>
LineCylinderIntersection(const std::vector<double>& _p, const std::vector<double>& _d,
	const std::vector<double>& _a, const std::vector<double>& _b, const double _r) noexcept;

/// Compute the positive signed distance of the sphere's surface from the start point
/// @param _p Point of the line
/// @param _d Direction vector of the line
/// @param _c center of the sphere
/// @param _r radius of the sphere 
std::pair<double, double>
LineSphereIntersection(const Vector3d& _p, const Vector3d& _d,
	const std::vector<double>& _c, const double _r) noexcept;

/// Compute the signed distance of the ellipsoid's surface from the start point
/// Assumption: Ellipsoid centered at origin
/// @param _p Point of the line
/// @param _d Direction vector of the line
/// @param _r Semi-ranges of the ellipsoid 
std::pair<double, double>
LineEllipsoidIntersection(const Vector3d& _p, const Vector3d& _d,
	const std::vector<double>& _r) noexcept;

/// Compute the signed distance of the box's surface from the start point
/// Assumption: Box centered at origin
/// @param _p Point of the line
/// @param _d Direction vector of the line
/// @param _r Semi-ranges of the box 
std::pair<double, double>
LineBoxIntersection(const Vector3d& _p, const Vector3d& _d,
	const std::vector<double>& _r) noexcept;

/// Compute the signed distance of the cylinder's surface from the start point
/// Assumption: Infinite cylinder
/// @param _p Point of the line
/// @param _d Direction vector of the line
/// @param _a,_b End-points of the underlying line of the cylinder
std::pair<double, double>
LineCylinderIntersection(const Vector3d& _p, const Vector3d& _d,
	const std::vector<double>& _a, const std::vector<double>& _b, const double _r) noexcept;

#endif
