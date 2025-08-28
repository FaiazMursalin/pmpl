#ifndef N_SHAPE_H_
#define N_SHAPE_H_

#include <cstddef>
#include <iostream>
#include <vector>

#include "Geometry/Boundaries/Range.h"
#include "LineIntersection.h"
#include "Vector.h"

using namespace mathtool;

////////////////////////////////////////////////////////////////////////////////
/// Abstract class for n-dimensional shape
////////////////////////////////////////////////////////////////////////////////
class NShape {
  public:

    ///@name Construction
    ///@{

    /// Construct a n-dimensional shape
    NShape(char _t = 'o') : m_type(_t) {}

    virtual ~NShape() = default;

    ///@}
    ///@name Accessors
    ///@{

    /// Get the dimension of this box.
    virtual size_t GetDimension() const noexcept = 0;

    /// Set the center point.
    virtual void SetCenter(const std::vector<double>& _c) noexcept = 0;

    /// Get the center point.
    virtual const std::vector<double>& GetCenter() const noexcept = 0;
    /// Get Type
    char GetType() { return m_type; }
    
    ///@}
    ///@name Point Testing
    ///@{
    /// If the box and point have different dimensions, the missing values will
    /// be assumed to be 0.

    /// Test if a given point lies within the shape.
    /// @param _p The point to test.
    /// @return True if _p lies within the shape.
    virtual bool Contains(const std::vector<double>& _p) const noexcept = 0;
    virtual bool Contains(const Vector3d& _p) const noexcept = 0;

    /// Compute the minimum distance to the shape's surface from a given point.
    /// This is bounding-box style, so clearance is positive if the point is
    /// inside the shape and negative if it is outside.
    /// @param _p The point of interest.
    /// @return The minimum distance from _p to the shape's surface.
    virtual double Clearance(const std::vector<double>& _p) const noexcept = 0;
    virtual double Clearance(const Vector3d& _p) const noexcept = 0;

    /// Find the point on the shape that is nearest to a given reference point.
    /// @param _p The reference point.
    /// @return The point on the surface that is nearest to _p.
    virtual std::vector<double> ClearancePoint(std::vector<double> _p) const noexcept = 0;

    /// Compute the minimum distance to the shape's surface from a given point 
    /// along a given direction.
    /// This is bounding-box style, so clearance is positive if the point is
    /// inside the shape and negative if it is outside.
    /// @param _p The point of interest.
    /// @return The minimum distance from _p to the shape's surface.
    virtual double DirectedClearance(const std::vector<double>& _p, const std::vector<double>& _d) const noexcept = 0;
    virtual double DirectedClearance(const Vector3d& _p, const Vector3d& _d) const noexcept = 0;


    /// Returns the extreme points of the AABB of the shape
    virtual std::pair<Point3d, Point3d> AxisAlignedExtremePoints() = 0;
   
    ///@}
  private:
    char m_type; ///< Type of shape 

};


#endif
