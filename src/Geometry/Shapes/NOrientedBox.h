#ifndef N_ORIENTEDBOX_H_
#define N_ORIENTEDBOX_H_

#include <cstddef>
#include <iostream>
#include <vector>

#include "Geometry/Boundaries/Range.h"
#include "Geometry/Shapes/NBox.h"
#include "NShape.h"

#include "Orientation.h"
using namespace mathtool;

////////////////////////////////////////////////////////////////////////////////
/// An object oriented rectangular prism in n dimensions.
////////////////////////////////////////////////////////////////////////////////
class NOrientedBox : public NBox {

  public:

    ///@name Construction
    ///@{

    /// Construct an infinite box centered at the origin in _n dimensions.
    /// @param _n The number of dimensions.
    /// @param _r The box axis semi ranges (infinite by default).
    /// @param _e The orientation provided as euler angles
    explicit NOrientedBox(const size_t _n, 
      const std::vector<double>& _r = std::vector<double>(),
	    const EulerAngle& _e = EulerAngle());

    /// Construct an infinite box with a designated center point.
    /// @param _center The center point, which is assumed to be of full
    ///                dimension.
    /// @param _r The box axis semi ranges (infinte by default).
    /// @param _e The orientation provided as euler angles
    explicit NOrientedBox(const std::vector<double>& _center,
      const std::vector<double>& _r = std::vector<double>(),
	    const EulerAngle& _e = EulerAngle());

    virtual ~NOrientedBox() = default;

    ///@}
    ///@name Accessors
    ///@{

    /// Get the dimension of this box.
    size_t GetDimension() const noexcept;

    /// Set the center point.
    void SetCenter(const std::vector<double>& _c) noexcept;

    /// Get the center point.
    const std::vector<double>& GetCenter() const noexcept;

    /// Get a range in a given dimension.
    /// @param _i The dimension index.
    /// @return The range of this volume in dimension _i.
    const Range<double>& GetRange(const size_t _i) const noexcept;

    /// Get all of the ranges.
    const std::vector<Range<double>>& GetRanges() const noexcept;

    /// Set the range in a given dimension.
    /// @param _i The dimension index.
    /// @param _r The new range to use.
    void SetRange(const size_t _i, const Range<double>& _r) noexcept;
    void SetRange(const size_t _i, Range<double>&& _r) noexcept;

    /// Set the range in a given dimension.
    /// @param _i The dimension index.
    /// @param _min The new lower bound.
    /// @param _max The new upper bound.
    void SetRange(const size_t _i, const double _min, const double _max) noexcept;

    /// Get the orientation matrix of the ellipsoid
    Orientation GetOrientation() const noexcept { return m_orientation; }
    
    /// Returns the extreme points of the AABB of the shape
    std::pair<Point3d, Point3d> AxisAlignedExtremePoints();

    ///@}
    ///@name Point Testing
    ///@{
    /// If the box and point have different dimensions, the missing values will
    /// be assumed to be 0.

    /// Test if a given point lies within the box.
    /// @param _p The point to test.
    /// @return True if _p lies within the box.
    bool Contains(const std::vector<double>& _p) const noexcept;
    bool Contains(const Vector3d& _p) const noexcept;

    /// Compute the minimum distance to the box's surface from a given point.
    /// This is bounding-box style, so clearance is positive if the point is
    /// inside the box and negative if it is outside.
    /// @param _p The point of interest.
    /// @return The minimum distance from _p to the box's surface.
    double Clearance(const std::vector<double> &_p) const noexcept;
    double Clearance(const Vector3d& _p) const noexcept;


    /// Find the point on the box that is nearest to a given reference point.
    /// @param _p The reference point.
    /// @return The point on the surface that is nearest to _p.
    std::vector<double> ClearancePoint(std::vector<double> _p) const noexcept;
    
    /// Compute the minimum distance to the box's surface from a given point 
    /// along a given direction.
    /// This is bounding-box style, so clearance is positive if the point is
    /// inside the box and negative if it is outside.
    /// @param _p The point of interest.
    /// @return The minimum distance from _p to the box's surface.
    virtual double DirectedClearance(const std::vector<double>& _p, const std::vector<double>& _d) const noexcept;
    virtual double DirectedClearance(const Vector3d& _p, const Vector3d& _d) const noexcept;
    ///@}

  private:
    ///@name Helper Functions
    ///@{
    /// Rotate the point to the co-ordinate frame of the OBBB
    /// @param _p The reference point
    /// @param _f Forward transformation (true) or Reverse transformation
    void RotatePoint(std::vector<double>& _p, bool _f = true) const noexcept;
		/// Transform the point to the co-ordinate frame of the OBBB
		/// @param _p The reference point
		/// @param _f Forward transformation (true) or Reverse transformation
		void TransformPoint(std::vector<double>& _p, bool _f = true) const noexcept;

    ///@name Internal State
    ///@{
    std::vector<double> m_center;        ///< The center point of the box.
    Orientation m_orientation; ///< Orientation of the axes 
    std::vector<double> m_semiRanges; ///< Length of the semi-ranges
    ///@}

};

/*----------------------------------------------------------------------------*/

#endif
