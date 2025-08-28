#ifndef N_ELLIPSOID_H_
#define N_ELLIPSOID_H_

#include <cstddef>
#include <iostream>
#include <limits>
#include <vector>
using namespace std;

#include "NShape.h"

#include "Orientation.h"
using namespace mathtool;

////////////////////////////////////////////////////////////////////////////////
/// A general representation of a ellipsoid in N dimensions.
////////////////////////////////////////////////////////////////////////////////
class NEllipsoid : public NShape {

  public:

    ///@name Construction
    ///@{

    /// Construct an n-ellipsoid at the origin with a given dimension and 
    /// n-axes ranges.
    /// @param _n The dimension of the n-ellipsoid.
    /// @param _r The ellipsoid axis semi ranges (infinite by default).
    /// @param _e The orientation provided as euler angles
    explicit NEllipsoid(const size_t _n,
        const std::vector<double>& _r = std::vector<double>(),
	      const EulerAngle& _e = EulerAngle());

    /// Construct an n-sphere with a given center point and radius.
    /// @param _c The center point of the n-ellipsoid, which is assumed to be of
    ///           full dimension.
    /// @param _r The ellipsoid axis semi ranges (infinte by default).
    /// @param _e The orientation provided as euler angles
    explicit NEllipsoid(const std::vector<double>& _center,
        const std::vector<double>& _r = std::vector<double>(),
	      const EulerAngle& _e = EulerAngle());

    virtual ~NEllipsoid() = default;

    ///@}
    ///@name Accessors
    ///@{

    /// Get the dimension of this ellipsoid.
    size_t GetDimension() const noexcept;

    /// Set the center point.
    /// @param _c The new center point.
    void SetCenter(const std::vector<double>& _c) noexcept;

    /// Get the center point.
    const std::vector<double>& GetCenter() const noexcept;

    /// Get the axis semi range.
    double GetAxisSemiRange(size_t _i) const noexcept;

    /// Set the axis semi range.
    void SetAxisSemiRange(size_t _i, const double _r) noexcept;

    /// Get the orientation matrix of the ellipsoid
    Orientation GetOrientation() const noexcept;

    /// Translate the entire n-sphere.
    /// @param _v The translation vector to apply.
    void Translate(const std::vector<double>& _v) noexcept;

    /// Returns the extreme points of the AABB of the shape
    std::pair<Point3d, Point3d> AxisAlignedExtremePoints();

    ///@}
    ///@name Point Testing
    ///@{
    /// If the ellipsoid and point have different dimensions, the missing values
    /// will be assumed to be 0.

    /// Test if a given point lies within the n-ellipsoid.
    /// @param _p The point to test.
    /// @return True if _p lies within the n-ellipsoid.
    bool Contains(const std::vector<double>& _p) const noexcept;
    bool Contains(const Vector3d& _p) const noexcept;


    /// Compute the minimum distance to the ellipsoid's surface from a given 
    /// point.
    /// This is bounding-ellipsoid style, so clearance is positive if 
    /// the point is inside the ellipsoid and negative if it is outside.
    /// @param _p The point of interest.
    /// @return The minimum distance from _p to the ellipsoid's surface.
    double Clearance(const std::vector<double>& _p) const noexcept;
    double Clearance(const Vector3d& _p) const noexcept;

    /// Compute the minimum distance to the object based bounding box surface  
    /// from a given point.
    /// This is ellipsoid style, so clearance is positive if 
    /// the point is inside the ellipsoid OBBB and negative if it is outside.
    /// @param _p The point of interest.
    /// @return The minimum distance from _p to the ellipsoid OBBB's surface.
    double BoxClearance(std::vector<double> _p) const noexcept;

    /// Compute the minimum distance to the ellipsoid surface  
    /// from a given point.
    /// This is ellipsoid style, so clearance is positive if 
    /// the point is inside the ellipsoid and negative if it is outside.
    /// @param _p The point of interest.
    /// @return The minimum distance from _p to the ellipsoid's surface.
    double ExactClearance(std::vector<double> _p) const noexcept;


    /// Find the point on the elliposid that is nearest to a given 
    /// reference point.
    /// @param _p The reference point.
    /// @return The point on the surface that is nearest to _p.
    std::vector<double> ClearancePoint(std::vector<double> _p) const noexcept;

    /// Compute the minimum distance to the ellipsoid's surface from a given point 
    /// along a given direction.
    /// This is bounding-box style, so clearance is positive if the point is
    /// inside the ellipsoid and negative if it is outside.
    /// @param _p The point of interest.
    /// @return The minimum distance from _p to the ellipsoid's surface.
    virtual double DirectedClearance(const std::vector<double>& _p, const std::vector<double>& _d) const noexcept;
    virtual double DirectedClearance(const Vector3d& _p, const Vector3d& _d) const noexcept;
    ///@}

  private:
    ///@name Helper Functions
    ///@{
    /// Transform the point to the co-ordinate frame at the center of ellipsoid
    /// @param _p The reference point
    /// @param _f Forward transformation (true) or Reverse transformation
    void TransformPoint(std::vector<double>& _p, bool _f = true) const noexcept;

    /// Rotate the point to the co-ordinate frame of the OBBB
    /// @param _p The reference point
    /// @param _f Forward transformation (true) or Reverse transformation
    void RotatePoint(std::vector<double>& _p, bool _f = true) const noexcept;

    /// Find the scaling factor which is placing the point in ellipsoid equation
    /// @param _p The reference point
    /// @param _t Should apply transformation
    double ScaleFactor(std::vector<double>& _p) const noexcept;

    ///@}
    ///@name Internal State
    ///@{

    std::vector<double> m_center; ///< The center point.
    std::vector<double> m_axes; ///< The axis semi-ranges.
    Orientation m_orientation; ///< Orientation of the axes 

    ///@}

};

/*----------------------------------------------------------------------------*/

#endif
