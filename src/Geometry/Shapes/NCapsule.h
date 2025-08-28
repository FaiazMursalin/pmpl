#ifndef N_CAPSULE_H_
#define N_CAPSULE_H_

#include <cstddef>
#include <iostream>
#include <limits>
#include <vector>

#include "Geometry/Boundaries/Range.h"
#include "NShape.h"

////////////////////////////////////////////////////////////////////////////////
/// A general representation of a capsule  in N dimensions.
/// A capsule is a swept volume of a sphere on a line-segment
////////////////////////////////////////////////////////////////////////////////
class NCapsule : public NShape {

  public:
    ///@name Local Types
    ///@{

    typedef Range<std::vector<double>> LineSegmentType;

    ///@}
    ///@name Construction
    ///@{

    /// Construct an n-capsule at the origin with a given dimension and radius.
    /// @param _n The dimension of the n-capsule.
    /// @param _r The capsule radius (infinite by default).
    explicit NCapsule(const size_t _n,
        const double _r = std::numeric_limits<double>::max());

    /// Construct an n-capsule with a given line segment and radius.
    /// @param _l The backbone line segment, which is assumed to be of
    ///           full dimension.
    /// @param _r The capsule radius (infinite by default).
    explicit NCapsule(const LineSegmentType& _l,
        const double _r = std::numeric_limits<double>::max());

    /// Construct an n-capsule with a given line segment and radius.
    /// @param _l The backbone line segment, which is assumed to be of
    ///           full dimension.
    /// @param _r The capsule radius (infinite by default).
    explicit NCapsule(const std::vector<double>&  _p1, const std::vector<double>& _p2,
			const double _r = std::numeric_limits<double>::max());

    virtual ~NCapsule() = default;

    ///@}
    ///@name Accessors
    ///@{

    /// Get the dimension of this sphere.
    size_t GetDimension() const noexcept;

    /// Set the center point.
    /// @param _c The new center point.
    void SetCenter(const std::vector<double>& _c) noexcept;

    /// Get the center point.
    const std::vector<double>& GetCenter() const noexcept;

    /// Get the radius.
    double GetRadius() const noexcept;

    /// Set the radius.
    void SetRadius(const double _r) noexcept;

    /// Get the line segment.
    const LineSegmentType& GetLineSegment() const noexcept;

    /// Set the line segment.
    /// @param _l The new line segment to use.
    void SetLineSegment(const LineSegmentType& _l) noexcept;
    void SetLineSegment(LineSegmentType&& _l) noexcept;

    /// Set the line segment.
    /// @param _p1, _p2 End-points of the line segment
    void SetLineSegment(const size_t _i, std::vector<double>& _p1, std::vector<double>& _p2) noexcept;

    /// Returns the extreme points of the AABB of the shape
    std::pair<Point3d, Point3d> AxisAlignedExtremePoints();

    ///@}
    ///@name Point Testing
    ///@{
    /// If the capsule and point have different dimensions, the missing values
    /// will be assumed to be 0.

    /// Test if a given point lies within the n-capsule.
    /// @param _p The point to test.
    /// @return True if _p lies within the n-capsule.
    bool Contains(const std::vector<double>& _p) const noexcept;
    bool Contains(const Vector3d& _p) const noexcept;


    /// Compute the minimum distance to the capsule's surface from a given point.
    /// This is bounding volume style, so clearance is positive if the point is
    /// inside the capsule and negative if it is outside.
    /// @param _p The point of interest.
    /// @return The minimum distance from _p to the capsule's surface.
    double Clearance(const std::vector<double>& _p) const noexcept;
    double Clearance(const Vector3d& _p) const noexcept;


    /// Find the point on the capsule that is nearest to a given reference point.
    /// @param _p The reference point.
    /// @return The point on the capsule that is nearest to _p.
    std::vector<double> ClearancePoint(std::vector<double> _p) const noexcept;

    /// Compute the minimum distance to the capsule's surface from a given point 
    /// along a given direction.
    /// This is bounding-box style, so clearance is positive if the point is
    /// inside the capsule and negative if it is outside.
    /// @param _p The point of interest.
    /// @return The minimum distance from _p to the capsule's surface.
    virtual double DirectedClearance(const std::vector<double>& _p, const std::vector<double>& _d) const noexcept;
    virtual double DirectedClearance(const Vector3d& _p, const Vector3d& _d) const noexcept;

    ///@}

  private:

    ///@{ 
    ///@name Helper Functions

    /// Resize the dimension of the end-points of the line-segment and calculates 
    /// center. Missing values are assumed to be 0.
    void ResizeDimension();

    /// Return the nearest point on the line segment of the capsule
    double NearestPoint(const std::vector<double>& _p) const noexcept;
    /// State the line segment length and normal
    void LineSegment() noexcept;
		
    ///@}
    ///@name Internal State
    ///@{

    std::vector<double> m_center;        ///< The center point of the capsule.
    std::vector<double> m_line;     ///< Unit vector of the line
    LineSegmentType m_segment; ///< The center line.
    double m_radius;              ///< The radius.
    double m_length;   ///< Length of the line 
    ///@}

};

/*----------------------------------------------------------------------------*/

#endif
