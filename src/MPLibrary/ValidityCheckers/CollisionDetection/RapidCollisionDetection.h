#ifndef RAPID_COLLISION_DETECTION_H_
#define RAPID_COLLISION_DETECTION_H_

#include "CollisionDetectionMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// @ingroup CollisionDetection
/// RAPID is typically used to simply and quickly determine collision between
/// two objects, but cannot compute any distance information. The contacts
/// found, i.e., triangle IDs are stored in CDInfo.
////////////////////////////////////////////////////////////////////////////////
class Rapid: public CollisionDetectionMethod {

  public:

    Rapid(bool _solid = false);

    static void Build(Body* const _body);

    virtual bool IsInCollision(const Body* const _body1,
        const Body* const _body2, CDInfo& _cdInfo, bool _cache=true);

    /// Shoot a pseudo-ray outward from a reference point to determine if it
    /// lies within a given body.
    /// @param[in] _pt The reference point of interest.
    /// @param[in] _body The body to check against.
    /// @return True if _pt is inside _body.
    virtual bool IsInsideObstacle(const Vector3d& _pt, const Body* const _body, bool _cache=true)
        override;

    ///@}

    private:
      bool m_isSolid; ///< To check for surface or solid check

 };

#endif
