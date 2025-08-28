#include "RapidCollisionDetection.h"

#ifndef NO_RAPID

#include <RAPID.H>

#include "CDInfo.h"
#include "Geometry/Bodies/Body.h"


/*------------------------------- Construction -------------------------------*/

Rapid::
Rapid(bool _solid) : CollisionDetectionMethod("RAPID"), m_isSolid(_solid) { }

/*------------------------------- CD Interface -------------------------------*/

void
Rapid::
Build(Body* const _body) {
  const GMSPolyhedron& poly = _body->GetPolyhedron();
  unique_ptr<RAPID_model> rapidBody(new RAPID_model);
  rapidBody->BeginModel();
  for(size_t q = 0; q < poly.m_polygonList.size(); q++) {
    double point[3][3];
    for(int i = 0; i < 3; i++) {
      const Vector3d& tmp = poly.m_polygonList[q].GetPoint(i);
      for(int j = 0; j < 3; j++)
        point[i][j] = tmp[j];
    }
    rapidBody->AddTri(point[0], point[1], point[2], q);
  }
  rapidBody->EndModel();
  _body->SetRapidBody(move(rapidBody));
}


bool
Rapid::
IsInCollision(const Body* const _body1, const Body* const _body2,
    CDInfo& _cdInfo, bool _cache) {

  auto body1 = _body1->GetRapidBody();
  auto body2 = _body2->GetRapidBody();
  /// @TODO See if we can modify RAPID_Collide to take const double arrays
  ///       instead of just double arrays so we don't have to copy.
  //const Transformation& t1 = _body1->GetWorldTransformation();
  //const Transformation& t2 = _body2->GetWorldTransformation();
  Transformation t1 = _body1->GetWorldTransformation();
  Transformation t2 = _body2->GetWorldTransformation();

  if(RAPID_Collide(
        t1.rotation().matrix(), t1.translation(), body1,
        t2.rotation().matrix(), t2.translation(), body2,
        RAPID_FIRST_CONTACT))
    throw RunTimeException(WHERE, "RAPID_ERR_COLLIDE_OUT_OF_MEMORY");

  if(RAPID_num_contacts) {
    _cdInfo.m_rapidContactID1 = RAPID_contact[0].id1;
    _cdInfo.m_rapidContactID2 = RAPID_contact[0].id2;
    return true;
  }

  if(m_isSolid) {
    return IsInsideObstacle(_body1->GetPolyhedron().m_vertexList[0]*t1,
                            _body2);
  }
  return false;
}

bool
Rapid::
IsInsideObstacle(const Vector3d& _pt, const Body* const _body, bool _cache) {
  // Set up a pseudo-ray for a ray-shooting test.
  double p1[3] = {0, 0, 0};
  double p2[3] = {0, 1e-10, 0};
  double x[3] = {1e10, 0, 0};
  RAPID_model* ray = new RAPID_model();
  ray->BeginModel();
  ray->AddTri(p1, p2, x, 0);
  ray->EndModel();
  static double rotation[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  double translation[3] = {_pt[0], _pt[1], _pt[2]};

  // Get obstacle info.
  auto body = _body->GetRapidBody();
  /// @TODO See if we can modify RAPID_Collide to take const double arrays
  ///       instead of just double arrays so we don't have to copy.
  Transformation t2 = _body->GetWorldTransformation();

  // Perform ray-shooting collision test.
  RAPID_Collide(rotation, translation, ray,
      t2.rotation().matrix(), t2.translation(), body);

  // Sort collisions by relative X-value.

  static const double tolerance = 10 * numeric_limits<double>::epsilon();
  static const Vector3d r(10e6, 0, 0); // Vector-representation of the ray.
  const auto& vertices = _body->GetPolyhedron().GetVertexList();
  const auto& polygons = _body->GetPolyhedron().GetPolygonList();

  enum TransitionType {Entering = 0, Exiting = 1};
  typedef pair<double, TransitionType> Transition;

  // We will store the processed collisions in a set to sort them and remove
  // duplicate transitions of the same type and x-value. Duplicate removal guards
  // against the case where the psuedo-ray passes through an edge or vertex that
  // is shared by multiple triangles with the same facing.
  static auto compare = [](const Transition& _t1, const Transition& _t2) -> bool {
    if(abs(_t1.first - _t2.first) > tolerance)
      return _t1.first < _t2.first;
    else
      return _t1.second < _t2.second;
  };
  static set<Transition, decltype(compare)> collisions(compare);

  // Process each collision.
  collisions.clear();
  for(int i = 0; i < RAPID_num_contacts; ++i) {
    const auto& triangle = polygons[RAPID_contact[i].id2];
    const auto& v = vertices[triangle[0]] * t2;
    const auto& n = (triangle.GetNormal() * t2.rotation()).normalize();

    // Skip collisions against triangles whose normals are perpendicular to the
    // ray: these are scrapes and don't affect inside/outside-ness.
    if(abs(n[0]) < tolerance) continue;

    // The collision occurs at some fraction of r. This fraction is the ratio of
    // |pt to the triangle plane| over |r's projection along n|.
    const double alpha = ((v - _pt) * n) / (r * n);

    // We are exiting the triangle if the normal has positive x value and
    // entering it otherwise (zero x values handled above).
    collisions.emplace(alpha * r[0], TransitionType(n[0] > 0));
  }

  // Check the ordered collisions to see what happened. Skip over any points
  // where we enter and exit at the same point.
  while(!collisions.empty()) {
    if(collisions.size() == 1 ||
        abs(collisions.begin()->first - (++collisions.begin())->first) >
        tolerance)
      return collisions.begin()->second;
    else
      collisions.erase(collisions.begin(), ++++collisions.begin());
  }

  // If we're still here, there are no valid collisions.
  return false;
}

/*----------------------------------------------------------------------------*/

#endif
