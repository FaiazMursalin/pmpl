#ifndef PLACE_PRIMITIVES_H_
#define PLACE_PRIMITIVES_H_

#include <vector>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <queue>

using namespace std;

#include "Vector.h"
using namespace mathtool;
#include "Workspace/WorkspaceSkeleton.h"
#include "Workspace/PropertyMap.h"
#include "Geometry/Shapes/NShape.h"
#include "Geometry/Shapes/NSphere.h"
#include "Geometry/Shapes/NEllipsoid.h"
#include "Geometry/Shapes/NCapsule.h"
#include "Geometry/Shapes/NOrientedBox.h"


////////////////////////////////////////////////////////////////////////////////
/// Constructs each candidate type of primitive
////////////////////////////////////////////////////////////////////////////////
class PlacePrimitives{
  public:
  ///@name Local types
  ///{
  typedef PropertyMap<NShape*,NShape*> PrimitiveStorageType;
  typedef vector<Point3d> SpokesVectorType;
  typedef WorkspaceSkeleton::GraphType SkeletonGraphType;
  typedef typename SkeletonGraphType::vertex_descriptor VD;
  typedef typename SkeletonGraphType::vertex_iterator VI;
  typedef typename SkeletonGraphType::edge_iterator EI;
  typedef typename SkeletonGraphType::adj_edge_iterator AEI;
    		
  ///@}
  ///@name Construction
  ///@{
  PlacePrimitives(WorkspaceSkeleton* _sk, 
    const PropertyMap<vector<pair<double,double>>, pair<double,double>>& _clMap,
    const PropertyMap<vector<SpokesVectorType>, SpokesVectorType>& _spMap, 
    double _dim) :
    m_skeleton(_sk), m_clearances(_clMap), m_spokes(_spMap), m_dimension(_dim) {}

  ~PlacePrimitives(){
     m_skeleton = nullptr;
  }

  ///@}
  ///@name Accessors
  ///@{
 
  void SetThresholds(double _l, double _c, double _g) {
    m_threshold =_g;
    m_lineThreshold = _l;
    m_overlapThreshold =_c;
  }
  
  void SetSkeletonType(bool _m = true){
	m_isMin = _m;
  }
  
  /// Place a capsule on an edge
  /// @param _ei Edge iterator
  /// @param _s, _t source and target pointer
  /// @param _r radius
  /// @return pair of Capsule pointer and their score(volume)
  pair<NCapsule*, double> PlaceCapsule(vector<double>&  _s, vector<double>& _t,const double& _r, double& _l);

  /// Place an ellipsoid on an edge
  /// @param _ei Edge iterator
  /// @param _c center
  /// @param _s, _t source and target pointer
  /// @param _r minimum clearance
  /// @return pair of Ellipsoid pointer and their score(volume)
  pair<NEllipsoid*, double> PlaceEllipsoid(EI& _ei, vector<double>&  _c, Point3d&  _s, Point3d& _t, double&  _r, const double& _vCl);

  /// Place an box on an edge
  /// @param _ei Edge iterator
  /// @param _c center
  /// @param _s, _t source and target pointer
  /// @param _r minimum clearance
  /// @return pair of OBB pointer and their score(volume)
  pair<NOrientedBox*, double> PlaceBox(EI& _ei, vector<double>&  _c, Point3d&  _s, Point3d& _t, double&  _r);
  /// Place a box on vertex
  pair<NOrientedBox*, double> PlaceBox(VI& _vi, vector<double>&  _c, Point3d&  _s, double&  _r);

  ///@}

 private:
   ///@name Helper Functions
   ///@{

   /// Find the third range for Ellipsoid
   /// @param _ei Edge iterator
   /// @param _r ranges
   /// @param _m Mid-point
   /// @param _i Index of the minimum 
   /// @return The angle of x-axis
   double FindThirdRangeEllipsoid(EI& _ei, Point3d& _s, Point3d& _t, vector<double>&  _r, size_t _i);
   /// Find the third range for box
   /// @param _ei Edge iterator
   /// @param _s, _t source and target pointer
   /// @param _r ranges
   /// @return The direction of y-axis
   double FindBoxThirdRange(EI& _ei, Point3d& _s, Point3d& _t, vector<double>&  _r, vector<double>&  _c);
   Vector3d FindBoxThirdRange(vector<Vector3d>& _spokeVectors, vector<double>&  _r, Vector3d& _xdir);

   /// Find the third range for box
   /// @param _v Set of vectors
   /// @param _yv, _yrange vector and range along y-axis
   /// @param _r ranges
   /// @return The distance of horizontal range
   double FindHorizontalRange(vector<Vector3d>& _v, Vector3d& _yv, double _yrange, double _err=0);
   double FindHorizontalRange(vector<Vector3d>& _v, Vector3d& _zv, Vector3d& _yv, double _yrange, double _zrange, double _err=0);

   /// Extend the ranges for box
   /// @param _ei Edge iterator
   /// @param _src source vertex descriptor
   /// @param _s, _t source & target vertex point
   /// @param _r radius of semi-circle
   /// @return The direction of y-axis
   double ExtendBoxRange(VD _src, Point3d& _s, Point3d& _t, double&  _r, double& _vr, size_t _dim=2, double _zr=0, Vector3d _ydir=Vector3d());

   /// Spokes Vector Helpers

   /// Get spokes vector of an edge or at particular index 
   /// such that all the vectors are normal to the line
   /// @param _s, _t source and target pointer
   /// @param _ei Edge iterator
   /// @param _index Intermediate index / default value: entire edge
   /// @return pair of start and end points for the spokes vector
   void GetSpokeVectors(EI& _ei, Point3d& _s, Point3d& _t, vector<pair<Vector3d,Vector3d>>& _spoke, size_t _index = numeric_limits<size_t>::max());

   /// Get spokes vector of an edge beyond the end vertex within a semi circle
   /// @param _s Starting end vertex
   /// @param _r semi circle radius
   /// @param _dir Direction of line
   void GetSpokeVectorsByRadius(VD _s, double _r, Vector3d _dir, vector<Vector3d>& _spoke);
   /// Get spokes vector of a vertex within a sphere
   void GetSpokeVectorsByRadius(VD _s, double _r, vector<Vector3d>& _spoke);
	 void GetSpokeVectorsByRadius(VD _src, double _r, const Point3d& _s, const Point3d& _t, vector<pair<Vector3d, Vector3d>>& _spoke);
   

   /// Euler Angles Helpers

   /// Get angle of rotation along z-axis/ of x-y plane
   /// @param _p1, _p2 Points in the vector
   /// @param _m Mid-point of the vector
   /// @return angle of rotation along z axis
   double GetZAngle(Point3d& _p1, Point3d& _p2, Point3d& _m);

   /// Get angle of rotation along y-axis /of x-z plane
   /// @param _p1, _p2 Points in the vector
   /// @param _m Mid-point of the vector
   /// @return angle of rotation along y axis / of x-z plane
   double GetYAngle(Point3d& _p1, Point3d& _p2, Point3d& _m);

   /// Get angle of rotation along x-axis /of y-z plane
   /// @param _p1, _p2 Points in the vector
   /// @param _m Mid-point of the vector
   /// @param _y unit vector along new-y axis
   /// @return angle of rotation along x axis / of y-z plane
   double GetXAngle(Point3d& _p1, Point3d& _p2, Point3d& _m, Vector3d& _y);

   ///@}
   ///@name Internal State
   ///@{
   WorkspaceSkeleton* m_skeleton; 
   PropertyMap<vector<pair<double,double>>,pair<double, double>> m_clearances;	///< Clearance property map associated with skeleton
   PropertyMap<vector<SpokesVectorType>, SpokesVectorType> m_spokes;	///< Spoke vectors property map associated with skeleton

   size_t m_dimension{3}; ///< Dimension of the problem or environment
   double m_threshold{0.0002};
   double m_lineThreshold{0.00}; ///< Threshold for line simplification
   double m_overlapThreshold{0.00};
   bool m_isMin{true};
   ///@}
}; 

/*------------------------------------------------------------------------*/

#endif


