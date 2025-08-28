#ifndef ELLIPSOID_TREES_H_
#define ELLIPSOID_TREES_H_

#include <memory>
#include <vector>
#include <limits>
using namespace std;

#include "Vector.h"
using namespace mathtool;
#include "IOUtils.h"
#include "ConfigurationSpace/Cfg.h"
#include "MPLibrary/MPBaseObject.h"
#include "Geometry/Boundaries/Boundary.h"
#include "Geometry/Boundaries/WorkspaceBoundingBox.h"
#include "Geometry/Bodies/ActiveMultiBody.h"
#include "Workspace/WorkspaceSkeleton.h"
#include "Workspace/WorkspaceDecomposition.h"
#include "Workspace/PropertyMap.h"
#include "Geometry/Shapes/NSphere.h"
#include "Geometry/Shapes/NEllipsoid.h"
#include "Utilities/MedialAxis2D.h"
#include "MPLibrary/MPTools/ReebGraphConstruction.h"

class Environment;

////////////////////////////////////////////////////////////////////////////////
/// Inner Ellipsoid Tree
////////////////////////////////////////////////////////////////////////////////
class EllipsoidTrees {

  public:
    ///@name Local types
    ///{

		struct reebEdgeHash{
			size_t operator()(const ReebGraphConstruction::RGEID&  _ed) const {
    		return hash<typename ReebGraphConstruction::RGEID::edge_id_type>()(_ed.id());
  		}
    };		
		typedef unordered_map<ReebGraphConstruction::RGEID, size_t, reebEdgeHash> ReebEdgeRegionMapType;
		typedef PropertyMap<NEllipsoid,NSphere> EllipsoidStorageType;
		
    ///@}
    ///@name Construction
    ///@{

    EllipsoidTrees(Environment* _e, size_t _d = 2);

    template<typename MPTraits>
    EllipsoidTrees(MPBaseObject<MPTraits>* _mp, double _ep, size_t _d=3);

    ~EllipsoidTrees();

    ///@}
    ///@name Modifiers
    ///@{

    /// Build the Ellipsoid Trees
    void BuildEllipsoidTree(double _epsilon = numeric_limits<float>::epsilon(), double _th = numeric_limits<float>::epsilon() );

    ///@}
    ///@name Accessors
    ///@{

    /// Get the number of ellipsoids in the tree
    /// @return Number of output ellipsoids
    size_t GetNumberEllipsoids() {
      return m_ellipsoids.GetEdgeMap().size(); 
    }
		/// Check whether ellipsoid is there on the edge
    /// @param _ed Edge descriptor of edge in resulting skeleton
    /// @return true/false if ellipsoid is there on edge _ed
    bool IsEllipsoid(const WorkspaceSkeleton::ED& _ed)	{
			return m_ellipsoids.IsEdgePropertyAssigned(_ed);
    }
		/// Check whether sphere is there on the vertex
    /// @param _vd Vertex descriptor of vertex in resulting skeleton
    /// @return true/false if sphere is there on vertex _vd
    bool IsSphere(const WorkspaceSkeleton::VD& _vd)	{
			return m_ellipsoids.IsVertexPropertyAssigned(_vd);
    }

    /// Get the output ellipsoid by edge descriptor
    /// @param _ed Edge descriptor of edge in resulting skeleton
    /// @return Output ellipsoid at edge _ed
    const NEllipsoid& GetEllipsoid(const WorkspaceSkeleton::ED& _ed)	{
      return m_ellipsoids.GetEdgeProperty(_ed);
    }

    /// Get the number of spheres in the tree
    /// @return Number of output spheres
    size_t GetNumberSpheres() {
      return m_ellipsoids.GetVertexMap().size();
    }

    /// Get the output sphere by index
    /// @param _vd Vertex descriptor of vertexin resulting skeleton
    /// @return Output ellipsoid at vertex _vd
    const NSphere& GetSphere(const WorkspaceSkeleton::VD& _vd) {
      return m_ellipsoids.GetVertexProperty(_vd);
    }

		/// Get the Ellipsoid Map
		EllipsoidStorageType::EdgeMapType& GetEllipsoidMap() { return m_ellipsoids.GetEdgeMap(); } 

		/// Get the Sphere Map
		EllipsoidStorageType::VertexMapType& GetSphereMap() { return m_ellipsoids.GetVertexMap(); } 

    ///  Get the dimension of the problem
    ///  @return dimension 
    size_t GetDimension() { return m_dimension;	}

    ///  Set the dimension of the problem
    ///  @param _d dimension
    void SetDimension(size_t _d = 3) {	m_dimension = _d; }

    ///  Get the skeleton
    WorkspaceSkeleton& GetSkeleton() { return m_skeleton; }

		vector<Boundary*>& GetRegions() { return m_regions; }

		void GetRegionMap(unordered_map<size_t, vector<WorkspaceSkeleton::ED>>& _rMap);

		unordered_map<WorkspaceSkeleton::VD, WorkspaceSkeleton::VD>& GetConcentricSphereMap() { return m_vertexToSphereVertexMap; }

		friend ostream& operator<<(ostream& _os, EllipsoidTrees& _et);

    ///@}

  private:
    ///@name Helper Functions
    ///@{
	
    /// Simplify the skeleton with only straight line segment edges
    void SimplifySkeleton(double _ep = numeric_limits<double>::epsilon());
    
    /// Simplify a line 
		void SimplifyLine(double _ep, vector<Point3d>& _pts, vector<vector<Point3d>>& _r);

    /// Build the spheres for the vertices in the skeleton graph
    void BuildSpheres(double _th = numeric_limits<double>::epsilon());

    /// Build the ellipsoids for the edges in the skeleton graph
    void BuildEllipsoids(double _th = numeric_limits<double>::epsilon());

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

		WorkspaceBoundingBox* GetBoundingBox(vector<Point3d>& _pts);

		void FloodRegions(ReebGraphConstruction::ReebGraph* _rg, WorkspaceDecomposition* _wd, ReebEdgeRegionMapType& _reToBoxMap);

		pair<bool, WorkspaceSkeleton::VD> IsNearlyConcentricSpheres(vector<double>& _c, double _r, double _t = numeric_limits<float>::epsilon());
		
    ///@}
    ///@name Internal State
    ///@{

    WorkspaceSkeleton m_skeleton; ///< Workspace Skeleton 

    PropertyMap<vector<double>,double> m_annotation;	///< Property map associated with skeleton

    PropertyMap<NEllipsoid,NSphere> m_ellipsoids; ///< Output set of ellipsoids

		vector<Boundary*> m_regions;
   	unordered_map<WorkspaceSkeleton::ED, size_t, edgeHash> m_regionMap;
		unordered_map<WorkspaceSkeleton::VD, WorkspaceSkeleton::VD> m_vertexToSphereVertexMap;
	
    size_t m_dimension{3}; ///< Dimension of the problem or environment

    bool m_debug{false};         ///< Toggle debug messages.

    ///@}
};


/*------------------------------------------------------------------------*/

template<typename MPTraits>
EllipsoidTrees::
EllipsoidTrees(MPBaseObject<MPTraits>* _mp, double _ep, size_t _d) : m_dimension(_d) {
  // Construct the skeleton from reeb graph
  ReebGraphConstruction* reeb = new ReebGraphConstruction();
  reeb->Construct(_mp->GetEnvironment());
  //m_skeleton = reeb->GetSkeleton();
  if(m_debug)
    cout<<"Created Reeb graph"<<endl;
  // Push the skeleton vertices and edge points to medial axis
  //auto mau = _mp->GetMPLibrary()->GetMPTools()->GetMedialAxisUtility("maUtil");
  //auto boundary = _mp->GetEnvironment()->GetBoundary();
  //auto pointRobot = _mp->GetMPProblem()->GetRobot("point");
  WorkspaceSkeleton::GraphType& g = m_skeleton.GetGraph();

  // Function to push point _p to Medial Axis.
  /*auto pushToMA = [&](const Point3d& _p) -> Point3d {
    typename MPTraits::CfgType cfg(_p, pointRobot);
    if (mau->PushToMedialAxis(cfg, boundary))
      return cfg.GetPoint();
    else
      return _p;
  };*/
  // Insert vertices.
  auto rg = reeb->GetReebGraph();
	ReebEdgeRegionMapType rmap;
	FloodRegions(&rg, _mp->GetEnvironment()->GetDecomposition(), rmap);
  for (auto vit = rg.begin(); vit != rg.end(); ++vit)
    g.add_vertex(vit->descriptor(), /*pushToMA(*/vit->property().m_vertex/*)*/);

  // Simplify edges
	/*vector<vector<Point3d>> paths;
	typedef typename WorkspaceSkeleton::ED ED;
	vector<ED> eds;
	vector<size_t> regions;
	for (auto eit = rg.edges_begin(); eit != rg.edges_end(); ++eit) {
    // Make sure this is a real path.
    if (!eit->property().m_path.empty()) {
      vector<Point3d> path;
      for (auto p : eit->property().m_path)
        path.push_back(p); //path.push_back(pushToMA(p));
			auto bs = paths.size();
      SimplifyLine(_ep, path, paths);
			auto as = paths.size();
			auto s = eit->source();
			size_t r = m_regions.size();
			auto rit = rmap.find(eit->descriptor());
			if(rit != rmap.end())
				r = rit->second; 
			for(size_t i = bs; i < as; i++)	{
				auto t = eit->target();
				if(as - i > 1) 
         t = g.add_vertex(paths[i].back()); 
				eds.push_back(ED(s, t));
				regions.push_back(r);
				s = t;
      }
    }
  }
	// Insert edges
	for(size_t i = 0; i < eds.size(); i++) {
		auto ed = g.add_edge(eds[i], paths[i]);
		if(regions[i] < m_regions.size())
			m_regionMap.emplace(ed, regions[i]);
  }*/
  // Insert edges.
  for (auto eit = rg.edges_begin(); eit != rg.edges_end(); ++eit) {
    // Make sure this is a real path.
    if (!eit->property().m_path.empty()) {
      vector<Point3d> path;
      for (auto p : eit->property().m_path)
        path.push_back(p);//path.push_back(pushToMA(p));
      auto ed = g.add_edge(eit->descriptor(), path);
			auto rit = rmap.find(eit->descriptor());
			if(rit != rmap.end())
				m_regionMap.emplace(ed, rit->second);
    }
  }

    cout<<"Created Skeleton"<<endl;  
  // Have the skeleton clearance annotated
  m_annotation = *(ClearanceAnnotatedSkeleton(_mp, &m_skeleton));
  if(m_debug)
    cout<<m_annotation.GetVertexMap().size() << " "<<m_annotation.GetEdgeMap().size()<<endl;
}
/*--------------------------------------------------------------------------*/
#endif
