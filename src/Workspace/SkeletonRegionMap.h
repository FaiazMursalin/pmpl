#ifndef SKELETON_REGION_MAP_H_
#define SKELETON_REGION_MAP_H_

#include <vector>
#include <set>
#include <map>
#include <unordered_map>

#include "ConfigurationSpace/Cfg.h"
#include "Geometry/Bodies/ActiveMultiBody.h"
#include "Geometry/Boundaries/Boundary.h"
#include "WorkspaceSkeleton.h"
#include "WorkspaceDecomposition.h"

#include <containers/sequential/graph/directed_preds_graph.h>
#include <boost/functional/hash.hpp>

#include "Vector.h"

using namespace mathtool;
using namespace std;

class ActiveMultiBody;
class Cfg;


////////////////////////////////////////////////////////////////////////////////
/// Relationship between workspace decomposition and workspace skeleton
////////////////////////////////////////////////////////////////////////////////
class SkeletonRegionMap {

  public:

    ///@name Local Types
    ///@{

    /// Graph type is the graph type for the skeleton.
    typedef typename WorkspaceSkeleton::GraphType GraphType;
    typedef typename GraphType::vertex_descriptor VD;
    typedef typename GraphType::edge_descriptor   ED;

		// Hash for edge descriptor
		struct edgeHash {
    	size_t operator()(const ED& _ed) const {
				size_t seed = 0;
				boost::hash_combine(seed,_ed.id());
				boost::hash_combine(seed,boost::hash_value(make_pair(_ed.source(),_ed.target())));
				return seed;
        //return hash<typename ED::edge_id_type>()(_ed.id());
    	}
		};

		// Hash for region and edge descriptor
		struct regionEdgeHash {
    	size_t operator()(const pair<size_t, ED>& _key) const {
				size_t seed = 0;
				boost::hash_combine(seed,_key.first);
				boost::hash_combine(seed,_key.second.id());
				boost::hash_combine(seed,boost::hash_value(make_pair(_key.second.source(),_key.second.target())));
				return seed;
    	}
		};
    
		///@}
		///@name Constructors
  	///@{
		SkeletonRegionMap(): m_decomposition(nullptr), m_skeleton(nullptr) {}
		
		SkeletonRegionMap(WorkspaceDecomposition* const _wd, 
											WorkspaceSkeleton* const _ws);
		
		/// Set the decomposition and compute the maps
		void SetWorkspaceDecomposition(WorkspaceDecomposition* const _wd);
		/// Set the skeleton and compute the maps
		void SetWorkspaceSkeleton(WorkspaceSkeleton* const _ws);

		/// Destructors
		~SkeletonRegionMap();

		///@}
		///@name Accessors
		///@{
    
		/// Get the skeleton vertex to region map
    const unordered_map<VD, size_t>& GetVertexRegionMap() const {
      return m_vertexToRegion;
    }
		/// Get the skeleton edge to region map
		const unordered_map<ED, vector<size_t>,edgeHash>& GetEdgeRegionMap() {
      return m_edgeToRegions;
    }

		/// Given a skeleton vertex, return the region the vertex belongs to
    const WorkspaceRegion& GetRegion(const VD& _v) {
      return m_decomposition->GetRegion(m_vertexToRegion[_v]);
    }

		/// Given a skeleton edge, return the regions it passes through
		vector<WorkspaceRegion> GetRegions(const ED& _e);
		/// Given a skeleton edge, return the _ith region it passes through
		const WorkspaceRegion& GetRegion(const ED& _e, size_t _i);
		/// Given a skeleton edge, return the number of regions it passes through
		size_t GetNumRegions(const ED& _e);
		
		/// Given a region, return the edges that passes through these points
		vector<VD> GetRegionVertices(size_t _i);

		/// Given a region, return the edges that passes through these points
		vector<ED> GetRegionEdge(size_t _i);

		/// Given a region and edge, return the intersecting edge  points
		vector<Point3d> GetEdgePoints(size_t _i, const ED& _e);

		/// Given a region, return the edges as a discretized set of points
		vector<pair<ED,vector<Point3d>>> GetRegionEdgePoints(size_t _i);

		/// Given a edge, return the regions and the edge points passing through it
		vector<pair<size_t,vector<Point3d>>> GetRegionEdgePoints(const ED& _e);

    ///@}

  private:

    ///@name Helpers
    ///@{
		/// Given a 3d workspace point finds the workspace region it belongs to 
		const size_t FindDecompositionRegion(const Point3d& _p);

		/// Create the maps from skeleton to workspace decomposition regions
		void CreateMaps();

		/// Create the maps from skeleton vertices to workspace decomposition regions
		void CreateVertexMaps();

		/// Create the maps from skeleton edges to workspace decomposition regions
		void CreateEdgeMaps();

		/// Find regions for a given skeleton edge
		void FindEdgeRegions(GraphType::const_edge_iterator& _ei);
	  
    ///@}
    ///@name Internal State
    ///@{

		WorkspaceDecomposition* m_decomposition;	///< Workspace decomposition
		WorkspaceSkeleton* m_skeleton;	///< Workspace skeleton

    unordered_map<VD, size_t> m_vertexToRegion; ///< Map from skeleton vertex to workspace decomposition region.
    unordered_map<ED, vector<size_t>, edgeHash> m_edgeToRegions; ///< Map from skeleton edge to workspace decomposition regions.

		unordered_map<size_t, vector<VD>> m_regionToVertices;	///< Map from workspace decomposition region to skeleton vertices.
		unordered_map<size_t, vector<ED>> m_regionToEdges;	///< Map from workspace decomposition region to skeleton edges.

		unordered_map<pair<size_t, ED>, vector<pair<Point3d, Point3d>>, regionEdgeHash> m_edgeRegionIntersections; ///< Map storing the intersection points of region and skeleton edge

    bool m_debug{false};  ///< Show debug messages

    ///@}
};

#endif
