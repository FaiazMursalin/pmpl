#ifndef SCORE_SKELETON_H_
#define SCORE_SKELETON_H_

#include <memory>
#include <vector>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <boost/heap/fibonacci_heap.hpp>
using namespace std;

#include "Vector.h"
using namespace mathtool;
#include "IOUtils.h"
#include "Geometry/Boundaries/Boundary.h"
#include "Geometry/Boundaries/WorkspaceBoundingBox.h"
#include "Workspace/WorkspaceSkeleton.h"
#include "Workspace/WorkspaceDecomposition.h"
#include "Workspace/PropertyMap.h"
#include "Workspace/GridOverlay.h"
#include "Geometry/Shapes/NShape.h"

////////////////////////////////////////////////////////////////////////////////
/// Abstract Skeleton Score 
////////////////////////////////////////////////////////////////////////////////
class SkeletonScore {
public: 
	///@name Local types
	///{
	typedef pair<double, size_t> PQElementType;
	typedef PropertyMap<vector<pair<double,double>>, pair<double,double>> ClearanceMapType;
	typedef WorkspaceSkeleton::GraphType SkeletonGraphType;
	typedef typename SkeletonGraphType::vertex_descriptor VD;
	typedef typename SkeletonGraphType::vertex_iterator VI;
	typedef typename SkeletonGraphType::edge_iterator EI;
	typedef typename SkeletonGraphType::adj_edge_iterator AEI;

	SkeletonScore() {}
	virtual ~SkeletonScore() {}
	
	virtual void InitializeScore(SkeletonGraphType& _g, ClearanceMapType& _cmap) = 0;
    virtual size_t GetScore(NShape* _s) = 0;
	virtual size_t UpdateScore(NShape* _s) = 0;
	virtual bool IsEmpty() = 0;
	virtual pair<size_t, char> PopNext() = 0;
	///@}
protected: 
	///@name Helper Functions
	///@{
	 
	pair<size_t, char> GetTopIndex(PQElementType& _t);

	///@}
	///@name Internal State
	///@{
	size_t m_vertexStartIndex{ 0 };
	size_t m_edgeStartIndex{ 0 };
	///@}
};

////////////////////////////////////////////////////////////////////////////////
/// Basic Skeleton Score by Property (Non adaptable PQ)
////////////////////////////////////////////////////////////////////////////////
class BasicPropertyScoring : public SkeletonScore {

public:
	///@name Local types
	///{
	typedef pair<double, size_t> PQElementType;
	// Using vector as total insert will be nlogn (same as heap) 
	// but deletion can be constant compared to logn for heap
	typedef vector<PQElementType> PQType;
	

	///@}
	///@name Construction
	///@{

	BasicPropertyScoring() : SkeletonScore() {}

	~BasicPropertyScoring() { m_heap.clear(); }

	///@}
	///@name Modifiers
	///@{

	/// Initialize score
	/// Initializes the score of the skeletal elements 
	/// based on the approximation of occupancy through the clearance
	/// @param _g Skeleton graph 
	/// @param _cmap Clearance Map
	void InitializeScore(SkeletonGraphType& _g, ClearanceMapType& _cmap);

	/// Update the score  
	/// Empty as the PQ is non-adaptable
	/// @param _s Primitive Shape inserted recently
	size_t UpdateScore(NShape* _s) { return 1;}
  size_t GetScore(NShape* _s) { return 1; }
	///@}
	///@name Accessors
	///@{
	
	///  Get the skeletal element of highest priority
	///  @return pair.first distance from skeleton elements begin 
	///  @return pair.second skeletal element type: v vertex, e edge
	pair<size_t, char> PopNext();

	///  Whether no elements in PQ
	bool IsEmpty() { return m_heap.empty(); }

	///@}

private:
	///@name Helper Functions
	///@{

	
	///@}
	///@name Internal State
	///@{
	PQType m_heap; ///< Priority queue storing score and id
	///@}
};

////////////////////////////////////////////////////////////////////////////////
/// Primitives Skeleton Occupancy Score
////////////////////////////////////////////////////////////////////////////////
class PrimitiveOccupancyScoring : public SkeletonScore {

  public:
    ///@name Local types
    ///{
    struct comparePQElements{
      bool operator()(const PQElementType&  _e1, const PQElementType&  _e2) const {
        return ((_e1.first < _e2.first)||
					(_e1.first == _e2.first && _e1.second < _e2.second));
      }
    };		
    typedef boost::heap::fibonacci_heap<PQElementType, boost::heap::compare<comparePQElements>> PQType;
    typedef typename PQType::handle_type PQHandle;
        		
    ///@}
    ///@name Construction
    ///@{
		PrimitiveOccupancyScoring() : SkeletonScore() {}
    PrimitiveOccupancyScoring(const Boundary* const _b, const double _s);

    ~PrimitiveOccupancyScoring();

    ///@}
    ///@name Modifiers
    ///@{

    /// Initialize score
    /// Initializes the score of the skeletal elements 
    /// based on the approximation of occupancy through the clearance
    /// @param _g Skeleton graph 
    /// @param _cmap Clearance Map
    void InitializeScore(SkeletonGraphType& _g, ClearanceMapType& _cmap);

    /// Update the score
    /// Updates the score of the adjacent skeletal elements due to placement of 
    /// primitive shape _s
    /// @param _s Primitive Shape inserted recently
    size_t UpdateScore(NShape* _s);
    size_t GetScore(NShape* _s);
    ///@}
    ///@name Accessors
    ///@{
    
    ///  Get the underlying occupancy grid
    ///  @return dimension 
    OccupancyGrid* GetOccupancyGrid() { return m_grid;	}

    ///  Set the underlying occupancy grid
    ///  @param _b Boundary box
    ///  @param _s Step length
    void SetOccupancyGrid(const Boundary* const _b, const double _s);
    
    ///  Get the skeletal element of highest priority
    ///  @return pair.first distance from skeleton elements begin 
    ///  @return pair.second skeletal element type: v vertex, e edge
    pair<size_t, char> PopNext();

    ///  Whether no elements in PQ or cannot pop
    bool IsEmpty() { return m_heap.empty() || (m_heap.top().first <= m_threshold); }

    ///@}

  private:
    ///@name Helper Functions
    ///@{
    
    /// Place AABB in segment tree
    /// This places a skeltal AABB in spatial index tree
    /// @param _i Skelton element index
    /// @param _ep Extreme points of the bounding boxes
    void SetSpatialIndexing(size_t _i, pair<Point3d,Point3d>& _ep);
    /// Populate the voxels which are at _d distance away from a given point
    /// @param _pt center
    /// @param _d distance 
    /// @param _vs Storage of voxels
    void PopulateVoxels(Point3d& _pt, double& _d, unordered_set<size_t>& _vs);


    void PopulateVoxels(vector<Point3d>& _pts, double& _d, unordered_set<size_t>& _vs, pair<Point3d, Point3d>& _r);

    ///@}
    ///@name Internal State
    ///@{
    size_t m_threshold{0};
    PQType m_heap; ///< Priority queue storing score and id
    unordered_map<size_t, PQHandle> m_handleMap;///< Map of skeletal element id to PQ handle
    unordered_map<size_t, unordered_set<size_t>> m_voxelMap;///< Map of skeletal element id to voxels ids
    vector<Boundary*> m_boundaries; ///< Set of boundaries
    OccupancyGrid* m_grid{nullptr}; ///< Occupancy grid
    SegmentTrees<size_t>* m_location{nullptr};	///< Segment Trees of AABB of skeletal elements regions for fast access
    ///@}
};


/*------------------------------------------------------------------------*/

#endif


