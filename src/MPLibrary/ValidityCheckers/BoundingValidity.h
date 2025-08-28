#ifndef BOUNDING_VALIDITY_H
#define BOUNDING_VALIDITY_H

#include <unordered_map>
#include <algorithm> 
#include <tuple>
#include <fstream>

#include "ValidityCheckerMethod.h"
#include "ValidityCheckerFunctor.h"
//#include "Utilities/AggregationHierarchyDecomposition.h"
#include "Utilities/SegmentTrees.h"
#include "Utilities/PrimitiveSkeleton.h"
#include "Workspace/WorkspaceSkeleton.h"
#include "Geometry/GMSPolyhedron.h"
#undef PI
#include <vector>
#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/boost/graph/convert_nef_polyhedron_to_polygon_mesh.h>
#include <boost/function_output_iterator.hpp>
#include <CGAL/IO/Nef_polyhedron_iostream_3.h>
#include <CGAL/Nef_3/SNC_indexed_items.h>
#include <CGAL/convex_decomposition_3.h> 

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ValidityCheckers
/// @brief A wrapper for validity checker that uses workspace decomposition or
///				bounding volumes to do quick check on validity of the configuration
/// @tparam MPTraits Motion planning universe
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class BoundingValidity : public ValidityCheckerMethod<MPTraits> {
  public:
    /// @name  Local Types
    /// @{
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPLibrary::ValidityCheckerPointer ValidityCheckerPointer;
    /// @}
    /// @name Contructions & Destructions
    /// @{
    BoundingValidity(string _vcLabel = "");
    BoundingValidity(XMLNode& _node);
    virtual ~BoundingValidity() {}
    /// @}
    /// @name Accessors
    /// @{
    virtual bool IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo,
			     const string& _callName);
    /// @}
		
  protected: 
    /// @name  Helper Functions
    /// @{
    /// @brief Initialize the skeleton, segment trees, robot parameters
    void InitializePrimitives();
    /// @brief Initializa the primitive skeleton 
    void InitializePrimitiveSkeleton();
    /// 
    void DecomposePolyhedron();
		
    /// Collision check routines
    bool SingleBodyCollision(const CfgType& _cfg, CDInfo& _cdInfo, 
      const string& _callName);
    bool MultiBodyCollision(const CfgType& _cfg, 
      CDInfo& _cdInfo, const string& _callName);

    bool CollisionWithObstacle(FreeBody* _body, CDInfo& _cdInfo);
    bool IsInSelfCollision(CollisionDetectionMethod* _cd, ActiveMultiBody* _rob,
	    CDInfo& _cdInfo, const string& _callName);
 
    /// @brief Given a point find in which space it is guranteed to be contained
    /// @param[in] _p The point co-ordinates.
    /// @param[in] _r Radius
    /// @return If inside any primitive (0 if not, 1 -if free space, 2 - if obstacle space)
    size_t SpaceContainmentCheck(const Point3d& _p, const double& _r);
    /// @brief Given workspace regions and a point find its clearance in the 
    ///        region (diastance from the boundaries of the region)
    /// @param[in] _p The point co-ordinates.
    /// @param[in] _vi Region vertex ids
    /// @param[in] _r Radius
    /// @return If inside any primitive (0 if not, 1 -if free space, 2 - if obstacle space)
    size_t DeepCheck(const vector<size_t>& _vi, const Point3d& _p, const double& _r);

    /// @brief Given a edge descriptor and point, check if the point is in 
    /// edge primitive
    /// @param[in] _ed vertex descriptor
    /// @param[in] _p The point co-ordinates.
    /// @param[in] _r Radius
    /// @param[in] _space In which space to check
    /// @return If inside any primitive 
    bool IsOutsidePrimitive(WorkspaceSkeleton::ED& _ed, const Vector3d& _p,
     const double& _r, const size_t& _space);
          
  private:
    ///@}
    ///@name Internal State
    ///@{
		// Robot and input parameters
    string m_vcLabel;					///< Label for underlying validity checker
    size_t m_dimension; 			///< Dimension of the problem
    vector<Vector3d> m_centers; ///< Centers of the multibody
    vector<double> m_radii; ///< Radii of the multibody
    vector<vector<Vector3d>> m_boxes; ///< Bounding box points of each body in multibody
    CollisionDetectionValidity<MPTraits>* m_vcMethod {nullptr};
    bool m_ignoreSelfCollision{false};

		// Data structures for containment check
    SegmentTrees<>* m_segmentTree{nullptr};	///< Segment Trees of bounding volume for faster location
    PrimitiveSkeleton* m_freePrimitives{nullptr}; ///< Primitive Skeleton of the free space
    PrimitiveSkeleton* m_obstPrimitives{nullptr}; ///< Primitive Skeleton of the obstacle space
    vector<Boundary*> m_freeRegions; ///< Bounded regions for faster location
    unordered_map<size_t, vector<WorkspaceSkeleton::ED>> m_freeRegionEdgeMap; ///< Map of the edges passing through the edge
    vector<Boundary*> m_obstRegions; ///< Bounded regions for faster location
    unordered_map<size_t, vector<WorkspaceSkeleton::ED>> m_obstRegionEdgeMap; ///< Map of the edges passing through the edge

    double m_lineThreshold;  ///< Threshold to simplify the curve skeleton
    double m_circleThreshold;  ///< Threshold for near concentric circles
    size_t m_space{0};  ///< Space in which skeletons are generated: 0- both, 1 - free, 2 - obstacle
    size_t m_score{0};
    bool m_multi{false};  /// Type for multilink: 0- each step separate for each body, 1 - all step for each body, 2 - normal & partial together for each body
    bool m_decomposition{false};  ///< External decomposition is provided
    bool m_partial{false};
    MeanCurvatureParams m_freeParameters, m_obstParameters; ///< MCS parameters
    string m_freeFile, m_obstFile; 
    ///@}
};

/*********************** Construction *****************************/

template<class MPTraits>
BoundingValidity<MPTraits>::
BoundingValidity(string _vcLabel) : ValidityCheckerMethod<MPTraits>(), 
  m_vcLabel(_vcLabel), m_dimension(2), 
  m_segmentTree(nullptr), m_freePrimitives(nullptr), 
  m_obstPrimitives(nullptr) {
  this->SetName("BoundingValidity");
  if(!m_vcLabel.empty()){
    m_vcMethod = static_cast<CollisionDetectionValidity<MPTraits>*>(this->GetValidityChecker(m_vcLabel).get());
    m_ignoreSelfCollision = m_vcMethod->GetIgnoreSelfCollision();
  }

  //InitializePrimitives();
}

template<class MPTraits>
BoundingValidity<MPTraits>::
BoundingValidity(XMLNode& _node) :
ValidityCheckerMethod<MPTraits>(_node) {
  this->SetName("BoundingValidity");
  this->m_vcLabel = _node.Read("vcLabel", true, "", "validity checker method");
  this->m_lineThreshold = _node.Read("simplification", false, 0.1, 0.0, MAX_DBL,
    "Threshold for line simplication");
  this->m_circleThreshold = _node.Read("concentric", false, 0.5, 0.0, MAX_DBL,
    "Threshold for concentric sphere placemnet");
  this->m_space = _node.Read("space", false, 0, 0, MAX_INT, 
    "Both/free/obstacle space wrapper");
  this->m_score = _node.Read("score", false, 0, 0, MAX_INT, "Score function to use");
  this->m_multi = _node.Read("multi", false, false, 
    "Multi Link Checker type(0,1,2)");
  this->m_decomposition = _node.Read("decomposition", false, false, 
    "External complete decomposition provided");
  this->m_freeParameters.m_iterations = _node.Read("fIteration", false, 1000,
    0, MAX_INT, "Number of iteration");
  this->m_freeParameters.m_wH = _node.Read("fQuality", false, 0.1, 0.0, 
    MAX_DBL, "Quality Parameter");
  this->m_freeParameters.m_wM = _node.Read("fMedial", false, 0.2, 0.0,
    MAX_DBL, "Medial parameter");
  this->m_obstParameters.m_iterations = _node.Read("oIteration", false, 1000,
    0, MAX_INT, "Number of iteration");
  this->m_obstParameters.m_wH = _node.Read("oQuality", false, 0.1, 0.0,
    MAX_DBL, "Quality Parameter");
  this->m_obstParameters.m_wM = _node.Read("oMedial", false, 0.2, 0.0,
    MAX_DBL, "Medial parameter");
  this->m_partial = _node.Read("partial", false, false, 
    "Partial Check");
  this->m_freeFile = _node.Read("fInput", false, "",
      "filename for input files free space");
  this->m_obstFile = _node.Read("oInput", false, "",
      "filename for input files obstacle space");
  
  this->m_freePrimitives = nullptr;
  this->m_obstPrimitives = nullptr;
  //InitializePrimitives();
}
/************* Construction Helpers *************************************/
template<class MPTraits>
void
BoundingValidity<MPTraits>::InitializePrimitives()	{
  //string clockName = this->GetNameAndLabel()+"::Validity Initialization";
  //this->GetStatClass()->StartClock(clockName);
  //auto stat = this->GetStatClass();
  this->m_vcMethod = static_cast<CollisionDetectionValidity<MPTraits>*>(this->GetValidityChecker(m_vcLabel).get());
  m_ignoreSelfCollision = m_vcMethod->GetIgnoreSelfCollision();

  // Find dimension
  m_dimension = this->GetEnvironment()->GetBoundary()->GetDimension();
  // Find the centers, radius of each body in multibody
  auto rob = this->GetTask()->GetRobot()->GetMultiBody();
  for(size_t i = 0; i < rob->NumFreeBody(); i++){
    m_centers.push_back(rob->GetFreeBody(i)->GetPolyhedron().GetCentroid());
    m_radii.push_back(rob->GetFreeBody(i)->GetBoundingSphereRadius());
    m_boxes.push_back(rob->GetFreeBody(i)->GetBoundingBox().GetVertexList());
    // If the dimension is less than 3
    if(m_dimension < 3) {
      size_t indices[] = {7, 5, 3, 1};
      for(auto j = 0; j < 4; j++)
        m_boxes[i].erase(m_boxes[i].begin()+indices[j]);
      for(size_t j = 0; j < m_boxes[i].size(); ++j)
        m_boxes[i][j][2] = m_centers[i][2];
    }
    
    // Arrange the box vertices such that center is at 0
    for(size_t j = 0; j < m_boxes[i].size(); ++j)
      m_boxes[i][j] -= m_centers[i];
  }

  InitializePrimitiveSkeleton();
  
  //this->GetStatClass()->StopClock(clockName);
  cout<<"Initialized Quick Validity"<<endl;
  cout<<"Max radius of robot:"<<m_radii[0]<<endl;
}

template<class MPTraits>
void
BoundingValidity<MPTraits>::InitializePrimitiveSkeleton()	{
  // Build primitive skeleton
  this->GetStatClass()->StartClock("SkeletonConstruct");
  if(m_space < 2) {
    delete m_freePrimitives;
    if(m_dimension == 2)
      m_freePrimitives = new PrimitiveSkeleton(this->GetEnvironment(), 1, 2, m_decomposition);
    else if(m_freeFile.empty())
      m_freePrimitives = new PrimitiveSkeleton(this, 1, m_lineThreshold, m_freeParameters, m_decomposition);
    else
      m_freePrimitives = new PrimitiveSkeleton(this, m_freeFile, 1, m_lineThreshold, 3, m_decomposition);
    m_freePrimitives->BuildPrimitiveGraph(m_lineThreshold, m_circleThreshold, m_score);
  }
  if(m_space != 1) {
    delete m_obstPrimitives;
    if(m_dimension == 2)
      m_obstPrimitives = new PrimitiveSkeleton(this->GetEnvironment(), 2, 2, m_decomposition);
    else if(m_obstFile.empty())
      m_obstPrimitives = new PrimitiveSkeleton(this, 2, m_lineThreshold, m_obstParameters, m_decomposition);
    else
      m_obstPrimitives = new PrimitiveSkeleton(this, m_obstFile, 2, m_lineThreshold, 3, m_decomposition);
    m_obstPrimitives->BuildPrimitiveGraph(m_lineThreshold, m_circleThreshold, m_score);
  }
  this->GetStatClass()->StopClock("SkeletonConstruct");
  // Get regions
  if(m_freePrimitives != nullptr) {
    m_freeRegions = m_freePrimitives->GetRegions(); 
    m_freePrimitives->GetRegionMap(m_freeRegionEdgeMap); 
  }
  if(m_obstPrimitives != nullptr) {
    m_obstRegions = m_obstPrimitives->GetRegions(); 
    m_obstPrimitives->GetRegionMap(m_obstRegionEdgeMap); 
  }
  // Build Segment Tree out of bounding volumes for faster location of configurations
  m_segmentTree = new SegmentTrees<>(m_dimension);
  for(auto i = 0; i < m_obstRegions.size(); ++i)	
    m_segmentTree->AddBoundary(m_obstRegions[i]);
  for(auto i = 0; i < m_freeRegions.size(); ++i)	{
    m_segmentTree->AddBoundary(m_freeRegions[i]);
  }
  m_segmentTree->BuildSegmentTrees();
}

template<class MPTraits>
void
BoundingValidity<MPTraits>::
DecomposePolyhedron() {
  typedef CGAL::Nef_polyhedron_3<typename GMSPolyhedron::CGALKernel, CGAL::SNC_indexed_items> NefPolyhedronIndexed;
  auto env = this->GetEnvironment();
	auto cp = env->GetBoundary()->CGAL();
	NefPolyhedronIndexed obstspace(NefPolyhedronIndexed::EMPTY);
  NefPolyhedronIndexed freespace(cp);
  // Add all obstacle
	for (size_t i = 0; i < env->NumObstacles(); ++i) {
		StaticMultiBody* obst = env->GetObstacle(i);
		if (!obst->IsInternal()) {
			// Make CGAL representation of this obstacle.
			auto ocp = obst->GetFixedBody(0)->GetWorldPolyhedron().CGAL();
			// Find the intersection of the obstacle with the boundary 
			NefPolyhedronIndexed obst(cp);
			obst *= NefPolyhedronIndexed(ocp);
			// Add it to the obstaclespace.
			obstspace += obst;//NefPolyhedron(ocp);
      freespace -= NefPolyhedronIndexed(ocp);
		}
	}
	vector<typename GMSPolyhedron::CGALPolyhedron> freeResults;
  vector<typename GMSPolyhedron::CGALPolyhedron> obstResults;
  typedef typename NefPolyhedronIndexed::Volume_const_iterator VCI;
  CGAL::convex_decomposition_3(obstspace);
  CGAL::convex_decomposition_3(freespace);
  VCI ci = ++obstspace.volumes_begin();
	for (; ci != obstspace.volumes_end(); ++ci) {
		if (ci->mark()) {
			GMSPolyhedron::CGALPolyhedron poly;
			obstspace.convert_inner_shell_to_polyhedron(ci->shells_begin(), poly);
			obstResults.push_back(poly);
		}
	}
  ci = ++freespace.volumes_begin();
	for (; ci != freespace.volumes_end(); ++ci) {
		if (ci->mark()) {
			GMSPolyhedron::CGALPolyhedron poly;
			freespace.convert_inner_shell_to_polyhedron(ci->shells_begin(), poly);
			freeResults.push_back(poly);
		}
	}
}

/********************** Accessors *************************************/

template<class MPTraits>
bool
BoundingValidity<MPTraits>::
IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo, const string& _callName) {
  if(m_freePrimitives == nullptr && m_obstPrimitives == nullptr)
    InitializePrimitives();

  string clockName = "CollisionDetection::" + this->GetLabel() + "::ValidityCheck";
  string clockObst = "CollisionDetection::" + this->GetLabel() +"::ObstCheck";
  auto stat = this->GetStatClass();
  stat->StartClock(clockName); stat->StartClock(clockObst);
  stat->IncCfgIsColl(_callName);
  //position environment
  _cfg.ConfigureRobot();

  bool inOut = false;
  // If the robot is single body
  if(m_centers.size() <= 1){
    this->GetStatClass()->IncStat("TotalCalls");
    stat->StartClock(clockObst);
    inOut = SingleBodyCollision(_cfg, _cdInfo, _callName);
    stat->StopClock(clockObst);
  }
  else {
    inOut = MultiBodyCollision(_cfg, _cdInfo, _callName);
  }
  
  stat->StartClock(clockObst);
  _cfg.SetLabel("VALID", !inOut);
  stat->StopClock(clockName); 
  stat->StopClock(clockObst);
  return !inOut;
}

/******************** Accessor Helpers *************************************/
template<class MPTraits>
bool
BoundingValidity<MPTraits>::
SingleBodyCollision(const CfgType& _cfg, CDInfo& _cdInfo, 
    const string& _callName) {

  auto stat = this->GetStatClass();
  string clockPart = "CollisionDetection::" + this->GetLabel()+"::Partial";
  string clockReg = "CollisionDetection::" + this->GetLabel()+"::Regular";

  auto multiBody = _cfg.GetMultiBody();
  
  // Check against the decomposition
  size_t inOut = 0;
	inOut = SpaceContainmentCheck(_cfg.GetPoint(), m_radii[0]);
  
  switch(inOut) { 
    case 1: // Free space
      return false;
    case 2: // Obstacle space
      return true;
    default: // Undecided
      stat->IncStat("RegularCalls");
      stat->StartClock(clockReg);
    
       auto num = multiBody->NumFreeBody();
       for(auto i = 0; i< num; ++i) {
         if(CollisionWithObstacle(multiBody->GetFreeBody(i), _cdInfo)) {
           stat->StopClock(clockReg);
           return true;
         }
       }
       stat->StopClock(clockReg);
       stat->IncStat("RegularValid");
  }
  return false;
}

template<class MPTraits>
bool
BoundingValidity<MPTraits>::
MultiBodyCollision(const CfgType& _cfg, CDInfo& _cdInfo, 
    const string& _callName) {

  auto stat = this->GetStatClass();
  string clockObst = "CollisionDetection::" + this->GetLabel() +"::ObstCheck";
  string clockReg = "CollisionDetection::" + this->GetLabel() +"::Regular";
  

  stat->StartClock(clockObst);
  //Get multibody
  auto multiBody = _cfg.GetMultiBody();
  size_t numBody = multiBody->NumFreeBody();
  stat->StopClock(clockObst);

  // Check for self collision
  if(m_multi && !m_ignoreSelfCollision && numBody > 1 
      && m_vcMethod->IsInSelfCollision(_cdInfo, multiBody, _callName)){
      _cdInfo.m_collidingObstIndex = -1;
      return true; 
  }
   
  stat->StartClock(clockObst); 
  // to store indices of the body whose validity is undecided
  vector<size_t> indices;
  
  // for each body check 
  for(size_t i = 0; i < numBody; ++i) {
    stat->IncStat("TotalCalls");
    // Transformation
    auto t = multiBody->GetFreeBody(i)->GetWorldTransformation();
    // Get the center of the link and transform it
    auto pos = t * m_centers[i];
    // Complete containment check
    auto result = SpaceContainmentCheck(pos, m_radii[i]);
    // Check space
    switch(result) {
      case 1: // Free space
        break;
      case 2: // Obstacle space
        _cdInfo.m_collidingObstIndex = 0;
        stat->StopClock(clockObst);
        return true;
      default : // Undecided
        // Store for regular chack 
        indices.push_back(i);
    }
  } 
  // Still free with undecided parts - do regular check
  if(!indices.empty()){
    stat->StartClock(clockReg);
    for(size_t index : indices) {
      stat->IncStat("RegularCalls");
      if(CollisionWithObstacle(multiBody->GetFreeBody(index), _cdInfo)) {
        stat->StopClock(clockReg);
        stat->StopClock(clockObst);
        return true;
      }
      //stat->IncStat("RegularValid");
    }
    stat->StopClock(clockReg);
  }
  stat->StopClock(clockObst);

  // Check for self collision
  if(!m_multi && !m_ignoreSelfCollision && numBody > 1 &&
     m_vcMethod->IsInSelfCollision(_cdInfo, multiBody, _callName)) {
    _cdInfo.m_collidingObstIndex = -1;
    return true; 
  }
  return false;
}

template<class MPTraits>
bool 
BoundingValidity<MPTraits>::
CollisionWithObstacle(FreeBody* _body,  CDInfo& _cdInfo) {
  auto env = this->GetEnvironment();
  size_t numObst = env->NumObstacles();
	auto cdMethod = m_vcMethod->GetCDMethod();
  // Check against each obstacle
  for (size_t i = 0; i < numObst; ++i) {
    if(cdMethod->IsInCollision(_body, env->GetObstacle(i)->GetFixedBody(0), _cdInfo)) {
      _cdInfo.m_collidingObstIndex = i;
      return true;
    }
  }
  return false;
}

template<class MPTraits>
bool
BoundingValidity<MPTraits>::
IsInSelfCollision(CollisionDetectionMethod* _cd, ActiveMultiBody* _rob,
	CDInfo& _cdInfo, const string& _callName) {
   auto stat = this->GetStatClass();
   string clockName = this->GetNameAndLabel()+"::SelfCheck";
   auto cdM = m_vcMethod->GetCDMethod();
   auto iAdjacentLinks = m_vcMethod->GetAdjacentLinks();

   if(!_cdInfo.m_retAllInfo) {
     stat->IncNumCollDetCalls(cdM->GetName(), _callName);
     stat->IncStat("SelfCalls");
     stat->StartClock(clockName); 
   }

  size_t numBody = _rob->NumFreeBody(); 
  for(size_t i = 0; i < numBody - 1; ++i) {
    for(size_t j = i+1; j < numBody; ++j) {
      auto body1 = _rob->GetFreeBody(i);
      auto body2 = _rob->GetFreeBody(j);

      if(body1->IsWithinI(body2, iAdjacentLinks))
        continue;
      
      if(cdM->IsInCollision(body1, body2, _cdInfo)) {
        if(!_cdInfo.m_retAllInfo) stat->StopClock(clockName); 
        return true; 
      }
    }
  }
  
  if(!_cdInfo.m_retAllInfo) stat->StopClock(clockName); 
  return false;
}

template<class MPTraits>
size_t
BoundingValidity<MPTraits>::
SpaceContainmentCheck(const Point3d& _p, const double& _r) {
  // Get the enclosing bounding volumes from segment trees
  auto nBoundings = m_segmentTree->FindEnclosingBoundaries(_p, _r);
  vector<size_t> regions;
  // Checking the nearest enclosure
  for(auto k = 0; k < nBoundings; ++k) 
    regions.push_back(m_segmentTree->GetOutputBoundaryIndex(k));
	// regions not found
	if(regions.empty()){
		// No regions found while checking with free space decomposition : obst
		if(m_space == 1)
			return 2;
		// No regions found while checking with obst space decomp only : free
		else if(m_space == 2)
			return 1;
		else // normal check
			return 0;
	}
	// First candidacy check
	// for every regions : check which space region
	size_t space = 0;
	for(auto i : regions) {
		// if only obstacle regions - outside free : obst, only free - outside obst : free
		size_t outstate = (i < m_obstRegions.size()) ? 2 : 1;
		// not initialized
		if (space == 0)
			space = outstate;
		// not in same space : Do a detailed check with the convex bodies
		if (space != outstate) 
			return DeepCheck(regions, _p, _r);
	}
	return space;
}

template<class MPTraits>
size_t
BoundingValidity<MPTraits>::
DeepCheck(const vector<size_t>& _vi, const Point3d& _p, const double& _r)	{
	bool allOutFree = true, allOutObst = true;
  // For each region find the corresponding edge
  for(auto i : _vi) {
    // Check if they are from obstacle space primitives
		if (m_obstPrimitives != nullptr && i < m_obstRegions.size()) {
			auto edit = m_obstRegionEdgeMap.find(i);
			if(edit == m_obstRegionEdgeMap.end()) continue;
			for (auto& e : edit->second) {
				// Check if outside the convex body
				allOutObst = IsOutsidePrimitive(e, _p, 0, 2) && allOutObst;
				// Mixed type
				if(!allOutObst && !allOutFree)
					return 0;
			}// end for
		}// end if
		// Check if they are from free space primitives
		else if (m_freePrimitives != nullptr && i >= m_obstRegions.size()) {
			i -= m_obstRegions.size();
			auto edit = m_freeRegionEdgeMap.find(i);
			if(edit == m_freeRegionEdgeMap.end()) continue;
			for (auto& e : edit->second) {
				// Check if outside the convex body
				allOutFree = IsOutsidePrimitive(e, _p, _r, 1) && allOutFree;
				// Mixed type
				if (!allOutObst && !allOutFree)
					return 0;
			}// end for
		} //end else if
  }// end for
	//If all out of free region it is in obst region
	if(allOutFree && !allOutObst)
		return 2;
	//if all out of obst regions it is in free region
	else if(!allOutFree && allOutObst)
		return 1;
	else return 0;
}

template<class MPTraits>
bool
BoundingValidity<MPTraits>::
IsOutsidePrimitive(GMSPolyhedron& _poly, const Vector3d& _p, 
  const double& _r, const size_t& _space)	{
	const auto polys = _poly.GetPolygonList();
	double minD = numeric_limits<double>::max();
	for (auto p : polys) {
		bool outside = p.PointIsAbove(_p);
		if (outside) {
			double d = (_p - p.GetPoint(0)) * p.GetNormal();
			minD = min(minD, fabs(d));
		}
	}
	if (minD >= numeric_limits<double>::max() || minD < _r) return false;
	else return true;
}

#endif
