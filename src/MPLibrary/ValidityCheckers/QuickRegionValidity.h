#ifndef QUICK_REGION_VALIDITY_H
#define QUICK_REGION_VALIDITY_H

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


////////////////////////////////////////////////////////////////////////////////
/// @ingroup ValidityCheckers
/// @brief A wrapper for validity checker that uses workspace decomposition and
///				clearance to do quick check on validity of the configuration
/// @tparam MPTraits Motion planning universe
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class QuickValidity : public ValidityCheckerMethod<MPTraits> {
  public:
    /// @name  Local Types
    /// @{
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPLibrary::ValidityCheckerPointer ValidityCheckerPointer;
    /// @}
    /// @name Contructions & Destructions
    /// @{
    QuickValidity(string _vcLabel = "");
    QuickValidity(XMLNode& _node);
    virtual ~QuickValidity() {
      m_freeRegions.clear();
      m_obstRegions.clear();
      m_centers.clear();
      m_radii.clear();
      m_boxes.clear();

      delete m_freePrimitives;
      delete m_obstPrimitives;
      delete m_segmentTree;
    }
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
    pair<size_t, NShape*> SpaceContainmentCheck(const Point3d& _p, const double& _r, 
       bool _partial = false);
    /// @brief Given workspace regions and a point find its clearance in the 
    ///        region (diastance from the boundaries of the region)
    /// @param[in] _p The point co-ordinates.
    /// @param[in] _vi Region vertex ids
    /// @param[in] _r Radius
    /// @return If inside any primitive (0 if not, 1 -if free space, 2 - if obstacle space)
    pair<size_t, NShape*> ClearanceCheck(const vector<size_t>& _vi, const Point3d& _p, const double& _r, 
      bool _partial = false);

    /// @brief Given a edge descriptor and point, check if the point is in 
    /// edge primitive
    /// @param[in] _ed vertex descriptor
    /// @param[in] _p The point co-ordinates.
    /// @param[in] _r Radius
    /// @param[in] _space In which space to check
    /// @return If inside any primitive 
    tuple<bool, NShape*, double> IsInsidePrimitive(WorkspaceSkeleton::ED& _ed, const Vector3d& _p,
     const double& _r, const size_t& _space);
    /// @brief Check if the primitive is large enough
    /// @param[in] _i Region id
    /// @param[in] _space Space in which the primitive lies.
    /// @return If the primitive is large enough
    bool CheckLargeRegion(size_t _i, size_t _space = 1);
    /// @brief Do a partwise intersection check on the body 
    /// @param[in] _ed Sets of edge descriptor 
    /// @param[in] _v Set of vertices on the boundary of the body 
    /// @return If inside any primitive (0 if not, 1 -if free space, 2 - if obstacle space)
    size_t PartialValidityCheck(NShape* _s, vector<Vector3d> _v, const Transformation& _t);
    /// @brief Clip the body and check whether the part outside the primitive is 
    /// inside any primitive
    /// @param[in] _st Status of the vertices inside/outside the primitive 
    /// @param[in] _v Set of vertices on the boundary of the body 
    /// @param[in] _p Primitive
    /// @return If inside any primitive (0 if not, 1 -if free space, 2 - if obstacle space)
    size_t ClipBody(const vector<Vector3d>& _v, const vector<bool>& _st, NShape* _p);
   
  private:
    ///@}
    ///@name Internal State
    ///@{
    string m_vcLabel;					///< Label for underlying validity checker
    size_t m_dimension; 			///< Dimension of the problem
    vector<Vector3d> m_centers; ///< Centers of the multibody
    vector<double> m_radii; ///< Radii of the multibody
    vector<vector<Vector3d>> m_boxes; ///< Bounding box points of each body in multibody
    CollisionDetectionValidity<MPTraits>* m_vcMethod {nullptr};
    bool m_ignoreSelfCollision{false};

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
QuickValidity<MPTraits>::
QuickValidity(string _vcLabel) : ValidityCheckerMethod<MPTraits>(), 
  m_vcLabel(_vcLabel), m_dimension(2), 
  m_segmentTree(nullptr), m_freePrimitives(nullptr), 
  m_obstPrimitives(nullptr) {
  this->SetName("QuickValidity");
  if(!m_vcLabel.empty()){
    m_vcMethod = static_cast<CollisionDetectionValidity<MPTraits>*>(this->GetValidityChecker(m_vcLabel).get());
    m_ignoreSelfCollision = m_vcMethod->GetIgnoreSelfCollision();
  }

  //InitializePrimitives();
}

template<class MPTraits>
QuickValidity<MPTraits>::
QuickValidity(XMLNode& _node) :
ValidityCheckerMethod<MPTraits>(_node) {
  this->SetName("QuickValidity");
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
QuickValidity<MPTraits>::InitializePrimitives()	{
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
QuickValidity<MPTraits>::InitializePrimitiveSkeleton()	{
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
  for(size_t i = 0; i < m_obstRegions.size(); ++i)	
    m_segmentTree->AddBoundary(m_obstRegions[i]);
  for(size_t i = 0; i < m_freeRegions.size(); ++i)	{
    m_segmentTree->AddBoundary(m_freeRegions[i]);
  }
  m_segmentTree->BuildSegmentTrees();
}

/********************** Accessors *************************************/

template<class MPTraits>
bool
QuickValidity<MPTraits>::
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
QuickValidity<MPTraits>::
SingleBodyCollision(const CfgType& _cfg, CDInfo& _cdInfo, 
    const string& _callName) {

  auto stat = this->GetStatClass();
  string clockPart = "CollisionDetection::" + this->GetLabel()+"::Partial";
  string clockReg = "CollisionDetection::" + this->GetLabel()+"::Regular";

  auto multiBody = _cfg.GetMultiBody();
  
  // Check against the primitive skeleton
  size_t inOut = 0;
  auto result = SpaceContainmentCheck(_cfg.GetPoint(), m_radii[0], m_partial);
  // If partial check is on and the result is still undecided
  if(m_partial && result.first == 0 && result.second != nullptr) {
    stat->IncStat("PartialCalls");
    stat->StartClock(clockPart);
    inOut = PartialValidityCheck(result.second, m_boxes[0], multiBody->GetFreeBody(0)->GetWorldTransformation());
    stat->StopClock(clockPart); 
    if(inOut > 0) stat->IncStat("PartialSuccessCalls");
  }
  else
    inOut = result.first;
  switch(inOut) { 
    case 1: // Free space
      return false;
    case 2: // Obstacle space
      return true;
    default: // Undecided
      stat->IncStat("RegularCalls");
      stat->StartClock(clockReg);
    
       auto num = multiBody->NumFreeBody();
       for(size_t i = 0; i< num; ++i) {
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
QuickValidity<MPTraits>::
MultiBodyCollision(const CfgType& _cfg, CDInfo& _cdInfo, 
    const string& _callName) {

  auto stat = this->GetStatClass();
  string clockObst = "CollisionDetection::" + this->GetLabel() +"::ObstCheck";
  string clockReg = "CollisionDetection::" + this->GetLabel() +"::Regular";
  string clockPart = "CollisionDetection::" + this->GetLabel() +"::Partial";

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
  // to store indices of the body that needs to be partial check
  vector<pair<size_t, NShape*>> partialShapes; 

  // for each body check 
  for(size_t i = 0; i < numBody; ++i) {
    stat->IncStat("TotalCalls");
    // Transformation
    auto t = multiBody->GetFreeBody(i)->GetWorldTransformation();
    // Get the center of the link and transform it
    auto pos = t * m_centers[i];
    // Complete containment check
    auto result = SpaceContainmentCheck(pos, m_radii[i], m_partial);
    // Check space
    switch(result.first) {
      case 1: // Free space
        break;
      case 2: // Obstacle space
        _cdInfo.m_collidingObstIndex = 0;
        stat->StopClock(clockObst);
        return true;
      default : // Undecided
        // Partial check is on and partial intersection detected
        if(m_partial && result.second != nullptr)
          partialShapes.push_back(make_pair(i, result.second));
        else // Store for regular chack 
          indices.push_back(i);
    }
  } 
  // Partial checks
  
  if(!partialShapes.empty()) {
    stat->StartClock(clockPart);
    for(auto s: partialShapes){
      size_t index = s.first;
      stat->IncStat("PartialCalls");
      size_t inOut = PartialValidityCheck(s.second, m_boxes[index], multiBody->GetFreeBody(index)->GetWorldTransformation());
      switch(inOut) {
        case 1: // Free space
          stat->IncStat("PartialSuccessCalls");
          break;
        case 2: // Obstacle space
          stat->IncStat("PartialSuccessCalls");
          stat->StopClock(clockObst);
          stat->StopClock(clockPart);
          return true;
        default : // Undecided
          indices.push_back(index);
      }
    }
    stat->StopClock(clockPart);
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
QuickValidity<MPTraits>::
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
QuickValidity<MPTraits>::
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
pair<size_t, NShape*>
QuickValidity<MPTraits>::
SpaceContainmentCheck(const Point3d& _p,const double& _r, bool _partial) {
  // Get the enclosing bounding volumes from segment trees
  auto nBoundings = m_segmentTree->FindEnclosingBoundaries(_p);
  vector<size_t> regions;
  // Checking the nearest enclosure
  for(size_t k = 0; k < nBoundings; ++k) 
    regions.push_back(m_segmentTree->GetOutputBoundaryIndex(k));

 if(!regions.empty())
    return ClearanceCheck(regions, _p, _r, _partial);
  // If an external complete decomposition of the space is provided 
  // no regions found in free space - in obstacle space
  else if(m_decomposition && m_space == 1)
    return make_pair(2, nullptr);
  // no regions found in obstacle space - in free space
  else if(m_decomposition && m_space == 2)
		return make_pair(1, nullptr);
  return make_pair(0, nullptr);;
}

template<class MPTraits>
pair<size_t, NShape*>
QuickValidity<MPTraits>::
ClearanceCheck(const vector<size_t>& _vi, const Point3d& _p, const double& _r, 
    bool _partial)	{
  //pair<size_t,NShape*> returnState = make_pair(0, nullptr);
  double maxClearance = -_r , clearance;
	NShape *sp, *returnShape = nullptr;
  size_t result;
  // For each region find the corresponding edge
  for(auto i : _vi) {
    // Check if they are from obstacle space primitives
		if (m_obstPrimitives != nullptr && i < m_obstRegions.size()) {
			//if(!CheckLargeRegion(i, space)) continue;
			auto edit = m_obstRegionEdgeMap.find(i);
			if(edit == m_obstRegionEdgeMap.end()) continue;
			for (auto& e : edit->second) {
				tie(result, sp, clearance) = IsInsidePrimitive(e, _p, _r, 2);
				if (result)
					return make_pair(2, nullptr);
			}// end for
		}// end if
		// Check if they are from free space primitives
		else if (m_freePrimitives != nullptr && i >= m_obstRegions.size()) {
			i -= m_obstRegions.size();
			//if(!CheckLargeRegion(i, space)) continue;
			auto edit = m_freeRegionEdgeMap.find(i);
			if(edit == m_freeRegionEdgeMap.end()) continue;
			for (auto& e : edit->second) {
				// Check if inside the edge primitive
				tie(result, sp, clearance) = IsInsidePrimitive(e, _p, _r, 1);
				if (result) return make_pair(1, nullptr);
				else if (_partial && sp != nullptr && clearance > maxClearance) {
					maxClearance = clearance;
					returnShape = sp;
				}
			}// end for
		} //end else if
  }// end for
	if (maxClearance < 0)
		return make_pair(0, nullptr);
  return make_pair(0, returnShape);
}

template<class MPTraits>
tuple<bool, NShape*, double>
QuickValidity<MPTraits>::
IsInsidePrimitive(WorkspaceSkeleton::ED& _ed, const Vector3d& _p, 
  const double& _r, const size_t& _space)	{
  auto skeleton = (_space == 1)? m_freePrimitives : m_obstPrimitives;
  double clearance = (_space == 1)? _r : 0.0;
	auto s = _ed.source();
	auto isVertex = (_ed.target() == (size_t)-1);
	if(isVertex && skeleton->IsPrimitive(s)) {
		auto vp = skeleton->GetVertexPrimitive(s);
		auto cl = vp->Clearance(_p);
		return make_tuple((cl >= clearance), vp, cl);
	}
  else if(!isVertex && skeleton->IsPrimitive(_ed))	{
    auto ep = skeleton->GetEdgePrimitive(_ed);
    auto cl = ep->Clearance(_p); 
    return make_tuple((cl >= clearance), ep, cl);
  }
  else
    return make_tuple(false, nullptr, -1);
}

template<class MPTraits>
bool 
QuickValidity<MPTraits>::
CheckLargeRegion(size_t _i, size_t _space) {
  auto bndry = (_space == 1)? m_freeRegions[_i] : m_obstRegions[_i];
  for(size_t j = 0; j < m_dimension; ++j){
    auto r = bndry->GetRange(j);
    if(r.Length() < m_radii[0]*2) {
      //cout<<"Too small a range in "<< _space << " space"<<endl;
      return false;
    }
  }
  return true;
}

/*********** Helpers - Partial Check ************************************/
template<class MPTraits>
size_t 
QuickValidity<MPTraits>::
PartialValidityCheck(NShape* _s, vector<Vector3d> _v, const Transformation& _t) {
  // Transform the  points
  size_t n = _v.size();
  for(size_t i = 0; i < n; i++){
    _v[i] = _v[i] * _t; 
  }
  // store the inclusion status of the points
  vector<bool> status(n, false);
  // Create the status
  size_t count = 0;
  for (size_t i = 0; i < n; ++i) {
    status[i] = _s->Contains(_v[i]);
    if (status[i]) {
      count++;
    }
  }
 
  if(count == n){
    return 1;
  }
  // If less than half vertices are inside the primitive
  else if(count >= n / 2) {
    return ClipBody(_v, status, _s); 
  }
  return 0;
}

template<class MPTraits>
size_t
QuickValidity<MPTraits>::
ClipBody(const vector<Vector3d>& _v, const vector<bool>& _st, NShape* _p) {
  // Store the vertices that are outside and at the intersection of rays
  vector<Vector3d> output;
  size_t n = _v.size();
	size_t j;
  // For every vertex check whether they are inside the primitive and consider
  // their neighbors which are outside to form the rays
  for(size_t i = 0; i < n; ++i) {
    //Check if the vertex is inside the primitive
    if(_st[i]){
      // Get the neighbors index 
      /*neighbors[0] = (i % 2 == 0) ? i+1 : i-1;
      neighbors[1] = ((i%4) < 2)? i+2 : i-2;
      if(m_dimension > 2) 
        neighbors[2] = (i < 4)? i+4 : i-4;*/
			// for every neighbor which is outside the primitive
			for (size_t k = 1; k < n; k=2*k) {
				 j = ((i % (2*k)) < k) ? i + k : i - k;
        // neighbor is outside the primitive
        if(!_st[j]){
          // compose the ray
          Vector3d r = (_v[j] - _v[i]).normalize();
          // Get intersection distance on the ray
					auto d = _p->DirectedClearance(_v[i], r);

          // Store the intersection point
          if(d >= 0 && d < numeric_limits<double>::max())
            output.push_back(_v[i] + r * d); 
        }
      }
    }
    // if outside the primitive store in output list
    else
      output.push_back(_v[i]);
  }

  if(output.empty())
   return 0;
  // Recompute the center and radius and recheck
  Vector3d center;
  double radius = 0;
  // Compute the center
  for(auto& v : output)
    center += v;
  center /= ((double)output.size());
  // compute radius
  for(auto& v : output) 
    radius = max(radius, (v - center).norm()); 
  return SpaceContainmentCheck(center, radius).first;
}

#endif
