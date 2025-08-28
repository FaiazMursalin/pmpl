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
    virtual ~QuickValidity() {}
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
		
    /// @brief Given a point find in which workspace region it lies
    /// @param[in] _p The point co-ordinates.
    /// @return The id of the workspace region; number of regions if not found
    vector<size_t> FindRegions(const Point3d& _p);
    size_t SingleBodyCollision(CollisionDetectionValidity<MPTraits>* _vc, 
    CfgType& _cfg, CDInfo& _cdInfo, const string& _callName);
    size_t MultiBodyCollision(CollisionDetectionValidity<MPTraits>* _vc, 
    CfgType& _cfg, CDInfo& _cdInfo, const string& _callName);
    bool CollisionWithObstacle(CollisionDetectionMethod* _cd, 
      FreeBody* _body, CDInfo& _cdInfo);
     
    /// @brief Given a point find in which space it is guranteed to be contained
    /// @param[in] _p The point co-ordinates.
    /// @param[in] _r Radius
    /// @return If inside any primitive (0 if not, 1 -if free space, 2 - if obstacle space)
    pair<size_t, NShape*> SpaceContainmentCheck(const Point3d& _p, double& _r, 
       bool _partial = false);
    /// @brief Given workspace regions and a point find its clearance in the 
    ///        region (diastance from the boundaries of the region)
    /// @param[in] _p The point co-ordinates.
    /// @param[in] _vi Region vertex ids
    /// @param[in] _r Radius
    /// @return If inside any primitive (0 if not, 1 -if free space, 2 - if obstacle space)
    pair<size_t, NShape*> ClearanceCheck(vector<size_t>& _vi, const Point3d& _p, double& _r, 
      bool _partial = false);
    /// @brief Given a vertex descriptor and point, check if the point is in 
    /// vertex primitive
    /// @param[in] _vd vertex descriptor
    /// @param[in] _p The point co-ordinates.
    /// @param[in] _r Radius
    /// @param[in] _space In which space to check
    /// @return If inside any primitive 
    tuple<bool, NShape*, double> IsInsideVertexPrimitive(WorkspaceSkeleton::VD& _vd, vector<double>& _p,
      double& _r, size_t _space=1);
    /// @brief Given a edge descriptor and point, check if the point is in 
    /// edge primitive
    /// @param[in] _ed vertex descriptor
    /// @param[in] _p The point co-ordinates.
    /// @param[in] _r Radius
    /// @param[in] _space In which space to check
    /// @return If inside any primitive 
    tuple<bool, NShape*, double> IsInsideEdgePrimitive(WorkspaceSkeleton::ED& _ed, vector<double>& _p, 
      double& _r, size_t _space=1);
    /// @brief Check if the primitive is large enough
    /// @param[in] _i Region id
    /// @param[in] _space Space in which the primitive lies.
    /// @return If the primitive is large enough
    bool CheckLargeRegion(size_t _i, size_t _space = 1);
    /// @brief Do a partwise intersection check on the body 
    /// @param[in] _ed Sets of edge descriptor 
    /// @param[in] _v Set of vertices on the boundary of the body 
    /// @return If inside any primitive (0 if not, 1 -if free space, 2 - if obstacle space)
    size_t PartialCheck(NShape* _s, vector<Vector3d>& _v);
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
    size_t ClipBody(vector<Vector3d>& _v, vector<vector<double>>& _pts, vector<bool>& _st, NShape* _p);
    /// @brief Recompute the radius and center from the given vertices
    pair<Vector3d, double> RecomputeCenterRadius(vector<Vector3d>& _v);

  private:
    ///@}
    ///@name Internal State
    ///@{
    string m_vcLabel;					///< Label for underlying validity checker
    size_t m_dimension; 			///< Dimension of the problem
    vector<Vector3d> m_centers; ///< Centers of the multibody
    vector<double> m_radii; ///< Radii of the multibody
    vector<vector<Vector3d>> m_boxes; ///< Bounding box points of each body in multibody

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
    size_t m_multi{0};  /// Type for multilink: 0- each step separate for each body, 1 - all step for each body, 2 - normal & partial together for each body
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
  this->m_multi = _node.Read("multi", false, 0, 0, MAX_INT, 
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
  this->GetStatClass()->SetStat("TotalCalls", 0);
  this->GetStatClass()->SetStat("RegularCalls", 0);
  this->GetStatClass()->SetStat("PartialCalls", 0);
  this->GetStatClass()->SetStat("PartialSuccessCalls", 0);
  this->GetStatClass()->SetStat("RegularValid", 0);
  if(rob->NumFreeBody() > 1) {
    this->GetStatClass()->SetStat("SelfCalls", 0);
    this->GetStatClass()->SetStat("Part", 0);
  }
  cout<<"Initialized Quick Validity"<<endl;
  cout<<"Max radius of robot:"<<m_radii[0]<<endl<<m_multi<<endl;
}

template<class MPTraits>
void
QuickValidity<MPTraits>::InitializePrimitiveSkeleton()	{
  // Build primitive skeleton
  if(m_space < 2) {
    delete m_freePrimitives;
    if(m_dimension == 2)
      m_freePrimitives = new PrimitiveSkeleton(this->GetEnvironment(), 1, 2, m_decomposition);
    else if(m_freeFile.empty())
      m_freePrimitives = new PrimitiveSkeleton(this, 1, m_lineThreshold, m_freeParameters, m_decomposition);
    else
      m_freePrimitives = new PrimitiveSkeleton(this, m_freeFile, 1, m_lineThreshold, 3, m_decomposition);
    m_freePrimitives->BuildPrimitiveGraph(m_lineThreshold, m_circleThreshold);
  }
  if(m_space != 1) {
    delete m_obstPrimitives;
    if(m_dimension == 2)
      m_obstPrimitives = new PrimitiveSkeleton(this->GetEnvironment(), 2, 2, m_decomposition);
    else if(m_obstFile.empty())
      m_obstPrimitives = new PrimitiveSkeleton(this, 2, m_lineThreshold, m_obstParameters, m_decomposition);
    else
      m_obstPrimitives = new PrimitiveSkeleton(this, m_obstFile, 2, m_lineThreshold, 3, m_decomposition);
    m_obstPrimitives->BuildPrimitiveGraph(m_lineThreshold, m_circleThreshold);
  }
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

/********************** Accessors *************************************/

template<class MPTraits>
bool
QuickValidity<MPTraits>::
IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo, const string& _callName) {
  if(m_freePrimitives == nullptr && m_obstPrimitives == nullptr)
    InitializePrimitives();
  
  string clockName = this->GetNameAndLabel()+"::Validity Check";
  this->GetStatClass()->StartClock(clockName);
  
  auto vcMethod = static_cast<CollisionDetectionValidity<MPTraits>*>(this->GetValidityChecker(m_vcLabel).get());
  size_t inOut = 0;
  // If the robot is single body
  if(m_centers.size() <= 1){
    this->GetStatClass()->IncStat("TotalCalls");
    inOut = SingleBodyCollision(vcMethod, _cfg, _cdInfo, _callName);
  }
  else {
    inOut = MultiBodyCollision(vcMethod, _cfg, _cdInfo, _callName);
  }
  
  // If inside free primitives
  if(inOut == 1) {
    _cfg.SetLabel("VALID", true); 
    this->GetStatClass()->StopClock(clockName); 
    return true;
  }
  // If inside obstacle primitives
  else {
    _cfg.SetLabel("VALID", false); 
    this->GetStatClass()->StopClock(clockName); 
    return false;
  }
}

/******************** Accessor Helpers *************************************/
template<class MPTraits>
size_t
QuickValidity<MPTraits>::
SingleBodyCollision(CollisionDetectionValidity<MPTraits>* _vc, 
    CfgType& _cfg, CDInfo& _cdInfo, const string& _callName) {
  //position environment
  _cfg.ConfigureRobot();
  auto multiBody = _cfg.GetMultiBody();
  
  // Check against the primitive skeleton
  size_t inOut = 0;
  auto result = SpaceContainmentCheck(_cfg.GetPoint(), m_radii[0], m_partial);
  // If partial check is on and the result is still undecided
  if(m_partial && result.first == 0 && result.second != nullptr) {
    inOut = PartialValidityCheck(result.second, m_boxes[0], multiBody->GetFreeBody(0)->GetWorldTransformation());
  }
  else
    inOut = result.first;
   
  if(inOut > 0) {
    return inOut;
  }
  else{
    this->GetStatClass()->IncStat("RegularCalls");
    string clockName = this->GetNameAndLabel()+"::Regular Check";
    this->GetStatClass()->StartClock(clockName);
    
    auto num = multiBody->NumFreeBody();
    for(auto i = 0; i< num; ++i) {
      if(CollisionWithObstacle(_vc->GetCDMethod(),
        multiBody->GetFreeBody(i), _cdInfo)) {
        this->GetStatClass()->StopClock(clockName);
        return 2;
      }
    }
    this->GetStatClass()->StopClock(clockName);
    this->GetStatClass()->IncStat("RegularValid");
  }
  return 1;
}

template<class MPTraits>
size_t
QuickValidity<MPTraits>::
MultiBodyCollision(CollisionDetectionValidity<MPTraits>* _vc, CfgType& _cfg, 
    CDInfo& _cdInfo, const string& _callName) {
  //position environment and get multibody
  _cfg.ConfigureRobot();
  auto multiBody = _cfg.GetMultiBody();
  string clockReg = this->GetNameAndLabel()+"::Regular Check";

 // Check for self collision
  if(_vc->IsInSelfCollision(_cdInfo, multiBody, _callName)) {
    this->GetStatClass()->IncStat("SelfCalls");
    return 2; 
  } 
   
  size_t numBody = multiBody->NumFreeBody();
  size_t inOut;
  vector<size_t> indices;
  vector<pair<size_t, NShape*>> partialShapes; 
  
  // for each body check in which space it belongs 
  for(size_t i = 0; i < numBody; ++i) {
    this->GetStatClass()->IncStat("TotalCalls");
    // Transformation
    auto t = multiBody->GetFreeBody(i)->GetWorldTransformation();
    // Get the center of the link and transform it
    auto pos = t * m_centers[i];
    // Complete containment check
    auto result = SpaceContainmentCheck(pos, m_radii[i], m_partial);
    // if partial check needs to be done now (1, 2) and a primitive returned
    if(m_partial && m_multi > 0 && result.first == 0 && result.second != nullptr)
      inOut = PartialValidityCheck(result.second, m_boxes[i], t);
    else
      inOut = result.first;
    // Already in obstacle space - discontinue
    if(inOut > 1){
      return inOut;
    }
    // Undecided - store the body index and continue
    else if(inOut < 1) {
      if(m_partial && m_multi == 0 && result.second != nullptr)
        partialShapes.push_back(make_pair(i, result.second));
      else if(m_multi != 1)
        indices.push_back(i);
      else{
        this->GetStatClass()->IncStat("RegularCalls");
        this->GetStatClass()->StartClock(clockReg);
        auto state = CollisionWithObstacle( _vc->GetCDMethod(),
           multiBody->GetFreeBody(i), _cdInfo);
        this->GetStatClass()->StopClock(clockReg );
        if(state) return 2;
        else 
          this->GetStatClass()->IncStat("RegularValid");
      }
    }
  }

  if(!partialShapes.empty()) {
    for(auto s: partialShapes){
      size_t i = s.first;
      auto t = multiBody->GetFreeBody(i)->GetWorldTransformation();
      inOut = PartialValidityCheck(s.second, m_boxes[i], t);
      if(inOut > 1)
        return inOut; 
      else if(inOut < 1){
        indices.push_back(i);
      }
    }
  }
  // Still free with undecided parts - do regular check
  if(!indices.empty()){
    this->GetStatClass()->StartClock(clockReg );
    
    for(size_t i : indices) {
      this->GetStatClass()->IncStat("RegularCalls");
      if(CollisionWithObstacle( _vc->GetCDMethod(),
           multiBody->GetFreeBody(i), _cdInfo)) {
        this->GetStatClass()->StopClock(clockReg);
        return 2;
      }
      this->GetStatClass()->IncStat("RegularValid");
    }
    this->GetStatClass()->StopClock(clockReg);
  }
  return 1;
}

template<class MPTraits>
bool 
QuickValidity<MPTraits>::
CollisionWithObstacle(CollisionDetectionMethod* _cd, FreeBody* _body, 
    CDInfo& _cdInfo) {
  auto env = this->GetEnvironment();
  size_t numObst = env->NumObstacles();
  // Check against each obstacle
  for (size_t i = 0; i < numObst; ++i) {
    if(_cd->IsInCollision(_body, env->GetObstacle(i)->GetFixedBody(0), _cdInfo))
      return true;
  }
  return false;
}

template<class MPTraits>
pair<size_t, NShape*>
QuickValidity<MPTraits>::
SpaceContainmentCheck(const Point3d& _p, double& _r, bool _partial) {
  auto regions = FindRegions(_p);
  pair<size_t,NShape*> inOut = make_pair(0, nullptr);
  if(!regions.empty())
    inOut = ClearanceCheck(regions, _p, _r, _partial);
  // If an external complete decomposition of the space is provided 
  // no regions found in free space - in obstacle space
  else if(m_decomposition && m_space == 1)
    inOut.first = 2;
  // no regions found in obstacle space - in free space
  else if(m_decomposition && m_space == 2)
    inOut.first = 1; 
  return inOut;
}


template<class MPTraits>
vector<size_t> 
QuickValidity<MPTraits>::
FindRegions(const Point3d& _p)	{
  // Get the enclosing bounding volumes from segment trees
  auto nBoundings = m_segmentTree->FindEnclosingBoundaries(_p);
  vector<size_t> returnIndices;

  // Checking the nearest enclosure
  for(auto k = 0; k < nBoundings; ++k) 
    returnIndices.push_back(m_segmentTree->GetOutputBoundaryIndex(k));
  
  return returnIndices;
}

template<class MPTraits>
pair<size_t, NShape*>
QuickValidity<MPTraits>::
ClearanceCheck(vector<size_t>& _vi, const Point3d& _p, double& _r, 
    bool _partial)	{
  vector<double> position; 
  pair<size_t,NShape*> returnState = make_pair(0, nullptr);
  double maxClearance = -numeric_limits<double>::max();
  for(auto i=0; i<m_dimension; i++)
    position.push_back(_p[i]);
  // For each region find the corresponding edge
  for(auto i : _vi) {
    vector<WorkspaceSkeleton::ED> edges;
    size_t space = 2;
    // Check if they are from obstacle space primitives
    if(m_obstPrimitives != nullptr && i < m_obstRegions.size()) {
      //if(!CheckLargeRegion(i, space)) continue;
      auto edit = m_obstRegionEdgeMap.find(i);
      if(edit == m_obstRegionEdgeMap.end()) continue; 
      edges = edit->second;
    }
    // Check if they are from free space primitives
    if(m_freePrimitives != nullptr && i >= m_obstRegions.size()) { 
      i -= m_obstRegions.size(); 
      space = 1;
      //if(!CheckLargeRegion(i, space)) continue;
      auto edit = m_freeRegionEdgeMap.find(i);
      if(edit == m_freeRegionEdgeMap.end()) continue; 
      edges = edit->second;
    }
    auto partialCheck = (_partial && space == 1);
    // For every mapped edge
    for(auto e : edges) {
      auto s = e.source();
      auto t = e.target(); 
      // Check if the region is for edge or vertex (target is invalid (size_t -1))
      if(t == (size_t) -1) {
        auto result = IsInsideVertexPrimitive(s, position, _r, space);
        if(get<0>(result)){
          returnState.first = space;
          return returnState;
        }
        else if(partialCheck && get<1>(result)!= nullptr && get<2>(result) > maxClearance) {
          maxClearance = get<2>(result);
          returnState.second = get<1>(result); 
        }
      }
      else {
        // Check if inside the primitive of the end-vertices
        /*if(get<0>(IsInsideVertexPrimitive(s, position, _r, space)) || 
           get<0>(IsInsideVertexPrimitive(t, position, _r, space)))
          return space;*/
        // Check if inside the edge primitive
        auto result = IsInsideEdgePrimitive(e, position, _r, space);
        if(get<0>(result)){
          returnState.first = space;
          return returnState;
        }
        else if(partialCheck && get<1>(result)!= nullptr && get<2>(result) > maxClearance) {
          maxClearance = get<2>(result);
          returnState.second = get<1>(result); 
        }
      }
    }
  }
  
  return returnState;
}

template<class MPTraits>
tuple<bool, NShape*, double>
QuickValidity<MPTraits>:: 
IsInsideVertexPrimitive(WorkspaceSkeleton::VD& _vd, vector<double>& _p, 
    double& _r, size_t _space)	{
  auto skeleton = (_space == 1)? m_freePrimitives : m_obstPrimitives;
  double clearance = (_space == 1)? _r : 0.0;
  if(skeleton->IsPrimitive(_vd))  {
    auto vp = skeleton->GetVertexPrimitive(_vd); 
    auto cl = vp->Clearance(_p);
    return make_tuple((cl >= clearance), vp, cl);
  }
  return make_tuple(false, nullptr, -1);
}

template<class MPTraits>
tuple<bool, NShape*, double>
QuickValidity<MPTraits>::
IsInsideEdgePrimitive(WorkspaceSkeleton::ED& _ed, vector<double>& _p, 
    double& _r, size_t _space)	{
  auto skeleton = (_space == 1)? m_freePrimitives : m_obstPrimitives;
  double clearance = (_space == 1)? _r : 0.0;
  if(skeleton->IsPrimitive(_ed))	{
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
  this->GetStatClass()->IncStat("PartialCalls");
  string clockPart = this->GetNameAndLabel()+"::Partial Check";
  this->GetStatClass()->StartClock(clockPart); 
  for(auto i = 0; i < _v.size(); i++){
    _v[i] = _v[i] * _t; 
  }
  auto result =  PartialCheck(_s, _v); 
  this->GetStatClass()->StopClock(clockPart);
  return result;
}

template<class MPTraits>
size_t 
QuickValidity<MPTraits>::
PartialCheck(NShape* _s, vector<Vector3d>& _v) {
  size_t n = _v.size();
  // store the inclusion status of the points
 
  vector<bool> status(n, false);
  vector<vector<double>> points(n, vector<double>(m_dimension, 0));
  for (size_t i = 0; i < n; i++) {
    for(size_t j = 0; j < m_dimension; j++)
      points[i][j] = _v[i][j];
  }
  
  // Create the status
  size_t count = 0;
  //double avgClearance = 0; 
  for (size_t i = 0; i < n; ++i) {
    status[i] = _s->Contains(points[i]);
    if (status[i]) {
      //avgClearance += clearance;
      count++;
    }
    //avgClearance /= (double) count;
  }
 
  if(count == n){ 
    this->GetStatClass()->IncStat("PartialSuccessCalls");
    return 1;
  }
  // If less than half vertices are inside the primitive
  else if(count >= n / 2) {
    auto result = ClipBody(_v, points, status, _s);
    if(result > 0) 
      this->GetStatClass()->IncStat("PartialSuccessCalls");
    return result;
  }
  return 0;
}

template<class MPTraits>
size_t
QuickValidity<MPTraits>::
ClipBody(vector<Vector3d>& _v, vector<vector<double>>& _pts, 
  vector<bool>& _st, NShape* _p) {
  // Store the vertices that are outside and at the intersection of rays
  vector<Vector3d> output;
  vector<size_t> neighbors(m_dimension, 0);
  vector<double> ray(m_dimension, 0);
  // For every vertex check whether they are inside the primitive and consider
  // their neighbors which are outside to form the rays
  for(size_t i = 0; i < _v.size(); ++i) {
    //Check if the vertex is inside the primitive
    if(_st[i]){
      // Get the neighbors index 
      neighbors[0] = (i % 2 == 0) ? i+1 : i-1;
      neighbors[1] = ((i%4) < 2)? i+2 : i-2;
      if(m_dimension > 2) 
        neighbors[2] = (i < 4)? i+4 : i-4;
      
      // for every neighbor which is outside the primitive
      for(auto j : neighbors){
        // neighbor is outside the primitive
        if(!_st[j]){
          // compose the ray
          Vector3d r = (_v[j] - _v[i]).normalize();
          for(auto k = 0; k < m_dimension; k++)
            ray[k] = r[k];
          // Get intersection distance on the ray
          auto d = _p->DirectedClearance(_pts[i], ray);
          
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
   return 1;
  
 
  // Recompute the center and radius and recheck
  auto cr = RecomputeCenterRadius(output);
  // If very small chip then use normal collision detection
  //if(cr.second > 1e-08)
  return SpaceContainmentCheck(cr.first, cr.second).first;
  //else
  //  return 0;
}

template<class MPTraits>
pair<Vector3d, double>
QuickValidity<MPTraits>::
RecomputeCenterRadius(vector<Vector3d>& _v) {
  Vector3d center(0,0,0);
  double radius = 0;
  // Compute the center
  for(auto& v : _v)
    center += v;
  center /= ((double)_v.size());
  // compute radius
  for(auto v : _v) 
    radius = max(radius, (v - center).norm()); 
  return make_pair(center, radius);
}
#endif
