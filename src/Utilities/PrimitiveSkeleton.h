#ifndef PRIMITIVE_SKELETON_H_
#define PRIMITIVE_SKELETON_H_

#include <memory>
#include <vector>
#include <limits>
#include <unordered_set>
#include <fstream>
#include <boost/heap/fibonacci_heap.hpp>
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
#include "Geometry/Shapes/NShape.h"
#include "Geometry/Shapes/NSphere.h"
#include "Geometry/Shapes/NEllipsoid.h"
#include "Geometry/Shapes/NCapsule.h"
#include "Geometry/Shapes/NOrientedBox.h"
#include "Utilities/MedialAxis2D.h"
#include "Utilities/MeanCurvatureSkeleton3D.h"
#include "MPLibrary/MPTools/ReebGraphConstruction.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/CDInfo.h"
#include "PlacePrimitives.h"


class Environment;
class SkeletonScore;

////////////////////////////////////////////////////////////////////////////////
/// Inner/Outer Primitives Skeleton Graph
////////////////////////////////////////////////////////////////////////////////
class PrimitiveSkeleton {

  public:
    ///@name Local types
    ///{

    struct reebEdgeHash{
      size_t operator()(const ReebGraphConstruction::RGEID&  _ed) const {
        return hash<typename ReebGraphConstruction::RGEID::edge_id_type>()(_ed.id());
      }
    };		
    typedef unordered_map<ReebGraphConstruction::RGEID, size_t, reebEdgeHash> ReebEdgeRegionMapType;
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

    PrimitiveSkeleton(const Environment* _e, size_t _s = 1, size_t _d = 2, bool _de = true);

    /// Using Reeb graph
    template<typename MPTraits>
    PrimitiveSkeleton(MPBaseObject<MPTraits>* _mp, double _ep, size_t _d=3);

    /// Using Mean Curvature Skeleton
    template<typename MPTraits>
    PrimitiveSkeleton(MPBaseObject<MPTraits>* _mp, size_t _s, double _ep, MeanCurvatureParams _p = MeanCurvatureParams(), bool _d = true);

    /// Using File skeleton
    template<typename MPTraits>
    PrimitiveSkeleton(MPBaseObject<MPTraits>* _mp, string& _filebase, size_t _s, double _ep, size_t _dim= 3, bool _d = true);

    ~PrimitiveSkeleton();

    ///@}
    ///@name Modifiers
    ///@{

    /// Build the Primitive Graph
    void BuildPrimitiveGraph(double _epsilon = numeric_limits<float>::epsilon(), double _th = numeric_limits<float>::epsilon(), size_t  _score = 0);

    ///@}
    ///@name Accessors
    ///@{
    
    ///  Get the dimension of the problem
    ///  @return dimension 
    size_t GetDimension() { return m_dimension;	}

    ///  Set the dimension of the problem
    ///  @param _d dimension
    void SetDimension(size_t _d = 3) {	m_dimension = _d; }

    ///@}
    ///@name Skeleton Accessors
    ///@{

    ///  Get the skeleton
    WorkspaceSkeleton& GetSkeleton() { return m_skeleton; }
    /// Get the edge Map
    PrimitiveStorageType::EdgeMapType& GetEdgePrimitiveMap(size_t _index=0) { return m_primitives[_index].GetEdgeMap(); }
    /// Get the vertex Map
    PrimitiveStorageType::VertexMapType& GetVertexPrimitiveMap(size_t _index=0) { return m_primitives[_index].GetVertexMap(); }

    /// Get the number of edge primitives in the graph
    /// @return Number of output edge primitives
    size_t GetNumberEdgePrimitives(size_t _index=0) {
      return (_index<2)?m_primitives[_index].GetEdgeMap().size(): 0;
    }
    /// Get the number of vertex primitives in the graph
    /// @return Number of output vertex primitives
    size_t GetNumberVertexPrimitives(size_t _index=0) {
      return (_index<2)?m_primitives[_index].GetVertexMap().size():0;
    }
    /// Used by the quick validity checkers
    /// Check whether primitive is there on the edge
    /// @param _ed Edge descriptor of edge in resulting skeleton
    /// @return true/false if primitive is there on edge _ed
    bool IsPrimitive(const WorkspaceSkeleton::ED& _ed,size_t _index=0)	{
      return (_index<2)?m_primitives[_index].IsEdgePropertyAssigned(_ed): false;
    }
    /// Check whether primitive is there on the vertex
    /// @param _vd Vertex descriptor of vertex in resulting skeleton
    /// @return true/false if primitive is there on vertex _vd
    bool IsPrimitive(const WorkspaceSkeleton::VD& _vd, size_t _index=0)	{
      return (_index<2)?m_primitives[_index].IsVertexPropertyAssigned(_vd):false;
    }

    /// Get the output primitive by edge descriptor
    /// @param _ed Edge descriptor of edge in resulting skeleton
    /// @return Output primitive at edge _ed
    NShape* GetEdgePrimitive(const WorkspaceSkeleton::ED& _ed,size_t _index=0)	{
      return (_index<2)?m_primitives[_index].GetEdgeProperty(_ed):nullptr;
    }
    /// Get the output vertex primitive by index
    /// @param _vd Vertex descriptor of vertex in resulting skeleton
    /// @return Output primitive at vertex _vd
    NShape* GetVertexPrimitive(const WorkspaceSkeleton::VD& _vd,size_t _index=0) {
      return (_index<2)?m_primitives[_index].GetVertexProperty(_vd):nullptr;
    }

    ///@}
    ///@name Primitive Accessors
    ///@{

    /// Get total number of spheres
    size_t GetNumberSpheres() { return m_spheres.size(); }
    /// Get total number of capsules
    size_t GetNumberCapsules() { return m_capsules.size(); }
    /// Get total number of ellipsoids
    size_t GetNumberEllipsoids() { return m_ellipsoids.size(); }
    /// Get total number of boxes
    size_t GetNumberBoxes() { return m_boxes.size(); }
    /// Get the sphere by number
    NSphere* GetSphereIndex(size_t _i) { 
      return (_i < m_spheres.size())? m_spheres[_i]: nullptr; }
    /// Get the capsule by number
    NCapsule* GetCapsuleIndex(size_t _i) { 
      return (_i < m_capsules.size())? m_capsules[_i]: nullptr; }
    /// Get the ellipsoid by number
    NEllipsoid* GetEllipsoidIndex(size_t _i) { 
      return (_i < m_ellipsoids.size())? m_ellipsoids[_i]: nullptr; }
    /// Get the boxes by number
    NOrientedBox* GetBoxIndex(size_t _i) { 
      return (_i < m_boxes.size())? m_boxes[_i]: nullptr; }
 
    ///@}
    ///@name Region Accessors
    ///@{

    vector<Boundary*>& GetRegions() { return m_regions; }
    void GetRegionMap(unordered_map<size_t, vector<WorkspaceSkeleton::ED>>& _rMap);
    WorkspaceSkeleton::GraphType& GetDeletedGraph() { return deletedGraph; }
    
    ///@}
    ///@name I/O Function
    ///@{

    friend ostream& operator<<(ostream& _os, PrimitiveSkeleton& _et);

    ///@}

  private:
    ///@name Helper Functions
    ///@{
    
    template<typename MPTraits>
    void ValidateSkeleton(MPBaseObject<MPTraits>* _mp, bool _free = true, double _ep =0.01);

    /// Update property maps with additional spokes vector
    void AddSpokesVectors(PropertyMap<vector<Vector3d>,Vector3d>* _w);
    /// Update clearance map with additional clearance information
	void UpdateClearances(PropertyMap<vector<double>,double>* _clearances);

    /// Medial Skeleton construction
    /// _s 1 - free space, 2 - obstacle space
    void MedialSkeleton(const Environment* _e, size_t _s = 1);

    /// Mean Curvature Skeleton construction
    void MeanCurvatureSkeleton(const Environment* _e, size_t _s = 1, bool _spokes = false, MeanCurvatureParams _p = MeanCurvatureParams());
	
    /// Simplify the skeleton with only straight line segment edges
    void SimplifySkeleton(double _ep = numeric_limits<double>::epsilon(), bool _push = false);

    /// Place the Primitives
    /// This scores and sort the skeletal elements and 
    /// calls them by their score to place primitives
    /// @param _t threshold for vertex overlap check
    /// @param _score Scoring function to use
    void PlacePrimitive(double _t = numeric_limits<double>::epsilon(), size_t  _score = 0, bool _minSkl=true);

    /// Place primitive on vertex
    /// @param _vi Skeleton vertex iterator
    /// @param _t threshold for vertex overlap check
    /// @return shape if placed a newly created shape otherwise nullptr
    pair<size_t,NShape*> PlaceVertexPrimitive(PlacePrimitives* _p, SkeletonScore* _sc, VI& _vi, double& _t, bool _minSkl=true);
   
    /// Place primitive on edge iterator
    /// @param _ei Skeleton edge iterator
    /// @return shape if placed a newly created shape otherwise nullptr
    pair<size_t,NShape*> PlaceEdgePrimitive(PlacePrimitives* _p, SkeletonScore* _sc, EI& _ei, double& _t, bool _minSkl=true);

    
    bool ValidBox(Point3d& _s, Vector3d& _xdir, double& _xr, double& _yr);
    
    
    /// Check the overlap of vertex
    /// @param _vi vertex iterator
    /// @param _c center
    /// @param _r minimum clearance
    /// @param _t threshold for overlap
    /// @param _i index of the primitive skeleton
    bool CheckVertexOverlap(VI& _vi, const vector<double>& _c, double& _r, double& _t, size_t _i);

    /// Check the overlap of ege
    /// @param _s, _t source and target
    /// @param _sd, _td source and target descriptor
    /// @param _l length
    bool CheckEdgeOverlap(const vector<double>& _s, const vector<double>& _t, VD  _sd, VD  _td, double& _l);

    /// Given a set of points find a bounding box region
    WorkspaceBoundingBox* GetBoundingBox(vector<Point3d>& _pts);

    /// Get Extra spokes from neighboring edges
    void GetNeighboringSpokes(EI& _ei, VD& _s, vector<Vector3d>& _spks);
    void GetExtraSpokes();


    void FloodRegions(ReebGraphConstruction::ReebGraph* _rg, WorkspaceDecomposition* _wd, ReebEdgeRegionMapType& _reToBoxMap);

    ///@}
    ///@name Internal State
    ///@{
    const Environment* m_env; ///< Environment
    WorkspaceSkeleton m_skeleton; ///< Workspace Skeleton 
    PropertyMap<vector<pair<double,double>>, pair<double, double>> m_clearances;	///< Clearance property map associated with skeleton
	PropertyMap<vector<SpokesVectorType>, SpokesVectorType> m_spokes;	///< Spoke vectors property map associated with skeleton

    PrimitiveStorageType m_primitives[2]; ///< Output set of primitive annotation
    vector<NSphere*> m_spheres;      ///< Total set of spheres
    vector<NCapsule*> m_capsules;    ///< Total set of capsules
    vector<NEllipsoid*> m_ellipsoids;  ///< Total set of ellipsoids
    vector<NOrientedBox*> m_boxes;     ///< Total set of OBBB

    size_t lastIndices[4]{0, 0, 0, 0};  ///<Total number of primitives: sphere, capsule, ellipsoid and boxes stored for inner skeleton

    vector<Boundary*> m_regions;    ///< Decomposition regions
    unordered_map<WorkspaceSkeleton::ED, size_t, edgeHash> m_regionMap; ///< Edges to decomposition regions map
    unordered_map<WorkspaceSkeleton::VD, size_t> m_regionVMap; ///< Vertex to decomposition regions map
			
    size_t m_dimension{3}; ///< Dimension of the problem or environment
    bool m_decomposition{true};
   
    double m_threshold{0.0002};
    double m_lineThreshold{0.00}; ///< Threshold for line simplification
    double m_overlapThreshold{0.00};
    bool m_debug{false};         ///< Toggle debug messages.
    WorkspaceSkeleton::GraphType deletedGraph; ///<Removed sub-graph
    ///@}
};


/*------------------------------------------------------------------------*/

template<typename MPTraits>
PrimitiveSkeleton::
PrimitiveSkeleton(MPBaseObject<MPTraits>* _mp, double _ep, size_t _d) : m_dimension(_d) {
  m_env = _mp->GetEnvironment();
  // Construct the skeleton from reeb graph
  ReebGraphConstruction* reeb = new ReebGraphConstruction();
  reeb->Construct(_mp->GetEnvironment());
  WorkspaceSkeleton::GraphType& g = m_skeleton.GetGraph();
  if(m_debug)
    cout<<"Created Reeb graph"<<endl;
 
  // Insert vertices.
  auto rg = reeb->GetReebGraph();
	ReebEdgeRegionMapType rmap;
	FloodRegions(&rg, _mp->GetEnvironment()->GetDecomposition(), rmap);
  for (auto vit = rg.begin(); vit != rg.end(); ++vit)
    g.add_vertex(vit->descriptor(), vit->property().m_vertex);

  // Insert edges.
  for (auto eit = rg.edges_begin(); eit != rg.edges_end(); ++eit) {
    // Make sure this is a real path.
    if (!eit->property().m_path.empty()) {
      vector<Point3d> path;
      for (auto p : eit->property().m_path)
        path.push_back(p);
      auto ed = g.add_edge(eit->descriptor(), path);
			auto rit = rmap.find(eit->descriptor());
			if(rit != rmap.end())
				m_regionMap.emplace(ed, rit->second);
    }
  }
  
  // Simplify edges
  //SimplifySkeleton(_ep, true);
   
  if(m_debug)
    cout<<"Created Skeleton"<<endl;  

  // Have the skeleton clearance annotated
  auto clearances = *(ClearanceAnnotatedSkeleton(_mp, &m_skeleton));
  // Make a copy in m_clearances with 2 range in clearance
  if(m_debug)
    cout<<m_clearances.GetVertexMap().size() << " "<<m_clearances.GetEdgeMap().size()<<endl;
}

template<typename MPTraits>
PrimitiveSkeleton::
PrimitiveSkeleton(MPBaseObject<MPTraits>* _mp, size_t _s, double _ep,  
  MeanCurvatureParams _p, bool _d) : m_dimension(3), m_decomposition(_d) {
  m_env = _mp->GetEnvironment();
  MeanCurvatureSkeleton(_mp->GetEnvironment(), _s, true, _p);
  // Have the skeleton clearance annotated
  SimplifySkeleton(_ep, true);
  if(_s == 2) 
   ValidateSkeleton(_mp, false, _ep);
  else
   ValidateSkeleton(_mp, true, _ep);
  PropertyMap<vector<Vector3d>,Vector3d>* witnessMap = new 
    PropertyMap<vector<Vector3d>,Vector3d>(&m_skeleton);
  auto clearances = *(ClearanceWitnessAnnotatedSkeleton(_mp, &m_skeleton, witnessMap));
  UpdateClearances(&clearances);
  AddSpokesVectors(witnessMap);
}

template<typename MPTraits>
PrimitiveSkeleton::
PrimitiveSkeleton(MPBaseObject<MPTraits>* _mp, string& _filebase, size_t _s, 
  double _ep, size_t _dim, bool _d) : m_dimension(_dim), m_decomposition(_d) {
  m_env = _mp->GetEnvironment();

  // Stream to the file containing the skeleton and spokes
  ifstream ifsg(_filebase+".graph");
  ifstream ifss(_filebase+".spokes");

  WorkspaceSkeleton::GraphType& g = m_skeleton.GetGraph();
  // Read in number of edges and vertices
  size_t numVert, numEdges;
  ifsg>>numVert>>numEdges;
  vector<WorkspaceSkeleton::VD> addedVertices;
  vector<WorkspaceSkeleton::ED> addedEdges;
  // Read in vertices
  for(size_t i = 0; i < numVert; ++i) {
    VD v; 
    Vector3d pt;
    ifsg>>v>>pt;
    addedVertices.push_back(g.add_vertex(v, pt));
  }
  // Read in edges
  for(size_t i = 0; i < numEdges; ++i){
    size_t src, trgt, pathSz;
    ifsg>>src>>trgt>>pathSz;
    vector<Point3d> path(pathSz);
    for(size_t j = 0; j < pathSz; ++j){
      ifsg>>path[j];
    }
    addedEdges.push_back(g.add_edge(src, trgt, path));
  } 

  // Read in the spokes for vertices
  for(size_t i = 0; i < numVert; ++i) {
    VD v; 
    size_t sz;
    ifss>>v>>sz;
    if(sz <= 0) continue;
    SpokesVectorType spokes(sz);
    for(size_t j=0; j<sz; ++j)
      ifss>>spokes[j];
    m_spokes.SetVertexProperty(addedVertices[i], spokes);
  }
  // Read in the spokes for edges
  for(size_t i = 0; i < numEdges; ++i) {
    size_t src, trgt, sz;
    ifss>>src>>trgt>>sz;
    if(sz <= 0) continue;
    vector<SpokesVectorType> spokes;
    for(size_t j=0; j<sz; ++j){
      size_t ssz;
      ifss>>ssz;
      SpokesVectorType spoke(ssz);
      for(size_t k=0; k<ssz; ++k)
        ifss>>spoke[k];
      spokes.push_back(spoke);
    }
    m_spokes.SetEdgeProperty(addedEdges[i], spokes);
  }
  // Have the skeleton simplified and clearance annotated
  SimplifySkeleton(_ep, true);
  //GetExtraSpokes();
  if(_s == 2) 
   ValidateSkeleton(_mp, false, _ep);
  else
   ValidateSkeleton(_mp, true, _ep);
  PropertyMap<vector<Vector3d>,Vector3d>* witnessMap = new 
    PropertyMap<vector<Vector3d>,Vector3d>(&m_skeleton);
  auto clearances = *(ClearanceWitnessAnnotatedSkeleton(_mp, &m_skeleton, witnessMap));
  UpdateClearances(&clearances);
  AddSpokesVectors(witnessMap);

  // Initialize the vertex clearnaces from spokes information
  for(auto vit = g.begin(); vit != g.end(); ++vit){
    auto vp = vit->property();
    auto spokes = m_spokes.GetVertexProperty(vit->descriptor());
    std::pair<double, double> vertexCl = (m_clearances.IsVertexPropertyAssigned(vit->descriptor()))?
      m_clearances.GetVertexProperty(vit->descriptor()):
      make_pair(std::numeric_limits<double>::max(), 0.0);
    for(auto p: spokes){
      auto d = (p - vp).norm();
      vertexCl.first = min(vertexCl.first, d);
      vertexCl.second = max(vertexCl.second, d);
    }
    m_clearances.ReplaceVertexProperty(vit->descriptor(),vertexCl);
  }
  // Initialize edge clearances from spokes information
  for (auto eit = g.edges_begin(); eit != g.edges_end(); ++eit) {
    auto inter = eit->property();
    auto spokes = m_spokes.GetEdgeProperty(eit->descriptor());
    vector<pair<double, double>> edgeClearances(inter.size(),std::make_pair(numeric_limits<double>::max(),0.0));
    if(m_clearances.IsEdgePropertyAssigned(eit->descriptor())){
      edgeClearances.clear();
      edgeClearances = m_clearances.GetEdgeProperty(eit->descriptor());
    }
    for (size_t i = 0; i < inter.size(); ++i) {
      for (auto p : spokes[i]){
        auto d = (p- inter[i]).norm();
        edgeClearances[i].first = std::min(d, edgeClearances[i].first);
        edgeClearances[i].second = std::max(d, edgeClearances[i].second);
	  }
    }
    m_clearances.ReplaceEdgeProperty(eit->descriptor(), edgeClearances);
  }
}

template<typename MPTraits>
void
PrimitiveSkeleton::
ValidateSkeleton(MPBaseObject<MPTraits>* _mp, bool _free, double _ep) {
  typedef typename WorkspaceSkeleton::VD VD;
  typedef typename WorkspaceSkeleton::ED ED;
  typedef typename MPTraits::CfgType CfgType;
  unordered_set<VD> deletedVertices;
  vector<ED> deletedEdges;
  vector<vector<Point3d>> edgesProp;

  WorkspaceSkeleton::GraphType& g = m_skeleton.GetGraph();
  auto vc = _mp->GetValidityChecker("pqp_solid");
  auto pointRobot = _mp->GetMPProblem()->GetRobot("point");

  // Function to check validity 
  auto isFree = [&](const Point3d& _p) -> bool {
    // Check against obstacles using a point robot.
    CfgType cfg(_p, pointRobot);
    CDInfo cdInfo(true);
    return vc->IsValid(cfg, cdInfo, "Skeleton Point Validity", false);
  };
  
  // Graph edges check.
  double step = _ep;
  for(auto eit = g.edges_begin(); eit != g.edges_end(); ++eit)	{
    auto src = g.find_vertex(eit->source())->property();
    auto trgt = g.find_vertex(eit->target())->property();
    double len = (trgt - src).norm();
    auto intermediates = eit->property();
    if(intermediates.size() > 2){
      for(auto pit = intermediates.begin(); pit < intermediates.end(); ++pit)
        if((*pit) == src || (*pit) == trgt) continue;
        else if(isFree(*pit) != _free) {
          deletedEdges.push_back(eit->descriptor()); edgesProp.push_back(eit->property());
          break;
        }
     }
     else {
       // Interpolate
       eit->property().pop_back();
       vector<SpokesVectorType> eSpokes;
       if(m_spokes.IsEdgePropertyAssigned(eit->descriptor()))
         eSpokes = m_spokes.GetEdgeProperty(eit->descriptor());
       for (size_t i = 1; step*i < len; i++) {
         auto p = src + (trgt - src)*((step*i)/len);
         if(isFree(p) != _free) {
           deletedEdges.push_back(eit->descriptor()); edgesProp.push_back(eit->property());
           break;
          }
          else {
            eit->property().push_back(p);
            eSpokes.insert(eSpokes.begin()+i, SpokesVectorType());
          }
       }
       eit->property().push_back(trgt);
       m_spokes.ReplaceEdgeProperty(eit->descriptor(), eSpokes);
     }// end else - Interpolate
  }

 // Delete the edges
  for(auto ed : deletedEdges)
    g.delete_edge(ed);

  // Graph vertices check.
  for(auto vit = g.begin(); vit != g.end(); ++vit)
    if((g.get_out_degree(vit->descriptor()) +
        g.get_in_degree(vit->descriptor())) != 1 &&  
      isFree(vit->property()) != _free)
      deletedVertices.insert(vit->descriptor());

  // Delete the vertices
  for(auto v : deletedVertices) 
    g.delete_vertex(v);

  cout<<"Invalid vertices: "<<deletedVertices.size()
     <<" Invalid edges: "<<deletedEdges.size() << endl;
}
/*--------------------------------------------------------------------------*/
#endif
