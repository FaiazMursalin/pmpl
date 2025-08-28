#include "PrimitiveSkeleton.h"
#include "ScoreSkeleton.h"
#include "EulerAngle.h"
#include "MPProblem/Environment/Environment.h"
#include "MPProblem/MPProblem.h"
#include "Geometry/Bodies/StaticMultiBody.h"
#include "Geometry/Bodies/FixedBody.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/PQPCollisionDetection.h"
#include <algorithm>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <tuple> 
#include "nonstd/container_ops.h"

/*------------------------------- Construction -------------------------------*/
PrimitiveSkeleton::
PrimitiveSkeleton(const Environment* _e, size_t _s, size_t _d, bool _de) : 
  m_env( _e), m_dimension(_d), m_decomposition(_de) {
  
  if (m_dimension == 2)
    MedialSkeleton(_e, _s);
  else
    MeanCurvatureSkeleton(_e, _s);
}

void 
PrimitiveSkeleton::
AddSpokesVectors(PropertyMap<vector<Vector3d>,Vector3d>* _w) {
  WorkspaceSkeleton::GraphType& g = m_skeleton.GetGraph();
  // Graph vertices witnesses.
  for(auto vit = g.begin(); vit != g.end(); ++vit) {
    if(_w->IsVertexPropertyAssigned(vit->descriptor())) {
      SpokesVectorType vSpokes;
      if(m_spokes.IsVertexPropertyAssigned(vit->descriptor())){
        vSpokes = m_spokes.GetVertexProperty(vit->descriptor());
      }
      vSpokes.emplace_back(_w->GetVertexProperty(vit->descriptor()));
      m_spokes.ReplaceVertexProperty(vit->descriptor(), vSpokes);
    }
  }
  // Graph edge witnesses
  for(auto eit = g.edges_begin(); eit != g.edges_end(); ++eit)	{
    if(_w->IsEdgePropertyAssigned(eit->descriptor())){
      auto wSpokes = _w->GetEdgeProperty(eit->descriptor());
      vector<SpokesVectorType> eSpokes;
      if(m_spokes.IsEdgePropertyAssigned(eit->descriptor())){
        eSpokes = m_spokes.GetEdgeProperty(eit->descriptor());
      }
      else
        eSpokes.resize(wSpokes.size());
      for(size_t i = 0; i < eSpokes.size(); i++)
        eSpokes[i].emplace_back(wSpokes[i]);
      m_spokes.ReplaceEdgeProperty(eit->descriptor(), eSpokes);
    }
  }
}

void 
PrimitiveSkeleton::
UpdateClearances(PropertyMap<vector<double>,double>* _clearances){
  WorkspaceSkeleton::GraphType& g = m_skeleton.GetGraph();
  // Update vertices clearances
  for(auto vit = g.begin(); vit != g.end(); ++vit){
	if(_clearances->IsVertexPropertyAssigned(vit->descriptor())) {
	   pair<double, double> vertexClearances(std::numeric_limits<double>::max(), 0.0);
	   if(m_clearances.IsVertexPropertyAssigned(vit->descriptor()))
		  vertexClearances = m_clearances.GetVertexProperty(vit->descriptor());
	   double clearance = _clearances->GetVertexProperty(vit->descriptor());
	   if(vertexClearances.first > clearance)
		 vertexClearances.first = clearance;
       if(vertexClearances.second < clearance)
		 vertexClearances.second = clearance;	 
	   m_clearances.ReplaceVertexProperty(vit->descriptor(), vertexClearances);
	} 
  } 
  // Update edge clearances 
  for(auto eit = g.edges_begin(); eit != g.edges_end(); ++eit)	{
    if(_clearances->IsEdgePropertyAssigned(eit->descriptor())){
      auto eClearances = _clearances->GetEdgeProperty(eit->descriptor());
      vector<pair<double,double>> edgeClearances;
      if(m_clearances.IsEdgePropertyAssigned(eit->descriptor()))
        edgeClearances = m_clearances.GetEdgeProperty(eit->descriptor());
      else
        edgeClearances.resize(eClearances.size(), std::make_pair(std::numeric_limits<double>::max(), 0.0));
      for(size_t i = 0; i < eClearances.size(); i++){
		if(edgeClearances[i].first > eClearances[i])
	      edgeClearances[i].first = eClearances[i];
		if(edgeClearances[i].second < eClearances[i])
	      edgeClearances[i].second = eClearances[i];
	  }
      m_clearances.ReplaceEdgeProperty(eit->descriptor(), edgeClearances);
    }
  }
}

void 
PrimitiveSkeleton::
MedialSkeleton(const Environment* _e, size_t _s) {
  //Intitialize for 2D Medial axis construction
  size_t numObjects = _e->NumObstacles();
  vector<GMSPolyhedron> polyhedra;
  for (size_t i = 0; i<numObjects; ++i) {
    auto obstacle = _e->GetObstacle(i);
    polyhedra.emplace_back(obstacle->GetFixedBody(0)->GetWorldPolyhedron());
  }
  MedialAxis2D* axis = new MedialAxis2D(polyhedra, _e->GetBoundary());

  // Construct the medial axis
  axis->BuildMedialAxis();
  auto anSk = axis->GetSkeleton(_s);
  m_skeleton = get<0>(anSk);
  auto g = m_skeleton.GetGraph();

  // Set up the property map
  auto vmap = get<1>(anSk);
  for (auto vit = g.begin(); vit != g.end(); ++vit) {
	auto vertexClearances = vmap[vit->descriptor()];
	double clearance = std::numeric_limits<double>::max();
	double maxClearance = 0;
	SpokesVectorType spokes;
	for(auto cit = vertexClearances.begin(); cit != vertexClearances.end(); ++cit){
	  if(clearance > cit->first)
		clearance = cit->first;
	  if(maxClearance < cit->first)
		maxClearance = cit->first;
	  spokes.push_back(cit->second);
	}
    m_clearances.SetVertexProperty(vit->descriptor(), make_pair(clearance, maxClearance));
    m_spokes.SetVertexProperty(vit->descriptor(), spokes);
  }
	
  auto emap = get<2>(anSk);
  for (auto eit = g.edges_begin(); eit != g.edges_end(); ++eit) {
    vector<pair<double, double>> clearances;
    vector<SpokesVectorType> spokes;
    auto ecl = emap[eit->descriptor()];
    for (auto e : ecl) {
      clearances.push_back(make_pair(e.first, e.first));
      SpokesVectorType spoke;
      spoke.push_back(e.second.first);
      spoke.push_back(e.second.second);
      spokes.push_back(spoke);
    }
    m_clearances.SetEdgeProperty(eit->descriptor(), clearances);
    m_spokes.SetEdgeProperty(eit->descriptor(), spokes);
    if(m_decomposition) {
      // Get the points for the bounding box region calculation
      vector<Point3d> pts;
      pts.assign(eit->property().begin(), eit->property().end());
      for(auto e : ecl){
        pts.push_back(e.second.first);
        pts.push_back(e.second.second);
      }
      size_t regionId = m_regions.size();
      m_regions.push_back(GetBoundingBox(pts));
      m_regionMap.emplace(eit->descriptor(), regionId);
    }
  }
  delete axis;
}

void 
PrimitiveSkeleton::
MeanCurvatureSkeleton(const Environment* _e, size_t _s, bool _spokes, MeanCurvatureParams _p) {
  //Intitialize for 3D Mean curvature skeleton construction
  MeanCurvature3D* axis = new MeanCurvature3D(_e, _s);
  axis->SetParameters(_p);

  // Construct the mean curvature skeleton
  axis->BuildSkeleton();
  auto anSk = axis->GetSkeleton();
  m_skeleton = anSk.first;
   
  m_spokes = anSk.second;
  auto g = m_skeleton.GetGraph();
  // Get the points for the bounding box region calculation
  if(m_decomposition) {
    for (auto eit = g.edges_begin(); eit != g.edges_end(); ++eit) {
      auto inter = eit->property();
      auto spokes = m_spokes.GetEdgeProperty(eit->descriptor());
      vector<Point3d> pts;
      pts.assign(inter.begin(), inter.end());
      for (auto s : spokes) 
        pts.insert(pts.end(), s.begin(), s.end());
      size_t regionId = m_regions.size();
      m_regions.push_back(GetBoundingBox(pts));
      m_regionMap.emplace(eit->descriptor(), regionId);
    }
  }

  if(_spokes) return;
	
  // Set up the clearance property map from spokes vector
  for (auto vit = g.begin(); vit != g.end(); ++vit) {
    auto v = vit->property();
    auto spks = m_spokes.GetVertexProperty(vit->descriptor());
    double d = numeric_limits<double>::max();
	double md = 0;
    for (auto p : spks){
      d = min(d, (p - v).norm());
	  md = max(md, (p - v).norm());
	}
    if(d < numeric_limits<double>::max()) 
      m_clearances.SetVertexProperty(vit->descriptor(), make_pair(d,md));
    else
      m_clearances.SetVertexProperty(vit->descriptor(), make_pair(0, 0));
   }
		
  for (auto eit = g.edges_begin(); eit != g.edges_end(); ++eit) {
    vector<pair<double, double>> clearances;
    auto inter = eit->property();
    auto spokes = m_spokes.GetEdgeProperty(eit->descriptor());
    for (size_t i = 0; i < inter.size(); ++i) {
      double d = numeric_limits<double>::max();
	  double md = 0;
      for (auto p : spokes[i]){
        d = min(d, (p - inter[i]).norm());
		md = max(md, (p - inter[i]).norm());
	  }
      if(d < numeric_limits<double>::max()) 
        clearances.push_back(make_pair(d, md));
      else
        clearances.push_back(make_pair(0, 0));
    }
    m_clearances.SetEdgeProperty(eit->descriptor(), clearances);
  }
  delete axis;
}



PrimitiveSkeleton::
~PrimitiveSkeleton() {
  m_regionMap.clear();
  m_regionVMap.clear();

  while(!m_boxes.empty()){
    auto r = m_boxes.back();
    m_boxes.pop_back();
    delete r;
  }
  while(!m_spheres.empty()){
    auto r = m_spheres.back();
    m_spheres.pop_back();
    delete r;
  }
  while(!m_ellipsoids.empty()){
    auto r = m_ellipsoids.back();
    m_ellipsoids.pop_back();
    delete r;
  }
  while(!m_capsules.empty()){
    auto r = m_capsules.back();
    m_capsules.pop_back();
    delete r;
  }
  while(!m_regions.empty()){
    auto r = m_regions.back();
    m_regions.pop_back();
    delete r;
  }
}
/*------------------------------ Accessors------------------------------------*/
void 
PrimitiveSkeleton::
GetRegionMap(unordered_map<size_t, vector<WorkspaceSkeleton::ED>>& _rMap) {
  for(auto r : m_regionMap) {
    auto rId = r.second;
    auto it = _rMap.find(rId);
    if(it == _rMap.end())
      _rMap.emplace(rId, vector<WorkspaceSkeleton::ED>(1, r.first));
    else
      it->second.push_back(r.first);
  }
  if(!m_regionVMap.empty()) {
    for(auto r : m_regionVMap) {
      auto rId = r.second;
      WorkspaceSkeleton::ED ed(r.first, (size_t)-1);
      auto it = _rMap.find(rId);
      if(it == _rMap.end())
        _rMap.emplace(rId, vector<WorkspaceSkeleton::ED>(1, ed));
      else
        it->second.push_back(ed);
    }
  }
}

ostream& operator<<(ostream& _os, PrimitiveSkeleton& _et) {
  _os << _et.GetDimension() << endl;
  _os << _et.GetNumberSpheres() + _et.GetNumberEllipsoids()
         +  _et.GetNumberCapsules() + _et.GetNumberBoxes() << endl;
  
  for(auto s: _et.m_spheres){
    _os<<"s"<< " ";
    for(auto c: s->GetCenter())
      _os<<c<<" ";
      if(_et.m_dimension == 2)
        _os<<"0"<<" ";
      _os<<s->GetRadius()<<endl;
  }
  for(auto c: _et.m_capsules){
    _os<<"c"<< " ";
    auto l = c->GetLineSegment();
    for(auto p: l.min)
      _os<<p<<" ";
      if(_et.m_dimension == 2)
        _os<<"0"<<" ";
     for(auto p: l.max)
      _os<<p<<" ";
      if(_et.m_dimension == 2)
        _os<<"0"<<" ";
      _os<<c->GetRadius()<<endl;
  }
  for(auto e: _et.m_ellipsoids){
    _os<<"e"<< " ";
    for(auto c: e->GetCenter())
      _os<<c<<" ";
    if(_et.m_dimension == 2)
      _os<<"0"<<" ";
    for(size_t i = 0; i < _et.m_dimension; ++i)
      _os<<e->GetAxisSemiRange(i)<<" ";
	/*if(_et.m_dimension == 2)
	_os<<"0"<<" ";*/
    _os<<e->GetOrientation()<<endl;
  }
  for(auto b: _et.m_boxes){
    _os<<"b"<< " ";
    for(auto c: b->GetCenter())
      _os<<c<<" ";
    if(_et.m_dimension == 2)
      _os<<"0"<<" ";
    for(size_t i = 0; i < _et.m_dimension; ++i)
      _os<<0.5*(b->GetRange(i).Length())<<" ";
	/*if(_et.m_dimension == 2)
	_os<<"0"<<" ";*/
    _os<<b->GetOrientation()<<endl;
  }
  return _os;
}

/*------------------------------- Modifiers ----------------------------------*/
void PrimitiveSkeleton::BuildPrimitiveGraph(double _epsilon, double _th, size_t  _score) {
  if(m_dimension == 2)
    SimplifySkeleton(_epsilon,true);
  m_lineThreshold = _epsilon;
  m_overlapThreshold = _th;
  size_t type = 1; // 0 : inner, 1: both  2: outer
  if(type < 2){
    m_primitives[0].SetSkeleton(&m_skeleton); //inner
    PlacePrimitive(_th, _score, true);
    lastIndices[0] = m_spheres.size();
    lastIndices[1] = m_capsules.size();
    lastIndices[2] = m_ellipsoids.size();
    lastIndices[3] = m_boxes.size();
    std::cout<< "Spheres: "<<lastIndices[0]<<"\t"
       << "Capsules: "<<lastIndices[1]<<"\t"
       << "Ellipsoids: "<<lastIndices[2]<<"\t"
       << "Boxes: "<<lastIndices[3]<<std::endl;
  }
  if(type > 0){
    m_primitives[1].SetSkeleton(&m_skeleton); //outer
    PlacePrimitive(_th, _score, false);
  }
}

/*------------------------------- Helper Functions ---------------------------*/

void PrimitiveSkeleton::SimplifySkeleton(double _ep, bool _push) {
  typedef typename WorkspaceSkeleton::VD VD;
  typedef typename WorkspaceSkeleton::ED ED;
  // NOTE: Doing the changes while traversal changes the internal property 
  // information. So we store the information about new insertions and deletions

  // Information New vertices to be created
  vector<Point3d> vertices;  // position info
  vector<pair<double,double>> vClearances;  // clearance info
  vector<SpokesVectorType> vSpokes; // spokes vector info
  // Information New edges to be created
  vector<pair<size_t,size_t>> evIndices; // indices of the end points
  vector<ED> edges;   // Descriptor of the end vertices
  vector<vector<Point3d>> edgeProps;  // intermediates info
  vector<vector<pair<double,double>>> edgeClearance;  // clearance info
  vector<vector<SpokesVectorType>> edgeSpokes; // spokes vector info
  // Information Old edges to be deleted
  vector<ED> deleteEdges;
  
  // Function to find the distance of a point from a line
  static auto DistanceToLine = [&](Point3d& _p0, Point3d& _p1, Point3d& _p2) -> double {
    auto v = (_p2 - _p1);
    return ((v % (_p1 - _p0)).norm())/(v.norm());
  };

  // Function to find the projection of a point on a line
  static auto ProjectionToLine = [&](Point3d& _p0, Point3d& _p1, Point3d& _p2) -> void {
    auto v = (_p2 - _p1).normalize();
    _p0 = _p1 + v * ((_p0-_p1)* v);
  };

  // Get the skeleton graph
  auto g = m_skeleton.GetGraph(); 
  	
  // Iterate through each edge in graph
  for (auto eit = g.edges_begin(); eit != g.edges_end(); ++eit) {
    auto intermediates = eit->property();
    if(intermediates.size() < 3) continue;
    vector<pair<double, double>> clearances;
    vector<SpokesVectorType> spokes;
    if(!m_clearances.IsEdgeMapEmpty())
      clearances = m_clearances.GetEdgeProperty(eit->descriptor());
    
    if(!m_spokes.IsEdgeMapEmpty())
      spokes = m_spokes.GetEdgeProperty(eit->descriptor());
    
    // Get the LP simplification
    auto simplified = LineSimplification(_ep, intermediates.begin(), intermediates.end());
    
    size_t nvb = vertices.size();
    // Add new vertex for the simplified segments end points
    for (size_t i =  1; i < simplified.size() - 1; i++) {
      if (simplified[i]) {
        vertices.push_back(intermediates[i]);
        if(i < clearances.size()) 
          vClearances.push_back(clearances[i]);
        if (i < spokes.size())
	        vSpokes.push_back(spokes[i]);
      }
    }
    size_t nve = vertices.size();	
	
    // If new vertex is added, add new segments too
    if (nvb != nve) {
      // add in the list of to be deleted edge
      deleteEdges.push_back(eit->descriptor()); 
      // add the start and the end indices of new vertex added for this edge to be deleted
      evIndices.push_back(make_pair(nvb,nve));
      size_t start = 0;
      for (size_t i = 1; i < simplified.size(); i++) {
        if (simplified[i]) {
          // We store the edge details and not add them immediately to avoid
          // re-checking through Line Simplification algo
          edgeProps.push_back(vector<Point3d>(intermediates.begin() + start, intermediates.begin() + i + 1));
          if(!clearances.empty())
            edgeClearance.push_back(vector<pair<double, double>>(clearances.begin() + start, clearances.begin() + i + 1));
          if(!spokes.empty())
            edgeSpokes.push_back(vector<SpokesVectorType>(spokes.begin() + start, spokes.begin() + i + 1));
          auto inter = edgeProps.back();
          auto s = inter.front();
          auto t = inter.back();
          // adjust clearance value
          if(!clearances.empty())
	          for(size_t j =  0; j < inter.size(); j++){
				double distanceToLine = DistanceToLine(inter[j], s, t);
	            edgeClearance.back()[j].first -= distanceToLine;
				edgeClearance.back()[j].second += distanceToLine;
			  }
          // store the pushed intermediate
          if(_push)
            for(size_t j =  1; j < inter.size()-1; j++)
              ProjectionToLine(edgeProps.back()[j], s, t);
          start = i;
        }
      }//end for
    }//end if
  }

  // Add new vertices
  vector<VD> newVertices;
  for(size_t i =  0; i < vertices.size(); i++) {
    auto vd = g.add_vertex(vertices[i]);
    newVertices.push_back(vd);
    if(i < vClearances.size())
      m_clearances.SetVertexProperty(vd, vClearances[i]);
    if(i < vSpokes.size())
      m_spokes.SetVertexProperty(vd, vSpokes[i]);
  }
  
  // Get the descriptor for the edges to add
  vector<size_t> regionIDs;
  for(size_t i = 0; i < evIndices.size(); i++) {
    // Get region map
    size_t rId;
    auto it = m_regionMap.find(deleteEdges[i]);
    if(it == m_regionMap.end())
      rId = m_regions.size();
    else {
      rId = it->second; 
      m_regionMap.erase(it);
    }
    if(rId == m_regions.size())
      regionIDs.push_back(rId);	
    // store the end points descriptor pairs
    auto s = deleteEdges[i].source();
    for(size_t j = evIndices[i].first; j < evIndices[i].second; j++) {
      edges.push_back(ED(s, newVertices[j]));
      s = newVertices[j];
      regionIDs.push_back(rId);
    }
    edges.push_back(ED(s, deleteEdges[i].target()));	
  }
  
  // Add the new edges
  for (size_t i = 0; i < edges.size(); i++) {
    auto ed = g.add_edge(edges[i], edgeProps[i]);
    // Add clearance info 
    if(i < edgeClearance.size())
      m_clearances.SetEdgeProperty(ed, edgeClearance[i]);
    if (i < edgeSpokes.size())
      m_spokes.SetEdgeProperty(ed, edgeSpokes[i]);
    // Add decomposition region info
    if(i < regionIDs.size() && regionIDs[i] < m_regions.size())
      m_regionMap.emplace(ed, regionIDs[i]);
  }
  // Delete the original curved edges
  for(auto ed : deleteEdges)
    g.delete_edge(ed);
  // Set the updated graph
  m_skeleton.SetGraph(g);
}

void 
PrimitiveSkeleton::PlacePrimitive(double  _t, size_t  _score, bool _minSkl) {
  // Get the skeleton graph
  auto g = m_skeleton.GetGraph();
  size_t count = 0;
  // Create the function to place the primitives
  PlacePrimitives* placement = new PlacePrimitives(&m_skeleton, m_clearances, m_spokes, m_dimension);
  placement->SetThresholds(m_lineThreshold, m_overlapThreshold, m_threshold);

  // Create a scoring function and initialize the score
  SkeletonScore* scoreFn;
  if(_score == 1)
    scoreFn = new PrimitiveOccupancyScoring(m_env->GetBoundary(), m_lineThreshold);
  else
    scoreFn = new BasicPropertyScoring();
  scoreFn->InitializeScore(g, m_clearances); 
  // Iterate through the priority queue till empty
  while(!scoreFn->IsEmpty()) {
    // Get top element
    auto element = scoreFn->PopNext();
    NShape* primitive = nullptr;
    size_t type = 0;
    // If next element is vertex
    if(element.second == 'v') { 
      VI vi = g.begin() + element.first;
      tie(type,primitive) = PlaceVertexPrimitive(placement, scoreFn, vi, _t, _minSkl);
    }
    // If next element is edge
    else if(element.second == 'e'){
      EI ei = g.edges_begin() + element.first;
      tie(type,primitive) = PlaceEdgePrimitive(placement, scoreFn, ei, _t, _minSkl);
      if(type == 2) count++;
    }
    // If new primitive is placed, update the score
    if(primitive != nullptr)
      scoreFn->UpdateScore(primitive);
  }
  delete scoreFn;
  delete placement;
}

pair<size_t,NShape*>
PrimitiveSkeleton::
PlaceVertexPrimitive(PlacePrimitives* _p, SkeletonScore* _sc, VI& _vi, double& _t, bool _isMinSkl) {
  size_t index = (_isMinSkl)? 0: 1;
  // Get the center 
  vector<double> center;
  auto p = _vi->property();
  for (size_t i = 0; i < m_dimension; ++i)
    center.push_back(p[i]);
  // Get radius 
  auto radius = _isMinSkl?m_clearances.GetVertexProperty(_vi->descriptor()).first
                :m_clearances.GetVertexProperty(_vi->descriptor()).second;

  // Check for overlap and small radius
  if(radius < numeric_limits<float>::epsilon() || CheckVertexOverlap(_vi, center, radius, _t, index))
    return make_pair(0,nullptr);

  // Place primitive if not 
  NShape* sp = nullptr;
  // Compute sphere volume 
  double sphereVolume = PI * pow(radius, m_dimension);
  if(m_dimension > 2) 
    sphereVolume *= (4/3);
  // Place Box on vertex 
  auto box = _p->PlaceBox(_vi, center, p, radius);
  size_t type = 0;
  // if the volume of box is more than the sphere 
  if((_isMinSkl && box.second >= sphereVolume) ||
     (!_isMinSkl && box.second <= sphereVolume)) { 
    sp = box.first;
    m_boxes.push_back(box.first);
    type = 2;
  }
  else {
    delete box.first;
    auto sphere = new NSphere(center,radius);
    m_spheres.push_back(sphere);
    sp = sphere;
    type = 1; 
  }
  if(sp != nullptr &&
    _sc->GetScore(sp) < max(1.0,ceil(m_overlapThreshold/m_lineThreshold))) {
      if(type == 1) m_spheres.pop_back();
      else if(type == 2) m_boxes.pop_back();
      sp = nullptr;
      type = 0;
  }
  if(sp != nullptr) {
    m_primitives[index].SetVertexProperty(_vi->descriptor(), sp);
  }
  
  // Place the region information
  if(!m_decomposition && sp != nullptr) {
    vector<Point3d> pts;
    auto pairs = sp->AxisAlignedExtremePoints();
    pts.push_back(pairs.first);
    pts.push_back(pairs.second); 
    size_t regionId = m_regions.size();
    m_regions.push_back(GetBoundingBox(pts));
    m_regionVMap.emplace(_vi->descriptor(), regionId);
  }
  return make_pair(type,sp);
}

pair<size_t,NShape*>
PrimitiveSkeleton::
PlaceEdgePrimitive(PlacePrimitives* _p, SkeletonScore* _sc, EI& _ei, double& _t, bool _isMinSkl) {
  // Get the skeleton graph
  auto g = m_skeleton.GetGraph(); 
  // Get general information
  auto sit = g.find_vertex(_ei->source());
  auto tit = g.find_vertex(_ei->target());

  // Get source and target clearance
  auto scl = _isMinSkl? m_clearances.GetVertexProperty(sit->descriptor()).first: m_clearances.GetVertexProperty(sit->descriptor()).second;
  auto tcl = _isMinSkl? m_clearances.GetVertexProperty(tit->descriptor()).first: m_clearances.GetVertexProperty(tit->descriptor()).second;

  // Get source and target points
  auto s = sit->property();
  auto t = tit->property();

  // Find the mid point, length of the edge
  auto mid = (s + t) * 0.5;
  auto len = (t - s).norm();

  // Ignore small edges
  if(len <= _t) return make_pair(0,nullptr);

  // Get the minimum clearance along the edge
  auto edCl = m_clearances.GetEdgeProperty(_ei->descriptor());
  double minCl = _isMinSkl? numeric_limits<double>::max(): 0;
  for(auto cl : edCl)
    if (_isMinSkl && cl.first < minCl)
      minCl = cl.first;
    else if(!_isMinSkl && cl.second > minCl)
	  minCl = cl.second;
	
  // Get the center, source, target
  vector<double> center, source, target;
  for (size_t i = 0; i < m_dimension; ++i) {
    center.push_back(mid[i]);
    source.push_back(s[i]);
    target.push_back(t[i]);
  }

  // Check for overlap
  if (CheckEdgeOverlap(source, target, sit->descriptor(), tit->descriptor(), len))
    return make_pair(0,nullptr);
  // Radius for capsule
  auto cr = _isMinSkl? min(minCl,min(scl,tcl)) : max(minCl,max(scl,tcl));
  // Place a capsule (1) , ellipsoid (3) and box(2)
     
  size_t type = 0; 
  double maxVolume = _isMinSkl? 0: numeric_limits<double>::max();
  size_t unoccupied = 0;
  // Get OBB
  auto box = _p->PlaceBox(_ei, center, s, t, minCl); 
  auto score = 1*_sc->GetScore(box.first);
  // Static function to check the volume is greater (for inbound primitives) or smaller for enclosing
  static auto checkVolume = [&] (double maxVolume, double primitiveVolume){
	if(_isMinSkl)
	  return (maxVolume < primitiveVolume);
    else
	  return (primitiveVolume > 0 && maxVolume > primitiveVolume);
  };
  // Static function to make the volume comparison
  if(score > unoccupied || (score == unoccupied && checkVolume(maxVolume , 1*box.second))){
    unoccupied = score;
    maxVolume = box.second; 
    type = 2;
  }
  
  // Get capsule 
  auto capsule = _p->PlaceCapsule(source, target, cr, len);
  score = 1*_sc->GetScore(capsule.first);
  if(score > unoccupied || (score == unoccupied && checkVolume(maxVolume , 1*capsule.second))){
    unoccupied =  score;
    maxVolume = capsule.second;
    type = 1;
  }

  // Get ellipsoid
  auto ext = _isMinSkl? min(scl,tcl): max(scl, tcl);
  auto ellipsoid = _p->PlaceEllipsoid(_ei, center, s, t, minCl, ext);
  score = 1*_sc->GetScore(ellipsoid.first);
  if(score > unoccupied || (score == unoccupied && checkVolume(maxVolume , 1*ellipsoid.second))){
    unoccupied =  score;
    maxVolume = ellipsoid.second;
    type = 3;
  }
 
  NShape* assignedPrimitive = nullptr;
  size_t index = _isMinSkl? 0: 1;
  // Store the necessary primitive and delete rest
  switch (type) {
    case 1: 
      delete box.first;
      delete ellipsoid.first;
      if(unoccupied  < max(1.0,ceil(m_overlapThreshold/m_lineThreshold))) break;
      m_capsules.push_back(capsule.first); 
      m_primitives[index].SetEdgeProperty(_ei->descriptor(), capsule.first);
      if(!m_primitives[index].IsVertexPropertyAssigned(_ei->source()) && scl <= cr)
        m_primitives[index].SetVertexProperty(_ei->source(), capsule.first);
      if(!m_primitives[index].IsVertexPropertyAssigned(_ei->target()) && tcl <= cr)
        m_primitives[index].SetVertexProperty(_ei->target(), capsule.first);
      assignedPrimitive = capsule.first;
      break;
    case 2:
      delete capsule.first;
      delete ellipsoid.first; 
      if(unoccupied  < max(1.0,ceil(m_overlapThreshold/m_lineThreshold))) break;
      m_boxes.push_back(box.first); 
      m_primitives[index].SetEdgeProperty(_ei->descriptor(), box.first);
      assignedPrimitive = box.first;
      break;
    case 3:
      delete capsule.first;
      delete box.first;
      if(unoccupied  < max(1.0,ceil(m_overlapThreshold/m_lineThreshold))) break;
      m_ellipsoids.push_back(ellipsoid.first); 
      m_primitives[index].SetEdgeProperty(_ei->descriptor(), ellipsoid.first);
      assignedPrimitive = ellipsoid.first;
      break;
  }
  
  // Place the region information
  if(!m_decomposition && type > 0 && assignedPrimitive != nullptr) {
    vector<Point3d> pts;
    auto pairs = assignedPrimitive->AxisAlignedExtremePoints();
    pts.push_back(pairs.first);
    pts.push_back(pairs.second);
    size_t regionId = m_regions.size();
    m_regions.push_back(GetBoundingBox(pts));
    m_regionMap.emplace(_ei->descriptor(), regionId);
  }
  if(type > 0 && assignedPrimitive != nullptr) 
   return make_pair(type,assignedPrimitive);
  else
   return make_pair(0,nullptr);
}


bool 
PrimitiveSkeleton::
ValidBox(Point3d& _s, Vector3d& _xdir, double& _xr, double& _yr) {
  vector<Vector3d> pts;
  Vector3d ydir(-_xdir[1], _xdir[0], 0);
  pts.push_back(_s + ydir*(_yr-m_lineThreshold));
  pts.push_back(_s - ydir*(_yr-m_lineThreshold));
  Point3d t = _s + _xdir*(_xr-m_lineThreshold);
  pts.push_back(t - ydir*(_yr-m_lineThreshold));
  pts.push_back(t + ydir*(_yr-m_lineThreshold));
  // Create a new free body with a nullptr as owning multibody
  FixedBody* boxBody = new FixedBody(nullptr);
  // Create body geometry with two triangles
  GMSPolyhedron poly;
  poly.GetVertexList() = pts;
  poly.GetPolygonList() = vector<GMSPolygon>{GMSPolygon(0, 1, 2,
      poly.GetVertexList()), GMSPolygon(0, 2, 3,
      poly.GetVertexList())};
  boxBody->SetPolyhedron(poly);
  // Check against every obstacle
  PQP* cdMethod = new PQP();
  // Default behaviour do not store the cd info
  CDInfo cdInfo;
  size_t numObjects = m_env->NumObstacles();
  for (size_t i = 0; i<numObjects; ++i) {
    auto obstacle = m_env->GetObstacle(i);
    if(cdMethod->IsInCollision(boxBody,obstacle->GetFixedBody(0),cdInfo))
      return false;
  }
  // Check against boundary
  auto boundary = m_env->GetBoundary();
  for(auto p : pts)
    if(!boundary->InBoundary(p))
      return false;
  return true;
}

bool
PrimitiveSkeleton::
CheckVertexOverlap(VI& _vi, const vector<double>& _c, double& _r, double& _t, size_t _i) {
  if(m_primitives[_i].IsVertexPropertyAssigned(_vi->descriptor()))
    return true;

  // Function to check whether the vertex is covered by primitive
  const static auto VertexWithinPrimitive = [&](NShape* _sh, const vector<double>& c) -> bool {
    auto cl = _sh->Clearance(c);
    return (cl > 0 && (_r - cl) <= _t);
  };

  // Check with every sphere - 0
  for (auto i = lastIndices[0]; i < m_spheres.size(); ++i) {
    if(VertexWithinPrimitive(m_spheres[i], _c)) {
      m_primitives[_i].SetVertexProperty(_vi->descriptor(), m_spheres[i]);
      return true;
    }
  }
  // Check with every capsule -1
  for (auto i = lastIndices[1]; i < m_capsules.size(); ++i) {
    if(VertexWithinPrimitive(m_capsules[i], _c)) {
      m_primitives[_i].SetVertexProperty(_vi->descriptor(), m_capsules[i]);
      return true;
    }
  }
  // Check with every boxes - 3
  for (auto i = lastIndices[3]; i < m_boxes.size(); ++i) {
   if(VertexWithinPrimitive(m_boxes[i], _c)) {
     m_primitives[_i].SetVertexProperty(_vi->descriptor(), m_boxes[i]);
     return true;
   }
  }
  // Check with every ellipsoid - 2
  for (auto i = lastIndices[2]; i < m_ellipsoids.size(); ++i) {
   if(VertexWithinPrimitive(m_ellipsoids[i], _c)) {
     m_primitives[_i].SetVertexProperty(_vi->descriptor(), m_ellipsoids[i]);
     return true;
   }
  }
  return false;
}

bool 
PrimitiveSkeleton::
CheckEdgeOverlap(const vector<double>& _s, const vector<double>& _t, VD  _sd, VD  _td, double& _l) {
  double maxcl1 = 0, maxcl2 = 0;
  // Compose the vectors for the line
  vector<double> sv(_t), tv(_s);
   for(size_t i =0; i < _s.size(); ++i) {
    sv[i] -= _s[i];
    tv[i] -= _t[i];
  }
  sv = nonstd::unit(sv);
  tv = nonstd::unit(tv);

  // Function to check whether the line is covered by primitive
  const static auto LineWithinPrimitive = [&](NShape* _sh,
                                              const vector<double>& s, const vector<double>& t,
                                              const vector<double>& sv, const vector<double>& tv) -> bool {
    auto cl1 = _sh->Clearance(s);
    auto cl2 = _sh->Clearance(t);
    maxcl1 = 0; maxcl2 = 0;
    if(cl1 >= 0 && cl2 >= 0)
      return true;
    if(cl1 > 0){
      cl1 = _sh->DirectedClearance(s, sv);
    }
    if(cl2 > 0){
      cl2 = _sh->DirectedClearance(t, tv);
    }
    if (cl1 > 0 && cl1 > maxcl1) {
      maxcl1 = cl1;
    }
    if (cl2 > 0 && cl2 > maxcl2) {
      maxcl2 = cl2;
    }
    return (_l - (maxcl1 + maxcl2) <= m_overlapThreshold);
  };

  // Check with every sphere - 0
  for (auto i = lastIndices[0]; i < m_spheres.size(); ++i) {
    if(LineWithinPrimitive(m_spheres[i], _s, _t, sv, tv)) return true;
  }
  // Check with every capsule -1
  for (auto i = lastIndices[1]; i < m_capsules.size(); ++i) {
    if(LineWithinPrimitive(m_capsules[i], _s, _t, sv, tv)) return true;
  }
  // Check with every boxes - 3
  for (auto i = lastIndices[3]; i < m_boxes.size(); ++i) {
   if(LineWithinPrimitive(m_boxes[i], _s, _t, sv, tv)) return true;
  }
  // Check with every ellipsoid - 2
  for (auto i = lastIndices[2]; i < m_ellipsoids.size(); ++i) {
   if(LineWithinPrimitive(m_ellipsoids[i], _s, _t, sv, tv)) return true;
  }
  return false;
}



WorkspaceBoundingBox* 
PrimitiveSkeleton::
GetBoundingBox(vector<Point3d>& _pts){
  double minx, miny, minz, maxx, maxy, maxz;
  minx = maxx = _pts[0][0];
  miny = maxy = _pts[0][1];
  minz = maxz = _pts[0][2];

  for(auto p: _pts) {
    minx = min(minx, p[0]);
    maxx = max(maxx, p[0]);
    miny = min(miny, p[1]);
    maxy = max(maxy, p[1]);
    minz = min(minz, p[2]);
    maxz = max(maxz, p[2]);
  }
	
  WorkspaceBoundingBox* b = new WorkspaceBoundingBox(m_dimension);
  b->SetRange(0, minx, maxx); 
  b->SetRange(1, miny, maxy); 
  if(m_dimension > 2) b->SetRange(2, minz, maxz);
  return b; 
}

void 
PrimitiveSkeleton::
FloodRegions(ReebGraphConstruction::ReebGraph* _rg, WorkspaceDecomposition* _wd, PrimitiveSkeleton::ReebEdgeRegionMapType& _reToBoxMap) {
  unordered_map<size_t, ReebGraphConstruction::RGEID> regionToREMap;
  queue<size_t> q;
  unordered_set<size_t> visited;
  unordered_map<ReebGraphConstruction::RGEID, vector<Point3d>, reebEdgeHash> reToRegionMap;

  // Initialize the seed from the graph embeddings
  for (auto eit = _rg->edges_begin(); eit != _rg->edges_end(); ++eit) {
    auto tetras = eit->property().m_tetra;
    for (auto t : tetras) {
      if (regionToREMap.find(t) != regionToREMap.end())
	      regionToREMap.emplace(t, eit->descriptor());
	    q.push(t);
    }
  }

  // flood adjacent tetras
  auto dg = _wd->GetDualGraph();
  while (!q.empty()) {
    auto t = q.front(); q.pop();
    // check if the tetrahedron is already checked
    if (visited.find(t) != visited.end()) continue;
    // if not, mark it as visited
    visited.insert(t);
    auto vit = dg.find_vertex(t);
    // for every outgoing edge check targets
    for (auto eit = vit->begin(); eit != vit->end(); ++eit) {
      auto trgt = eit->target();
      // search if it is visited
      if (visited.find(trgt) != visited.end())
	      continue;
      // search if it is marked
      if (regionToREMap.find(trgt) == regionToREMap.end())
	      regionToREMap.emplace(trgt, regionToREMap[t]); 
      // Push in the queue
      q.push(trgt);
    }
	}

	// Push to eid to regions map
  for (auto rre : regionToREMap) {
    auto pts = _wd->GetRegion(rre.first).GetPoints();
    auto rit = reToRegionMap.find(rre.second);
    if (rit == reToRegionMap.end())
      reToRegionMap.emplace(rre.second, vector<Point3d>(pts.begin(), pts.end()));
    else
      rit->second.insert(rit->second.end(), pts.begin(), pts.end());
  }

  // Find the bounding box
  for(auto rre : reToRegionMap) {
    auto pts = rre.second;
    size_t regionId = m_regions.size();
    m_regions.push_back(GetBoundingBox(pts));
    _reToBoxMap.emplace(rre.first, regionId);
  }
}

void 
PrimitiveSkeleton::GetExtraSpokes() {
  // Get the skeleton graph
  auto g = m_skeleton.GetGraph();
  for (auto eit = g.edges_begin(); eit != g.edges_end(); ++eit) {
    vector<Vector3d> vSpokes;
    if (m_spokes.IsVertexPropertyAssigned(eit->source()))
      vSpokes = m_spokes.GetVertexProperty(eit->source());
    size_t prev = vSpokes.size();
    VD v = eit->source();
    GetNeighboringSpokes(eit, v, vSpokes);
    if(prev < vSpokes.size())
      m_spokes.ReplaceVertexProperty(eit->source(), vSpokes);
    vSpokes.clear();
    if (m_spokes.IsVertexPropertyAssigned(eit->target()))
      vSpokes = m_spokes.GetVertexProperty(eit->target());
    prev = vSpokes.size();
    v = eit->target();
    GetNeighboringSpokes(eit, v, vSpokes);
    if (prev < vSpokes.size())
      m_spokes.ReplaceVertexProperty(eit->target(), vSpokes);
   }
}

void
PrimitiveSkeleton::
GetNeighboringSpokes(EI& _ei, VD& _s, vector<Vector3d>& _spks) {
  queue<size_t> q;
  unordered_set<size_t> visitedVertices;
  unordered_set<WorkspaceSkeleton::ED, edgeHash> visitedEdges;
	
  // Initialization 
  // Get the skeleton graph
  auto g = m_skeleton.GetGraph();
  // Get source and target vertex
  auto sit = g.find_vertex(_ei->source());
  auto tit = g.find_vertex(_ei->target());
  // direction of the line 
  Vector3d direction = (tit->property() - sit->property());
  double length = 0.5 * direction.norm();
  direction = direction.normalize();
  auto mid = (sit->property() + tit->property()) * 0.5;

  // Store the start vertex in queue
  q.push(_s);
  visitedEdges.insert(_ei->descriptor());
  // BFS traversal
   while (!q.empty()) {
     auto v = q.front(); q.pop();
     // if v not visited
     if(visitedVertices.find(v) != visitedVertices.end())
       continue;
     // mark v to be visited
     visitedVertices.insert(v);
     auto vit = g.find_vertex(v);
     // for every outgoing edge check targets
     for(auto eit = vit->begin(); eit != vit->end(); ++eit) {
       // If edge is already visited
       if(visitedEdges.find(eit->descriptor()) != visitedEdges.end())
         continue;
       // Mark the edge visited
       visitedEdges.insert(eit->descriptor());
       size_t prev = _spks.size();
       // Check whether the spokes vector are inside the line
       auto spokes = m_spokes.GetEdgeProperty(eit->descriptor());
       for(auto spks : spokes) {
         for(auto s : spks) {
           auto d = (s - mid)*direction;
           if(fabs(d) <= length)
             _spks.push_back(s);
         }
       }
       // if no change in size of spks, dont push the other vertex to queue
       if(_spks.size() <= prev)
         continue;
       // Get the other vertex
       auto o = eit->target();
       // if already visited vertex then outgoing edges already considered
       if(visitedVertices.find(o) != visitedVertices.end())
         continue;
       q.push(o);
     }
     // for all incoming edges
     auto pred = vit->predecessors();
     for(auto pit = pred.begin(); pit != pred.end(); ++pit) {
       // get other vertex
       auto o = *pit;
       auto oit = g.find_vertex(o);
       // if already visited vertex then outgoing edges already considered
       if(visitedVertices.find(o) != visitedVertices.end())
         continue;
       size_t prev = _spks.size();
       for(auto eit = oit->begin(); eit != oit->end(); ++eit)
         if(eit->target() == v 
           && visitedEdges.find(eit->descriptor()) == visitedEdges.end()) {
	         // Mark the edge visited
           visitedEdges.insert(eit->descriptor());
           // Check whether the spokes vector are inside the line
           auto spokes = m_spokes.GetEdgeProperty(eit->descriptor());
           for(auto spks : spokes) {
	           for(auto s : spks) {
		           auto d = (s - mid)*direction;
		           if(fabs(d) <= length)
		             _spks.push_back(s);
             }
           }
         }
			
      if(_spks.size() <= prev)
	      continue;
      // Push in the queue if within the semi-circle
      q.push(o);
    }
  }
}


