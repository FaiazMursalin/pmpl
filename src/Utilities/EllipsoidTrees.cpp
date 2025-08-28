#include "EllipsoidTrees.h"
#include "EulerAngle.h"
#include "MPProblem/Environment/Environment.h"
#include "MPProblem/MPProblem.h"
#include "Geometry/Bodies/StaticMultiBody.h"
#include "Geometry/Bodies/FixedBody.h"
#include <algorithm>
#include <queue>
#include <unordered_map>
#include <unordered_set>

/*------------------------------- Construction -------------------------------*/
EllipsoidTrees::
EllipsoidTrees(Environment* _e, size_t _d) : m_dimension(_d) {
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
  auto anSk = axis->GetSkeleton(1);
  m_skeleton = get<0>(anSk);
  auto g = m_skeleton.GetGraph();

  // Set up the property map
  auto vmap = get<1>(anSk);
  for (auto vit = g.begin(); vit != g.end(); ++vit){
	double clearance = std::numeric_limits<double>::max();
	auto vertexClearances = vmap[vit->descriptor()];
	for(auto c : vertexClearances)
	  if(c.first < clearance)
		  clearance = c.first;
    m_annotation.SetVertexProperty(vit->descriptor(), clearance);
  }
	
  auto emap = get<2>(anSk);
  for (auto eit = g.edges_begin(); eit != g.edges_end(); ++eit) {
    vector<double> clearances;
    auto ecl = emap[eit->descriptor()];
    for (auto e : ecl)
      clearances.push_back(e.first);
    m_annotation.SetEdgeProperty(eit->descriptor(), clearances);
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
  for(auto eit = g.edges_begin(); eit != g.edges_end(); eit++)
    if(m_regionMap.find(eit->descriptor()) == m_regionMap.end())
      cout<<"Edge not mapped first"<<endl;

  delete axis;
}


EllipsoidTrees::
~EllipsoidTrees() {
}
/*------------------------------ Accessors------------------------------------*/
void 
EllipsoidTrees::
GetRegionMap(unordered_map<size_t, vector<WorkspaceSkeleton::ED>>& _rMap) {
  for(auto r : m_regionMap) {
    auto rId = r.second;
    auto it = _rMap.find(rId);
    if(it == _rMap.end())
      _rMap.emplace(rId, vector<WorkspaceSkeleton::ED>(1, r.first));
    else
      it->second.push_back(r.first);
   }
}

ostream& operator<<(ostream& _os, EllipsoidTrees& _et) {
  _os << _et.GetDimension() << endl;
  _os << _et.GetNumberSpheres() + _et.GetNumberEllipsoids() << endl;
  auto vmap = _et.m_ellipsoids.GetVertexMap();
  for(auto s: vmap){
    _os<<"s"<< " ";
    for(auto c: s.second.GetCenter())
      _os<<c<<" ";
      if(_et.m_dimension == 2)
        _os<<"0"<<" ";
      _os<<s.second.GetRadius()<<endl;
  }
  auto emap = _et.m_ellipsoids.GetEdgeMap();
  for(auto e: emap){
    _os<<"e"<< " ";
    for(auto c: e.second.GetCenter())
      _os<<c<<" ";
    if(_et.m_dimension == 2)
      _os<<"0"<<" ";
    for(size_t i = 0; i < _et.m_dimension; ++i)
      _os<<e.second.GetAxisSemiRange(i)<<" ";
	/*if(_et.m_dimension == 2)
	_os<<"0"<<" ";*/
    _os<<e.second.GetOrientation()<<endl;
  }
  return _os;
}

/*------------------------------- Modifiers ----------------------------------*/
void EllipsoidTrees::BuildEllipsoidTree(double _epsilon, double _th) {
	if(m_dimension == 2)
  	SimplifySkeleton(_epsilon);
  m_ellipsoids.SetSkeleton(&m_skeleton);
  BuildSpheres(_th);
  BuildEllipsoids(_th);
}

/*------------------------------- Helper Functions ---------------------------*/

void EllipsoidTrees::SimplifySkeleton(double _ep) {
  typedef typename WorkspaceSkeleton::VD VD;
  typedef typename WorkspaceSkeleton::ED ED;
  vector<ED> edges;
  vector<vector<Point3d>> edgeProps;
  vector<vector<double>> edgeClearance;
  vector<ED> deleteEdges;
  vector<pair<size_t,size_t>> evIndices;
  vector<Point3d> vertices;
  vector<double> vClearances;
  static auto DistanceToLine = [&](Point3d& _p0, Point3d& _p1, Point3d& _p2) -> double {
		auto v = (_p2 - _p1);
		return ((v % (_p1 - _p0)).norm())/(v.norm());
	};

  // Get the skeleton graph
  auto g = m_skeleton.GetGraph(); 
	
  // Iterate through each edge in graph
  for (auto eit = g.edges_begin(); eit != g.edges_end(); ++eit) {
    auto intermediates = eit->property();
    auto clearances = m_annotation.GetEdgeProperty(eit->descriptor());
		
    // Get the LP simplification
    auto simplified = LineSimplification(_ep, intermediates.begin(), intermediates.end());
    
    size_t nvb = vertices.size();
    // Add new vertex for the simplified segments end points
    for (size_t i =  1; i < simplified.size() - 1; i++) {
      if (simplified[i]) {
        vertices.push_back(intermediates[i]); 
        vClearances.push_back(clearances[i]);
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
          edgeClearance.push_back(vector<double>(clearances.begin() + start, clearances.begin() + i + 1));
          auto inter = edgeProps.back();
          auto s = inter.front();
          auto t = inter.back();
					for(size_t j =  0; j < inter.size(); j++)
						edgeClearance.back()[j] -= DistanceToLine(inter[j], s, t);
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
    m_annotation.SetVertexProperty(vd, vClearances[i]);
  }
  
  // Get the descriptor for the edges to add
  vector<size_t> regionIDs;
  for(size_t i = 0; i < evIndices.size(); i++) {
    auto s = deleteEdges[i].source();
    // Get region map
    size_t rId;
    auto it = m_regionMap.find(deleteEdges[i]);
    if(it == m_regionMap.end())
      rId = m_regions.size();
    else {
      rId = it->second; 
      m_regionMap.erase(it);
    }
    for(size_t j = evIndices[i].first; j < evIndices[i].second; j++) {
      edges.push_back(ED(s, newVertices[j]));
      s = newVertices[j];
      regionIDs.push_back(rId);
    }
    edges.push_back(ED(s, deleteEdges[i].target()));	
    regionIDs.push_back(rId);	
  }
  
  // Add the new edges
  for (size_t i = 0; i < edges.size(); i++) {
    auto ed = g.add_edge(edges[i], edgeProps[i]);
    m_annotation.SetEdgeProperty(ed, edgeClearance[i]);
    if(regionIDs[i] < m_regions.size())
      m_regionMap.emplace(ed, regionIDs[i]);
  }
  // Delete the original curved edges
  for(auto ed : deleteEdges)
    g.delete_edge(ed);
  // Set the updated graph
  m_skeleton.SetGraph(g);
}


void EllipsoidTrees::SimplifyLine(double _ep, vector<Point3d>& _pts, vector<vector<Point3d>>& _r)	{
	static auto ProjectionToLine = [&](Point3d& _p0, Point3d& _p1, Point3d& _p2) -> void {
		auto v = (_p2 - _p1).normalize();
		_p0 = _p1 + v * ((_p0-_p1)* v);
	};
	// Get the LP simplification
  auto simplified = LineSimplification(_ep, _pts.begin(), _pts.end());
	// Loop through start index
	size_t i = 0;
  while (i < simplified.size() - 1) {
		vector<Point3d> intermediates;
		size_t j = i+1;
		for(; j < simplified.size() && !simplified[j]; ++j);
		// Insert first vertex
		intermediates.push_back(_pts[i]);
		for(size_t k = i+1; k < j; k++){
			intermediates.push_back(_pts[k]);
			ProjectionToLine(intermediates.back(), _pts[i], _pts[j]);
		}
		// Insert last vertex
		intermediates.push_back(_pts[j]);
		_r.push_back(intermediates);
		i = j;
	}
}

void EllipsoidTrees::BuildSpheres(double _th) {
  // Get the skeleton graph
  auto g = m_skeleton.GetGraph();
  vector<tuple<WorkspaceSkeleton::VD, Point3d, double>> verticesClearance;

  // Function to sort in descending order of clearance
  static auto CompareVertex = [&](const tuple<WorkspaceSkeleton::VD, Point3d, double>& _vl1, 
      const tuple<WorkspaceSkeleton::VD, Point3d, double>& _vl2) -> bool {
    return get<2>(_vl1) > get<2>(_vl2);
  };
		
  // Iterate through each vertex and store the information in a vector
  for (auto vit = g.begin(); vit != g.end(); ++vit) {
    // Get the clearance
    auto cl = m_annotation.GetVertexProperty(vit->descriptor());
    verticesClearance.emplace_back(make_tuple(vit->descriptor(),vit->property(),cl));
  }
  // Sort by decreasing value of clearance
  sort(verticesClearance.begin(), verticesClearance.end(), CompareVertex);

  // Iterate through each vertex and create sphere when necessary
  for(auto v: verticesClearance)	{
    auto radius = get<2>(v);
    if (radius <= numeric_limits<double>::epsilon())
      continue;
    // Get the center 
    vector<double> center;
    auto p = get<1>(v);
    for (size_t i = 0; i < m_dimension; ++i)
      center.push_back(p[i]);
		
    auto vd = get<0>(v); 
    auto concentric = IsNearlyConcentricSpheres(center, radius, _th);
    // if the sphere is not conecntric with any other sphere
    if(!concentric.first)
      m_ellipsoids.SetVertexProperty(vd, NSphere(center,radius));
    else
      m_vertexToSphereVertexMap.emplace(vd, concentric.second);
  }
}

void EllipsoidTrees::BuildEllipsoids(double _th) {
  // Get the skeleton graph
  auto g = m_skeleton.GetGraph();
  // Iterate through each edge in graph
  for (auto eit = g.edges_begin(); eit != g.edges_end(); ++eit) {
    auto sit = g.find_vertex(eit->source());
    auto tit = g.find_vertex(eit->target());
    // Get source and target clearance
    auto scl = m_annotation.GetVertexProperty(sit->descriptor());
    auto tcl = m_annotation.GetVertexProperty(tit->descriptor());

    // Get source and target points
    auto s = sit->property();
    auto t = tit->property();
		
    // Find the mid point, length of the edge
    auto mid = (s + t) * 0.5;
    auto len = (t - s).norm();

    // Get the minimum clearance along the edge
    auto edCl = m_annotation.GetEdgeProperty(eit->descriptor());
    auto minit = min_element(edCl.begin(), edCl.end());
    double minCl = numeric_limits<double>::max();
    if (minit != edCl.end())
      minCl = (*minit);
    minCl = min(minCl, min(scl, tcl));
    
    // If major axis is too small or already covered major axis
    if (len < numeric_limits<double>::epsilon() || len <= (scl + tcl)) 
      continue;
    // If minor axis range is too small
    if (minCl < numeric_limits<double>::epsilon()/*0.5*_th*/)
      continue;

    // Get the center
    vector<double> center;
    for (size_t i = 0; i < m_dimension; ++i)
      center.push_back(mid[i]);
    // Get the semi-axes ranges
    vector<double> axes;
    axes.push_back(len*0.5);
    for (size_t i = 1; i < m_dimension; ++i)
      axes.push_back(minCl);
    // Create new ellipsoid
    if (m_dimension == 2)
      m_ellipsoids.SetEdgeProperty(eit->descriptor(), NEllipsoid(center, axes, EulerAngle(GetZAngle(s, t, mid), 0, 0)));
    else
      m_ellipsoids.SetEdgeProperty(eit->descriptor(), NEllipsoid(center, axes, EulerAngle(GetZAngle(s, t, mid), GetYAngle(s, t, mid), 0)));
  }
}

double 
EllipsoidTrees::
GetZAngle(Point3d& _p1, Point3d& _p2, Point3d& _m) {
  const double denom = sqrt(pow((_p2[0] - _m[0]), 2) + pow((_p2[1] - _m[1]), 2)); 
  if (_p2[1] > _m[1] || (_p2[1] == _m[1] &&  _p2[0] > _m[0]))
    return acos((_p2[0] - _m[0]) / denom);
  else
    return acos((_p1[0] - _m[0]) / denom);
}

double
EllipsoidTrees::
GetYAngle(Point3d& _p1, Point3d& _p2, Point3d& _m) {
  if (_p2[1] > _m[1] || (_p2[1] == _m[1] && _p2[0] > _m[0]))
    return -asin((_p2[2] - _m[2]) / (_p2 - _m).norm());
  else
    return -asin((_p1[2] - _m[2]) / (_p1 - _m).norm());
}

WorkspaceBoundingBox* 
EllipsoidTrees::
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
EllipsoidTrees::
FloodRegions(ReebGraphConstruction::ReebGraph* _rg, WorkspaceDecomposition* _wd, EllipsoidTrees::ReebEdgeRegionMapType& _reToBoxMap) {
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
		// for every incoming edge
		/*auto pred = vit->predecessors();
    for(auto pit = pred.begin(); pit != pred.end(); ++pit){
      auto trgt = *pit;
      // search if it is visited
      if (visited.find(trgt) != visited.end())
	      continue;
      // search if it is marked
      if (regionToREMap.find(trgt) == regionToREMap.end())
	      regionToREMap.emplace(trgt, regionToREMap[t]);
      // Push in the queue
      q.push(trgt);
		}*/
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

pair<bool, WorkspaceSkeleton::VD> 
EllipsoidTrees::
IsNearlyConcentricSpheres(vector<double>& _c, double _r, double _t)	{
  auto vmap = m_ellipsoids.GetVertexMap(); 
  for(auto s : vmap) {
    auto cl = s.second.Clearance(_c);
    if(cl > 0 && (_r - cl) <= _t)
      return make_pair(true, s.first);
  }
  return make_pair(false,0);
}
