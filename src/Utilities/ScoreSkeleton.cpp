#include "ScoreSkeleton.h"
#include <algorithm>

pair<size_t, char>
SkeletonScore::
GetTopIndex(PQElementType& _t) {
	pair<size_t, char> index;
	if (m_vertexStartIndex < m_edgeStartIndex) {
		if (_t.second < m_edgeStartIndex)
			index = make_pair(_t.second, 'v');
		else
			index = make_pair(_t.second - m_edgeStartIndex, 'e');
	}
	else {
		if (_t.second < m_vertexStartIndex)
			index = make_pair(_t.second, 'e');
		else
			index = make_pair(_t.second - m_vertexStartIndex, 'v');
	}
	return index;
}

/*---------------Basic Scoring--------------------------------------------*/
/*------------------------------ Accessors------------------------------------*/

pair<size_t, char>
BasicPropertyScoring::
PopNext() {
  
	// Get top element
	auto top = m_heap.back();
	auto index = GetTopIndex(top);
	// Remove from the heap
	m_heap.pop_back();
	return index;
}

/*------------------------------- Modifiers ----------------------------------*/
void
BasicPropertyScoring::
InitializeScore(SkeletonGraphType& _g, ClearanceMapType& _cmap) {
	// Running index for skeletal elements
	size_t index = 0;
	// Set the vertex start index
	m_vertexStartIndex = index;
	// For each vertex in the graph 
	for (VI vit = _g.begin(); vit != _g.end(); ++vit, ++index) {
		auto radius = _cmap.GetVertexProperty(vit->descriptor()).first;
		// Set the heap score, and maps
		if (radius >= numeric_limits<double>::epsilon())
			m_heap.push_back(make_pair((radius*radius), index));
	}
	m_edgeStartIndex = index;
	// For each edge in the graph
	for (EI eit = _g.edges_begin(); eit != _g.edges_end(); ++eit, ++index) {
		// Length calculation
		auto s = _g.find_vertex(eit->source())->property();
		auto t = _g.find_vertex(eit->target())->property();
		double len = (t - s).norm();
		auto clearances = _cmap.GetEdgeProperty(eit->descriptor());
		double cl = std::numeric_limits<double>::max();
		for(auto c: clearances)
			if (c.first < cl) cl = c.first;
		// Only push if the minimum clearance is higher
		if (cl >= numeric_limits<double>::epsilon()) {
			if (cl <= len)
				m_heap.push_back(make_pair((max(len, cl)*cl), index));
			else
				m_heap.push_back(make_pair(((len + cl)*cl), index));
		}
	}
	// Function to sort vertices in increasing order of score
	// as deletion is available at end for vectors
	static auto CompareElements = [&](const PQElementType&  _e1, 
		const PQElementType&  _e2) -> bool {
		return ((_e1.first < _e2.first) ||
			(_e1.first == _e2.first && _e1.second < _e2.second));
	};
	// Sort the list by their first value
	sort(m_heap.begin(), m_heap.end(), CompareElements);
}
/*----------------------------------------------------------------------------*/
/*---------------Occupancy Scoring--------------------------------------------*/
/*------------------------------- Construction -------------------------------*/
PrimitiveOccupancyScoring::
PrimitiveOccupancyScoring(const Boundary* const _b, const double _s)
: SkeletonScore() {
  SetOccupancyGrid(_b, _s);
}

PrimitiveOccupancyScoring::
~PrimitiveOccupancyScoring() {
  m_heap.clear();
  m_handleMap.clear();
  m_voxelMap.clear();
  delete m_grid;
  delete m_location;
  while(!m_boundaries.empty()) {
    auto temp = m_boundaries.back();
    m_boundaries.pop_back();
    delete temp;
  }
}

/*------------------------------ Accessors------------------------------------*/
void 
PrimitiveOccupancyScoring::
SetOccupancyGrid(const Boundary* const _b, const double _s) {
  m_grid = new OccupancyGrid(_b,_s);
}

pair<size_t, char>
PrimitiveOccupancyScoring::
PopNext() {
  pair<double, size_t> top;
  pair<size_t, char> index; 
  // Get top element
  top = m_heap.top();
	index = GetTopIndex(top); 
  // Erase the element from map of handle ids
  m_handleMap.erase(top.second);
  // Remove from the heap
  m_heap.pop();
  return index;
}

/*------------------------------- Modifiers ----------------------------------*/
void 
PrimitiveOccupancyScoring::
InitializeScore(SkeletonGraphType& _g, ClearanceMapType& _cmap) {
  // Get the dimension 
  auto dimension = m_grid->GetDimension();
  // Initialize the segment tree 
  m_location = new SegmentTrees<size_t>(dimension);
  // Running index for skeletal elements
  size_t index = 0;
  // Set the vertex start index
  m_vertexStartIndex = index;
  // For each vertex in the graph 
  for (VI vit = _g.begin(); vit != _g.end(); ++vit, ++index) {
    auto center = vit->property();
    auto radius = _cmap.GetVertexProperty(vit->descriptor()).first;
		if (radius < numeric_limits<double>::epsilon())
			continue;
    unordered_set<size_t> voxels;
    pair<Point3d,Point3d> range = make_pair(Point3d(0,0,0), Point3d(0,0,0));
    PopulateVoxels(center, radius, voxels); 
    for(size_t j = 0; j < dimension; j++) {
      range.first[j] = center[j] - radius;
      range.second[j] = center[j] + radius;
    } 
    
    // Set the heap score, and maps
    auto hid = m_heap.push(make_pair((double)voxels.size(), index));
    m_handleMap.insert(make_pair(index, hid));
    m_voxelMap.insert(make_pair(index, voxels));
    // Set the AABB of the vertex 
    SetSpatialIndexing(index, range);   
  } 
  m_edgeStartIndex = index; 
  // For each edge in the graph
  for (EI eit = _g.edges_begin(); eit != _g.edges_end(); ++eit, ++index) {
    auto intermediates = eit->property();
    auto clearances = _cmap.GetEdgeProperty(eit->descriptor());
	double cl = std::numeric_limits<double>::max();
	for(auto c : clearances)
	  if(cl > c.first)
		  cl = c.first;
    unordered_set<size_t> voxels;
	pair<Point3d, Point3d> range = make_pair(Point3d(0, 0, 0), Point3d(0, 0, 0)); 
    Point3d s = intermediates.front();
    Point3d t = intermediates.back();
    for(size_t j = 0; j < dimension; j++){
      range.first[j] = min(s[j], t[j]) - cl;
      range.second[j] = max(s[j], t[j]) + cl;
    }
    
    /*for(size_t i = 0; i < intermediates.size(); i++) { 
      //PopulateVoxels(intermediates[i], cl, voxels);
      for(size_t j = 0; j < dimension; j++) {
        range.first[j] = (i > 0)? min(range.first[j], intermediates[i][j] - clearances[i]) : intermediates[i][j] - clearances[i];
        range.second[j] = (i > 0)? max(range.second[j], intermediates[i][j] + clearances[i]) : intermediates[i][j] + clearances[i];
      }
    } */
    PopulateVoxels(intermediates, cl, voxels, range);
    // Set the heap score, and maps
    auto hid = m_heap.push(make_pair((double)voxels.size(), index));
    m_handleMap.insert(make_pair(index, hid));
    m_voxelMap.insert(make_pair(index, voxels));
    // Set the AABB of the vertex 
    SetSpatialIndexing(index, range);
  }
  // Build the segment tree
  m_location->BuildSegmentTrees();
  cout<<"Initialized scoring"<<endl;
}

size_t 
PrimitiveOccupancyScoring::GetScore(NShape* _s) {
  size_t unoccupied = 0;
  if(_s == nullptr) return unoccupied;
  auto cells = m_grid->LocatePrimitiveCells(_s);
  for(auto c : cells) {
    if(!m_grid->IsCellOccupied(c)) unoccupied++;
  }
  if(unoccupied <= 0) 
    return 0;
  else return unoccupied;
}

size_t 
PrimitiveOccupancyScoring::
UpdateScore(NShape* _s) {
  // Update the occupancy grid on new shape
  size_t unoccupied = 0;
  auto cells = m_grid->LocatePrimitiveCells(_s);
  for(auto c : cells) {
    if(!m_grid->IsCellOccupied(c)) unoccupied++;
    m_grid->SetCell(c);
  }
  if(unoccupied <= 0) 
    return 0;
  // Update scores in the priority queue 
  // Get the AABB of the stored primitive
  auto extremePts = _s->AxisAlignedExtremePoints();
  auto s = m_location->FindEnclosingBoundaries(extremePts);
  // Get each AABB colliding with the primitive's AABB
  for(size_t i = 0; i < s; ++i){
    // Get the index of the skeletal element
    auto index = m_location->GetOutputBoundaryProperty(i);
    auto hid = m_handleMap.find(index);
    // If the skeletal element has an associated primitive, then skip
    if(hid == m_handleMap.end()) continue;
    // Get the voxel maps of the skeleton
    auto voxels = m_voxelMap[index];
    size_t count = 0; 
    // Get the next score by iterating through the voxels that are not occupied
    for(auto v : voxels) 
      if(!m_grid->IsCellOccupied(v)) count++;
    // Update the score in heap
    m_heap.update(hid->second, make_pair(double(count),index));
  }
  return unoccupied;
}

/*------------------------------- Helper Functions ---------------------------*/

void 
PrimitiveOccupancyScoring::
SetSpatialIndexing(size_t _i, pair<Point3d,Point3d>& _ep) {
  auto dimension = m_grid->GetDimension();
  WorkspaceBoundingBox* b = new WorkspaceBoundingBox(dimension);
  for(size_t i = 0; i < dimension; i++)
    b->SetRange(i, _ep.first[i], _ep.second[i]); 
  m_boundaries.push_back(b);
  m_location->AddBoundary(b, _i);
}

void
PrimitiveOccupancyScoring::
PopulateVoxels(Point3d& _pt, double& _d, unordered_set<size_t>& _vs) {
  if(_d <= m_grid->GetLength()){
    _vs.insert(m_grid->LocateCell(_pt));
  }
  else {
    auto dimension = m_grid->GetDimension();
    Vector3d unit(1, 1, 1);
    if(dimension < 3)
      unit[2] = 0;
  	unit.normalize();
  	auto minPt = _pt - ( _d * unit);
  	auto maxPt = _pt + ( _d * unit); 
  	auto voxels = m_grid->LocateBBXCells(minPt, maxPt); 
  	for(auto v: voxels){
    	auto p = m_grid->CellMidPoint(v); 
    	if((p -_pt).norm() <= _d)
      	_vs.insert(v);
    }
  }
}

void
PrimitiveOccupancyScoring::
PopulateVoxels(vector<Point3d>& _pts, double& _d, unordered_set<size_t>& _vs, pair<Point3d, Point3d>& _r) {
  if(_d <= m_grid->GetLength()){
    for(auto pt: _pts)
      _vs.insert(m_grid->LocateCell(pt));
  }
  else {
    Vector3d p1 = _pts.front();
    Vector3d line = (_pts.back() - p1);
    double len = line.norm();
    if(_pts.size() < 2 || len < numeric_limits<double>::epsilon()){
      for(auto pt: _pts)
        PopulateVoxels(pt, _d, _vs);
      return;
    }
    auto voxels = m_grid->LocateBBXCells(_r.first, _r.second); 
  	for(auto v: voxels){
    	auto p = m_grid->CellMidPoint(v); 
      auto dist = (line % (p1 - p)).norm()/len;
    	if(dist <= _d)
      	_vs.insert(v);
    }
  }
}

