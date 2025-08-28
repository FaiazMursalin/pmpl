#include <cmath>
#include <queue>
#include <algorithm>
#include <limits>
#include "OctreeDecomposition.h"

#include "Geometry/Boundaries/Boundary.h"
#include "Geometry/Bodies/FixedBody.h"
#include "Geometry/Bodies/StaticMultiBody.h"
#include "Geometry/Boundaries/WorkspaceBoundingBox.h"
#include "MPProblem/Environment/Environment.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/PQPCollisionDetection.h"

/////////////// VoxMap methods ////////////////////////////////////////////////

VoxMap::VoxMap(size_t _d): m_dimension(_d) {
  if(m_dimension == 2) m_tree2 = new Tree2D();
  else m_tree3 = new Tree3D();
}

VoxMap::~VoxMap() {
  delete m_tree2;
  delete m_tree3;
}

void VoxMap::BuildTree() { 
  if(m_dimension == 2) m_tree2->build();
  else m_tree3->build();
}

void VoxMap::InsertPoint(Point3d&  _pt) {
  if(m_dimension == 2) {
    Point2 p(_pt[0], _pt[1]);
    m_tree2->insert(p);
  }
  else {
    Point3 p(_pt[0], _pt[1], _pt[2]);
    m_tree3->insert(p);
  }
}

size_t VoxMap::Search(Point3d& _min, Point3d& _max) {
  double tolerance = numeric_limits<double>::epsilon();
  if(m_dimension == 2) {
    Point2 p(_min[0] + tolerance, _min[1] + tolerance);
    Point2 q(_max[0] - tolerance, _max[1] - tolerance);
    vector<Point2> result;
    QueryBox2D range(p,q);
    m_tree2->search(std::back_inserter(result), range);
    return result.size();
  }
  else {
    Point3 p(_min[0] + tolerance, _min[1] + tolerance, _min[2] + tolerance);
    Point3 q(_max[0] - tolerance, _max[1] - tolerance, _max[2] - tolerance);
    vector<Point3> result;
    QueryBox3D range(p,q);
    m_tree3->search(std::back_inserter(result), range);
    return result.size();
  }
}


////////////// Octree Node methods ////////////////////////////////////////////

OctreeNode::OctreeNode() {
	for(size_t i = 0; i < 8; ++i)
		m_children[i] = nullptr;
}

OctreeNode::
OctreeNode(Point3d& _pt, OctreeNode* _p) {
  if(_p) {
    m_parent = _p; 
    m_level = _p->GetLevel() + 1; 
  }
  m_corner = _pt;
  for(size_t i = 0; i < 8; ++i)
		m_children[i] = nullptr;
}

// Use this for bottom-up construction
OctreeNode::
OctreeNode(OctreeNode** _c, size_t _dim) {
	// Number of children
	size_t number = (_dim == 2) ? 4 : 8;
	// Iterate through each of the children 
  for(size_t i = 0; i < number; ++i) {
		if(_c[i] != nullptr) {
			// set the child
			m_children[i] = _c[i];
			auto type = m_children[i]->GetType();
			//setting type
			if(m_type == 3) {
				m_type = type;
        // set the level as 1 above the child
        m_level = m_children[i]->GetLevel() + 1;
				// set the corner as the corner of the first child cell which should be at 0
				m_corner = m_children[i]->GetCorner();
			}
			else if(m_type != type)
				m_type = 2;
			// set the parent for the children
			m_children[i]->SetParent(this);
		}
		else
			m_children[i] = nullptr;
	}
}

OctreeNode::~OctreeNode() {
	for(size_t i = 0; i < 8; ++i) {
		m_children[i]->SetParent(nullptr);
		delete m_children[i];
	}
}

bool OctreeNode::IsLeaf() {
	for(size_t i = 0; i < 8; ++i)
		if(m_children[i])
			return false;
	return true; 
}

bool OctreeNode::IsRoot() {
  return (m_parent == nullptr);
}

Point3d OctreeNode::GetCorner() {
  return m_corner;
}
size_t OctreeNode::GetType() {
  return m_type;
}
size_t OctreeNode::GetLevel(){
  return m_level;
}
OctreeNode* OctreeNode::GetParent() {
  return m_parent;
}
OctreeNode* OctreeNode::GetChild(size_t _i) {
  if(_i > 7) return nullptr;
  return m_children[_i];
}

void OctreeNode::SetCorner(Point3d& _p) {
  m_corner = _p;
}
void OctreeNode::SetType(size_t _t) {
  m_type = _t;
}
void OctreeNode::SetParent(OctreeNode* _p) {
  m_parent = _p;
}
void OctreeNode::SetChild(size_t _i, OctreeNode* _c) {
  m_children[_i] = _c;
}

///////////////Octree decomposition/////////////////////////////////////////////

OctreeDecomposition::
OctreeDecomposition(const Environment* _e, const double _length, StatClass* _stat):
    m_env(_e), m_dimension(3) {
  auto boundary = m_env->GetBoundary();
  m_dimension = boundary->GetDimension();
  // Compute the maximum range in each dimension
  Point3d minP, maxP;
  double maxRange = 0;
  size_t num = 0;
  for(size_t i = 0; i < m_dimension; ++i){
    auto range = boundary->GetRange(i);
    minP[i] = range.min;
    maxP[i] = range.max;
    num = max((size_t)ceil(range.Length()/_length),num);
    maxRange = max(range.Length(), maxRange);
  }
  m_length = maxRange;
  // Build the data structures to quickly determine type
  auto segTree = BuildSegmentTrees(m_env); 
  if(_stat != nullptr)
    _stat->StartClock("VoxmapBasic");
  auto voxels = BuildVoxMap(segTree, minP, maxP, _length);
  if(_stat != nullptr)
    _stat->StopClock("VoxmapBasic");
  if(_stat != nullptr)
    _stat->StartClock("DecompositionBasic");
  //cout<<"possibleLevels = "<<ceil(log2(num))<<" "<<num<<endl;
  size_t numChild = (m_dimension == 3)? 8 : 4;
   
  // create root with free level 
  m_root = new OctreeNode(minP);
  queue<pair<OctreeNode*, double>> q;
  q.push(make_pair(m_root, maxRange));
  while(!q.empty()) {
    auto tp = q.front(); q.pop(); 
    auto node = tp.first; 
    auto len = tp.second;
    //m_length = min(len, m_length);
    m_levels = max(m_levels, node->GetLevel());
    // Detemine the type of the node
    auto corner = node->GetCorner();
    Point3d last;
    for(size_t i = 0; i < m_dimension; ++i)
      last[i] = min(corner[i] + len, maxP[i]);
    node->SetType(DetermineType(segTree, voxels, corner, last));
    // If type is mixed then divide node
    if(node->GetType() == 2 && len > _length) {
      for(size_t i = 0; i < numChild; ++i) {
        Point3d pt(corner);
        if(i%2 > 0) pt[0] += (len/2);
        if((i%4) > 1) pt[1] += (len/2);
        if(i > 3) pt[2] += (len/2);
        // if corner within the max range
        if(pt[0] >= maxP[0] || pt[1] >= maxP[1] || (m_dimension > 2 && pt[2] >= maxP[2]))
          continue;
        // Create the children
        OctreeNode* child = new OctreeNode(pt, node);
        node->SetChild(i, child);
        // push to queue
        q.push(make_pair(child, len/2));
      }//end for
    }//end if
  }//end while
  cout<<"No of levels" << m_levels<<endl;
  if(_stat != nullptr)
    _stat->StopClock("DecompositionBasic");
}


void 
OctreeDecomposition::
GetFreeRegions(vector<pair<Point3d, size_t>>& _ret, double _length) {
  size_t numChild = (m_dimension == 3)? 8 : 4;
  queue<OctreeNode*> q;
  //size_t prevLevel;
  // Do a BFS traversal of the tree s.t. the wide regions are stored first
  // Push root in the queue
  q.push(m_root);
  while(!q.empty()){
    auto node = q.front(); q.pop();
    auto type = node->GetType();
    auto level = m_length/pow(2,node->GetLevel());
    // if the node is free or leaf
    if(type == 0 || (type == 2 && (node->IsLeaf() || 0.5*level < _length))){
      // Store the regions
      _ret.push_back(make_pair(node->GetCorner(), node->GetLevel()));
    }
    // Mixed types
    else if(type == 2) {
      // Append the children to queue
      for(size_t i = 0; i < numChild; ++i){
        auto child = node->GetChild(i);
        if(child != nullptr)
          q.push(child);
      }// end for
    }// end else
  }// end while
}


SegmentTrees<size_t>*
OctreeDecomposition::
BuildSegmentTrees(const Environment* _e) {
	SegmentTrees<size_t>* stree = new SegmentTrees<size_t>(m_dimension);
  for(size_t i = 0; i < _e->NumObstacles(); ++i) {
    const auto& bbx = _e->GetObstacle(i)->GetFixedBody(0)->GetWorldPolyhedron().ComputeBoundingPolyhedron();
    const auto& minVertex = bbx.m_vertexList[0]; 
    const auto& maxVertex = bbx.m_vertexList[7];
    auto bndry = new WorkspaceBoundingBox(m_dimension);
    bndry->SetRange(0, minVertex[0], maxVertex[0]); 
    bndry->SetRange(1, minVertex[1], maxVertex[1]); 
    if(m_dimension > 2) bndry->SetRange(2, minVertex[2], maxVertex[2]); 
    stree->AddBoundary(bndry, i); 
  } 
  stree->BuildSegmentTrees();
  return stree;
}

VoxMap* 
OctreeDecomposition::
BuildVoxMap(SegmentTrees<size_t>* _st, Point3d& _min, Point3d& _max, const double& _length) {
  VoxMap* vmap = new VoxMap(m_dimension);
  size_t num[3] ={0,0,1};
  for(size_t i = 0; i < m_dimension; ++i)
    num[i] = (size_t)ceil((_max[i] - _min[i]) / _length);
  size_t mark = num[0];
  if(m_dimension > 2) mark *= num[1];
  vector<bool> state(mark, false);
  Point3d p;

  // Iterate through each voxel
  size_t id[3], next[3];
  next[0] = 1; next[1] = num[0]; next[2] = num[0] * num[1];
  for(id[2] = 0; id[2] < num[2]; id[2]++) {
    for(id[1] = 0; id[1] < num[1]; id[1]++) {
      for(id[0] = 0; id[0] < num[0]; id[0]++) {
	      // Get the voxel center
	      for(size_t i = 0; i < m_dimension; ++i)
	        p[i] = min(_min[i] + id[i]*_length + 0.5*_length, _max[i]);
	      // set the serial number of voxel
	      size_t sno = id[0] + num[0] * id[1];
	      if(m_dimension > 2) sno += (id[2] * num[0] * num[1]);
        // if first voxel get state
        if(sno == 0) state[0] = IsInsideObstacles(p, m_env, _st);
	      auto currState = state[sno % mark];
	      // for every dimension 
	      for(size_t i = 0; i < m_dimension; ++i) {
	        // if last voxel in dimension 
	        if(id[i] >= num[i] - 1) continue;
          // Get neighbor voxel
	        Point3d np(p);
          np[i] = min(p[i] + _length, _max[i]);
          bool nextState = IsInsideObstacles(np, m_env, _st);
          // if different validity state store the point
          if(nextState != currState) {
            Point3d newPoint(p);
            newPoint[i] += (0.5*_length);
            vmap->InsertPoint(newPoint); 
	        }
        // store the validity to avoid re-computation
        state[(sno + next[i]) % mark] = nextState;
	     }
      }// end for 0
    }// end for 1
  }// end for 2
  vmap->BuildTree(); 
  return vmap;
}

size_t 
OctreeDecomposition::
DetermineType(SegmentTrees<size_t>* _st, VoxMap* _vm, Point3d& _lp, Point3d& _gp) {
  auto key = make_pair(_lp, _gp);
  auto num = _st->FindEnclosingBoundaries(key);
  // No intersecting boundaries - free space
  if(num == 0) return 0; 
  auto vNum = _vm->Search(_lp, _gp);
  // Intersecting boundaries in the query box
  if(vNum > 0) return 2;
  for(size_t k = 0; k < num; k++) {
    auto b = _st->GetOutputBoundary(k);
    bool enclosed = true;
    for(size_t i = 0; i < m_dimension; ++i) {
      // Check if contained 
      auto r = b->GetRange(i);
      if(_lp[i] < r.min && _gp[i] > r.max) return 2;
      // Check if enclosed or partial 
      else if(_lp[i] < r.min || _gp[i] > r.max) enclosed = false;
    }
    if(enclosed) return 1;
  }
  return 0;
}


bool 
OctreeDecomposition::
IsInsideObstacles(Point3d& _p, const Environment* _e, SegmentTrees<size_t>* _st) {
	auto num = _st->FindEnclosingBoundaries(_p);
	// No intersecting obstacle bounding box - free
	if(num == 0) return false;
	// Check against each intersecting box's obstacles for deep collision check
	PQPSolid cd; 
	for(size_t k = 0; k < num; k++) {
		auto index = _st->GetOutputBoundaryIndex(k);
		auto state = cd.IsInsideObstacle(_p, _e->GetObstacle(index)->GetFixedBody(0));
		if(state) return true;
	}
	return false;
}

