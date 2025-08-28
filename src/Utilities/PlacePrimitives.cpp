#include "PlacePrimitives.h"
#include <algorithm>

/*************** Primitive Placement ******************************************/
pair<NCapsule*, double>
PlacePrimitives::
PlaceCapsule(vector<double>&  _s, vector<double>& _t,const double&  _r, double& _l) {
  // return if radius is too small
  if(_r < m_threshold * _l)
    return make_pair(nullptr, 0);
  // Compute volume
  double volume = 0;
  if (m_dimension == 2)
    volume = 2 * _r *_l + PI * _r * _r;
  else
    volume = PI * _r * _r *(_l + (4 * _r / 3));
  NCapsule* cp = new NCapsule(_s, _t, _r);  
  return make_pair(cp, volume);
}

pair<NEllipsoid*, double> 
PlacePrimitives::
PlaceEllipsoid(EI& _ei, vector<double>&  _c, Point3d&  _s, Point3d& _t, double&  _r, const double& _vCl) {
  vector<double> ranges;
  // Get mid and length
  auto mid = (_s + _t) * 0.5;
  auto l = (_t - _s).norm();  
  // Get the ranges
  ranges.push_back(0.5*l);
  if(_vCl > numeric_limits<float>::epsilon())
    ranges[0] += _vCl;
  auto inter = _ei->property();
  auto cl = m_clearances.GetEdgeProperty(_ei->descriptor());
  double minB = m_isMin?numeric_limits<double>::max() : 0;
  double maxC = 0;
  // iterate through each edge point and find min b through ellipsoid equation
  // substituting y/z as clearance, x as distance to center and a as previously
  // computed. This computes the largest b possible
  size_t minI = inter.size(); 
  for (size_t i = 0; i < inter.size(); ++i) {
    maxC = max(maxC, cl[i].first);
    double d = (inter[i] - mid).normsqr(); 
    d = 1 - (d/pow(ranges[0],2)); 
    if (d <= 0) 
      continue;
    d = cl[i].first / (sqrt(d)); 
    if((m_isMin && minB > d) || (!m_isMin && minB < d)) {
      minB = d;
      minI= i; 
    }
  } 
  
  // check whether the minimum b is within clearance limit along the edge
  if (minB < _r || minB > maxC) { 
    minB = m_isMin? _r: maxC;
    minI = inter.size();
  }
 
  // Return if min axis range is too small
  if(minB < /*max(*/m_threshold * l/*, m_lineThreshold)*/)
    return make_pair(nullptr, 0);
  ranges.push_back(minB);
  double xAngle = 0;
  if (m_dimension == 3) {
    if(!m_spokes.IsEdgeMapEmpty())
      xAngle = FindThirdRangeEllipsoid(_ei, _s, _t, ranges, minI);
    else
      ranges.push_back(minB);
  }
  // Compute the volume 
  double volume = PI;
  for (size_t i = 0; i < m_dimension; ++i)
    volume *= ranges[i];
  if (m_dimension == 3)
    volume *= (4 / 3);

  NEllipsoid* ellip;
  if (m_dimension == 2)
    ellip = new NEllipsoid(_c, ranges, EulerAngle(GetZAngle(_s, _t, mid), 0, 0));
  else
    ellip = new NEllipsoid(_c, ranges, EulerAngle(GetZAngle(_s, _t, mid), GetYAngle(_s, _t, mid), xAngle));
  return make_pair(ellip, volume);
}

pair<NOrientedBox*, double>
PlacePrimitives::
PlaceBox(EI& _ei, vector<double>&  _c, Point3d&  _s, Point3d& _t, double&  _r) {
  // Get mid and length
  auto mid = (_s + _t) * 0.5;
  auto l = (_t - _s).norm();
  // Return if min axis range is too small
  if(_r < m_threshold * l)
    return make_pair(nullptr, 0);
  vector<double> center(_c);
  // Get the ranges
  vector<double> ranges;
  ranges.push_back(0.5*l);
  ranges.push_back(_r);
  double xangle = 0;
  // Compute the volume 
  double volume = 1;
  if (m_dimension > 2){
    xangle = FindBoxThirdRange(_ei, _s, _t, ranges, center);
  }
  else {
	double e1, e2;
	if(m_isMin){
		double v1Cl = m_clearances.GetVertexProperty(_ei->source()).first; 
		e1 = min(ExtendBoxRange(_ei->source(), _s, _t, v1Cl , ranges[1]),v1Cl);//0.25*l);
		double v2Cl = m_clearances.GetVertexProperty(_ei->target()).first; 
		e2 = min(ExtendBoxRange(_ei->target(), _t, _s, v2Cl , ranges[1]),v2Cl);//0.25*l);
	}
	else{
	  e1 = 	m_clearances.GetVertexProperty(_ei->source()).second; 
	  e2 = m_clearances.GetVertexProperty(_ei->target()).second; 
	}
    auto e = /*min(e1, e2);*/0.5*(e1 + e2);
    auto line = (_t - _s).normalize(); 
    if(e1 > 0 && e2 > 0) {
      ranges[0] += e; 
      for(size_t i =0; i < m_dimension; ++i)
        center[i] += 0.5*(e2-e1)*line[i];
    }
  }
  // Compute the volume 
  for (size_t i = 0; i < m_dimension; ++i){ 
    volume *= (2 * ranges[i]);
  }
  NOrientedBox* box;
  if (m_dimension == 2)
    box = new NOrientedBox(center, ranges, EulerAngle(GetZAngle(_s, _t, mid), 0, 0));
  else
    box = new NOrientedBox(center, ranges, EulerAngle(GetZAngle(_s, _t, mid), GetYAngle(_s, _t, mid), xangle));
  return make_pair(box, volume);
}

pair<NOrientedBox*, double>
PlacePrimitives::
PlaceBox(VI& _vi, vector<double>&  _c, Point3d&  _s, double&  _r) {
  // Return if min axis range is too small
  if(_r < m_threshold)
    return make_pair(nullptr, 0);
  vector<double> center(_c);
  // Get the ranges
  vector<double> ranges;
  ranges.push_back(_r);

  vector<Vector3d> spokes, spokeVectors;
  GetSpokeVectorsByRadius(_vi->descriptor(), _r, spokes);
  // if spokes is empty
  if(spokes.empty()) 
    return make_pair(nullptr, 0);

  double minD = numeric_limits<double>::max(), maxC = 0;
  Vector3d minS; 
  // Find the minimum length spokes
  for(auto v: spokes){
    spokeVectors.push_back(v - _s);
    auto d = (v - _s).norm();
    if(d < minD){
      minD = d;
      minS = v;
    }
    maxC = max(maxC, d);
  } 
  // vector of x-axis 
  Vector3d xdir = (minS - _s).normalize();
  // get other end of the x-axis
  Vector3d t = _s - _r * xdir;
  double xangle = 0;
  if(m_dimension == 2) {
    auto yRange = FindHorizontalRange(spokeVectors, xdir, ranges[0]);
    if(yRange < numeric_limits<double>::max())
      ranges.push_back(yRange);
    else
      ranges.push_back(maxC);
  }
  else {
    Vector3d ydir = FindBoxThirdRange(spokeVectors, ranges, xdir);
    if(ranges.size() < 3) {
      ranges.push_back(minD); ranges.push_back(minD);
    }
    else
      xangle = GetXAngle(minS, t, _s, ydir);
  }
 
  // Compute the volume 
  double volume = 1;
  for (size_t i = 0; i < m_dimension; ++i){ 
    volume *= (2 * ranges[i]);
  }
 
  NOrientedBox* box;
  if (m_dimension == 2)
    box = new NOrientedBox(center, ranges, EulerAngle(GetZAngle(minS, t, _s), 0, 0));
  else
    box = new NOrientedBox(center, ranges, EulerAngle(GetZAngle(minS, t, _s), GetYAngle(minS, t, _s), xangle));
  return make_pair(box, volume);
}


/************ Helper - Extra dimension ****************************************/
double 
PlacePrimitives::
FindThirdRangeEllipsoid(EI& _ei, Point3d& _s, Point3d& _t, vector<double>&  _r, size_t _i) {
  // Get intermediates and spokes 
  auto inter = _ei->property();
  auto edgeSpokes = m_spokes.GetEdgeProperty(_ei->descriptor());
 
  // Find the vector for y-axis
  Vector3d yv(0,0,0);
  if(edgeSpokes.size() != inter.size()/*|| _i == 0 || _i == inter.size()-1*/) {
    _r.push_back(_r[1]);  
    return 0;
  }
  // Get the mid point 
  auto m = (_s + _t) * 0.5;
  // Get the spokes vectors at the minimum/maximum value point
  vector<pair<Vector3d, Vector3d>> spokes; 
  GetSpokeVectors(_ei, _s, _t, spokes, _i);
  double miny = m_isMin?numeric_limits<double>::max():0;
  for(auto s : spokes){
    auto v = s.second - s.first;
    auto d = v.norm();
    if((m_isMin && d < miny)||(!m_isMin && d > miny)){
      miny = d;
      yv = v;
    }
  }
  yv = yv.normalize();
  if(miny >= numeric_limits<double>::max() || miny <= 0) {
    _r.push_back(_r[1]); 
    return 0;
  }
  
  // Get the spokes vectors if maximum index was not entered
  if(_i < inter.size()){
    spokes.clear();
    GetSpokeVectors(_ei, _s, _t, spokes);
  }
	double radius = _r[0] - 0.5*(_t - _s).norm(); 
	if (radius > numeric_limits<double>::epsilon()) {
		GetSpokeVectorsByRadius(_ei->source(), radius, _s, _t, spokes);
		GetSpokeVectorsByRadius(_ei->target(), radius, _t, _s, spokes);
	}
  // Find the minimum/maximum z-axis range through each cross-section
  double minc = m_isMin? numeric_limits<double>::max(): 0;
  double maxC = 0;
  // For each spokes vector
  for(auto s: spokes){
    maxC = max(maxC, (s.second - s.first).norm());
    // Find the x^2/r[0]^2 at the intermediate position
    double x2 = (s.first - m).normsqr();
    x2 /= (_r[0]*_r[0]);
	// if x value is beyond range
    if(x2 >= 1) continue;
    // find y
    auto v = s.second - s.first;
    auto y2 = v * yv;
    y2 *= y2;
    // if y value is beyond range
    if(y2 >= (1-x2)*(_r[1]*_r[1])) continue;
	// find z
    auto c = sqrt(fabs(v.normsqr() - y2)); 
	y2 /= (_r[1] * _r[1]);
    // find z axis range by substituting y^2 and x^2 in 
    // ellipsoid equation
    c /= sqrt(1 - x2 - y2); 
    // Avoid outliers
    if(c > _r[1] && c <= maxC)
      minc = m_isMin? min(minc, c):max(minc, c);
  }
  if(minc <= maxC && minc > _r[1]){ 
    _r.push_back(minc);
    return GetXAngle(_s, _t, m, yv);
  } 
  _r.push_back(_r[1]);
  return 0;
}

double 
PlacePrimitives::
FindBoxThirdRange(EI& _ei, Point3d& _s, Point3d& _t, vector<double>&  _r, vector<double>&  _c) {
  // Get max bending of the line
  auto l = (_t - _s).normalize();
  auto inter = _ei->property();
  
  // Get the spokes vectors
  vector<pair<Vector3d, Vector3d>> spokes;
  GetSpokeVectors(_ei, _s, _t, spokes);
  // Get the direction for y-axis
  Vector3d yv;
  double miny = m_isMin?numeric_limits<double>::max():0;
  double maxC = 0;
  vector<Vector3d> v;
  for(auto s : spokes) {
    // find distance of the projected point from the intermediate
    auto d = (s.second - s.first).norm();
    if((m_isMin && d < miny)||(!m_isMin && d > miny)) {
      miny = d;
      yv = (s.second - s.first);
    }
    // maximum length spoke vector
    maxC = max(maxC, d);
    // store it in vector form
    v.push_back(s.second - s.first);
  }
  yv = yv.normalize();
  auto m = (_s + _t) * 0.5; 
  auto angle = GetXAngle(_s, _t, m, yv);
  double dev = 0; // deviation in line approximation
  for(auto p : inter){
    auto vec = (p - _s);
    auto proj = _s + (vec*l) * l; 
    dev = max(fabs((p - proj)*yv), dev);
  }
  // If checking for maximum clearance - no need to account for deviation
  if(!m_isMin)  
	dev = 0;
  // Get the z-range
  auto minZ = FindHorizontalRange(v, yv, _r[1]-dev/*, 0.5*m_lineThreshold*/);
  //auto diagonal = _r[1] / sqrt(2);

  // Found a positive z-axis range
  if(minZ <= maxC /*&& minZ >= m_lineThreshold*/) {
    _r[1] -= dev;
    _r.push_back(minZ); 
  }
  else {
    // Store the ranges as the diagonal of y and z-axes as the clearnace value
    //_r[1] = diagonal;
    _r.push_back(_r[1]);
    //yv = yv.rotate(l, 0.785398);
    //angle += 0.785398;
  } 
  // Extend the box beyond the end vertices of the line 
  double e1, e2;
  if(m_isMin){
	double v1Cl = m_clearances.GetVertexProperty(_ei->source()).first; 
	e1 = min(ExtendBoxRange(_ei->source(), _s, _t, v1Cl , _r[1], 3, _r[2], yv),v1Cl);
	double v2Cl = m_clearances.GetVertexProperty(_ei->target()).first; 
	e2 = min(ExtendBoxRange(_ei->target(), _t, _s, v2Cl , _r[1], 3, _r[2], yv),v2Cl);
  }
  else {
	e1 = m_clearances.GetVertexProperty(_ei->source()).second; 
	e2 = m_clearances.GetVertexProperty(_ei->target()).second;
  }
  auto e = 0.5*(e1 + e2);
  if(e1 > 0 || e2 > 0) {
    _r[0] += e; 
    for(size_t i =0; i < m_dimension; ++i)
      _c[i] += 0.5*(e2-e1)*l[i];
  }
  return angle;
}


Vector3d
PlacePrimitives::
FindBoxThirdRange(vector<Vector3d>& _spokeVectors, vector<double>&  _r, Vector3d& _xdir) {
  // Find the y-direction as the smallest vector on the y-z plane
  double  minD = numeric_limits<double>::max();
  Vector3d ydir; 
  for(auto v: _spokeVectors) {
    // Projection on x- axis 
    auto xd = v * _xdir;
    // Check whether within x-range
    if(fabs(xd) > _r[0]) continue;
    // projected vector on y-z plane
    auto yzproj = v - xd * _xdir;
    auto yd = yzproj.norm();
    if(minD > yd) {
      minD = yd;
      ydir = yzproj;
    }
  }
  // Found a minimum y-direction
  if(minD < numeric_limits<double>::max()){
    ydir.selfNormalize();
    auto zRange = FindHorizontalRange(_spokeVectors, ydir, _xdir, _r[0], minD);
    //auto diagonal = minD/sqrt(2);
    
    // found a z range that is large enough compared to y range
    if(zRange < numeric_limits<double>::max() /*&& zRange >= diagonal-m_lineThreshold*/) {
      _r.push_back(minD);
      _r.push_back(zRange);
    }
    else{ // Allign the corner of y-z plane along the minimum spoke
      _r.push_back(minD);
      _r.push_back(minD);
      //ydir = ydir.rotate(_xdir, 0.785398);
    }
  }
  return ydir;
}

double 
PlacePrimitives::
FindHorizontalRange(vector<Vector3d>& _v, Vector3d& _yv, double _yrange, double _err) {
  double minZ = m_isMin?numeric_limits<double>::max():0;
  // To get the maximum z value for vectors with y = _yrange 
  double eqYZ = 0; 
  bool hasOnRange = false; 
  for(auto v : _v) {
    // find y as projection of the spokes vector on the y-axis
    auto y = v * _yv;
    // Only consider the spoke vector which are within the y-axis range
    if(fabs(y) > _yrange + _err)
      continue;
    // find z 
    auto z = sqrt(fabs(v.normsqr() - y*y)); 
    // Find minimum z for spokes vector which has y-value less than y-axis range
    if(fabs(y) < _yrange - _err)
      minZ = m_isMin?min(minZ, z):max(minZ,z);
    // if spoke point lies on the y-axis range get their maximum value
    else {
      eqYZ = max(eqYZ, z);
      hasOnRange = true;
    }
  }
 
  // if there are only spokes that lie on the edge of the y-axis range
  // consider the maximum z-value of the spokes on the y-axis range
  if(hasOnRange){
	if(m_isMin)
	  minZ = min(minZ, eqYZ);
	else 
	  minZ = max(minZ, eqYZ);
  }
  return minZ;
}

double
PlacePrimitives::
FindHorizontalRange(vector<Vector3d>& _v, Vector3d& _zv, Vector3d& _yv, 
  double _yrange, double _zrange, double _err){
  double minX = numeric_limits<double>::max();
  // To get the maximum z value for vectors with y = _yrange 
  double eqYZ = 0; 
  bool hasOnRange = false; 
  
  for(auto v : _v) {
    // Get z projection
    auto z = v * _zv;
    // Only consider the vector within range
    if(fabs(z) > _zrange + _err)
      continue;
    // Get y projection
    auto y = v * _yv;
    // Only consider the vector within range
    if(fabs(y) > _yrange + _err)
      continue;
    auto x = sqrt(fabs(v.normsqr() - y*y - z*z));
    // Find minimum z for spokes vector which has y-value less than y-axis range
    if(fabs(y) < _yrange - _err && fabs(z) <  _zrange - _err)
      minX = min(minX, x);
    // if spoke point lies on the y-axis range get their maximum value
    else {
      eqYZ = max(eqYZ, x);
      hasOnRange = true;
    }
  }
  if(hasOnRange)
    minX = min(minX, eqYZ);
  return minX;
}

double
PlacePrimitives::
ExtendBoxRange(VD _src, Point3d& _s, Point3d& _t, double&  _r, double& _vr, size_t _dim,
  double _zr, Vector3d _ydir) {
  vector<Vector3d> wV;
  double range = 0;
 
  if(_r == 0 || _vr == 0)
    return range;
  
  // Initialization 
  // direction of the line directed towards the source vertex
  Vector3d direction = (_s - _t).normalize();
 
  // Store the start vertex in queue
  double radius = (_r*_r + _vr*_vr);
  if(_dim < 3)
    radius = sqrt(radius);
  else
    radius = sqrt(radius + _zr*_zr);
  // Get the spokes vector within the semi circle
  GetSpokeVectorsByRadius(_src, radius, direction, wV);
  
  // if it is 3 dimension and there are no extra spokes
  if(_dim > 2 && wV.empty()){
    return range;
  }

  Vector3d ydir(-direction[1], direction[0], 0);
  if(wV.empty())
    range = _r;
  else if(_dim == 2) {
    range = FindHorizontalRange(wV, ydir, _vr-0.1*m_lineThreshold);
    range = min(range, _r);
  }
  else {
    auto zdir = (direction%_ydir).normalize();
    range = FindHorizontalRange(wV, zdir, _ydir, _vr, _zr);
  }
  if(range == numeric_limits<double>::max())
    range = _r;
  return range;
}



/************ Helper - Spokes vectors *****************************************/
void 
PlacePrimitives::
GetSpokeVectors(EI& _ei, Point3d& _s, Point3d& _t, 
    vector<pair<Vector3d,Vector3d>>& _spoke, size_t _index) {
  // Get center and normalized vector of the line segment
  Vector3d mid = (_s + _t) * 0.5;
  Vector3d l = (_t - _s).normalize(); 
  double halflen = (mid - _s).norm();
 
  // Get intermediates and spokes 
  auto inter = _ei->property();
  auto edgeSpokes = m_spokes.GetEdgeProperty(_ei->descriptor()); 
  size_t si = 0, ei = inter.size();
  if(_index < inter.size()) {
    si = _index;
    ei = _index + 1; 
  }

  for(size_t i = si; i < ei; i++) {
    // For each spokes point of each intermediate
    for(auto s: edgeSpokes[i]){
      // projection of the spoke point on the line and its distance from midpoint
      Vector3d p = _s + l * (l * (s - _s));
      // if projection is within the line segment 
      double hl = (p - mid).norm(); 
      if (hl <= halflen)
        _spoke.emplace_back(make_pair(p, s));
    }
  }
  if(_index < inter.size())
    return;
  auto vSpokes = m_spokes.GetVertexProperty(_ei->source());
  // For each spokes point of each intermediate
  for(auto s : vSpokes) {
    // projection of the spoke point on the line and its distance from midpoint
      Vector3d p = _s + l * (l * (s - _s));
      // if projection is within the line segment 
      double hl = (p - mid).norm(); 
      if (hl <= halflen)
        _spoke.emplace_back(make_pair(p, s));
  }
  vSpokes.clear();
  vSpokes = m_spokes.GetVertexProperty(_ei->target());
  // For each spokes point of each intermediate
  for(auto s : vSpokes) {
    // projection of the spoke point on the line and its distance from midpoint
      Vector3d p = _s + l * (l * (s - _s));
      // if projection is within the line segment 
      double hl = (p - mid).norm(); 
      if (hl <= halflen)
        _spoke.emplace_back(make_pair(p, s));
  }
}

void
PlacePrimitives::
GetSpokeVectorsByRadius(VD _src, double _r, const Point3d& _s, const Point3d& _t, vector<pair<Vector3d, Vector3d>>& _spoke) {
	Vector3d dir = (_s - _t).normalize();
	vector<Vector3d> sVs;
	GetSpokeVectorsByRadius(_src, _r, dir, sVs);
	for (auto v : sVs) {
		Vector3d second = v + _s;
		Vector3d first = _s + (v*dir) *dir;
		_spoke.push_back(make_pair(first, second));
	}
}

void
PlacePrimitives::
GetSpokeVectorsByRadius(VD _s, double _r, Vector3d _dir, vector<Vector3d>& _spoke) {
  queue<size_t> q;
  unordered_set<size_t> visitedVertices;
  unordered_set<WorkspaceSkeleton::ED, edgeHash> storedEdges;

  // Initialization 
  // Get the skeleton graph
  auto g = m_skeleton->GetGraph(); 
  // Get source vertex
  auto sit = g.find_vertex(_s);
  
  // Store the start vertex in queue
  q.push(_s);
  // BFS traversal
  while(!q.empty()){
    auto v = q.front(); q.pop();
    // if v not visited
    if(visitedVertices.find(v) != visitedVertices.end())
     continue;
    // mark v to be visited
    visitedVertices.insert(v);
    auto vit = g.find_vertex(v);
    // for every outgoing edge check targets
    for (auto eit = vit->begin(); eit != vit->end(); ++eit) {
      // Get the other vertex
      auto o = eit->target();
      auto op = g.find_vertex(o)->property();
      Vector3d l = (op - sit->property());
      // if the skeleton edge is not at acute angle - semicircle containment check
      if((l.normalize()*_dir) < 0)
        continue;
      // if already visited vertex then outgoing edges already considered
      if(visitedVertices.find(o) != visitedVertices.end())
        continue;
      // store the edge
      storedEdges.insert(eit->descriptor());
      // Push in the queue if within the semi-circle
      if(l.norm() <= _r) 
        q.push(o);
    }
    // for all incoming edges
    auto pred = vit->predecessors();
    for(auto pit = pred.begin(); pit != pred.end(); ++pit){
      auto o = *pit;
      auto oit = g.find_vertex(o);
      auto op = oit->property();
      Vector3d l = (op - sit->property());
      // if the skeleton edge is not at acute angle - semicircle containment check
      if((l.normalize()*_dir) < 0)
        continue;
      // if already visited vertex then outgoing edges already considered
      if(visitedVertices.find(o) != visitedVertices.end())
        continue;
      for(auto eit = oit->begin(); eit != oit->end(); ++eit)
        if(eit->target() == v) 
          storedEdges.insert(eit->descriptor());
      // Push in the queue if within the semi-circle
      if(l.norm() <= _r)
        q.push(o);
    }
  }
 
  // create spokes vector out of the stored skeleton edge witnesses
  for(auto se: storedEdges) {
    auto edgeSpokes = m_spokes.GetEdgeProperty(se);
    for(auto seV : edgeSpokes) {
      for(auto s : seV) {
        Vector3d sv = s - sit->property();
        if(sv.normalize()*_dir >= 0)
          _spoke.push_back(sv);
      }
    }
  }
}


void 
PlacePrimitives::
GetSpokeVectorsByRadius(VD _s, double _r, vector<Vector3d>& _spoke){
  queue<size_t> q;
  unordered_set<size_t> visitedVertices;
  unordered_set<WorkspaceSkeleton::ED, edgeHash> storedEdges;

  // Initialization 
  // Get the skeleton graph
  auto g = m_skeleton->GetGraph(); 
  // Get source vertex
  auto sit = g.find_vertex(_s);
  // Store the start vertex in queue
  q.push(_s);
  // BFS traversal
  while(!q.empty()){
    auto v = q.front(); q.pop();
    // if v not visited
    if(visitedVertices.find(v) != visitedVertices.end())
     continue;
    // mark v to be visited
    visitedVertices.insert(v);
    auto vit = g.find_vertex(v);
    // for every outgoing edge check targets
    for (auto eit = vit->begin(); eit != vit->end(); ++eit) {
      // Get the other vertex
      auto o = eit->target();
      // if already visited vertex then outgoing edges already considered
      if(visitedVertices.find(o) != visitedVertices.end())
        continue;
      // store the edge
      storedEdges.insert(eit->descriptor());
      Vector3d l = (g.find_vertex(o)->property() - sit->property());
      // Push in the queue if within the semi-circle
      if(l.norm() <= _r) 
        q.push(o);
    }
    // for all incoming edges
    auto pred = vit->predecessors();
    for(auto pit = pred.begin(); pit != pred.end(); ++pit){
      auto o = *pit;
      auto oit = g.find_vertex(o);
      Vector3d l = (oit->property() - sit->property());
      // if already visited vertex then outgoing edges already considered
      if(visitedVertices.find(o) != visitedVertices.end())
        continue;
      for(auto eit = oit->begin(); eit != oit->end(); ++eit)
        if(eit->target() == v) 
          storedEdges.insert(eit->descriptor());
      // Push in the queue if within the semi-circle
      if(l.norm() <= _r)
        q.push(o);
    }
  }

  // create spokes vector out of the stored skeleton vertex witnesses
  for(auto se: storedEdges) {
    auto edgeSpokes = m_spokes.GetEdgeProperty(se);
    for(auto seV : edgeSpokes) {
      for(auto s : seV)
        _spoke.push_back(s);
    }
  }
}

/************ Helper - Euler Angles *******************************************/

double 
PlacePrimitives::
GetZAngle(Point3d& _p1, Point3d& _p2, Point3d& _m) {
  const double denom = sqrt(pow((_p2[0] - _m[0]), 2) + pow((_p2[1] - _m[1]), 2)); 
  // too small denominator
  if(fabs(denom) < numeric_limits<double>::epsilon()) return 0;
  double angle = 1;
  if (_p2[1] > _m[1] || (_p2[1] == _m[1] &&  _p2[0] > _m[0]))
    angle = (_p2[0] - _m[0]) / denom;
  else
    angle = (_p1[0] - _m[0]) / denom;
  if (angle > 1) 
    return acos(1);
  else if(angle < -1)
    return acos(-1);
  else
    return acos(angle);
}

double
PlacePrimitives::
GetYAngle(Point3d& _p1, Point3d& _p2, Point3d& _m) {
  if (_p2[1] > _m[1] || (_p2[1] == _m[1] && _p2[0] > _m[0])
    ||( _p2[1] == _m[1] &&  _p2[0] == _m[0] &&  _p2[2] > _m[2]))
    return -asin((_p2[2] - _m[2]) / (_p2 - _m).norm());
  else
    return -asin((_p1[2] - _m[2]) / (_p1 - _m).norm());
}

double 
PlacePrimitives::
GetXAngle(Point3d& _p1, Point3d& _p2, Point3d& _m, Vector3d& _y) {
  // Get a point on rotated y axis and x-axis
  Vector3d y0(0, 0, 0);
 // Get the x-axis along the line-segment : (XD, YD, ZD)
  Vector3d x;
  if (_p2[1] > _m[1] || (_p2[1] == _m[1] && _p2[0] > _m[0])
    ||( _p2[1] == _m[1] &&  _p2[0] == _m[0] &&  _p2[2] > _m[2])) 
    x = (_p2 - _m);
  else 
    x = (_p1 - _m);
  x = x.normalize();
  // get the y axis with x-axis along the line segment (-YD, XD, 0)
  y0[0] = -x[1];
  y0[1] = x[0];
  y0 = y0.normalize();
  if(y0.norm() < numeric_limits<double>::epsilon())
    y0 = Vector3d(0, 1, 0);

  auto y = _y;
  if(y*y0 < 0)
    y = -y;
  //Calculate the angle : ZYX format considering the rotation of y axis
  //auto angle = atan2((x % y0).normalize()* y, y0 * y);
	
  // New angle calculation :ZYX format
  // get the negative z-axis as cross of y0 and x 
  auto z0 = (y0 % x).normalize();
  auto z = (y % x).normalize();
  auto newangle = atan2(y0*z, z0*z);
 //cout << "Z angles " << angle << " " << newangle << endl;

  return newangle;
}

