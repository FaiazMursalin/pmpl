#include <cmath>

#include "GridOverlay.h"

#include "Geometry/Boundaries/Boundary.h"
#include "Utilities/MPUtils.h"
#include "Workspace/WorkspaceDecomposition.h"


/*------------------------------- Construction -------------------------------*/

GridOverlay::
GridOverlay(const Boundary* const _b, const double _length):
    m_boundary(_b), m_dimension(_b->GetDimension()), m_length(_length) {
  // Compute the number of cells in each dimension.
  for(size_t i = 0; i < m_dimension; ++i)
    m_num[i] = std::ceil(m_boundary->GetRange(i).Length() / m_length);

  if(m_debug) {
    std::cout << "Dimension = " << m_dimension << endl
              << "Computed grid overlay with " << Size() << " cells "
              << "(" << Size(0) << "x" << Size(1) << "x" << Size(2) << ") "
              << "of length " << m_length << "."
              << std::endl;
  }
}

/*------------------------------- Cell Finding -------------------------------*/

size_t
GridOverlay::
Size() const noexcept {
  size_t size = 1;
  for(size_t i = 0; i < m_dimension; i++) {
    size *= m_num[i];
  }
  return size;
}


size_t
GridOverlay::
Size(const size_t _i) const noexcept {
  if(_i > m_dimension-1)
    return 0;
  return m_num[_i];
}


size_t
GridOverlay::
LocateCell(const Cfg& _cfg) const {
  return LocateCell(_cfg.GetPoint());
}


size_t
GridOverlay::
LocateCell(const Point3d& _p) const {
  return CellIndex(Cell(_p));
}

Point3d 
GridOverlay::
CellMidPoint(const size_t& _i) const {
  Point3d p(0,0,0);
  size_t index[3];
  index[0] = XIndex(_i);
  index[1] = YIndex(_i);
  index[2] = ZIndex(_i);
  for(size_t i = 0; i < m_dimension; ++i){
    p[i] = m_boundary->GetRange(i).min + (0.5 + index[i]) * m_length;
  }
  return p;
}

std::vector<size_t>
GridOverlay::
LocateBBXCells(const Boundary* const _b) const {
  Point3d min, max;

  for(size_t i = 0; i < m_dimension; ++i) {
    auto range = _b->GetRange(i);
    min[i] = range.min;
    max[i] = range.max;
  }

  return LocateBBXCells(min, max);
}


std::vector<size_t>
GridOverlay::
LocateBBXCells(const Point3d& _min, const Point3d& _max) const {
  const auto min = Cell(_min);
  const auto max = Cell(_max);

  // Create space for the appropriate number of cells.
  std::vector<size_t> output;
  output.reserve((max[0] - min[0] + 1) *
                 (max[1] - min[1] + 1) *
                 (max[2] - min[2] + 1));

  // Populate the cell list.
  for(size_t z = min[2]; z <= max[2]; ++z) {
    for(size_t y = min[1]; y <= max[1]; ++y) {
      const size_t first = CellIndex(min[0], y, z);
      const size_t last  = CellIndex(max[0], y, z);
      for(size_t i = first; i <= last; ++i)
        output.push_back(i);
    }
  }

  return output;
}

/*-------------------------- Decomposition Mapping ---------------------------*/

GridOverlay::DecompositionMap
GridOverlay::
ComputeDecompositionMap(const WorkspaceDecomposition* const _decomposition) const
{
  DecompositionMap map(this->Size());

  // For each region, find the grid cells that are associated with it.
  for(auto iter = _decomposition->begin(); iter != _decomposition->end(); ++iter)
  {
    auto region = &iter->property();
    /// @TODO Support locating the colliding cells using PQP Solid.
    auto cells = LocateBBXCells(region->GetBoundary());
    for(auto index : cells)
      map[index].push_back(region);
  }

  return map;
}

std::vector<size_t>
GridOverlay::
LocatePrimitiveCells(NShape* _s) {
  std::vector<size_t> output;
  auto extremePts = _s->AxisAlignedExtremePoints();
  auto indicesAABB = LocateBBXCells(extremePts.first, extremePts.second);
  for(size_t i: indicesAABB) {
    auto pt = CellMidPoint(i);
    std::vector<double> point({pt[0],pt[1]}); 
    if(m_dimension > 2) 
      point.push_back(pt[2]);
    if(_s->Contains(point))
      output.push_back(i);
  }
  return output;
}

/*------------------------------- Helpers ------------------------------------*/

std::array<size_t, 3>
GridOverlay::
Cell(const Point3d& _p) const noexcept {
  std::array<size_t, 3> cell;

  for(size_t i = 0; i < m_dimension; ++i) {
    const auto range = m_boundary->GetRange(i);
    cell[i] = m_num[i] * std::max(0.0, (_p[i] - range.min)) / range.Length(); 
    // Catch edge-case for cells on the maximal boundary.
    cell[i] = std::min(cell[i], m_num[i] - 1);
  }
  if(m_dimension < 3) 
    cell[2] = 0;
  return cell;
}


size_t
GridOverlay::
CellIndex(const size_t _x, const size_t _y, const size_t _z) const noexcept {
  return (m_num[0] * m_num[1]) * _z + m_num[0] * _y + _x;
}


size_t
GridOverlay::
CellIndex(const array<size_t, 3>& _indexes) const noexcept {
  return CellIndex(_indexes[0], _indexes[1], _indexes[2]);
}


size_t
GridOverlay::
ZIndex(const size_t _index) const noexcept {
  // To find the z-index, we see how many x-y slices we can remove.
  return _index / (m_num[0] * m_num[1]);
}


size_t
GridOverlay::
YIndex(const size_t _index) const noexcept {
  // To find the y-index, we first remove the z components and then see how many
  // x rows we can remove.
  return (_index % (m_num[0] * m_num[1])) / m_num[0];
}


size_t
GridOverlay::
XIndex(const size_t _index) const noexcept {
  // To find the x-index, we remove the z and y components.
  return _index % (m_num[0] * m_num[1]) % m_num[0];
}

bool
GridOverlay::
ContainsPrimitive(const size_t _index, PrimitiveSkeleton* _ps) const noexcept {
  //Get the mid-point of the cell
  auto p = CellMidPoint(_index);
  vector<double> point({p[0],p[1]});
  if(m_dimension > 2) 
    point.push_back(p[2]);
  //iterate through sphere
	for (size_t v = 0; v < _ps->GetNumberSpheres(); v++) {
    if(_ps->GetSphereIndex(v)->Contains(point)) 
      return true;
  }
  //iterate through capsule
	for (size_t v = 0; v < _ps->GetNumberCapsules(); v++) {
    if(_ps->GetCapsuleIndex(v)->Contains(point)) 
      return true;
  }
	//iterate through ellipsoid
	for (size_t v = 0; v < _ps->GetNumberEllipsoids(); v++) {
    if(_ps->GetEllipsoidIndex(v)->Contains(point)) 
      return true;
  }
  //iterate through box
  for (size_t v = 0; v < _ps->GetNumberBoxes(); v++) {
    if(_ps->GetBoxIndex(v)->Contains(point)) 
      return true;
  }
  return false;
}

/*----------------------------------------------------------------------------*/
