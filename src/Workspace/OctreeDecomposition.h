#ifndef OCTREE_DECOMPOSITION_H_
#define OCTREE_DECOMPOSITION_H_

#include <vector>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Kd_tree.h>
#include <CGAL/Fuzzy_iso_box.h>
#include <CGAL/Search_traits_2.h>
#include <CGAL/Search_traits_3.h>

#include "ConfigurationSpace/Cfg.h"
#include "Vector.h"
#include "Utilities/SegmentTrees.h"
#include "Utilities/MetricUtils.h"

using namespace std;

class Boundary;
class Environment;
///////////////////////////////////////////////////////////////////////////////
///
/// Voxmap representation as k-d tree of the boundary walls mid-point
///
//////////////////////////////////////////////////////////////////////////////
class VoxMap {
  public:
    ///@name Local Types
    ///@{
		typedef CGAL::Simple_cartesian<double> K;
    typedef K::Point_2 Point2;
    typedef K::Point_3 Point3;
    typedef CGAL::Search_traits_2<K> Traits2D;
    typedef CGAL::Kd_tree<Traits2D> Tree2D;
    typedef CGAL::Fuzzy_iso_box<Traits2D> QueryBox2D;
    typedef CGAL::Search_traits_3<K> Traits3D;
    typedef CGAL::Kd_tree<Traits3D> Tree3D;
    typedef CGAL::Fuzzy_iso_box<Traits3D> QueryBox3D;
    ///@}
    ///@name Construction
    ///@{
    VoxMap(size_t _d);
    ~VoxMap();

    void BuildTree();
    void InsertPoint(Point3d&  _pt);
    
    ///@}
    ///@name Access
    ///@{
    size_t Search(Point3d& _min, Point3d& _max);
    ///@}
  private:
    size_t m_dimension{3};
    Tree2D* m_tree2{nullptr};
    Tree3D* m_tree3{nullptr};
};

///////////////////////////////////////////////////////////////////////////////
///
/// Octree node or cell representation
///
//////////////////////////////////////////////////////////////////////////////
class OctreeNode {
  public:
		///@}
		///@name Construction
		///@{

		OctreeNode(); 

    /// construction by passing the parent and corner pt and index
    OctreeNode(Point3d& _pt, OctreeNode* _p=nullptr);

		/// Construction by passing the children 
		OctreeNode(OctreeNode** _c, size_t _dim = 3);

		/// Destructor for the Octree node, deletes the sub-branch too
		~OctreeNode(); 



		///@}
		///@name Accessors
		///@{

		bool IsLeaf(); 
		bool IsRoot(); 
		Point3d GetCorner(); 
		size_t GetType(); 
    size_t GetLevel(); 
		OctreeNode* GetParent(); 
		OctreeNode* GetChild(size_t _i);

		///@}
		///@name Set functions
		///@{

		void SetCorner(Point3d& _p);
		void SetType(size_t _t);
		void SetParent(OctreeNode* _p);
		void SetChild(size_t _i, OctreeNode* _c);

		///@}
  private:

		///@name Internal State
		///@{
		Point3d m_corner; /// the bottom left corner of the cell
		size_t m_type{ 3 }; /// the data for the cell : 0 - free space, 1 - obstacle space, 2 - mixed type
    size_t m_level{ 0 }; /// level in the hierarchy : 0 - root
		OctreeNode* m_parent{ nullptr }; /// Pointer to the parent node
		OctreeNode* m_children[8];  /// Pointers to the children nodes
		///@}
};

///////////////////////////////////////////////////////////////////////////////
/// A octree representation of an environment
///
/// Each cell in each level denotes the occupancy data of the environment
/// The cell is marked 0 for free space, 1 for obstacle space and 2 on mixed
/// type. Lowest level provides a grid map of the environment.
///////////////////////////////////////////////////////////////////////////////
class OctreeDecomposition {

  public:

    ///@name Local Types
    ///@{

    
    ///@}
    ///@name Construction
    ///@{
    
    /// Construct a octree with lowest possible cells of a given length.
    /// @param _e The environment
    /// @param _length The cell length to use.
    OctreeDecomposition(const Environment* _e, const double _length, StatClass* _stat=nullptr);
    
    ///@}
    ///@name Accessors
    ///@{

    size_t GetDimension() { return m_dimension; }
    const double GetLength() { return m_length; }
    size_t GetNumberOfLevels() { return m_levels; } 

    ///@}
    ///@name traversals
    ///@{
 
    void GetFreeRegions(vector<pair<Point3d, size_t>>& _ret, double _length = 0);
    
    ///@}

  private:

    ///@name Helpers
    ///@{
		/// Build the segment trees of obstacle's bounding boxes
    SegmentTrees<size_t>* BuildSegmentTrees(const Environment* _e);
    /// Build voxel map storing the mid-point of wall between cell of different validity
		VoxMap* BuildVoxMap(SegmentTrees<size_t>* _st, Point3d& _min, Point3d& _max, const double& _length);
    /// Function to determine the validity of a point
		bool IsInsideObstacles(Point3d& _p, const Environment* _e, SegmentTrees<size_t>* _st);

		/// Determine the type as free(0), obstacle(1) or mixed
    size_t DetermineType(SegmentTrees<size_t>* _st, VoxMap* _vm, Point3d& _lp, Point3d& _gp);
    ///@}
    ///@name Internal State
    ///@{

    const Environment* m_env; ///< The environment.
    size_t m_dimension;         ///< The dimension of the boundary
    double m_length;            ///< Length of root node.

		OctreeNode* m_root;     ///< Root of the octree
    size_t m_levels{0};    ///< Number of levels in the octree
   
    static constexpr bool m_debug{true}; ///< Enable debugging messages?

    ///@}
};



#endif
