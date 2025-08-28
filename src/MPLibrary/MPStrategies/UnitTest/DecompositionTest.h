#ifndef DECOMPOSITION_TEST_H
#define DECOMPOSITION_TEST_H

#include "MPLibrary/MPStrategies/MPStrategyMethod.h"
#include "Geometry/GMSPolyhedron.h"
#undef PI
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/boost/graph/convert_nef_polyhedron_to_polygon_mesh.h>
#include <CGAL/subdivision_method_3.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_mesh_processing/refine.h>
#include <CGAL/Polygon_mesh_processing/fair.h>
#include <boost/function_output_iterator.hpp>
#include <CGAL/IO/Nef_polyhedron_iostream_3.h>
#include <CGAL/Nef_3/SNC_indexed_items.h>
#include <CGAL/convex_decomposition_3.h> 
////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @brief This is to test decompose a space into nearly convex parts
///
/// TODO
///
/// \internal This strategy is configured for pausible execution.
////////////////////////////////////////////////////////////////////////////////

template <typename MPTraits>
class DecompositionTest : public MPStrategyMethod<MPTraits> {

  public:

    typedef typename MPTraits::CfgType CfgType;
		typedef CGAL::Nef_polyhedron_3<typename GMSPolyhedron::CGALKernel, CGAL::SNC_indexed_items> NefPolyhedronIndexed;

    DecompositionTest();
    DecompositionTest(XMLNode& _node);

    virtual void Initialize();
    virtual void Iterate();
    virtual void Finalize();
    virtual void Print(ostream& _os) const;
		
		void GetObstacleSpace();
		void GetFreeSpace();
		void WriteToFile(string _filebase, GMSPolyhedron::CGALPolyhedron& _poly);
		void ConvexDecomposition();

  protected:
    size_t m_space;
		string m_filename;
		GMSPolyhedron::CGALPolyhedron m_input;
    bool m_debug;
};

template <typename MPTraits>
DecompositionTest<MPTraits>::
DecompositionTest() {
  this->SetName("DecompositionTest");
}

template <typename MPTraits>
DecompositionTest<MPTraits>::
DecompositionTest(XMLNode& _node) :
  MPStrategyMethod<MPTraits>(_node)	{
    this->m_space = _node.Read("space", false, 1, 0, MAX_INT, "Free/obstacle/both space construction");
  //  this->m_params.m_wH = _node.Read("quality", false, 0.1, 0.0, MAX_DBL,
  //    "Quality Parameter");
		this->m_filename = _node.Read("filebase", false, "example",
			"filename for output files");
    this->SetName("DecompositionTest");
}

template <typename MPTraits>
void
DecompositionTest<MPTraits>::
Print(ostream& _os) const {
  
}

template <typename MPTraits>
void
DecompositionTest<MPTraits>::
Initialize() {
	if(this->m_debug)
    std::cout << "DecompositionTest::Initialize()" << std::endl;
	if (m_space == 1)
		GetFreeSpace();
	else
		GetObstacleSpace();
  ConvexDecomposition();
}

template <typename MPTraits>
void
DecompositionTest<MPTraits>::
Iterate() {
 
}

template <typename MPTraits>
void
DecompositionTest<MPTraits>::
Finalize() {
//	WriteToFile(m_filename, m_input);
}

template <typename MPTraits>
void
DecompositionTest<MPTraits>::
GetObstacleSpace() {
	auto env = this->GetEnvironment();
	auto cp = env->GetBoundary()->CGAL();
	NefPolyhedronIndexed obstspace(NefPolyhedronIndexed::EMPTY);
	// Add all obstacle
	for (size_t i = 0; i < env->NumObstacles(); ++i) {
		if (m_debug)
			cout << "\tAdding obstacle " << i << "..." << endl;

		StaticMultiBody* obst = env->GetObstacle(i);
		if (!obst->IsInternal()) {
			// Make CGAL representation of this obstacle.
			auto ocp = obst->GetFixedBody(0)->GetWorldPolyhedron().CGAL();
			if (m_debug)
				cout << "\t\tobstacle is " << (ocp.is_closed() ? "" : "not ")
				<< "closed" << endl;

			// Find the intersection of the obstacle with the boundary 
			NefPolyhedronIndexed obst(cp);
			obst *= NefPolyhedronIndexed(ocp);
			// Add it to the obstaclespace.
			obstspace += obst;//NefPolyhedron(ocp);
		}
	}
	obstspace.convert_to_polyhedron(m_input);
}

template <typename MPTraits>
void
DecompositionTest<MPTraits>::
GetFreeSpace() {
	auto env = this->GetEnvironment();
  auto cp = env->GetBoundary()->CGAL();
	NefPolyhedronIndexed freespace(cp);
	// Subtract each obstacle from the freespace.
	for (size_t i = 0; i < env->NumObstacles(); ++i) {
		if (m_debug)
			cout << "\tAdding obstacle " << i << "..." << endl;

		StaticMultiBody* obst = env->GetObstacle(i);
		if (!obst->IsInternal()) {
			// Make CGAL representation of this obstacle.
			auto ocp = obst->GetFixedBody(0)->GetWorldPolyhedron().CGAL();

			if (m_debug)
				cout << "\t\tobstacle is " << (ocp.is_closed() ? "" : "not ")
				<< "closed" << endl;

			// Subtract it from the freespace.
			freespace -= NefPolyhedronIndexed(ocp);
		}
	}
	freespace.convert_to_polyhedron(m_input);
}

template <typename MPTraits>
void
DecompositionTest<MPTraits>::
WriteToFile(string _filebase, GMSPolyhedron::CGALPolyhedron& _poly) {
	ofstream ofs(_filebase + ".off");
	ofs << _poly;
	ofs.close();
}

template <typename MPTraits>
void
DecompositionTest<MPTraits>::
ConvexDecomposition() {
 NefPolyhedronIndexed input(m_input);
	CGAL::convex_decomposition_3(input);
	vector<typename GMSPolyhedron::CGALPolyhedron> results;
	// store convex polyhedron
	typedef typename NefPolyhedronIndexed::Volume_const_iterator VCI;
	VCI ci = ++input.volumes_begin();
	for (; ci != input.volumes_end(); ++ci) {
		if (ci->mark()) {
			GMSPolyhedron::CGALPolyhedron poly;
			input.convert_inner_shell_to_polyhedron(ci->shells_begin(), poly);
			results.push_back(poly);
		}
	}
  cout<<"Number of components : "<<results.size()<<endl;
	// write to file
	/*for(size_t i = 0; i < results.size(); ++i) {
		stringstream str(m_filename);
		str << i;
		string filename;
		str >> filename;
		WriteToFile(filename, results[i]);
	}*/
}

#endif
