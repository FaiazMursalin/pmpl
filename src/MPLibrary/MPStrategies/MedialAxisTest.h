#ifndef MEDIAL_AXIS_TEST_H
#define MEDIAL_AXIS_TEST_H

#include "MPStrategyMethod.h"
#include "Utilities/MedialAxis2D.h"
#include "Utilities/SegmentTrees.h"
#include "Workspace/OctreeDecomposition.h"
////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @brief This is to test 2d medial axis
///
/// TODO
///
/// \internal This strategy is configured for pausible execution.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class MedialAxisTest : public MPStrategyMethod<MPTraits> {

  public:

    typedef typename MPTraits::CfgType CfgType;

    MedialAxisTest(size_t _s = 1);
    MedialAxisTest(XMLNode& _node);

    virtual void Initialize();
    virtual void Iterate();
    virtual void Finalize();
    virtual void Print(ostream& _os) const;

  protected:
    size_t m_space; ///< 0: entire space, 1: free space, 2: obstacle space
		MedialAxis2D* m_axis{nullptr}; ///< Medial axis
		WorkspaceSkeleton m_ws;
};

template <typename MPTraits>
MedialAxisTest<MPTraits>::
MedialAxisTest(size_t _s): m_space(_s) {
  this->SetName("MedialAxisTest");
}

template <typename MPTraits>
MedialAxisTest<MPTraits>::
MedialAxisTest(XMLNode& _node) :
  MPStrategyMethod<MPTraits>(_node)	{
		this->m_space = _node.Read("space", false, 1, 0, MAX_INT, "Free/obstacle/both space medialaxis construction");
    this->SetName("MedialAxisTest");

}

template <typename MPTraits>
void
MedialAxisTest<MPTraits>::
Print(ostream& _os) const {
  
}

template <typename MPTraits>
void
MedialAxisTest<MPTraits>::
Initialize() {
	if(this->m_debug)
    std::cout << "MedialAxisTest::Initialize()" << std::endl;
	Environment* env = this->GetEnvironment();
	size_t numObjects = env->NumObstacles();
	vector<GMSPolyhedron> polyhedra;
	vector<Boundary*> boundaries;
  /*SegmentTrees<> ms(2);
  ms.AddBoundary(env->GetBoundary());
  ms.BuildSegmentTrees();
  cout<<"FindEnclosingBoundaries: "<<ms.FindEnclosingBoundaries(Point3d(-250,0,0), 50)<<endl;*/
  OctreeDecomposition oct(env, 1);
  cout<<"Constructed the octtree";
	for(size_t i=0; i<numObjects; ++i)	{
		auto obstacle = env->GetObstacle(i);
		polyhedra.emplace_back(obstacle->GetFixedBody(0)->GetWorldPolyhedron());
		
		/*auto b = obstacle->GetFixedBody(0)->GetWorldBoundingBox().GetVertexList();
		auto bndry = new WorkspaceBoundingBox(2);
		bndry->SetRange(0, b.front()[0], b.back()[0]); 
		bndry->SetRange(1, b.front()[1], b.back()[1]); 
		boundaries.emplace_back(bndry);*/
	}
	delete m_axis;
	m_axis = new MedialAxis2D (polyhedra, env->GetBoundary());
	cout<<"Intialized"<<endl;
	
}

template <typename MPTraits>
void
MedialAxisTest<MPTraits>::
Iterate() {
  /*m_axis->BuildMedialAxis();
	cout<<"Built Axis"<<endl;
	m_ws = get<0>(m_axis->GetSkeleton(m_space));*/
}

template <typename MPTraits>
void
MedialAxisTest<MPTraits>::
Finalize() {
	//auto g = m_ws.GetGraph();
  //cout<<g.get_num_vertices()<<" "<<g.get_num_edges()<<endl;
	/*for(auto vit = g.begin(); vit != g.end(); ++vit)
		cout<<vit->descriptor()<<" "<<vit->property()<<endl;
	for(auto eit = g.edges_begin(); eit != g.edges_end(); ++eit)	{
		cout<<eit->source()<<" "<<eit->target()<<" ";
		auto prop = eit->property();
		cout<<prop.size()<<" ";
		for(auto v: prop)
			cout<<v<<" ";
		cout<<endl;
	}*/
}

#endif
