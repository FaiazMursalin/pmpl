#ifndef GRID_OVERLAY_TEST_STRATEGY_H
#define GRID_OVERLAY_TEST_STRATEGY_H

#include "MPLibrary/MPStrategies/MPStrategyMethod.h"
#include "Workspace/GridOverlay.h"
#include "Utilities/PrimitiveSkeleton.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @brief This is to test GridOverlay
///
/// TODO
///
/// \internal This strategy is configured for pausible execution.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class GridOverlayTestStrategy : public MPStrategyMethod<MPTraits> {

  public:

    typedef typename MPTraits::CfgType CfgType;

    GridOverlayTestStrategy(size_t _d=3);
    GridOverlayTestStrategy(XMLNode& _node);

    virtual void Initialize();
    virtual void Iterate();
    virtual void Finalize();
    virtual void Print(ostream& _os) const;

  protected:
    size_t m_dimension;
		double m_length;
    size_t m_space; ///< 0: entire space, 1: free space, 2: obstacle space
		double m_lineThreshold;
		double m_circleThreshold;
    string m_inputFileName; 
    GridOverlay* m_go{nullptr};
		PrimitiveSkeleton* m_et{nullptr};
    bool m_debug = true;
};

template <typename MPTraits>
GridOverlayTestStrategy<MPTraits>::
GridOverlayTestStrategy(size_t _d) : m_dimension(_d), m_length(1)	{
    this->SetName("GridOverlayTestStrategy");
}


template <typename MPTraits>
GridOverlayTestStrategy<MPTraits>::
GridOverlayTestStrategy(XMLNode& _node) :
  MPStrategyMethod<MPTraits>(_node)	{

    this->m_dimension = _node.Read("dimension", false, 1, 0, MAX_INT, "2d/3d space");
		this->m_length = _node.Read("length", false, 0.1, 0.0, MAX_DBL, "Length");
    this->m_space = _node.Read("space", false, 1, 0, MAX_INT, "Free/obstacle/both space construction");
    this->m_lineThreshold = _node.Read("simplification", false, 0.1, 0.0, MAX_DBL,
      "Threshold for line simplication");
		this->m_circleThreshold = _node.Read("concentric", false, 0.5, 0.0, MAX_DBL,
      "Threshold for concentric sphere placemnet");
    this->m_inputFileName = _node.Read("inputfilebase", false, "",
      "filename for input files");
    this->SetName("GridOverlayTestStrategy");
}

template <typename MPTraits>
void
GridOverlayTestStrategy<MPTraits>::
Print(ostream& _os) const {

}

template <typename MPTraits>
void
GridOverlayTestStrategy<MPTraits>::
Initialize() {
    if(this->m_debug)
      std::cout << "GridOverlayTest::Initialize()" << std::endl;
    auto bndry = this->GetEnvironment()->GetBoundary();
    m_go = new GridOverlay(bndry, m_length);
}

template <typename MPTraits>
void
GridOverlayTestStrategy<MPTraits>::
Iterate() {
  cout<<"Dimension in test strategy:"<<m_dimension<<endl;
	Environment* env = this->GetEnvironment();

	if (m_dimension == 2) {
		m_et = new PrimitiveSkeleton(env, m_space);
	}
	else {
		//m_et = new PrimitiveSkeleton(this, m_space);
		m_et = new PrimitiveSkeleton(this, m_inputFileName, m_space, m_lineThreshold);
	}
  m_et->BuildPrimitiveGraph(m_lineThreshold,/*this->GetEnvironment()->GetPositionRes()*/m_circleThreshold, 1);
		size_t e = 0; size_t b = 0;
  for(size_t i = 1 ; i <= m_go->Size(); i++)  {
    std::cout << "Cell " << i;
		if (m_go->ContainsPrimitive(i, m_et)) {
			e++;
			std::cout << " is occupied by an ellipsoid";
		}
    else
      std::cout << " is not occupied by an ellipsoid";
		if (m_go->Occupied(this, i)) {
			b++;
			std::cout << " and occupied by an obstacle";
		}
    else
      std::cout << " and not occupied by an obstacle";
    std::cout << endl;
  }
	cout << "Ellipsoids:" << e << endl;
	cout << "Obstacle:" << b << endl;
	cout << "Total:" << m_go->Size() << endl;
}

template <typename MPTraits>
void
GridOverlayTestStrategy<MPTraits>::
Finalize() {
  /*
	cout<<"Number of Spheres : "<< m_go->GetNumberSpheres() << endl;
	cout<<"Number of Ellipsoids : "<< m_go->GetNumberEllipsoids() << endl;
	auto g = m_go->GetSkeleton().GetGraph();
	 cout<<g.get_num_vertices()<<" "<<g.get_num_edges()<<endl;
	for(auto vit = g.begin(); vit != g.end(); ++vit)
		cout<<vit->descriptor()<<" "<<vit->property()<<endl;
	for(auto eit = g.edges_begin(); eit != g.edges_end(); ++eit)	{
		cout<<eit->source()<<" "<<eit->target()<<" ";
		auto prop = eit->property();
		cout<<prop.size()<<" ";
		for(auto v: prop)
			cout<<v<<" ";
		cout<<endl;
	}
	ofstream ofs("example.e");
	ofs<<(*m_go);*/
}

#endif
