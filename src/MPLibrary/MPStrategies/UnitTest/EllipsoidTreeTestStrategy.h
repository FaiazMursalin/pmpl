#ifndef ELLIPSOID_TREE_TEST_STRATEGY_H
#define ELLIPSOID_TREE_TEST_STRATEGY_H

#include "MPLibrary/MPStrategies/MPStrategyMethod.h"
#include "Utilities/EllipsoidTrees.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @brief This is to test 2d ellipsoid tree
///
/// TODO
///
/// \internal This strategy is configured for pausible execution.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class EllipsoidTreeTestStrategy : public MPStrategyMethod<MPTraits> {

  public:

    typedef typename MPTraits::CfgType CfgType;

    EllipsoidTreeTestStrategy(size_t _d=2);
    EllipsoidTreeTestStrategy(XMLNode& _node);

    virtual void Initialize();
    virtual void Iterate();
    virtual void Finalize();
    virtual void Print(ostream& _os) const;

  protected:
		size_t m_dimension;
		double m_lineThreshold;
		double m_circleThreshold;
    EllipsoidTrees* m_et{nullptr};
};

template <typename MPTraits>
EllipsoidTreeTestStrategy<MPTraits>::
EllipsoidTreeTestStrategy(size_t _d) : m_dimension(_d)	{
    this->SetName("EllipsoidTreeTestStrategy");
}


template <typename MPTraits>
EllipsoidTreeTestStrategy<MPTraits>::
EllipsoidTreeTestStrategy(XMLNode& _node) :
  MPStrategyMethod<MPTraits>(_node)	{
		this->m_dimension = _node.Read("dimension", false, 1, 0, MAX_INT, "2d/3d space");
		this->m_lineThreshold = _node.Read("simplification", false, 0.1, 0.0, MAX_DBL,
      "Threshold for line simplication");
		this->m_circleThreshold = _node.Read("concentric", false, 0.5, 0.0, MAX_DBL,
      "Threshold for concentric sphere placemnet");
    this->SetName("EllipsoidTreeTestStrategy");
}

template <typename MPTraits>
void
EllipsoidTreeTestStrategy<MPTraits>::
Print(ostream& _os) const {
  
}

template <typename MPTraits>
void
EllipsoidTreeTestStrategy<MPTraits>::
Initialize() {
	if(this->m_debug)
    std::cout << "EllipsoidTreeTest::Initialize()" << std::endl;
	
}

template <typename MPTraits>
void
EllipsoidTreeTestStrategy<MPTraits>::
Iterate() {
	if(m_dimension == 2){
		Environment* env = this->GetEnvironment();
		m_et = new EllipsoidTrees(env);
	}
	else {
		m_et = new EllipsoidTrees(this,m_lineThreshold,3);
	}cout<<"EllipsoidTreeTestStrategyGetGraph"<<endl;
	auto g = m_et->GetSkeleton().GetGraph();
	cout<<g.get_num_vertices()<<" "<<g.get_num_edges()<<endl;
  m_et->BuildEllipsoidTree(m_lineThreshold,/*this->GetEnvironment()->GetPositionRes()*/m_circleThreshold);
}

template <typename MPTraits>
void
EllipsoidTreeTestStrategy<MPTraits>::
Finalize() {
	cout<<"Number of Spheres : "<< m_et->GetNumberSpheres() << endl;
	cout<<"Number of Ellipsoids : "<< m_et->GetNumberEllipsoids() << endl;
	auto g = m_et->GetSkeleton().GetGraph();
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
	ofs<<(*m_et);
}

#endif
