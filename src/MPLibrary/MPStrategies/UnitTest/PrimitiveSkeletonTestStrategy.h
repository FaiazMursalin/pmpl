#ifndef PRIMITIVE_SKELETON_TEST_STRATEGY_H
#define PRIMITIVE_SKELETON_TEST_STRATEGY_H

#include "MPLibrary/MPStrategies/MPStrategyMethod.h"
#include "Utilities/PrimitiveSkeleton.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @brief This is to test primitive skeleton
///
/// TODO
///
/// \internal This strategy is configured for pausible execution.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class PrimitiveSkeletonTestStrategy : public MPStrategyMethod<MPTraits> {

  public:

    typedef typename MPTraits::CfgType CfgType;

    PrimitiveSkeletonTestStrategy(size_t _d=2);
    PrimitiveSkeletonTestStrategy(XMLNode& _node);

    virtual void Initialize();
    virtual void Iterate();
    virtual void Finalize();
    virtual void Print(ostream& _os) const;

  protected:
    size_t m_dimension;
    size_t m_space; ///< 0: entire space, 1: free space, 2: obstacle space
    size_t m_score{0}; 
    double m_lineThreshold;
    double m_circleThreshold;
    string m_filename; ///< Output file name
    string m_inputFileName; 
    MeanCurvatureParams m_params;
    PrimitiveSkeleton* m_pt{nullptr};
    GridOverlay* m_go{nullptr};
};

template <typename MPTraits>
PrimitiveSkeletonTestStrategy<MPTraits>::
PrimitiveSkeletonTestStrategy(size_t _d) : m_dimension(_d), m_score(0)	{
    this->SetName("PrimitiveSkeletonTestStrategy");
}


template <typename MPTraits>
PrimitiveSkeletonTestStrategy<MPTraits>::
PrimitiveSkeletonTestStrategy(XMLNode& _node) :
  MPStrategyMethod<MPTraits>(_node)	{
    this->m_dimension = _node.Read("dimension", false, 1, 0, MAX_INT, "2d/3d space");
    this->m_space = _node.Read("space", false, 1, 0, MAX_INT, "Free/obstacle/both space construction");
    this->m_score = _node.Read("score", false, 0, 0, MAX_INT, "Score function to use");
    this->m_lineThreshold = _node.Read("simplification", false, 0.1, 0.0, MAX_DBL,
      "Threshold for line simplication");
    this->m_circleThreshold = _node.Read("concentric", false, 0.5, 0.0, MAX_DBL,
      "Threshold for concentric sphere placemnet");
    this->m_filename = _node.Read("filebase", false, "example",
      "filename for output files");
    this->m_inputFileName = _node.Read("inputfilebase", false, "",
      "filename for input files");
    this->m_params.m_iterations = _node.Read("iteration", false, 1000, 0, MAX_INT, "Number of iteration");
    this->m_params.m_wH = _node.Read("quality", false, 0.1, 0.0, MAX_DBL,
      "Quality Parameter");
    this->m_params.m_wM = _node.Read("medial", false, 0.2, 0.0, MAX_DBL,
      "Medial parameter");
    this->SetName("PrimitiveSkeletonTestStrategy");
}

template <typename MPTraits>
void
PrimitiveSkeletonTestStrategy<MPTraits>::
Print(ostream& _os) const {
  
}

template <typename MPTraits>
void
PrimitiveSkeletonTestStrategy<MPTraits>::
Initialize() {
	if(this->m_debug)
    std::cout << "EllipsoidTreeTest::Initialize()" << std::endl;
	 
}

template <typename MPTraits>
void
PrimitiveSkeletonTestStrategy<MPTraits>::
Iterate() {
  StatClass* stats = this->GetStatClass();
  stats->StartClock("Loading");
  Environment* env = this->GetEnvironment();
  if(m_dimension == 2){
    m_pt = new PrimitiveSkeleton(env, m_space);
  }
  else {
    if(m_inputFileName.empty())
      m_pt = new PrimitiveSkeleton(this, m_space, m_lineThreshold, m_params);
    else
      m_pt = new PrimitiveSkeleton(this, m_inputFileName, m_space, m_lineThreshold);
    //m_pt = new PrimitiveSkeleton(this->GetEnvironment(), m_space, 3);
  }cout<<"PrimitiveSkeletonStrategyGetGraph"<<endl;
  stats->StopClock("Loading");
  auto g = m_pt->GetSkeleton().GetGraph();
  cout<<g.get_num_vertices()<<" "<<g.get_num_edges()<<endl;
  stats->StartClock("Building");
  m_pt->BuildPrimitiveGraph(m_lineThreshold,m_circleThreshold, m_score);
  stats->StopClock("Building");
  
}

template <typename MPTraits>
void
PrimitiveSkeletonTestStrategy<MPTraits>::
Finalize() {
	cout<<"Number of Spheres : "<< m_pt->GetNumberSpheres() << endl;
	cout<<"Number of Ellipsoids : "<< m_pt->GetNumberEllipsoids() << endl;
  cout<<"Number of Capsules : "<< m_pt->GetNumberCapsules() << endl;
	cout<<"Number of Boxes : "<< m_pt->GetNumberBoxes() << endl;

 ofstream ofsg(m_filename+".graph");
  auto g = m_pt->GetSkeleton().GetGraph();
	ofsg<<g.get_num_vertices()<<" "<<g.get_num_edges()<<endl;
	for(auto vit = g.begin(); vit != g.end(); ++vit)
		ofsg<<vit->descriptor()<<" "<<vit->property()<<endl;
	for(auto eit = g.edges_begin(); eit != g.edges_end(); ++eit)	{
		ofsg<<eit->source()<<" "<<eit->target()<<" ";
		auto prop = eit->property();
		ofsg<<prop.size()<<" ";
		for(auto v: prop)
			ofsg<<v<<" ";
		ofsg<<endl;
	}

  /*g = m_pt->GetDeletedGraph();
  ofstream ofdg(m_filename+"-deleted.graph");
  ofdg<<g.get_num_vertices()<<" "<<g.get_num_edges()<<endl;
	for(auto vit = g.begin(); vit != g.end(); ++vit)
		ofdg<<vit->descriptor()<<" "<<vit->property()<<endl;
	for(auto eit = g.edges_begin(); eit != g.edges_end(); ++eit)	{
		ofdg<<eit->source()<<" "<<eit->target()<<" ";
		auto prop = eit->property();
		ofdg<<prop.size()<<" ";
		for(auto v: prop)
			ofdg<<v<<" ";
		ofdg<<endl;
	}*/

	ofstream ofsp(m_filename+".e");
	ofsp<<(*m_pt);
  this->GetStatClass()->PrintClock("Loading", cout);
  this->GetStatClass()->PrintClock("Building", cout);
}

#endif
