#ifndef CURVE_SKELETON_TEST_H
#define CURVE_SKELETON_TEST_H

#include "MPLibrary/MPStrategies/MPStrategyMethod.h"
#include "Utilities/MeanCurvatureSkeleton3D.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @brief This is to test 3d mean curvature skeleton
///
/// TODO
///
/// \internal This strategy is configured for pausible execution.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class MeanCurvatureSkeletonTest : public MPStrategyMethod<MPTraits> {

  public:

    typedef typename MPTraits::CfgType CfgType;
    typedef typename MeanCurvature3D::AnnotationType SpokesType;

    MeanCurvatureSkeletonTest();
    MeanCurvatureSkeletonTest(XMLNode& _node);

    virtual void Initialize();
    virtual void Iterate();
    virtual void Finalize();
    virtual void Print(ostream& _os) const;

  protected:
    MeanCurvatureParams m_params;
    size_t m_space;
    string m_filename;
    MeanCurvature3D* m_axis{nullptr}; ///< Mean curvature skeleton
    SpokesType m_spokes;
    WorkspaceSkeleton m_ws;
    
};

template <typename MPTraits>
MeanCurvatureSkeletonTest<MPTraits>::
MeanCurvatureSkeletonTest() {
  this->SetName("MeanCurvatureSkeletonTest");
}

template <typename MPTraits>
MeanCurvatureSkeletonTest<MPTraits>::
MeanCurvatureSkeletonTest(XMLNode& _node) :
  MPStrategyMethod<MPTraits>(_node)	{
    this->m_space = _node.Read("space", false, 1, 0, MAX_INT, "Free/obstacle/both space construction");
    this->m_filename = _node.Read("filebase", false, "example",
      "filename for output files");
    this->m_params.m_iterations = _node.Read("iteration", false, 1000, 0, MAX_INT, "Number of iteration");
    this->m_params.m_wH = _node.Read("quality", false, 0.1, 0.0, MAX_DBL,
      "Quality Parameter");
    this->m_params.m_wM = _node.Read("medial", false, 0.2, 0.0, MAX_DBL,
      "Medial parameter");
    this->SetName("MeanCurvatureSkeletonTest");
}

template <typename MPTraits>
void
MeanCurvatureSkeletonTest<MPTraits>::
Print(ostream& _os) const {
  
}

template <typename MPTraits>
void
MeanCurvatureSkeletonTest<MPTraits>::
Initialize() {
	if(this->m_debug)
    std::cout << "MeanCurvatureSkeletonTest::Initialize()" << std::endl;
	delete m_axis;
	m_axis = new MeanCurvature3D(this->GetEnvironment(),m_space);
  m_axis->SetParameters(m_params);
	cout<<"Intialized"<<endl;
	
}

template <typename MPTraits>
void
MeanCurvatureSkeletonTest<MPTraits>::
Iterate() {
  m_axis->BuildSkeleton();
	cout<<"Built Axis"<<endl;
  auto sk = m_axis->GetSkeleton();
	m_ws = sk.first;
  m_spokes = sk.second;
}

template <typename MPTraits>
void
MeanCurvatureSkeletonTest<MPTraits>::
Finalize() {
  ofstream ofsg(m_filename+".graph");
  ofstream ofss(m_filename+".spokes");
	auto g = m_ws.GetGraph();
  
  ofsg<<g.get_num_vertices()<<" "<<g.get_num_edges()<<endl;
	for(auto vit = g.begin(); vit != g.end(); ++vit){
		ofsg<<vit->descriptor()<<" "<<vit->property()<<endl;
    if(m_spokes.IsVertexPropertyAssigned(vit->descriptor())){
      auto spk = m_spokes.GetVertexProperty(vit->descriptor());
      ofss<<vit->descriptor()<<" "<<spk.size()<<" ";
      for(auto s : spk)
        ofss<<s<<" ";
      ofss<<endl;
    }
    else
      ofss<<vit->descriptor()<<" 0"<<endl;
  }
	for(auto eit = g.edges_begin(); eit != g.edges_end(); ++eit)	{
		ofsg<<eit->source()<<" "<<eit->target()<<" ";
		auto prop = eit->property();
		ofsg<<prop.size()<<" ";
		for(auto v: prop)
			ofsg<<v<<" ";
		ofsg<<endl;
    if(m_spokes.IsEdgePropertyAssigned(eit->descriptor())){
      auto spks = m_spokes.GetEdgeProperty(eit->descriptor());
      ofss<<eit->source()<<" "<<eit->target()<<" "<<spks.size()<<endl;
      for(auto spk : spks){
        ofss<<spk.size()<<" ";
        for(auto s : spk)
          ofss<<s<<" ";
        ofss<<endl;
      }
    }
    else
      ofss<<eit->source()<<" "<<eit->target()<<" 0"<<endl; 
	}
}

#endif
