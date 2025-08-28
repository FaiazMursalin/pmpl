#ifndef MANUAL_SKELETON_TEST_H
#define MANUAL_SKELETON_TEST_H

#include "MPLibrary/MPStrategies/MPStrategyMethod.h"
#include "Utilities/MeanCurvatureSkeleton3D.h"
#include "Workspace/WorkspaceSkeleton.h"
#include "Workspace/PropertyMap.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @brief This is to test spokes vector for manual skeleton
///
/// TODO
///
/// \internal This strategy is configured for pausible execution.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class ManualSkeletonTest : public MPStrategyMethod<MPTraits> {

  public:

    typedef typename MPTraits::CfgType CfgType;
    typedef typename MeanCurvature3D::AnnotationType SpokesType;

    ManualSkeletonTest();
    ManualSkeletonTest(XMLNode& _node);

    virtual void Initialize();
    virtual void Iterate();
    virtual void Finalize();
    virtual void Print(ostream& _os) const;

  protected:
    string m_input; 
    string m_filename;
    string m_merge;
    double m_step{0.02}; 
    double m_radial{0.05};
    WorkspaceSkeleton m_ws;
    PropertyMap<vector<vector<Vector3d>>, vector<Vector3d>>* m_spokes;
};

template <typename MPTraits>
ManualSkeletonTest<MPTraits>::
ManualSkeletonTest() {
  this->SetName("ManualSkeletonTest");
}

template <typename MPTraits>
ManualSkeletonTest<MPTraits>::
ManualSkeletonTest(XMLNode& _node) :
  MPStrategyMethod<MPTraits>(_node)	{
    this->m_filename = _node.Read("filebase", false, "example",
      "filename for output files");
    this->m_input = _node.Read("input", false, "example",
      "filename for input files");
    this->m_step = _node.Read("step", false, 0.1, 0.0, MAX_DBL,
      "Interpolation step");
    this->m_radial = _node.Read("radial", false, 0.05, 0.0, MAX_DBL,
      "Radial Interpolation step");
    this->m_merge = _node.Read("merge", false, "",
      "filename for merge files");
    this->SetName("ManualSkeletonTest");
}

template <typename MPTraits>
void
ManualSkeletonTest<MPTraits>::
Print(ostream& _os) const {
  
}

template <typename MPTraits>
void
ManualSkeletonTest<MPTraits>::
Initialize() {
	if(this->m_debug)
    std::cout << "ManualSkeletonTest::Initialize()" << std::endl;
	cout<<"Intialized"<<endl;
}

template <typename MPTraits>
void
ManualSkeletonTest<MPTraits>::
Iterate() {
  ifstream ifsg(m_input+".graph");
  size_t numVert, numEdges;
  ifsg>>numVert>>numEdges;
  WorkspaceSkeleton::GraphType& g = m_ws.GetGraph();
  size_t maxIndex = 0;
  // Read in vertices
  for(size_t i = 0; i < numVert; ++i) {
    size_t v;
    Vector3d pt;
    ifsg>>v>>pt;
    g.add_vertex(v, pt);
    maxIndex = max(maxIndex, v);
  }
 cout<<"Read total vertices"<<endl;
  // Read in edges
  for(size_t i = 0; i < numEdges; ++i){
    size_t src, trgt, pathSz;
    vector<Vector3d> path;
    ifsg>>src>>trgt>>pathSz;
    if(pathSz == 2){
      Point3d s, t;
      ifsg>>s>>t;
      double len = (s - t).norm();
      path.push_back(s);
      for(double j = m_step; j < len; j += m_step)
        path.push_back(s + (j / len)*(t-s));
      path.push_back(t);
    }
    else{
      for(size_t j = 0; j< pathSz; j++){
        Vector3d inter;
        ifsg>>inter;
        path.push_back(inter);
      }
    }
    g.add_edge(src, trgt, path);
  }
  cout<<"Done reading the graph and interpolating the edges"<<endl;
  ifsg.close();
  if(!m_merge.empty()) {
    maxIndex++;
    ifstream ifsg1(m_merge+".graph");
    ifsg1>>numVert>>numEdges;
    for(size_t i = 0; i < numVert; ++i) {
      size_t v;
      Vector3d pt;
      ifsg1>>v>>pt;
      g.add_vertex(v+ maxIndex, pt);
    }
    for(size_t i = 0; i < numEdges; ++i) {
      size_t src, trgt, pathSz;
      vector<Vector3d> path;
      ifsg1>>src>>trgt>>pathSz;
      for(size_t j = 0; j< pathSz; j++){
        Vector3d inter;
        ifsg1>>inter;
        path.push_back(inter);
      }
      g.add_edge(src + maxIndex, trgt + maxIndex, path);
    }
    ifsg1.close();
  }
  cout<<"Merged file read"<<endl;
  m_ws.SetGraph(g);
  m_spokes = SpokeVectorSkeleton(this, &m_ws, m_radial);
  cout<<"Created the spokes vector"<<endl;
}

template <typename MPTraits>
void
ManualSkeletonTest<MPTraits>::
Finalize() {
  ofstream ofsg(m_filename+".graph");
  ofstream ofss(m_filename+".spokes");
	auto g = m_ws.GetGraph();
  
  ofsg<<g.get_num_vertices()<<" "<<g.get_num_edges()<<endl;
	for(auto vit = g.begin(); vit != g.end(); ++vit){
		ofsg<<vit->descriptor()<<" "<<vit->property()<<endl;
    if(m_spokes->IsVertexPropertyAssigned(vit->descriptor())){
      auto spk = m_spokes->GetVertexProperty(vit->descriptor());
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
    if(m_spokes->IsEdgePropertyAssigned(eit->descriptor())){
      auto spks = m_spokes->GetEdgeProperty(eit->descriptor());
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
