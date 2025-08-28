#ifndef MORSE_QUERY_H_
#define MORSE_QUERY_H_

#include <map>
#include <vector>
#include <numeric>
#include <unordered_set>
#include <queue>
#include "QueryMethod.h"

#include "MPLibrary/LocalPlanners/LPOutput.h"
#include "MPLibrary/LocalPlanners/StraightLine.h"
#include "Utilities/MetricUtils.h"

#include <containers/sequential/graph/algorithms/astar.h>

////////////////////////////////////////////////////////////////////////////////
/// Evaluate a roadmap under construction to see if a query has been satisfied.
///
/// This query is specialized for PRM methods.
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class MorseQuery : public QueryMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::WeightType   WeightType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::GraphType GraphType;
    typedef typename RoadmapType::VID       VID;
    typedef typename MPTraits::Path         Path;

    ///@}
    ///@name Construction
    ///@{

    MorseQuery();
    MorseQuery(XMLNode& _node);
    virtual ~MorseQuery() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(ostream& _os) const override;

    ///@}
    ///@name QueryMethod Overrides
    ///@{

    virtual bool PerformSubQuery(const CfgType& _start, const CfgType& _goal)
        override;

    ///@}

  protected:

    ///@name Helpers
    ///@{

    /// Ensure a given configuration is in the roadmap, adding it if necessary.
    /// @param[in] _cfg The configuration to ensure.
    /// @return The VID of _cfg, and a bool indicating whether or not it had to
    ///         be added to the map.
    pair<VID, bool> EnsureCfgInMap(const CfgType& _cfg);

    /// Remove an ensured configuration if it was added to the map.
    /// @param[in] _temp The ensured configuration's VID and an indicator of
    ///                  whether or not it was added as a temporary.
    void RemoveTempCfg(pair<VID, bool> _temp);

    /// Get the CC stats of the roadmap.
    /// @return A vector with one element per CC. The elements contain the size
    ///         of a CC and one VID within it.
    vector<pair<size_t, VID>> FindCCs();

    /// Try to connect a given VID to the CC containing a second VID.
    /// @param[in] _toConnect The VID to connect.
    /// @param[in] _inCC One of the VIDs in the CC of interest.
    void ConnectToCC(const VID _toConnect, const VID _inCC);

    // vector<Path*> 
    void GetPaths(size_t k);

    void ReadDynamicAgentPath();

    ///@}
    ///@name MP Object Labels
    ///@{

    vector<string> m_ncLabels{"kClosest"};

    vector<Path*> m_paths;   // Diverse path set
    vector<CfgType> dynamicAgentPath;
    string m_inputDynamicAgentPathFile;
    // // unordered_map<VID, VID> m_feasible; // Map of feasible critical points to the critical points

    ///@}
    ///@name Graph Search
    ///@{

    bool m_deleteNodes{false}; ///< Delete any added nodes?

    ///@}

};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
MorseQuery<MPTraits>::
MorseQuery() : QueryMethod<MPTraits>() {
  this->SetName("MorseQuery");
  m_inputDynamicAgentPathFile = "";
}


template <typename MPTraits>
MorseQuery<MPTraits>::
MorseQuery(XMLNode& _node) : QueryMethod<MPTraits>(_node) {
  this->SetName("MorseQuery");
  m_inputDynamicAgentPathFile = _node.Read("inputMap", false, "",
      "filename of dynamic agent path");

  m_deleteNodes = _node.Read("deleteNodes", false, m_deleteNodes, "Whether or "
      "not to delete start and goal from roadmap");

  bool defaultsCleared = false;
  for(auto& child : _node) {
    if(child.Name() == "NodeConnectionMethod") {
      if(!defaultsCleared) {
        defaultsCleared = true;
        m_ncLabels.clear();
      }
      m_ncLabels.push_back(child.Read("method", true, "", "Connector method"));
    }
  }
}


template<typename MPTraits>
void
MorseQuery<MPTraits>::
ReadDynamicAgentPath() {
  m_inputDynamicAgentPathFile = MPProblem::GetPath(m_inputDynamicAgentPathFile);

  cout << "Reading dynamic agent path file \'" << m_inputDynamicAgentPathFile << endl;

  ifstream in(m_inputDynamicAgentPathFile);

  cout<<"File stream ready for dynamic agent path"<<endl;

  if(!in.good())
    throw ParseException(WHERE, "Can't open dynamic agent path file '" + m_inputDynamicAgentPathFile + "'.");

  if(!dynamicAgentPath.empty())
    dynamicAgentPath.clear();

  auto robot = this->GetTask()->GetRobot(); //Change here for dynamic agent

  cout<<"Gets robot"<<endl;

  // Skip the first two lines
  std::string line;
  for (int i = 0; i < 2; ++i) {
      std::getline(in, line);
  }

  cout<<"skips 1st 2 lines"<<endl;

  // Read the number from the third line
  int number;
  in >> number;
  in.ignore(); // Ignore the newline character

  // Read three values from each subsequent line until linecount == number
  CfgType tempCfg(robot);
  for (int i = 0; i < number; ++i) {
    in >> tempCfg;
    dynamicAgentPath.push_back(tempCfg);    
  }

  
  cout<<"Read "<<dynamicAgentPath.size()<<" Cfgs for dynamic agent path"<<endl;

}



// template<typename MPTraits> void
// MorseQuery<MPTraits>::SetFeasiblePoints(unordered_map<VID, VID> _inFeasibles) {
//   cout<<"SetFeasible called"<<endl;
//   m_feasible = _inFeasibles;
// }

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
MorseQuery<MPTraits>::
Print(ostream& _os) const {
  QueryMethod<MPTraits>::Print(_os);
  _os << "\n\tDelete Nodes: " << m_deleteNodes
      << "\n\tConnectors:" << endl;
  for(const auto& label : m_ncLabels)
    _os << "\t\t" << label << endl;
}

/*------------------------------ Query Interface -----------------------------*/

template <typename MPTraits>
bool
MorseQuery<MPTraits>::
PerformSubQuery(const CfgType& _start, const CfgType& _goal) {

  if(!m_inputDynamicAgentPathFile.empty()) {
    ReadDynamicAgentPath();
  }



  // if(this->m_debug)
    cout << "Evaluating sub-query:" << endl
         << "\tfrom " << _start << endl
         << "\tto   " << _goal << endl;

  size_t k = 3;
  GetPaths(k);


  for(auto cfg: dynamicAgentPath) {
    // remove the path vertices from the roadmap - Case 2

    // I need to find a way to retrieve the corressponding vertex/vid from
    // the roadmap graaph if cfg is in dynamic agent path

    auto vid = this->GetRoadmap()->GetGraph()->GetVertex(cfg);
	   this->GetRoadmap()->GetGraph()->DeleteVertex(v);
  }

  for(Path* p: m_paths) {
    this->GetPath()->Clear(); // Clear previous path data
    cout<<"Path: "<<p<<endl;
    // cout<<"Path*: "<<(*p)<<endl;
    cout<<"Roadmap: "<<p->GetRoadmap()<<endl;
    cout<<"Roadmap->Graph: "<<p->GetRoadmap()->GetGraph()<<endl;
    cout<<"this->GetRoadmap(): "<<this->GetRoadmap()<<"\n-------------------"<<endl;

    bool cfgFound = false;
    for(auto cfg: p->Cfgs()) {
      if (std::find(dynamicAgentPath.begin(), dynamicAgentPath.end(), cfg) != dynamicAgentPath.end()) {
        cfgFound = true;
        break;
      }
    }

    if(cfgFound) {
      cout<<"Path: "<<p<<" not valid."<<endl;
      continue;
    }

  }



    // this->GetRoadMap()->Set
    // Find connected components.
  auto ccs = FindCCs();

  // Add start and goal to roadmap (if not already there).
  auto start = EnsureCfgInMap(_start);
  auto goal  = EnsureCfgInMap(_goal);

  // Check each connected component.
  bool connected = false;
  for(auto cc : ccs) {

    CfgType ccCfg = this->GetRoadmap()->GetGraph()->GetVertex(cc.second);

    if (std::find(dynamicAgentPath.begin(), dynamicAgentPath.end(), ccCfg) != dynamicAgentPath.end()) {
      cout<<"cc VID: "<<cc.second<<" found in dynamic path. Skipping"<<endl;
      continue;
    }

    // Try connecting the start and goal to this CC.
    ConnectToCC(start.first, cc.second);
    ConnectToCC(goal.first, cc.second);

    // If start and goal are connected to the same CC, generate path and end.
    if(this->SameCC(start.first, goal.first)) {
      connected = true;
      this->GeneratePath(start.first, goal.first);
      break;
    }
  }

  if(this->m_debug) {
    if(connected)
      cout << "\tSuccess: found path from start node " << start.first
           << " to goal node " << goal.first << "." << endl;
    else
      cout << "\tFailed to connect start node " << start.first
           << " to goal node " << goal.first << "." << endl;
  }

  // Remove start and goal if necessary.
  if(m_deleteNodes) {
    RemoveTempCfg(start);
    RemoveTempCfg(goal);
  }

  return connected;
  
}


/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
pair<typename MorseQuery<MPTraits>::VID, bool>
MorseQuery<MPTraits>::
EnsureCfgInMap(const CfgType& _cfg) {
  auto g = this->GetRoadmap()->GetGraph();

  const bool exists = g->IsVertex(_cfg);

  if(exists)
    // The vertex already exists.
    return std::make_pair(g->GetVID(_cfg), false);
  else if(!m_deleteNodes)
    // The vertex is new.
    return std::make_pair(g->AddVertex(_cfg), true);
  else {
    // The vertex is new, but we are adding a temporary node. Do not run the hook
    // functions.
    g->DisableHooks();
    auto out = std::make_pair(g->AddVertex(_cfg), true) ;
    g->EnableHooks();
    return out;
  }
}


template <typename MPTraits>
void
MorseQuery<MPTraits>::
RemoveTempCfg(pair<VID, bool> _temp) {
  // Return if this wasn't added as a temporary cfg.
  if(!_temp.second)
    return;

  auto g = this->GetRoadmap()->GetGraph();
  g->DisableHooks();
  g->DeleteVertex(_temp.first);
  g->EnableHooks();
}


template <typename MPTraits>
vector<pair<size_t, typename MorseQuery<MPTraits>::VID>>
MorseQuery<MPTraits>::
FindCCs() {
  auto stats = this->GetStatClass();
  stats->IncStat("CC Operations");

  stats->StartClock("MorseQuery::FindCCs");
  vector<pair<size_t, VID>> ccs;
  stapl::sequential::vector_property_map<GraphType, size_t> cmap;
  get_cc_stats(*this->GetRoadmap()->GetGraph(), cmap, ccs);
  stats->StopClock("MorseQuery::FindCCs");

  if(this->m_debug)
    cout << "\tThere are " << ccs.size() << " CCs." << endl;

  return ccs;
}


template <typename MPTraits>
void
MorseQuery<MPTraits>::
ConnectToCC(const VID _toConnect, const VID _inCC) {
  // If the nodes are already in the same CC, return.
  if(this->SameCC(_toConnect, _inCC)) {
    // if(this->m_debug)
      cout << "\tNodes " << _toConnect << " and " << _inCC << " are already in "
           << "the same CC." << endl;
    return;
  }

  auto stats = this->GetStatClass();
  stats->IncStat("CC Operations");
  stats->StartClock("MorseQuery::ConnectToCC");

  // Get the CC containing _inCC.
  vector<VID> cc;
  stapl::sequential::vector_property_map<GraphType, size_t> cmap;
  stapl::sequential::get_cc(*this->GetRoadmap()->GetGraph(), cmap, _inCC, cc);

  // Try to join _toConnect to that CC using each connector.
  for(auto& label : m_ncLabels) {
    cout<<"MorseQuery::ConnectToCC::m_ncLabels::"<<label<<endl;
    this->GetConnector(label)->Connect(this->GetRoadmap(), _toConnect,
        cc.begin(), cc.end(), false);
  }

  stats->StopClock("MorseQuery::ConnectToCC");
}



template<typename MPTraits>
// vector<Path*> 
void
MorseQuery<MPTraits>::
GetPaths(size_t k) {
  // if(!this->UsingQuery()) return;

  // Copy the graph

  // vector<Path*> m_paths;

  unordered_map<VID, VID> oldNew;
  GraphType* prevg = new GraphType();
  auto gr = this->GetRoadmap()->GetGraph();
  for(auto vit = gr->begin(); vit != gr->end(); ++vit){
	auto newVID = prevg->AddVertex(vit->descriptor(), vit->property());
	oldNew.insert({vit->descriptor(), newVID});
  }
  for(auto eit = gr->edges_begin(); eit != gr->edges_end(); eit++){
    prevg->AddEdge(oldNew[eit->source()], oldNew[eit->target()], eit->property());
  }
  // unordered_map<VID, VID> m_feasible;

  // for(auto git = gr->begin(); git != gr->end(); ++git) {
  //   auto vertex = gr->GetVertex(git);
  //   cout <<"g->getVertex->git: " << vertex<<endl;
  //   cout<<"feasible: "<<git->descriptor()<<endl;
  //   cout<<" VID "<<gr->GetVID(git)<<endl;
  //   cout<<"getstat "<<vertex.GetStat("Critical")<<endl;
  //   cout<<"getLabel: "<<vertex.GetLabel("FeasibleCritical")<<endl;
  //   // if (vertex.GetLabel("FeasibleCritical")) {
  //   //   // m_feasible.insert({,git->descriptor()})
  //   //   cout<<"vertex is feasible"<<endl;
  //   // }

  // }

  vector<unordered_set<VID>> criticalPoints;
  unordered_set<VID> pCritical;
  // Get Diverse paths
  while(m_paths.size() < k){
	this->GetPath()->Clear(); // Clear previous path data
//     // auto mapPassed = this->EvaluateMap();
//     // if(!mapPassed) break; // no more path to be found
	//get the path and store it
    Path* p = new Path(this->GetRoadmap());
	vector<VID> pVID, fVID;
	for(auto v : this->GetPath()->VIDs()){
	  pVID.push_back(oldNew[v]);
	  // find the feasible critical points in the path
    if(prevg->GetVertex(oldNew[v]).GetLabel("FeasibleCritical")) {
      fVID.push_back(v);
      pCritical.insert(prevg->GetVertex(oldNew[v]).GetStat("Critical"));
    }
// 	  // if(m_feasible.find(v) != m_feasible.end())
// 		// fVID.push_back(v);
	}
	*p+= pVID;
	m_paths.push_back(p);
    // delete the vertices in the path from the roadmap
	// get the corresponding critical point and its' feasible critical point
	
	// for(auto f: fVID)
	//   pCritical.insert(m_feasible[f]);

	// check if there is exact match with any critical point set found earlier
	bool isSame = false;
	for(auto cp: criticalPoints){
	  isSame = true;
	  if(cp.size() != pCritical.size()){
		isSame = false;
	  }
	  else{// check if same set of critical points
	    for(auto cap: pCritical)
			if(cp.find(cap) == cp.end()){
				isSame = false;
				break;
			}
	  }
	  // early quit if equal to any
	  if(isSame) break;
	}

	// remove them from the current roadmap
	// remove the path vertices from the roadmap - Case 2
	 for(auto v : this->GetPath()->VIDs())
	   gr->DeleteVertex(v);
	// remove all the feasible critical points - case 1
	// /*for(auto f: fVID)
	//   gr->DeleteVertex(f);*/

    // store only if the critical points are different than before
	if(isSame){
	  delete p;
	  continue;
	}
	// store the critical points for future paths
	criticalPoints.push_back(pCritical);
	m_paths.push_back(p);
  }
  // restore the roadmap
  this->GetRoadmap()->SetGraph(prevg);
  cout<<"Total number of paths at the end of GetPaths = "<<m_paths.size()<<endl;

  // return m_paths;
}

/*----------------------------------------------------------------------------*/

#endif
