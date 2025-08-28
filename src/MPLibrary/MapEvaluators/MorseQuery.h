#ifndef MORSE_QUERY_H_
#define MORSE_QUERY_H_

#include <unordered_map>
#include <vector>
#include <numeric>
#include <unordered_set>
#include <queue>
#include <random>
#include "QueryMethod.h"
#include "SafeIntervalHandler.h"

#include "MPLibrary/LocalPlanners/LPOutput.h"
#include "MPLibrary/LocalPlanners/StraightLine.h"
#include "Utilities/MetricUtils.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/CDInfo.h"

#include <containers/sequential/graph/algorithms/astar.h>
#include <containers/sequential/graph/algorithms/breadth_first_search.h>

template <typename MPTraits>
class MorseQuery : public PRMQuery<MPTraits>
{
public:
    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::WeightType   WeightType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::GraphType GraphType;
    typedef typename RoadmapType::VID       VID;
    typedef typename MPTraits::Path         Path;

    typedef typename MPTraits::SafeIntervalHandlerType SafeIntervalHandler;
    typedef typename SafeIntervalHandler::DynamicAgentState DynamicAgentState;

    MorseQuery();
    MorseQuery(XMLNode &_node);
    virtual ~MorseQuery() = default;

    virtual void Print(ostream &_os) const override;
    virtual bool PerformSubQuery(const CfgType &_start, const CfgType &_goal) override;

    virtual void Initialize() override {
        if(m_numAgents > 0){
            // Read the multibody fotr dynamic agent
            cout << "Reading dynamic agent from: " << m_dynamicAgent << endl;
            const string inputDynamicAgentFile(m_dynamicAgent);
            CountingStreamBuffer cbs(inputDynamicAgentFile);
            std::istream ifs(&cbs);
            ActiveMultiBody * mb = new ActiveMultiBody;
            mb->Read(ifs, cbs);
            dynamicAgent = new Robot(this->GetMPProblem(), mb, "dynamic agent");
            cout << "Gets dynamic agent" << endl;
            cout << "DOF " << dynamicAgent->GetMultiBody()->DOF() << endl;
            cout << "PosDOF " << dynamicAgent->GetMultiBody()->PosDOF() << endl;
            cout << "OrientationDOF " << dynamicAgent->GetMultiBody()->OrientationDOF() << endl;
            cout << "JointDOF " << dynamicAgent->GetMultiBody()->JointDOF() << endl;
        }
        if(!m_agentRdmpFile.empty()){
            if(m_agentRdmpFile.find("map") !=std::string::npos)
                ReadMapFile(m_agentRdmpFile);
            else if(m_agentRdmpFile.find("path") !=std::string::npos)
                ReadDynamicAgentPath();
        }

        if(PRMQuery<MPTraits>::GetQuery().empty())
            PRMQuery<MPTraits>::Initialize();

        InitializeCritical();
    }

protected:
    double m_maxPlanningTime = 0;
    string m_vcLabel;
    string m_mapFileName;//filename of the map

    enum GraphSearchAlg {DIJKSTRAS, ASTAR}; ///< The supported sssp algorithms.
    GraphSearchAlg m_searchAlg{DIJKSTRAS};  ///< The sssp algorithm to use.

    enum class SocialNavigationStrategy {
        WAIT,
        DEFLECT,
        INCREASE_SPEED_OR_DIVERSE
    };

    // new stuff
    unordered_map<VID, vector<VID>> m_feasibleCritical; // map of critical point to fesible critical points
    unordered_map<VID, vector<VID>> m_clusters;  // map of critical point to region vid

    bool GetValidPath();
    std::pair<VID, VID> GetNextPathRegion(const VID _start, Path* _p, vector<VID>& _segment);
    bool GetNextSubPath(VID _start, vector<VID>& _segment, std::pair<VID, VID> _cpoints, vector<VID>& _addVID);
    bool GenerateValidSubPath(VID start, vector<VID>& _segment, std::pair<VID, VID> _cpoints, double& _startTick, Path* _newPath, int r_speed);
    bool DynamicValidate(vector<VID>& _segment, double& _startTick, vector<pair<VID, VID>>& _edgeToDelete, int r_speed);
    bool ValidCfg(CfgType& _cfg, double _tick, int r_speed);
    double GetDeflectionLength(Path* _sp, Path *origPath);
    double GetSegmentLength(const vector<VID>& _vids);
    vector<vector<pair<int, int>>> GetSafeWindows(vector<CfgType>& _segment, int _startTick, int r_speed);
    void GetSafeWindows(CfgType& _cfg, int _startTick, vector<vector<pair<int, int>>>& _safeWindows, int _max_length);
    vector<pair<int, int>> findNonCollisionWindows(vector<CfgType>& _segment, int startTick, int agentIdx);
    void findNonCollisionWindows(CfgType& _cfg, double startTick, int agentIdx, vector<pair<double, double>>& _windows, int  _max_length, int r_speed) ;
    bool ValidateCfgAgainstDynamicAgent(CfgType& _cfg, double _tick, int dynamicAgentNumber, int r_speed=1);
    bool InitializeCritical();
    void GetPaths(const VID _start, const VID _end, size_t k, unordered_map<VID, VID>& oldNew);
    void ReadDynamicAgentPath();
    void PrettyPrintSocialNavigationStrategy(SocialNavigationStrategy strategy);
    SocialNavigationStrategy SocialNavigation(double congestionValue, double deflectionCost, double waitCost, double diverseCost);
    double waitCost(double congestionvalue, double waitTime);
    double deflectionCost(double congestionValue, CfgType deflectingPoint, CfgType goal);
    double diversePathCost(double congestionValue, Path *newPath);
    int GetSegmentTickLength(vector<VID>& _segment);
    double calculateWaitTimeCfg(vector<CfgType>& robotcfg, double startTick, vector<double>& startWindowEndtime, int r_speed=1);
    void calculateIntermediates(VID start, VID goal, vector<Cfg>& _edgeCfg);
    void GetSafeWindowsInterval(CfgType& _cfg, double _startTick, vector<pair<double, double>>& _safeWindows,  double _max_length, size_t j=INT_MAX, int r_speed=1);
    pair<double, double> calculateWaitTime(vector<VID>& _segment, double startTick, vector<double>& _waitTimes);
    void ReadMapFile(const std::string& _filename);
    void InitializeAgentPaths(RoadmapType* _rdmp);

    vector<string> m_ncLabels{"kClosest"};
    vector<vector<CfgType>> dynamicAgentPaths;
    string m_agentRdmpFile;
    Robot *dynamicAgent;
    SafeIntervalHandler *m_safeIntervalHandler;
    string m_dynamicAgent;
    bool m_deleteNodes{false};
    vector<Path *> m_paths;
    CollisionDetectionMethod* m_cdMethod{nullptr};
    int m_numAgents{0};
//    string m_agentRdmpFile;
    int m_divPaths{10};
    int r_speed{1};
    int max_r_speed{3};
//    double speedMultiBodyPositionalResolution = 1;
};
template <typename MPTraits>
MorseQuery<MPTraits>::MorseQuery() : PRMQuery<MPTraits>() {
    this->SetName("MorseQuery");
    m_agentRdmpFile = "";
    m_dynamicAgent = "";
    dynamicAgent = nullptr;
    m_safeIntervalHandler = new SafeIntervalHandler();
    m_maxPlanningTime = 0.0;
    m_mapFileName = "";
    max_r_speed = 1;
}

template <typename MPTraits>
MorseQuery<MPTraits>::MorseQuery(XMLNode &_node) : PRMQuery<MPTraits>(_node) {
    this->SetName("MorseQuery");

//    m_inputDynamicAgentPathFile = _node.Read("agentPath", false, "",
//                                           "filename of dynamic agent path");//read dynamic agent file from xml

    m_mapFileName = _node.Read("inputMap", false, "", "filename of map file");//read map file from xml

    m_dynamicAgent = _node.Read("agent", true, "", "agent robot");

    m_deleteNodes = _node.Read("deleteNodes", false, m_deleteNodes,
                              "Whether or not to delete start and goal from roadmap");

    string cdMethod = _node.Read("cdMethod", true, "", "Collision detection method");

    max_r_speed = _node.Read("r_speed", false, max_r_speed, 1, MAX_INT, "Speed of the agent compared to speed of robot");
//	speedMultiBodyPositionalResolution = this->GetTask()->GetRobot()->GetMultiBody()->GetBoundingSphereRadius() / this->GetEnvironment()->GetPositionRes();
//        throw ParseException(_node.Where(),
//            "Unknown collision detection library '" + cdMethod + "' requested.");

    if(cdMethod == "RAPID")
        m_cdMethod = new Rapid();
    else if(cdMethod == "RAPID_SOLID")
        m_cdMethod = new Rapid(true);
    else
        throw ParseException(_node.Where(),
            "Unknown collision detection library '" + cdMethod + "' requested.");

    m_numAgents = _node.Read("numAgents", false, m_numAgents, 0, MAX_INT,
      "Number of dynamic agents");
    m_divPaths = _node.Read("numPaths", false, m_divPaths, 0, MAX_INT,
        "Number of diverse paths");

    m_agentRdmpFile = _node.Read("agentRdmp", false, "", "Roadmap used to simulate agent paths");

    if(m_numAgents> 0 && !m_agentRdmpFile.empty()) {
        if(!FileExists(m_agentRdmpFile))
            throw ParseException(WHERE,
                "Map file '" + m_agentRdmpFile + "' does not exist.");
    }

    bool defaultsCleared = false;
    for (auto &child : _node) {
        if (child.Name() == "NodeConnectionMethod") {
            if (!defaultsCleared) {
                defaultsCleared = true;
                m_ncLabels.clear();
            }
            m_ncLabels.push_back(child.Read("method", true, "", "Connector method"));
        }
    }

}

template <typename MPTraits>
void MorseQuery<MPTraits>::Print(ostream &_os) const {
    QueryMethod<MPTraits>::Print(_os);
    _os << "\n\tDelete Nodes: " << m_deleteNodes
        << "\n\tConnectors:" << endl;
    for (const auto &label : m_ncLabels)
        _os << "\t\t" << label << endl;
}

template <typename MPTraits>
void MorseQuery<MPTraits>::ReadDynamicAgentPath() {
    m_agentRdmpFile = MPProblem::GetPath(m_agentRdmpFile);

    cout << "Reading dynamic agent path file \'" << m_agentRdmpFile << endl;
    ifstream in(m_agentRdmpFile);
    cout << "File stream ready for dynamic agent path" << endl;

    if (!in.good())
        throw ParseException(WHERE, "Can't open dynamic agent path file '" + m_agentRdmpFile + "'.");

    if (!dynamicAgentPaths.empty())
        dynamicAgentPaths.clear();


    int numDynamicAgents;
    in >> numDynamicAgents;
    cout << "Number of dynamic agents: " << numDynamicAgents << endl;
    in.ignore();

    vector<CfgType> dynamicAgentPath;
    for (int i = 0; i < numDynamicAgents; i++) {
        int number;
        in >> number;
        in.ignore();

        cout << "Should read lines: " << number << " for path " << i << endl;
        dynamicAgentPath.clear();

        for (int j = 0; j < number; ++j) {
            CfgType tempCfg(dynamicAgent);
            double skip;
            in >> skip >> tempCfg;
            //skip the 0s and trailing
            dynamicAgentPath.push_back(tempCfg);
        }

        cout << "Read " << dynamicAgentPath.size() << " for path " << i << endl;
        m_maxPlanningTime = std::max(m_maxPlanningTime, 2.0*dynamicAgentPath.size());
        dynamicAgentPaths.push_back(dynamicAgentPath);
    }
    cout << "Read " << dynamicAgentPaths.size() << " paths for dynamic agent path" << endl;


    in.close();
}

template <typename MPTraits>
bool
MorseQuery<MPTraits>::
InitializeCritical() {
  cout << "InitializeCritical called" << endl;
  bool returnValue = false;
  // Initialize empty membership for each critical point
  auto og = this->GetBlockRoadmap()->GetGraph();
  for(auto cit = og->begin(); cit != og->end(); ++cit){
    m_feasibleCritical.insert({cit->descriptor(), vector<VID>()});
    m_clusters.insert({cit->descriptor(), vector<VID>()});
  }
  // Populate the information for every vertex in the graph
  auto g = this->GetRoadmap()->GetGraph();
  for(auto vit = g->begin(); vit != g->end(); ++vit){
    auto vp = g->GetVertex(vit);
    // only if the critical information is populated by strategy
    if(vp.IsStat("Critical")){
      returnValue = true;
      VID cVID = vp.GetStat("Critical");
      m_clusters[cVID].push_back(vit->descriptor());
      if(vp.IsLabel("FeasibleCritical") && vp.GetLabel("FeasibleCritical"))
        m_feasibleCritical[cVID].push_back(vit->descriptor());
    }
  }
  cout << "InitializeCritical return: " << returnValue << endl;
  return returnValue;
}

template <typename MPTraits>
void MorseQuery<MPTraits>::PrettyPrintSocialNavigationStrategy(SocialNavigationStrategy strategy) {
    switch (strategy) {
        case SocialNavigationStrategy::INCREASE_SPEED_OR_DIVERSE:
            cout << " INCREASE SPEED OR DIVERSE ";
            break;
        case SocialNavigationStrategy::WAIT:
            cout << " WAIT ";
            break;
        case SocialNavigationStrategy::DEFLECT:
            cout << " DEFLECT ";
            break;
        default:
            break;
    }
}

template <class MPTraits>
typename MorseQuery<MPTraits>::SocialNavigationStrategy
MorseQuery<MPTraits>::SocialNavigation(double congestionVal, double deflectCost, double waitCost, double diverseCost) {

    cout << "Congestion value: " << congestionVal << " Deflection Cost: " << deflectCost << " Wait Cost: " << waitCost << " Diverse Cost: " << diverseCost << endl;

    map<double, SocialNavigationStrategy> costStrategymap = {
        {(congestionVal * waitCost), SocialNavigationStrategy::WAIT},
        {(congestionVal * deflectCost), SocialNavigationStrategy::DEFLECT},
        {(congestionVal * diverseCost), SocialNavigationStrategy::INCREASE_SPEED_OR_DIVERSE}
    };

    auto minCost = std::min_element(costStrategymap.begin(), costStrategymap.end(),
                                   [](const auto &a, const auto &b) {
                                       return a.first < b.first;
                                   });

    return minCost->second;
}

template <class MPTraits>
double MorseQuery<MPTraits>::waitCost(double congestionValue, double waitTime) {
    double waitCost = 1;
    cout << "Wait cost: " << (waitCost * congestionValue) << endl;
    return waitTime * congestionValue;
}

template <class MPTraits>
double MorseQuery<MPTraits>::deflectionCost(double congestionValue, CfgType deflectionPoint, CfgType goal) {
    double distanceToGoalAfterDeflection = (goal - deflectionPoint).Magnitude();
    double alpha = 0;
    double deflectionPenalty = 0;
    double cost = (distanceToGoalAfterDeflection + (alpha * congestionValue) + deflectionPenalty);

    cout << "Distance to goal after deflection: " << distanceToGoalAfterDeflection
         << " and deflection path cost: " << cost << endl;

    return cost;
}

template <class MPTraits>
double MorseQuery<MPTraits>::diversePathCost(double congestionValue, Path *newPath) {
    vector<CfgType> path_cfgs = GetCfgs(newPath->VIDs());
    double alpha = 0;
    double distance = (path_cfgs.front() - path_cfgs.back()).Magnitude();
    cout << "Diverse path cost for new path: " << newPath << " = " << distance << endl;
    return (distance + (alpha * congestionValue));
}

template<class MPTraits>
void
MorseQuery<MPTraits>::
GetPaths(const VID _start, const VID _end,size_t k, unordered_map<VID, VID>& oldNew){
  // Copy the graph
  oldNew.clear();
  GraphType* prevg = new GraphType();
  auto gr = this->GetRoadmap()->GetGraph();
  for(auto vit = gr->begin(); vit != gr->end(); ++vit){
	auto newVID = prevg->AddVertex(vit->descriptor(), vit->property());
	oldNew.insert({vit->descriptor(), newVID});
  }
  for(auto eit = gr->edges_begin(); eit != gr->edges_end(); eit++){
    prevg->AddEdge(oldNew[eit->source()], oldNew[eit->target()], eit->property());
  }

  vector<unordered_set<VID>> criticalPoints;
  vector<tuple<VID, VID, WeightType>> deleteEdges;
  unordered_set<VID> chosenCriticalRemoved, store;
  auto start = gr->GetVertex(_start);
  auto goal = gr->GetVertex(_end);
  size_t nR = 1; // number of elements to remove
  // Get Diverse paths
  while(m_paths.size() < k){
    // add back old removed the edges before deleting new
    for(auto e: deleteEdges)
        gr->AddEdge(std::get<0>(e),std::get<1>(e), std::get<2>(e));
    deleteEdges.clear(); // delte the edges
    if(!store.empty()){
      // there are potential feasible critical points to be removed
      size_t i = 0;
      while(i < nR && !store.empty()){
        auto randIndex = rand() % store.size(); // get a random index
        auto it = store.begin();
        std::advance(it, randIndex); // find the random value
        auto randCritical= *(it);
        store.erase(randCritical); // remove them from store
        chosenCriticalRemoved.insert(randCritical); // move to already considered
//        std::cout<<"Chose to remove "<<randCritical<< " out of "<<store.size()<<std::endl;
        // delete the edges new
        for(auto v: m_feasibleCritical[randCritical])
          if(v != _start && v != _end) {
            // instead of deleting vertex delete edge connections - will remain disconnected to find path
            auto vi = gr->find_vertex(v);
            for(auto ei = vi->begin(); ei != vi->end(); ei = vi->begin()){
              deleteEdges.emplace_back(make_tuple(ei->source(), ei->target(), ei->property()));
              gr->DeleteEdge(ei);
            }
          }
        i++;
      }
    }// end of deleting feasible critical points
    else
      std::cout<<"No points to delete"<<std::endl;
    this->GetPath()->Clear(); // Clear previous path data
    auto mapPassed = PRMQuery<MPTraits>::PerformSubQuery(start, goal);
    if(!mapPassed)  // no more path to be found in the current map
    {
      std::cout<<"map not passed"<<std::endl;
      if(store.empty()) // no more new critical points to consirer
      {
        nR++;
        std::cout<<"Final no. of distinct critical points removed:"<<chosenCriticalRemoved.size()<<std::endl;
        if(nR < chosenCriticalRemoved.size()){
          store.insert(chosenCriticalRemoved.begin(), chosenCriticalRemoved.end());
        }
        else break;
      }  //break;
      //else
        //continue;
    }
    else std::cout<<"Map passed"<<std::endl;
	//get the path and store it
    Path* p = new Path(this->GetRoadmap());
	vector<VID> pVID, fVID;
    unordered_set<VID> criticalRegions;
	for(auto v : this->GetPath()->VIDs()){
      pVID.push_back(oldNew[v]); // push in the new vertex index as graph will be reset
      auto vp = gr->GetVertex(v);
	  // find the feasible critical points in the path
	  if(vp.IsLabel("FeasibleCritical") && vp.GetLabel("FeasibleCritical"))
		fVID.push_back(v);
      if(vp.IsStat("Critical"))
        criticalRegions.insert(vp.GetStat("Critical"));
	}
	*p+= pVID;
	// get the corresponding critical point point
	unordered_set<VID> pCritical;
	for(auto f: fVID)
      if(gr->GetVertex(f).IsStat("Critical"))
	    pCritical.insert(gr->GetVertex(f).GetStat("Critical"));

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
	// remove the path vertices from the roadmap excapt start and goal- Case 2
	/*for(auto v : this->GetPath()->VIDs())
	   if(v != _start && v != _end) gr->DeleteVertex(v);*/
	// remove all the feasible critical points - case 1 (from paper)
	/*for(auto f: fVID)
	  gr->DeleteVertex(f);*/
    // remove a random set of feasible critical points - case 1.5 (Aakrit's suggestion)
    if(pCritical.size() == 0) {
      //std::cout<<"Critical regions: "<<criticalRegions.size()<<std::endl;
      for(auto v : this->GetPath()->VIDs())
	   if(v != _start && v != _end) gr->DeleteVertex(v);
      continue;
    }
    else{
     //std::cout<<"Critical points it passes through: "<<pCritical.size()<<std::endl;
     // Remove the feasible critical points of random picked critical point
     for(auto cap: pCritical)
      if(store.find(cap) == store.end()
        && chosenCriticalRemoved.find(cap) == chosenCriticalRemoved.end()) // if not already considered and potential considered
        store.insert(cap);
      //std::cout<<"Inserted points: "<<store.size()<<std::endl;
    }
    // store only if the critical points are different than before
	if(!isSame){
	  // store the critical points for future paths
	  criticalPoints.push_back(pCritical);
	  m_paths.push_back(p);
    }
	else { delete p;
      //std::cout<<"Same path"<<std::endl;
      //if(store.empty()) std::cout<<"Final no. of distinct critical points removed:"<<chosenCriticalRemoved.size()<<std::endl;
      if(store.empty()) {
        nR++;
       // std::cout<<"Final no. of distinct critical points removed:"<<chosenCriticalRemoved.size()<<std::endl;
        if(nR < chosenCriticalRemoved.size()){
          store.insert(chosenCriticalRemoved.begin(), chosenCriticalRemoved.end());
        }
        else break;
      }
     }
  }
  std::cout<<"Number of diverse paths: "<<m_paths.size() <<" out of "<<k <<std::endl;
  // restore the roadmap
  this->GetRoadmap()->SetGraph(prevg);

  // restore the feasibility maps - vids changed with all deletions
  for(auto c: m_clusters)
    for(size_t i = 0; i < c.second.size(); ++i)
      c.second[i] = oldNew[c.second[i]];
  for(auto c: m_feasibleCritical)
    for(size_t i = 0; i < c.second.size(); ++i)
      c.second[i] = oldNew[c.second[i]];
}


template <typename MPTraits>
bool MorseQuery<MPTraits>::PerformSubQuery(const CfgType &_start, const CfgType &_goal) {
    cout << "Evaluating sub-query:" << endl
         << "\tfrom " << _start << endl
         << "\tto   " << _goal << endl;

//    size_t k = 10;
    auto ccs = this->FindCCs();
    cout << "cc size:" << ccs.size() << endl;
    auto start = this->EnsureCfgInMap(_start);
    auto goal = this->EnsureCfgInMap(_goal);
    auto prevPathIDs = this->GetPath()->VIDs();
    bool connected = false; // to store in same connected component
    for (auto cc : ccs) {
        this->ConnectToCC(start.first, cc.second);
        this->ConnectToCC(goal.first, cc.second);

        if (this->SameCC(start.first, goal.first)) {
            connected = true;
           // this->GeneratePath(start.first, goal.first);
            break;
        }
    }
    cout << "connected: " << connected << endl;
    if (connected) {
        InitializeCritical();
        // Find the clusters the start and goal belongs to
        auto g = this->GetRoadmap()->GetGraph();
        auto sit = g->find_vertex(start.first);
        auto git = g->find_vertex(goal.first);
        for(auto eit= sit->begin(); eit != sit->end(); eit++){
            auto op = g->GetVertex(eit->source());
            if(op.IsStat("Critical")){
                sit->property().SetStat("Critical",op.GetStat("Critical"));
                m_clusters[op.GetStat("Critical")].push_back(sit->descriptor());
                break;
            }
            op = g->GetVertex(eit->target());
            if(op.IsStat("Critical")){
                sit->property().SetStat("Critical",op.GetStat("Critical"));
                m_clusters[op.GetStat("Critical")].push_back(sit->descriptor());
                break;
            }
        }
        for(auto eit= git->begin(); eit != git->end(); eit++){
            auto op = g->GetVertex(eit->source());
            if(op.IsStat("Critical")){
                git->property().SetStat("Critical",op.GetStat("Critical"));
                m_clusters[op.GetStat("Critical")].push_back(git->descriptor());
                break;
            }
            op = g->GetVertex(eit->target());
            if(op.IsStat("Critical")){
                git->property().SetStat("Critical",op.GetStat("Critical"));
                m_clusters[op.GetStat("Critical")].push_back(git->descriptor());
                break;
            }
        }
        // diverse path planning
//        vector<Path *> m_paths;
        unordered_map<VID, VID> oldNew;
        string clockName = this->GetNameAndLabel() + "::waitDeflectQuery";
        this->GetStatClass()->StartClock(clockName);
        GetPaths(start.first, goal.first, m_divPaths, oldNew);
        std::sort(m_paths.begin(), m_paths.end(),
                 [](Path *path1, Path *path2) {
                     return path1->Size() < path2->Size();
                 });
        this->GetStatClass()->StopClock(clockName);
        cout << "Paths size: " << m_paths.size() << endl;
        clockName = this->GetNameAndLabel() + "::SocialNavigationQuery";
        this->GetStatClass()->StartClock(clockName);
        connected = GetValidPath();//(m_paths);
        this->GetStatClass()->StopClock(clockName);
        if(connected){
            // restore the previous path as its is a sub-query
            vector<VID> p;
            for(auto v: prevPathIDs)
                p.push_back(oldNew[v]);
            // append the sub-query path
            auto newVID = this->GetPath()->VIDs();
            p.insert(p.end(), newVID.begin(), newVID.end());
            this->GetPath()->Clear();
            *this->GetPath() += p;
        }
    }
    if (this->m_debug) {
        if (connected)
            cout << "\tSuccess: found path from start node " << start.first
                 << " to goal node " << goal.first << "." << endl;
        else
            cout << "\tFailed to connect start node " << start.first
                 << " to goal node " << goal.first << "." << endl;
    }
    // Remove start and goal nodes if necessary
    if (m_deleteNodes) {
        PRMQuery<MPTraits>::RemoveTempCfg(start);
        PRMQuery<MPTraits>::RemoveTempCfg(goal);
    }
    return connected;
}
template<class MPTraits>
void
MorseQuery<MPTraits>::
calculateIntermediates(VID start, VID goal, vector<Cfg>& _edgeCfg){
  // Get the next edge.
  		auto g= this->GetRoadmap()->GetGraph();
        typename GraphType::adj_edge_iterator ei;
        {
            typename GraphType::edge_descriptor ed(start, goal);
            typename GraphType::vertex_iterator vi;
            g->find_edge(ed, vi, ei);
        }
        typename MPTraits::MPLibrary::LocalPlannerPointer lp;
        try {
            lp = this->GetLocalPlanner(ei->property().GetLPLabel());
        }
        catch(...) {
            lp = this->GetLocalPlanner("sl");
        }
        vector<CfgType> inter = ei->property().GetIntermediates(); // intermediates represent each tick
        // Recreate this edge, including intermediates.
        inter.insert(inter.begin(), g->GetVertex(start));
        CfgType& e   = g->GetVertex(goal);
        inter.push_back(e);
        for(auto cit = inter.begin(); cit + 1 != inter.end(); ++cit) {
            vector<CfgType> edge = lp->ReconstructPath(*cit, *(cit+1),
                vector<CfgType>(), this->GetEnvironment()->GetPositionRes(), this->GetEnvironment()->GetOrientationRes());
            _edgeCfg.insert(_edgeCfg.end(), edge.begin(), edge.end());
        }
        _edgeCfg.push_back(e);
}
template<class MPTraits>
double MorseQuery<MPTraits>::calculateWaitTimeCfg(vector<CfgType>& robotcfgs, double startTick, vector<double>& startWindowEndtime, int r_speed) {
    vector<vector<pair<double, double>>> windows;
    int tickLimit = robotcfgs.size();
    windows.push_back(vector<pair<double, double>>()); // Create a slot for start vertex
    GetSafeWindowsInterval(robotcfgs[0], startTick, windows.back(), m_maxPlanningTime, 1);
    if(windows.back().empty()) return INT_MAX; // this would never happen except for the very start of the path
    startWindowEndtime.push_back(windows[0].front().second);
    double upperWaitTime = std::min(windows[0].front().second - startTick, m_maxPlanningTime);
    if (upperWaitTime>0) cout<<"upper wait time"<<upperWaitTime<<endl;
    for (int x = 1; x < tickLimit; x++){
       windows.push_back(vector<pair<double, double>>()); // Create a slot for cfg x
//       int agent_time_for_config_x = startTick + static_cast<int>(floor(static_cast<double>(x) / r_speed));
//       GetSafeWindowsInterval(robotcfgs[x], agent_time_for_config_x, windows.back(), upperWaitTime, r_speed);
       GetSafeWindowsInterval(robotcfgs[x], startTick + x/static_cast<double>(r_speed), windows.back(), upperWaitTime, r_speed);
    }
    double currentTick = startTick;
    double currentWait = 0;
    vector<size_t> indices(robotcfgs.size(), 0);
    while(currentWait <= upperWaitTime){
      bool safe = true;
      for (int x = 1; x < tickLimit; x++){
         if(indices[x] >= windows[x].size()){
           safe = safe & false;
         }
         else {
            // Get the window at indices[x]
            auto window = windows[x][indices[x]];
            // running tick for intermediate x
//            currentTick = startTick + currentWait + static_cast<int>(floor(static_cast<double>(x) / r_speed));
            currentTick = startTick + x + currentWait; // start tick + wait considered at start + index
            if(currentTick >= window.first && currentTick <= window.second){  // safe window
              safe = safe & true;
            }
            else if(currentTick < window.first){ // collison window
              safe = safe & false;
            }
            // update indices
            if(currentTick == window.second && indices[x] < windows[x].size()){
              indices[x]++;
            }
         }
      }// end of for
      if(safe){
        // found a safe wait Time for all intermediates
        return currentWait;
      }
      else // continue with next index
        currentWait+=1.0;
    }
    return INT_MAX;
 }


template<class MPTraits>
bool MorseQuery<MPTraits>::GetValidPath() {

    static auto AddToPath = [&](Path* _newPath, const vector<VID>& _segment) {
        vector<VID> addVID;
        if (!_newPath->VIDs().empty())
            addVID.insert(addVID.end(), _segment.begin() + 1, _segment.end());
        else
            addVID.insert(addVID.end(), _segment.begin(), _segment.end());

        *_newPath += addVID;
    };

    if (dynamicAgentPaths.empty()) {
        if (m_paths.empty()) return false;
        this->GetPath()->Clear();
        *this->GetPath() += m_paths[0]->VIDs();
        return true;
    }

    bool oneSuccess = false;
    Path* bestPath = nullptr;
    size_t bestPathIndex;
    int bestPathTick;

    std::unordered_map<size_t, std::vector<std::pair<VID, double>>> waitTimes;
    std::unordered_map<size_t, std::vector<std::pair<VID, int>>> speedRecords;

    for (size_t i = 0; i < m_paths.size(); ++i) {
        cout << "path size for path:" << i << " is " << m_paths[i]->Length() << endl;

        auto pathLengthBefore = m_paths[i]->Length();
        auto pathVIDs = m_paths[i]->VIDs();
        auto start = pathVIDs.front();
        vector<VID> segment;
        Path* newPath = new Path(this->GetRoadmap());
        bool result = true;
        double startTick = 0.0;
        r_speed = 1;

        double segmentCount = 0.0, currentCount = 0.0;

        // Pre-pass to count number of segments
        while (true) {
            segmentCount++;
            segment.clear();
            auto criticals = GetNextPathRegion(start, m_paths[i], segment);
            auto goal = segment.back();
            vector<VID> newSegment;

            if (!newPath->VIDs().empty() && newPath->VIDs().back() == pathVIDs.back())
                break;

            if (!newPath->VIDs().empty())
                start = newPath->VIDs().back();

            bool foundNext = GetNextSubPath(start, segment, criticals, newSegment);
            AddToPath(newPath, segment);

            if (!foundNext) {
                cout << "Path " << i << " cannot find the next segment " << endl;
                result = false;
                break;
            }

            if (result)
                start = goal;
        }

        // Reset state for real construction
        segment.clear();
        newPath->Clear();
        start = pathVIDs.front();

        while (true) {
            currentCount++;
            segment.clear();
            vector<pair<VID, VID>> edgeToDelete;
            vector<VID> newSegment;
            auto criticals = GetNextPathRegion(start, m_paths[i], segment);
            auto goal = segment.back();

            if (!newPath->VIDs().empty() && newPath->VIDs().back() == pathVIDs.back())
                break;

            if (!newPath->VIDs().empty())
                start = newPath->VIDs().back();

            bool foundNext = GetNextSubPath(start, segment, criticals, newSegment);

            if (!foundNext) {
                cout << "Path " << i << " cannot find the next segment " << endl;
                result = false;
                break;
            }

            segment.swap(newSegment);
            cout << "Evaluating segment: start " << start << " to goal " << goal << " with speed " << r_speed << endl;

            double tempTick = startTick;
            bool validSegment = DynamicValidate(segment, tempTick, edgeToDelete, r_speed);

            if (validSegment) {
                AddToPath(newPath, segment);
                start = segment.back();
                startTick = tempTick;
                result = true;
                for (auto vid : segment) {
                    cout << "Setting speed for " << vid << " : " << r_speed << endl;
                    speedRecords[i].emplace_back(vid, r_speed);
                }
                r_speed = 1;
            } else {
                double deflectionCost = MAXDOUBLE, waitCost = MAXDOUBLE, diversePathCost = MAXDOUBLE;

                cout << "Calculating wait costs" << endl;
                vector<double> pathWaitTimes;
                auto waitReturn = calculateWaitTime(segment, startTick, pathWaitTimes);
                double wt = waitReturn.first;
                waitCost = (wt < INT_MAX) ? (waitReturn.second - startTick) : MAXDOUBLE;
                cout << "wait cost is this :: " << waitCost << endl;

                cout << "Calculating deflection costs" << endl;
                Path* duplicatePath_Diverse = new Path(this->GetRoadmap());
                double lengthBeforeDeflection = GetSegmentLength(segment);
                double possibleDeflectTick = startTick;

                bool deflectResult = GenerateValidSubPath(start, segment, criticals, possibleDeflectTick, duplicatePath_Diverse, r_speed);
                cout << "Tick after deflection: " << possibleDeflectTick << " original: " << startTick << endl;

                if (deflectResult) { // defelction is valid
                    deflectionCost = (possibleDeflectTick - startTick);
                    double segmentLengthAfterDeflection = GetSegmentLength(duplicatePath_Diverse->VIDs());
                    cout << "Deflection cost: " << deflectionCost
                         << ", lengthAfter: " << segmentLengthAfterDeflection
                         << ", lengthBefore: " << lengthBeforeDeflection << endl;
                }

                double remainingSegments = segmentCount - currentCount;
                double weightForWait = 1 - (remainingSegments / segmentCount);

                cout << "Remaining segments: " << remainingSegments
                     << " weight: " << weightForWait << endl;

//                auto strategy = SocialNavigation(1, weightForWait * deflectionCost, (1 - weightForWait) * waitCost, diversePathCost);
                auto strategy = SocialNavigation(1, 1*deflectionCost, 1*waitCost, diversePathCost);

                cout << "Social Navigation Strategy returned: ";
                PrettyPrintSocialNavigationStrategy(strategy);
                cout << endl;

                switch (strategy) {
                    case SocialNavigationStrategy::DEFLECT:
                        result = deflectResult;
                        if (result) {
                            AddToPath(newPath, duplicatePath_Diverse->VIDs());
                            startTick = possibleDeflectTick;
                            for (auto vid : duplicatePath_Diverse->VIDs()) {
                                cout << "In deflect, setting speed for " << vid << " : " << r_speed << endl;
                                speedRecords[i].emplace_back(vid, r_speed);
                            }
                            r_speed = 1;
                        }
                        break;

                    case SocialNavigationStrategy::WAIT:
                        for (size_t index = 0; index < pathWaitTimes.size(); ++index) {
                            if (pathWaitTimes[index] > 0)
                                waitTimes[i].emplace_back(segment[index], pathWaitTimes[index]);
                        }

                        if (!waitTimes[i].empty() && waitCost < MAXDOUBLE) {
                            result = true;
                            AddToPath(newPath, segment);
                            startTick = waitReturn.second;
                            for (auto vid : segment) {
                                cout << "In wait, setting speed for " << vid << " : " << r_speed << endl;
                                speedRecords[i].emplace_back(vid, r_speed);
                            }
                            r_speed = 1;
                        } else result = false;
                        break;

                    default:
                        result = false;
                        break;
                }
                //we neeed to do  speed jump so need to find the radius of the robots free space to
                if (!result) { // if we did not solve by either wait or deflect, try increasing speed
                    auto next_speed = r_speed+1;
                    if (next_speed <= (max_r_speed) && foundNext) { // check whether speed can be increased and if we found next segment
                        std::cout<<"Increasing speed \ncurrent speed: "<<r_speed<<endl;
                        r_speed = next_speed;
                        std::cout<<"new speed: "<<r_speed<<endl;
                        currentCount -= 1; // since we did not travel this segment, reset current count.
                    } else break; // speed cannot be increased, go to next path
                } else { // segment traversed
                    start = goal;
                }
            }
        }

        cout << "Path " << i << " success: " << (result? "true" : "false") << endl;
        if (result) {
            if (!oneSuccess) {
                bestPath = newPath;
                oneSuccess = true;
                bestPathIndex = i;
                bestPathTick = startTick;
                break;
            } else if (newPath->Length() < bestPath->Length()) {
                bestPath = newPath;
                bestPathIndex = i;
            } else {
                delete newPath;
            }
        } else {
            delete newPath;
        }

        cout << "================================================xxxxxx================================================" << endl;
    }

    if (oneSuccess) {
        cout << "Stored path tick : " << bestPathTick << endl;
        cout << "Stored best path length: " << bestPath->Length() << endl;
        cout << "Deflection length: " << GetDeflectionLength(bestPath, m_paths[bestPathIndex]) << endl;

        if (waitTimes.find(bestPathIndex) != waitTimes.end()) {
            for (const auto& pair : waitTimes[bestPathIndex]) {
                cout << "(vid: " << pair.first << ", wait: " << pair.second << ") ";
                bestPath->GetRoadmap()->GetGraph()->GetVertex(pair.first).SetStat("waittime", pair.second);
            }
            cout << endl;
        }

        for (const auto& pair : speedRecords[bestPathIndex]) {
            cout << "(vid: " << pair.first << ", speed: " << pair.second << ") ";
            bestPath->GetRoadmap()->GetGraph()->GetVertex(pair.first).SetStat("speed", pair.second);
        }

        this->GetPath()->Clear();
        *this->GetPath() += bestPath->VIDs();
    }

    return oneSuccess;
}


template<class MPTraits>
std::pair<typename MPTraits::RoadmapType::VID,typename MPTraits::RoadmapType::VID>
MorseQuery<MPTraits>::
GetNextPathRegion(const VID _start, Path* _p, vector<VID>& _segment){
  auto vids = _p->VIDs();
  bool foundStart = false;
  auto gr = this->GetRoadmap()->GetGraph();
  vector <VID> criticals;
  // Look into each vid in the path
  for(auto id : vids) {
    if(!foundStart && id == _start){ // ignore vids if start is not found
      foundStart = true;
      _segment.push_back(id);
      if(gr->GetVertex(id).IsStat("Critical")){
        criticals.push_back(gr->GetVertex(id).GetStat("Critical"));
      }
    } else if(foundStart){  // start is found, need to find the end point-feasible region
      _segment.push_back(id);
      auto vp = gr->GetVertex(id);
      auto lastCritical = criticals.back();
      if(vp.IsStat("Critical")){
        if(lastCritical != vp.GetStat("Critical")){
          criticals.push_back(vp.GetStat("Critical"));
          //std::cout<<"Pushed"<<criticals.back()<<std::endl;
          break;
        }
      }
      else
        std::cout<<"No critical label"<<std::endl;
      if(vp.IsLabel("FeasibleCritical") && vp.GetLabel("FeasibleCritical") && criticals.back() != lastCritical)
		break;
    }
  }//end for

  if(criticals.empty()) return make_pair(VID(), VID());
  else
    return make_pair(criticals.front(), criticals.back());
}

template<typename MPTraits>
bool
MorseQuery<MPTraits>::
GetNextSubPath(VID _start, vector<VID>& _segment, std::pair<VID, VID> _cpoints, vector<VID>& _addVID){
  // end of segment or missing critical regions
  if(_cpoints.first == _cpoints.second && _cpoints.first == VID()){//cpoint is the region index where the start is there
    std::cout<<"Empty critical region()"<<std::endl;
    return false;
  }
  // Get the points in the region
  vector<VID> oldVIDs, feasibleVIDs;
  oldVIDs.insert(oldVIDs.end(),m_clusters[_cpoints.first].begin(), m_clusters[_cpoints.first].end());
  if(_cpoints.first == _cpoints.second){ // Last or goal point region
    feasibleVIDs.push_back(_segment.back()); // push the query goal as the only goal
  }
  else{ // Other segment rehion
   oldVIDs.insert(oldVIDs.end(),m_clusters[_cpoints.second].begin(), m_clusters[_cpoints.second].end());
   feasibleVIDs.assign(m_feasibleCritical[_cpoints.second].begin(), m_feasibleCritical[_cpoints.second].end());
  }
  // Generate the sub-graph
  GraphType* subgraph = new GraphType();
  unordered_map<VID, VID> oldNew, newOld;
  auto gr = this->GetRoadmap()->GetGraph();
  for(auto vid: oldVIDs){
	auto newVID = subgraph->AddVertex(gr->GetVertex(vid));
	oldNew.insert({vid, newVID});
    newOld.insert({newVID, vid});
  }
  for(auto vid: oldVIDs){
    auto vit = gr->find_vertex(vid);
    for(auto eit = vit->begin(); eit != vit->end(); eit++){
      if(oldNew.find(eit->source()) == oldNew.end()||oldNew.find(eit->target()) == oldNew.end()) continue;
      if(!subgraph->IsEdge(oldNew[eit->source()], oldNew[eit->target()]))
        subgraph->AddEdge(oldNew[eit->source()], oldNew[eit->target()], eit->property());
    }
  }
 auto start = oldNew[_start];
 unordered_map<VID, vector<VID>> segments;
  vector<pair<VID, double>> vidLengths;
  // Plan sub-path to every feasible critical points
  double minLength = std::numeric_limits<double>::max();
  _addVID.clear();
//  std::cout <<" Number of sub-goals: "<<feasibleVIDs.size()<<std::endl;
  //for(auto gid: feasibleVIDs){
       auto goal = oldNew[_segment.back()];//oldNew[gid];
       vector<VID> path;
       switch(m_searchAlg) {
        case DIJKSTRAS:
            find_path_dijkstra(*subgraph, start, goal, path, MPTraits::WeightType::MaxWeight());
            break;
        case ASTAR:
            Heuristic<MPTraits> heuristic(subgraph->GetVertex(goal),
            this->GetEnvironment()->GetPositionRes(),
            this->GetEnvironment()->GetOrientationRes());
            astar(*subgraph, start, goal, path, heuristic);
            break;
        } //end switch
       if(path.empty()) return false;//continue; // No path found
       // revise the segment with old ids
       vector<VID> newSegment;
       for(auto p: path)
        newSegment.push_back(newOld[p]);
       double segLength = GetSegmentLength(newSegment); //std::cout << "Found segment path" <<std::endl;
       if(segLength < minLength){ // store the minimum length path
         _addVID.swap(newSegment);
         minLength = segLength;
       }
   // }
    return !(_addVID.empty());
}

//look into this
template <typename MPTraits>
bool
MorseQuery<MPTraits>::
GenerateValidSubPath(VID _start, vector<VID>& _segment, std::pair<VID, VID> _cpoints, double& _startTick, Path* _newPath, int r_speed){
    cout << "Gets here: GenerateValidSubPath. current tick:" << _startTick << endl;
  // end of segment or missing critical regions
  if(_cpoints.first == _cpoints.second && _cpoints.first == VID()){//cpoint is the region index where the start is there
    std::cout<<"Empty critical region()"<<std::endl;
    return false;
  }
  // Get the points in the region
  vector<VID> oldVIDs, feasibleVIDs;
  oldVIDs.insert(oldVIDs.end(),m_clusters[_cpoints.first].begin(), m_clusters[_cpoints.first].end());
  if(_cpoints.first == _cpoints.second){ // Last or goal point region
    feasibleVIDs.push_back(_segment.back()); // push the query goal as the only goal
  }
  else{ // Other segment rehion
   oldVIDs.insert(oldVIDs.end(),m_clusters[_cpoints.second].begin(), m_clusters[_cpoints.second].end());
   feasibleVIDs.assign(m_feasibleCritical[_cpoints.second].begin(), m_feasibleCritical[_cpoints.second].end());
  }
  // Generate the sub-graph
  GraphType* subgraph = new GraphType();
  unordered_map<VID, VID> oldNew, newOld;
  auto gr = this->GetRoadmap()->GetGraph();
  for(auto vid: oldVIDs){
	auto newVID = subgraph->AddVertex(gr->GetVertex(vid));
	oldNew.insert({vid, newVID});
    newOld.insert({newVID, vid});
  }
  for(auto vid: oldVIDs){
    auto vit = gr->find_vertex(vid);
    for(auto eit = vit->begin(); eit != vit->end(); eit++){
      if(oldNew.find(eit->source()) == oldNew.end()||oldNew.find(eit->target()) == oldNew.end()) continue;
      if(!subgraph->IsEdge(oldNew[eit->source()], oldNew[eit->target()]))
        subgraph->AddEdge(oldNew[eit->source()], oldNew[eit->target()], eit->property());
    }
  }
  auto start = oldNew[_start];//(_newPath->VIDs().empty()) ? oldNew[_segment.front()] : oldNew[_newPath->VIDs().back()];
  unordered_map<VID, vector<VID>> segments;
  vector<pair<VID, double>> vidLengths;
  size_t i = 0;
  while(subgraph->get_num_edges() > 0){
    i = i+1;
    vidLengths.clear();
    segments.clear();
    vector<pair<VID, VID>> edgeToDelete;
    // Plan sub-path to every feasible critical points
    for(auto gid: feasibleVIDs){
       auto goal = oldNew[gid];
       vector<VID> path;
       switch(m_searchAlg) {
        case DIJKSTRAS:
            find_path_dijkstra(*subgraph, start, goal, path, MPTraits::WeightType::MaxWeight());
            break;
        case ASTAR:
            Heuristic<MPTraits> heuristic(subgraph->GetVertex(goal),
            this->GetEnvironment()->GetPositionRes(),
            this->GetEnvironment()->GetOrientationRes());
            astar(*subgraph, start, goal, path, heuristic);
            break;
        } //end switch
       if(path.empty()) continue; // No path found
       // revise the segment with old ids
       vector<VID> newSegment;
       for(auto p: path)
        newSegment.push_back(newOld[p]);
      segments.insert({gid, newSegment});
      vidLengths.push_back({gid,GetSegmentLength(newSegment)});
    }
    if(vidLengths.empty()) return false;  // No path found for any sub-goal
    // sort the segments based on the length
    std::sort(vidLengths.begin(), vidLengths.end(), [](pair<VID, double> a, pair<VID, double> b)
                                  {
                                      return a.second < b.second;
                                  });
    // Validate each segment one by one and return on first valid
    for(auto vid: vidLengths){
      // validate the segment
      if(DynamicValidate(segments[vid.first], _startTick, edgeToDelete, r_speed)){
        vector<VID> addVID;
        if(!_newPath->VIDs().empty()) //if not the first segment, do not include the start point its' a repeat'
          addVID.insert(addVID.end(), segments[vid.first].begin()+1, segments[vid.first].end());
        else
          addVID.insert(addVID.end(), segments[vid.first].begin(), segments[vid.first].end());
        *_newPath += addVID;
        std::cout<<"Found the local path at iteration:"<<i<< " and _startTick is: " << _startTick << std::endl;
        return true;//before trying the next thing just check if the segment has wait time. then return true with all the wait times and if the wait is true then add the start tick.
      }
    }//end-for
    if(edgeToDelete.empty()) return false;
    // Remove the edges if all the paths come up to be invalid
    for(auto eid: edgeToDelete){
      if(subgraph->IsEdge(oldNew[eid.first], oldNew[eid.second])){// std::cout<<"Deleted"<< eid.first << ", "<<eid.second<<std::endl;
        subgraph->DeleteEdge(oldNew[eid.first], oldNew[eid.second]);
      }
    }
  }//end-while
  return false;
}

template <class MPTraits>
bool
MorseQuery<MPTraits>::
DynamicValidate(vector<VID>& _segment, double &_startTick, vector<pair<VID, VID>>& _edgeToDelete, int r_speed){
  auto g = this->GetRoadmap()->GetGraph();
  // start configuration validity check
   CfgType startCfg = g->GetVertex(_segment.front());
  // validate the start _cfg
  if(!ValidCfg(startCfg, _startTick, r_speed)) return false;
  double tick = _startTick; // - 1; // using pre-increment on ticks on validation call
  // Validate each edge
  for(auto it = _segment.begin(); it + 1 < _segment.end(); ++it) {
    // Get the next edge.
    typename GraphType::adj_edge_iterator ei;
    {
      typename GraphType::edge_descriptor ed(*it, *(it+1));
      typename GraphType::vertex_iterator vi;
      g->find_edge(ed, vi, ei);
    }
    typename MPTraits::MPLibrary::LocalPlannerPointer lp;
    try {
        lp = this->GetLocalPlanner(ei->property().GetLPLabel());
      }
      catch(...) {
        lp = this->GetLocalPlanner("sl");
      }
    vector<CfgType> inter = ei->property().GetIntermediates(); // intermediates represent each tick
     // Recreate this edge, including intermediates.
    inter.insert(inter.begin(), g->GetVertex(*it));
    CfgType& e   = g->GetVertex(*(it+1));
    inter.push_back(e);
    // Construct a resolution-level path along the recreated edge.
    vector<CfgType> edgeCfg;
    for(auto cit = inter.begin(); cit + 1 != inter.end(); ++cit) {
      vector<CfgType> edge = lp->ReconstructPath(*cit, *(cit+1),
          vector<CfgType>(), this->GetEnvironment()->GetPositionRes(), this->GetEnvironment()->GetOrientationRes());
      edgeCfg.insert(edgeCfg.end(), edge.begin(), edge.end());
    }
    edgeCfg.push_back(e);
    double tickIncrement = 1.0 / r_speed;
//    cout << "before for loop tick increment " << tickIncrement << endl;
    for(auto& c: edgeCfg){
      tick += tickIncrement;
//      cout << "tick increment: " << tick << endl;
      if(!ValidCfg(c, tick, r_speed)  ) {
        _edgeToDelete.push_back(make_pair(*it, *(it+1)));
        return false;
      }
    }
   }
   _startTick = tick;//-1; // decrement for next segment start point = end point for this
  return true;
}

template <class MPTraits>
int
MorseQuery<MPTraits>::
GetSegmentTickLength(vector<VID>& _segment){
  auto g = this->GetRoadmap()->GetGraph();
  // start configuration validity check
  CfgType startCfg = g->GetVertex(_segment.front());
  int tick = 0;
  for(auto it = _segment.begin(); it + 1 < _segment.end(); ++it) {
    // Get the next edge.
    typename GraphType::adj_edge_iterator ei;
    {
        typename GraphType::edge_descriptor ed(*it, *(it+1));
        typename GraphType::vertex_iterator vi;
        g->find_edge(ed, vi, ei);
    }
    typename MPTraits::MPLibrary::LocalPlannerPointer lp;
    try {
        lp = this->GetLocalPlanner(ei->property().GetLPLabel());
    }
    catch(...) {
        lp = this->GetLocalPlanner("sl");
    }
    vector<CfgType> inter = ei->property().GetIntermediates(); // intermediates represent each tick
    // Recreate this edge, including intermediates.
    inter.insert(inter.begin(), g->GetVertex(*it));
    CfgType& e   = g->GetVertex(*(it+1));
    inter.push_back(e);
    // Construct a resolution-level path along the recreated edge.
    vector<CfgType> edgeCfg;
    for(auto cit = inter.begin(); cit + 1 != inter.end(); ++cit) {
        vector<CfgType> edge = lp->ReconstructPath(*cit, *(cit+1),
            vector<CfgType>(), this->GetEnvironment()->GetPositionRes(), this->GetEnvironment()->GetOrientationRes());
        edgeCfg.insert(edgeCfg.end(), edge.begin(), edge.end());
	}
    edgeCfg.push_back(e);
    for(auto& c: edgeCfg) {
      tick++;
    }
  }
  return tick;
}

template <class MPTraits>
vector<vector<pair<int, int>>>
MorseQuery<MPTraits>::
GetSafeWindows(vector<CfgType>& _segment, int _startTick, int r_speed) {
    // Get the first configuration from segment to check initial validity
    CfgType startCfg = _segment.front();
    // Initialize vector to store safe windows for each dynamic agent
    vector<vector<pair<int, int>>> safeWindows;
    // First check if start configuration is valid at start time
    if(!ValidCfg(startCfg, _startTick, r_speed)) {
        // If start invalid, create invalid windows for all agents
        vector<pair<int, int>> windows;
        for(size_t i= 0; i < dynamicAgentPaths.size(); i++) {
            //create invalid windows -1,-1 as tick cannot be negative
            windows.push_back(make_pair(-1,-1));
            safeWindows.push_back(windows);
        }
        cout << "Invalid safe window (-1, -1), start cfg not valid. start tick = " << _startTick << endl;
        return safeWindows;
    }
    // Store start tick for validation calls
    int tick = _startTick; // using pre-increment on ticks on validation call
    // Find non-collision windows for each dynamic agent
    for(size_t i= 0; i < dynamicAgentPaths.size(); i++) {
        // Get collision-free time windows for current agent
        vector<pair<int, int>> windows = findNonCollisionWindows(_segment, _startTick, i);
        // Add windows to collection for all agents
        safeWindows.push_back(windows);
    }
//    cout << "For start tick = " << _startTick << ", returning safewindows: " << safeWindows.size() <<endl;
    cout << endl;
    // Return collection of safe windows for all agents
    return safeWindows;
}

template <class MPTraits>
void
MorseQuery<MPTraits>::
GetSafeWindowsInterval(CfgType& _cfg, double _startTick, vector<pair<double, double>>& _safeWindows,  double _max_length, size_t j, int r_speed){
    // Function to validate across every agent
    static auto ValidateAgainstAll = [&](CfgType& _cfg, double tick){
      for(size_t i= 0; i < dynamicAgentPaths.size(); i++){
        if(!ValidateCfgAgainstDynamicAgent(_cfg, tick, i, r_speed))
           return false;  // early quit if invalid against at least one agent
      }
      return true;
    };
	// Initialize first window start time with starting tick
    double t1;
    // Check if the configuration collides at start time
    bool isColliding = !ValidateAgainstAll(_cfg, _startTick);
    // If starting with collision, invalidate window start time
    if(isColliding) {
       t1 = -1;  // Invalid t1 to indicate we're in collision so no valid window start time
    }
    else t1 = _startTick;
    // Initialize tick counter for checking future times
    double currentTick;
    // Check each future time step up to maximum planning time
    for(size_t cfgIdx = 1; cfgIdx <= _max_length; cfgIdx++) {
       // Calculate actual tick by adding offset to start time
       currentTick = _startTick + cfgIdx;
       // Check if configuration collides at current time
       bool currentCollision = !ValidateAgainstAll(_cfg, currentTick);
       // If collision state changed (from colliding to free or vice versa)
       if(currentCollision != isColliding) {
          if(currentCollision) {
            // transition from collsiion free to colliding so close current window with end time as previous tick
            _safeWindows.push_back({t1, currentTick - 1.0/r_speed});
            if (_safeWindows.size()>=j) return;
            t1 = -1;  // reset window start time
           } else {
            // transition from colliding to collision free so start new window at current tick
            t1 = currentTick;
           }
           //update collision state for next iteration
           isColliding = currentCollision;
      }
    }
    // if configuration ends with collision free state
    if(!isColliding) {
      //add final window extending to infinity
      _safeWindows.push_back({t1, INT_MAX});

    }
}

// Function finds safe time windows for a edge of robot configurations
template <class MPTraits>
void
MorseQuery<MPTraits>::
GetSafeWindows(CfgType& _cfg, int _startTick, vector<vector<pair<int, int>>>& _safeWindows, int _max_length) {
    // Store start tick for validation calls
    int tick = _startTick; // using pre-increment on ticks on validation call
    // Find non-collision windows for each dynamic agent
    for(size_t i= 0; i < dynamicAgentPaths.size(); i++) {
        // Get collision-free time windows for current agent
        vector<pair<int, int>> windows;
        findNonCollisionWindows(_cfg, _startTick, i, windows, _max_length);
        // Add windows to collection for all agents
        _safeWindows.push_back(windows);
    }
    //cout << "For start tick = " << _startTick << ", returning safewindows: " << _safeWindows.size() <<endl;
    //cout << endl;
    // Return collection of safe windows for all agents
    //return safeWindows; // DO not return vectors it makes a copy - expensive to memory
}


template<class MPTraits>
pair<double, double>
MorseQuery<MPTraits>::
calculateWaitTime(vector<VID>& _segment, double startTick, vector<double>& _waitTimes) {// Function calculates total wait time needed for a path segment
    // Vector to store configurations along edges
    vector<Cfg> edgeCfg;
    // Current time tracking
    auto tick = startTick;
    // Track total wait time for entire segment
    double totalWaitTime = 0;
    // Vectors to track window end times and start times for backtracking
    vector<double> startWindowEndtime, startTimes;
    // Store initial start time
    startTimes.push_back(startTick);
    // Process each vertex pair in segment edges
    for(auto it = _segment.begin(); it + 1 < _segment.end(); ++it) {
        // Clear previous edge configurations
        edgeCfg.clear();
        // Add start vertex configuration
        edgeCfg.push_back(this->GetRoadmap()->GetGraph()->GetVertex(*it));
        // Calculate intermediate configurations between vertices
        calculateIntermediates(*it, *(it+1), edgeCfg);
        // Calculate wait time needed for this edge's configurations
        double waitTime=calculateWaitTimeCfg(edgeCfg, tick, startWindowEndtime, r_speed);
        if(waitTime>0 && waitTime != INT_MAX) cout<< "calculated wait time in calculateWaitTime, waitTime: " << waitTime << endl;
        // If wait time is infinite, path is impossible
        if(waitTime==INT_MAX) {
            cout<< "calculated wait time is too big" << endl;
            return {INT_MAX,INT_MAX};
        }
        // Add wait time to total
        totalWaitTime += waitTime;
        // Update current tick (wait time + time to traverse edge)
        tick += waitTime + (edgeCfg.size() - 1)/static_cast<double>(r_speed);
        // Store wait time for this edge
        _waitTimes.push_back(waitTime);

        //check whether backtracking is possible so iterate through previous window end times from newest to oldest
        for(int i = startWindowEndtime.size()-1; i>=0; i--) {
            // Calculate time after waiting
            int sTime = startTimes[i] + waitTime;
            // If window end time allows for waiting
            if(startWindowEndtime[i] > sTime) {
              //possible wait points if not in the current window
              if(i != startWindowEndtime.size()-1)
                  std::cout<<"Can wait at index " << i <<std::endl;
              break;
            }
        }
        // update the starttime to be inclusive of wait at its start
        startTimes.back() += waitTime;
        // next start time will include edge traversal time
        startTimes.push_back(startTimes.back() + (edgeCfg.size() - 1)/static_cast<double>(r_speed));
    }
    cout<< "finished calculating waiting time and tick is " <<tick<<endl;
    // Return total wait time and final tick
    return {totalWaitTime,tick};
}

// NEW FUNCTION
// Function finds time windows where configurations don't collide with a specific dynamic agent
template <class MPTraits>
void
MorseQuery<MPTraits>::
findNonCollisionWindows(CfgType& _cfg, double startTick, int agentIdx, vector<pair<double, double>>& _windows, int  _max_length, int r_speed) {
    // Initialize first window start time with starting tick
    double t1;
    // Check if the configuration collides at start time
    bool isColliding = !ValidateCfgAgainstDynamicAgent(_cfg, startTick, agentIdx);
    // If starting with collision, invalidate window start time
    if(isColliding) {
       t1 = -1;  // Invalid t1 to indicate we're in collision so no valid window start time
    }
    else t1 = startTick;
    // Initialize tick counter for checking future times
    double currentTick;
    // Check each future time step up to maximum planning time
    for(size_t cfgIdx = 1; cfgIdx < _max_length; cfgIdx++) {
       // Calculate actual tick by adding offset to start time
       currentTick = startTick + cfgIdx;
       // Check if configuration collides at current time
       bool currentCollision = !ValidateCfgAgainstDynamicAgent(_cfg, currentTick, agentIdx, r_speed);
       // If collision state changed (from colliding to free or vice versa)
       if(currentCollision != isColliding) {
          if(currentCollision) {
            // transition from collsiion free to colliding so close current window with end time as previous tick
            _windows.push_back({t1, currentTick - 1});
            t1 = -1;  // reset window start time
           } else {
            // transition from colliding to collision free so start new window at current tick
            t1 = currentTick;
           }
           //update collision state for next iteration
           isColliding = currentCollision;
      }
    }
    // if configuration ends with collision free state
    if(!isColliding) {
      //add final window extending to infinity
      _windows.push_back({t1, INT_MAX});
    }
    // do not return vectors it makes a copy takes time
}


template <class MPTraits>
bool
MorseQuery<MPTraits>::
ValidateCfgAgainstDynamicAgent(CfgType& _cfg, double _tick, int dynamicAgentNumber, int r_speed) {
    // Find the agent configurations
    //std::cout<<"Agent paths"<<m_agentPaths.size()<<std::endl;
    size_t speed_adjusted_tick = static_cast<size_t>(floor(_tick));  // Apply speed adjustment
    size_t sz = dynamicAgentPaths[dynamicAgentNumber].size()-1;
    size_t mult = speed_adjusted_tick / sz;
    size_t rem = speed_adjusted_tick % sz;
    size_t agentTick;

    if(mult % 2 == 0)
        agentTick = rem;
    else
        agentTick = sz - rem;

    //position environment for robot
    _cfg.ConfigureRobot();
    auto multiBody = _cfg.GetMultiBody();
    ActiveMultiBody* mb = dynamicAgent->GetMultiBody();
    size_t numBody = multiBody->NumFreeBody();
    size_t numOtherBody = mb->NumFreeBody();
    // For every agent _cfg
    CDInfo cdInfo;
//    CollisionDetectionMethod *m_cdMethod = new Rapid();

    auto agentCfg = dynamicAgentPaths[dynamicAgentNumber][agentTick];
    mb->Configure(agentCfg); // configure the agent
    for(size_t j = 0; j < numBody; ++j) {  // for each body of robot
        for(size_t k = 0; k < numOtherBody; ++k) {   // for each body of the agent
            bool collisionFound =
              m_cdMethod->IsInCollision(multiBody->GetFreeBody(j),
                  mb->GetFreeBody(k), cdInfo);

            if(collisionFound) {  // early quit on collision
                //cout<< "Collision found for robot and dynamic agent " << dynamicAgentNumber << " at " << _cfg.PrettyPrint(5) << ". Agent Cfg: " << agentCfg.PrettyPrint(5) << endl;
                return false;
            }

        }
    }
    return true;
}

template <class MPTraits>
vector<pair<int, int>>
MorseQuery<MPTraits>::
findNonCollisionWindows(vector<CfgType>& _segment, int startTick, int agentIdx) {
    vector<pair<int, int>> windows;
    int t1 = startTick;
    for(auto it = _segment.begin(); it < _segment.end(); ++it) {
      bool isColliding = !ValidateCfgAgainstDynamicAgent(*it, startTick, agentIdx);
      // If starting with collision, don't set initial t1
      if(isColliding) {
          t1 = -1;  // Invalid t1 to indicate we're in collision
      }

      int currentTick = 0;
      for(size_t cfgIdx = 1; cfgIdx < m_maxPlanningTime; cfgIdx++) {
        currentTick = startTick + cfgIdx;
        bool currentCollision = !ValidateCfgAgainstDynamicAgent(*it, currentTick, agentIdx);

        if(currentCollision != isColliding) {  // State changed
            if(currentCollision) {
                // Was valid, now collision - close non-collision window
                windows.push_back({t1, currentTick - 1});
                t1 = -1;  // Reset t1
            } else {
                // Was collision, now valid - start new window
                t1 = currentTick;
            }
            isColliding = currentCollision;
        }
      }
      // If we end in a non-collision state, close the final window
      if(!isColliding) {
          windows.push_back({t1, INT_MAX});
      }
    }

    cout << "non-collision Windows found: " << windows.size() << " for startTick: " << startTick << " and agentIdx: " << agentIdx << ". windows are:" <<endl;
    for (const auto& pair : windows) {
        cout << "(" << pair.first << ", " << pair.second << ") ";
    }
    cout << "end windows" << endl;

    return windows;
}

template <class MPTraits>
bool
MorseQuery<MPTraits>::
ValidCfg(CfgType& _cfg, double _tick, int r_speed){
  // Find the agent configurations
//  std::cout<<"Agent paths"<<std::endl;
  vector<size_t> agentTicks;
  for(size_t i = 0; i < dynamicAgentPaths.size(); i++){
    size_t speed_adjusted_tick = static_cast<size_t>(floor(_tick));//instead of dividing it with r_speed  we can try with (r_speed*constant factor which could be the max_radius / postioanl resolution)
    size_t sz = dynamicAgentPaths[i].size()-1;
    size_t mult =  speed_adjusted_tick / sz;
    size_t rem =  speed_adjusted_tick % sz;
    if(mult % 2 == 0) agentTicks.push_back(rem);
    else agentTicks.push_back(sz - rem);
//    std::cout<<"Tick "<<_tick<<" agent index "<<agentTicks.back()<<std::endl;
  }
  //position environment for robot
  _cfg.ConfigureRobot();
  auto multiBody = _cfg.GetMultiBody();
  ActiveMultiBody* mb = dynamicAgent->GetMultiBody();
  size_t numBody = multiBody->NumFreeBody();
  size_t numOtherBody = mb->NumFreeBody();
  // For every agent _cfg
  CDInfo cdInfo;
//  CollisionDetectionMethod *m_cdMethod = new Rapid();
//  if (r_speed == 1) cout<< "When r_speed is "<< r_speed << ", for agent: " << 0 << ", when main tick is " << _tick << ", agentTick is " << agentTicks[0] << endl;
  for(size_t i = 0; i < agentTicks.size(); i++){  // for each agent
    if(dynamicAgentPaths[i].size() <= agentTicks[i]){
      cout << "agentTicks[" << i << "] out of bounds" << endl;
    }
    auto agentCfg = dynamicAgentPaths[i][agentTicks[i]];
    mb->Configure(agentCfg); // configure the agent
    for(size_t j = 0; j < numBody; ++j) {  // for each body of robot
    for(size_t k = 0; k < numOtherBody; ++k) {   // for each body of the agent
      bool collisionFound =
        m_cdMethod->IsInCollision(multiBody->GetFreeBody(j),
            mb->GetFreeBody(k), cdInfo);

      if(collisionFound) {  // early quit on collision
        //cout<< "Collision found for robot and dynamic agent " << i << " at " << _cfg.PrettyPrint(5) << ". Agent Cfg: " << agentCfg.PrettyPrint(5) << endl;
        return false;
      }

    }
    }
  }
  return true;
}

template <typename MPTraits>
double
MorseQuery<MPTraits>::
GetDeflectionLength(Path* _sp, Path *origPath){
  auto origPathCfgs = origPath->Cfgs();
  auto deflectPathCfgs = _sp->Cfgs();

  double maxDist = 0.0;
  double sumDist =0.0;
  // Every cfg in deflected path
  for(auto dc: deflectPathCfgs){
    double minDist = std::numeric_limits<double>::max();
    for(auto c: origPathCfgs)
      minDist = std::min(minDist, (c-dc).Magnitude());
    maxDist = std::max(maxDist, minDist);
    sumDist += minDist;
  }
  return sumDist;
}

template <typename MPTraits>
double
MorseQuery<MPTraits>::
GetSegmentLength(const vector<VID>& _vids){
  double length = 0.0;
  for(auto start = _vids.begin(); start + 1 < _vids.end(); ++start) {
      if(*start == *(start + 1))
        continue;  // Skip repeated vertices.
      typename GraphType::edge_descriptor ed(*start, *(start + 1));
      typename GraphType::vertex_iterator vi;
      typename GraphType::adj_edge_iterator ei;
      if(this->GetRoadmap()->GetGraph()->find_edge(ed, vi, ei))
        length += (*ei).property().GetWeight();
      else
        throw RunTimeException(WHERE, "Tried to compute length for a path "
            "containing the edge (" + to_string(*start) + "," +
            to_string(*(start + 1)) + "), but that edge was not found in the "
            "graph.");
   }
   return length;
}


template <typename MPTraits>
void
MorseQuery<MPTraits>::
ReadMapFile(const std::string& _filename){

    if(!dynamicAgentPaths.empty()) return;
    RoadmapType* agentRdmp = new RoadmapType(dynamicAgent);
    WeightType::inputRobot = dynamicAgent;

    ifstream ifs(_filename.c_str());
    if(!ifs)
        throw ParseException(WHERE, "Cannot open file " + _filename + ".");

    std::string tag;
    bool headerParsed = false;
    // Read the file until we find the GRAPHSTART tag.
    while(!headerParsed) {
        if(!(ifs >> tag))
            throw ParseException(WHERE, "Error reading map file '" + _filename + "' - "
                "GRAPHSTART tag is missing.");

        // If we find the GRAPHSTART tag, we are done.
        if(tag.find("GRAPHSTART") != string::npos)
            headerParsed = true;
    }
    size_t nVerts, nEdges, maxVID;
    ifs >> nVerts >> nEdges >> maxVID;

    double skip;
    for(size_t i = 0; i < nVerts; i++){
        VID vid1, vid2;
        CfgType data(dynamicAgent);
        ifs >> vid1;
        ifs >> skip;  // skip for vizmo
        ifs >> data;

        agentRdmp->GetGraph()->AddVertex(vid1, data);  // add the vertex
        size_t adjEdges;
        ifs >> adjEdges;
        for (size_t j = 0; j < adjEdges; j++){
            WeightType weight;
            ifs >> vid2 >> weight;
            agentRdmp->GetGraph()->AddEdge(vid1, vid2, weight);
        }
        //break;
    }
//    cout<<"agent roadmap"<<agentRdmp->GetGraph()->get_num_vertices()<<endl;
    InitializeAgentPaths(agentRdmp);
//    m_maxPlanningTime = std::max(m_maxPlanningTime, 2*dynamicAgentPaths.size());
    WeightType::inputRobot = nullptr;
    delete agentRdmp;
}

template <typename MPTraits>
void
MorseQuery<MPTraits>::
InitializeAgentPaths(RoadmapType* _rdmp){
  auto g = _rdmp->GetGraph();
  vector<VID> vids; // for random selection
  /*auto qrypoints = PRMQuery<MPTraits>::GetQuery();
  vector<Vector3d> qryPts;
  for(auto q: qrypoints)
    qryPts.push_back(q.GetPoint());
  double radiusThreshold = 1.0;*/
  for(auto v= g->begin(); v!= g->end(); v++){
    //for(auto q: qryPts)
      //auto q = qryPts[0];
     // if((q - v->property().GetPoint()).norm() >= radiusThreshold){
       // auto p = v->property().GetPoint();
       // if((p[0] < 0 /*|| p[0] > 35*/) && p[1] < 10)// bounding box clipping //state in which regions i want to
          vids.push_back(v->descriptor());
      //}
  }
  //std::shuffle(vids.begin(), vids.end(), std::default_random_engine(0));
  // find ccs
  vector<pair<size_t, VID>> ccs;
  stapl::sequential::vector_property_map<GraphType, size_t> cmap;
  get_cc_stats(*g, cmap, ccs);
  cout << "Finding path for random start and goal" << endl;
  for(int i= 0; i < m_numAgents; i++){
    bool connected = false;
    size_t attempt = 0;
    VID start, goal;
    do{
    start = vids[LRand() % vids.size()];
    goal = vids[LRand() % vids.size()];
    stapl::sequential::vector_property_map<GraphType, size_t> ccmap;
    connected = is_same_cc(*g, ccmap, start, goal);
    attempt++;
    }while(!connected && attempt < 10);
    // Find path
    if(!connected) break;
    cout<<"Found a path ";
    vector<VID> path;
    switch(m_searchAlg) {
    case DIJKSTRAS:
      find_path_dijkstra(*g, start, goal, path, MPTraits::WeightType::MaxWeight());
      break;
    case ASTAR:
      Heuristic<MPTraits> heuristic(g->GetVertex(goal),
          this->GetEnvironment()->GetPositionRes(),
          this->GetEnvironment()->GetOrientationRes());
      astar(*g, start, goal, path, heuristic);
      break;
    }
    Path* newPath = new Path(_rdmp);
    *newPath += path;
    //dynamicAgentPaths.push_back(newPath->FullCfgs(this->GetMPLibrary()));
    std::cout << "number of paths added: " << dynamicAgentPaths.size() << std::endl;
//      m_maxPlanningTime = std::max(m_maxPlanningTime, 2*dynamicAgentPaths.size());

  //}
    // Length-wise curbing - length in workspace
    double lengthLimit = 50;
    auto pathCfgs = newPath->FullCfgs(this->GetMPLibrary());
    if(pathCfgs.empty()) continue;
    size_t len = pathCfgs.size();
	int max_r_speed=10;
    std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<int> behaviorDist(0, 1); // 0 or 1
    int toggle = behaviorDist(gen);
    if(toggle == 0) {//wait/slowdown
      vector<CfgType> out;
      out.push_back(pathCfgs[0]);
      std::random_device rd;
      std::mt19937 gen(rd());
      std::uniform_int_distribution<size_t> distr(1, max_r_speed); // replace 10 with max_r_speed
	  int randomWait = distr(gen);
      double sum = 0;
      for(size_t k = 1; k < len && sum < lengthLimit; k++){
		for(int i = 0; i < randomWait; i++) {
          out.push_back(pathCfgs[k]);
        }
        sum += (out.back().GetPoint() - pathCfgs[k].GetPoint()).norm();
      }
      dynamicAgentPaths.push_back(out);
    }
    else{//speed
      vector<CfgType> curbedPath;
      curbedPath.push_back(pathCfgs[0]);
      double sum = 0;
      std::random_device rd;
      std::mt19937 gen(rd());
      std::uniform_int_distribution<size_t> distr(1, max_r_speed); // replace 10 with max_r_speed
	  int randomSpeed = distr(gen);
      //int accSpeed = 1;
      for(size_t k = 1; k < len && sum < lengthLimit; k+=randomSpeed){
        sum += (curbedPath.back().GetPoint() - pathCfgs[k].GetPoint()).norm();
        curbedPath.push_back(pathCfgs[k]);
      }
      dynamicAgentPaths.push_back(curbedPath);
    }
  }
  std::string outfileName = this->GetBaseFilename() + "agent" +std::to_string(dynamicAgentPaths.size()) + ".paths";
  std::ofstream ofs(outfileName);
  if(!ofs)
    throw RunTimeException(WHERE, "Cannot open file \"" + outfileName + "\"");

  // Print header.
  ofs << dynamicAgentPaths.size() << std::endl;
  // Print path.
  for(auto path : dynamicAgentPaths){
    ofs<< path.size() << std::endl;
    for(auto cit = path.begin(); cit != path.end(); ++cit)
    ofs << *cit << std::endl;
  }

  for (const auto& path : dynamicAgentPaths) {
    m_maxPlanningTime = std::max(m_maxPlanningTime, static_cast<double>(2*path.size()));
  }


  std::cout<<"Number of agent paths:"<<dynamicAgentPaths.size()<<std::endl;
}


//template <typename MPTraits>
//void
//MorseQuery<MPTraits>::
//InitializeAgentPaths(RoadmapType* _rdmp){
//  auto g = _rdmp->GetGraph();
//  vector<VID> vids; // for random selection
//  /*auto qrypoints = PRMQuery<MPTraits>::GetQuery();
//  vector<Vector3d> qryPts;
//  for(auto q: qrypoints)
//    qryPts.push_back(q.GetPoint());
//  double radiusThreshold = 1.0;*/
//  for(auto v= g->begin(); v!= g->end(); v++){
//    //for(auto q: qryPts)
//      //auto q = qryPts[0];
//     // if((q - v->property().GetPoint()).norm() >= radiusThreshold){
//       // auto p = v->property().GetPoint();
//       // if((p[0] < 0 /*|| p[0] > 35*/) && p[1] < 10)// bounding box clipping //state in which regions i want to
//          vids.push_back(v->descriptor());
//      //}
//  }
//  //std::shuffle(vids.begin(), vids.end(), std::default_random_engine(0));
//  // find ccs
//  vector<pair<size_t, VID>> ccs;
//  stapl::sequential::vector_property_map<GraphType, size_t> cmap;
//  get_cc_stats(*g, cmap, ccs);
//  cout << "Finding path for random start and goal" << endl;
//  for(int i= 0; i < m_numAgents; i++){
//    bool connected = false;
//    size_t attempt = 0;
//    VID start, goal;
//    do{
//    start = vids[LRand() % vids.size()];
//    goal = vids[LRand() % vids.size()];
//    stapl::sequential::vector_property_map<GraphType, size_t> ccmap;
//    connected = is_same_cc(*g, ccmap, start, goal);
//    attempt++;
//    }while(!connected && attempt < 10);
//    // Find path
//    if(!connected) break;
//    cout<<"Found a path ";
//    vector<VID> path;
//    switch(m_searchAlg) {
//    case DIJKSTRAS:
//      find_path_dijkstra(*g, start, goal, path, MPTraits::WeightType::MaxWeight());
//      break;
//    case ASTAR:
//      Heuristic<MPTraits> heuristic(g->GetVertex(goal),
//          this->GetEnvironment()->GetPositionRes(),
//          this->GetEnvironment()->GetOrientationRes());
//      astar(*g, start, goal, path, heuristic);
//      break;
//    }
//    Path* newPath = new Path(_rdmp);
//    *newPath += path;
//    //dynamicAgentPaths.push_back(newPath->FullCfgs(this->GetMPLibrary()));
//    std::cout << "number of paths added: " << dynamicAgentPaths.size() << std::endl;
////      m_maxPlanningTime = std::max(m_maxPlanningTime, 2*dynamicAgentPaths.size());
//
//  //}
//    // Length-wise curbing - length in workspace
//    double lengthLimit = 50;
//    auto pathCfgs = newPath->FullCfgs(this->GetMPLibrary());
//    if(pathCfgs.empty()) continue;
//    size_t len = pathCfgs.size();
//    vector<CfgType> curbedPath;
//    curbedPath.push_back(pathCfgs[0]);
//    double sum = 0;
//    for(size_t k = 1; k < len && sum < lengthLimit; k++){
//      sum += (curbedPath.back().GetPoint() - pathCfgs[k].GetPoint()).norm();
// //     for(int l = 0; l < m_speed; l++)
//        curbedPath.push_back(pathCfgs[k]);
//    }
//    dynamicAgentPaths.push_back(curbedPath);
//  }
//  std::string outfileName = this->GetBaseFilename() + "agent" +std::to_string(dynamicAgentPaths.size()) + ".paths";
//  std::ofstream ofs(outfileName);
//  if(!ofs)
//    throw RunTimeException(WHERE, "Cannot open file \"" + outfileName + "\"");
//
//  // Print header.
//  ofs << dynamicAgentPaths.size() << std::endl;
//  // Print path.
//  for(auto path : dynamicAgentPaths){
//    ofs<< path.size() << std::endl;
//    for(auto cit = path.begin(); cit != path.end(); ++cit)
//    ofs << *cit << std::endl;
//  }
//
//  for (const auto& path : dynamicAgentPaths) {
//    m_maxPlanningTime = std::max(m_maxPlanningTime, 2.0*path.size());
//  }
//
//
//  std::cout<<"Number of agent paths:"<<dynamicAgentPaths.size()<<std::endl;
//}


#endif