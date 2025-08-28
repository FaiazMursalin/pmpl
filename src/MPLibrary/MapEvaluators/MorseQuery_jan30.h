#ifndef MORSE_QUERY_H_
#define MORSE_QUERY_H_

#include <map>
#include <vector>
#include <numeric>
#include <unordered_set>
#include <queue>
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
        if(PRMQuery<MPTraits>::GetQuery().empty())
            PRMQuery<MPTraits>::Initialize();

        if (!m_inputDynamicAgentPathFile.empty()) {
            cout << "Reading dynamic agent path" << endl;
            ReadDynamicAgentPath();
        } else {
            cout << "Dynamic agent path not found" << endl;
        }

        // if(this->GetRoadmap()->GetGraph()->get_num_vertices() != 0 && this->GetBlockRoadmap()->GetGraph()->get_num_vertices() != 0)
        InitializeCritical();
    }

protected:
    size_t m_maxPlanningTime = 0;

    enum GraphSearchAlg {DIJKSTRAS, ASTAR}; ///< The supported sssp algorithms.
    GraphSearchAlg m_searchAlg{DIJKSTRAS};  ///< The sssp algorithm to use.

    enum class SocialNavigationStrategy {
        WAIT,
        DEFLECT,
        DIVERSE
    };

    // new stuff
    unordered_map<VID, vector<VID>> m_feasibleCritical; // map of critical point to fesible critical points
    unordered_map<VID, vector<VID>> m_clusters;  // map of critical point to region vid
    // vector<Path*> m_paths;   // Diverse path set

    bool GetValidPath(vector<Path *> m_paths);
    std::pair<VID, VID> GetNextPathRegion(const VID _start, Path* _p, vector<VID>& _segment);
    bool GenerateValidSubPath(vector<VID>& _segment, std::pair<VID, VID> _cpoints, int& _startTick, Path* _newPath);
    bool DynamicValidate(vector<VID>& _segment, int& _startTick, vector<pair<VID, VID>>& _edgeToDelete);

    bool ValidCfg(CfgType& _cfg, int _tick);
    double GetDeflectionLength(Path* _sp, Path *origPath);
    double GetSegmentLength(const vector<VID>& _vids);

    vector<vector<pair<int, int>>> GetSafeWindows(vector<CfgType>& _segment, int _startTick);
    vector<pair<int, int>> findNonCollisionWindows(CfgType& edgeCfgs, int startTick, int agentIdx);
    bool ValidateCfgAgainstDynamicAgent(CfgType& _cfg, int _tick, int dynamicAgentNumber);

    bool InitializeCritical();

    // end new stuff

    vector<Path *> GetPaths(VID start, VID goal, size_t k, unordered_map<VID, VID>& oldNew);
    // bool CheckCollision(CfgType robotCfg, CfgType dynamicAgentCfg);
    void ReadDynamicAgentPath();
    // vector<CfgType> GetCfgs(const vector<VID> &_segment);
    // bool ValidPath(vector<typename MPTraits::CfgType> path_segment, CfgType &collisionCfg, int previousSegmentSize);
    // vector<pair<double, double>> findSafeIntervals(const vector<pair<double, double>>& collisionIntervals,
    //                                               double startTime, double endTime);
    // double CalculateMinimumWaitTime(const CfgType &cfg);
    // double CalculateWaitTimeForSafeMotion(const CfgType &start, const CfgType &end, double startTime);

    void PrettyPrintSocialNavigationStrategy(SocialNavigationStrategy strategy);
    // SocialNavigationStrategy SocialNavigation(double congestionValue, CfgType delcetionPoint, CfgType goal, Path *newPath);
    SocialNavigationStrategy SocialNavigation(double congestionValue, double deflectionCost, double waitCost, double diverseCost);
    double waitCost(double congestionvalue, double waitTime);
    double deflectionCost(double congestionValue, CfgType deflectingPoint, CfgType goal);
    double diversePathCost(double congestionValue, Path *newPath);
    int GetSegmentTickLength(vector<VID>& _segment);
    int calculateWaitTime(vector<VID>& _segment, int startTick, vector<int>& _waitTimes);
    // CfgType findDeflectToPoint(CfgType from, int clusterNumber);
    int calculateWaitTimeCfg(vector<CfgType>& robotcfg, int startTick);
    void calculateIntermediates(VID start, VID goal, vector<Cfg>& _edgeCfg);

    vector<string> m_ncLabels{"kClosest"};
    vector<vector<CfgType>> dynamicAgentPaths;
    string m_inputDynamicAgentPathFile;
    Robot *dynamicAgent;
    SafeIntervalHandler *m_safeIntervalHandler;
    string m_dynamicAgent;
    bool m_deleteNodes{false};
};
template <typename MPTraits>
MorseQuery<MPTraits>::MorseQuery() : PRMQuery<MPTraits>() {
    this->SetName("MorseQuery");
    m_inputDynamicAgentPathFile = "";
    m_dynamicAgent = "";
    dynamicAgent = nullptr;
    m_safeIntervalHandler = new SafeIntervalHandler();
    m_maxPlanningTime = 0;
}

template <typename MPTraits>
MorseQuery<MPTraits>::MorseQuery(XMLNode &_node) : PRMQuery<MPTraits>(_node) {
    this->SetName("MorseQuery");
    m_inputDynamicAgentPathFile = _node.Read("inputMap", false, "",
                                           "filename of dynamic agent path");

    m_dynamicAgent = _node.Read("agent", true, "", "agent robot");

    m_deleteNodes = _node.Read("deleteNodes", false, m_deleteNodes,
                              "Whether or not to delete start and goal from roadmap");

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
    m_safeIntervalHandler = new SafeIntervalHandler();
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
    m_inputDynamicAgentPathFile = MPProblem::GetPath(m_inputDynamicAgentPathFile);

    cout << "Reading dynamic agent path file \'" << m_inputDynamicAgentPathFile << endl;
    ifstream in(m_inputDynamicAgentPathFile);
    cout << "File stream ready for dynamic agent path" << endl;

    if (!in.good())
        throw ParseException(WHERE, "Can't open dynamic agent path file '" + m_inputDynamicAgentPathFile + "'.");

    if (!dynamicAgentPaths.empty())
        dynamicAgentPaths.clear();

    cout << "Reading dynamic agent from: " << m_dynamicAgent << endl;
    const string inputDynamicAgentFile(m_dynamicAgent);
    CountingStreamBuffer cbs(inputDynamicAgentFile);
    std::istream ifs(&cbs);
    ActiveMultiBody *const mb = new ActiveMultiBody;
    mb->Read(ifs, cbs);

    dynamicAgent = new Robot(this->GetMPProblem(), mb, "dynamic agent");
    cout << "Gets dynamic agent" << endl;
    cout << dynamicAgent << endl;

    cout << "DOF " << dynamicAgent->GetMultiBody()->DOF() << endl;
    cout << "PosDOF " << dynamicAgent->GetMultiBody()->PosDOF() << endl;
    cout << "OrientationDOF " << dynamicAgent->GetMultiBody()->OrientationDOF() << endl;
    cout << "JointDOF " << dynamicAgent->GetMultiBody()->JointDOF() << endl;

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
        m_maxPlanningTime = std::max(m_maxPlanningTime, 2*dynamicAgentPath.size());
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
        case SocialNavigationStrategy::WAIT:
            cout << " WAIT ";
            break;
        case SocialNavigationStrategy::DEFLECT:
            cout << " DEFLECT ";
            break;
        case SocialNavigationStrategy::DIVERSE:
            cout << " DIVERSE ";
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
        {(congestionVal * diverseCost), SocialNavigationStrategy::DIVERSE}
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

template <class MPTraits>
vector<typename MPTraits::Path *>
MorseQuery<MPTraits>::GetPaths(VID _start, VID _goal, size_t k, unordered_map<VID, VID>& oldNew) {
    vector<Path *> m_paths;
    GraphType *prevg = new GraphType();
    auto gr = this->GetRoadmap()->GetGraph();

    for (auto vit = gr->begin(); vit != gr->end(); ++vit) {
        auto newVID = prevg->AddVertex(vit->descriptor(), vit->property());
        oldNew.insert({vit->descriptor(), newVID});
    }

    for (auto eit = gr->edges_begin(); eit != gr->edges_end(); eit++) {
        prevg->AddEdge(oldNew[eit->source()], oldNew[eit->target()], eit->property());
    }

    vector<unordered_set<VID>> criticalPoints;
    unordered_set<VID> pCritical;

    while (m_paths.size() < k) {
        this->GetPath()->Clear();
        auto mapPassed = PRMQuery<MPTraits>::PerformSubQuery(gr->GetVertex(_start), gr->GetVertex(_goal));
        cout << "map passed " << mapPassed << endl;

        Path *p = new Path(this->GetRoadmap());
        vector<VID> pVID, fVID;

        for (auto v : this->GetPath()->VIDs()) {
            pVID.push_back(oldNew[v]);
            auto vp = gr->GetVertex(v);
            if (vp.IsLabel("FeasibleCritical") && vp.GetLabel("FeasibleCritical")) {
                fVID.push_back(v);
            }
        }

        *p += pVID;
        cout << "path size found for GetPaths: " << this->GetPath()->Length() << endl;

        for (auto f : fVID) {
            if (gr->GetVertex(f).IsStat("Critical"))
                pCritical.insert(gr->GetVertex(f).GetStat("Critical"));
        }

        bool isSame = false;
        for (auto cp : criticalPoints) {
            isSame = true;
            if (cp.size() != pCritical.size()) {
                isSame = false;
            } else {
                for (auto cap : pCritical)
                    if (cp.find(cap) == cp.end()) {
                        isSame = false;
                        break;
                    }
            }
            if (isSame)
                break;
        }

        for (auto v : this->GetPath()->VIDs())
            if (v != _start && v != _goal)
                gr->DeleteVertex(v);

        if (!isSame) {
            criticalPoints.push_back(pCritical);
            m_paths.push_back(p);
        } else {
            delete p;
        }
    }

    this->GetRoadmap()->SetGraph(prevg);
    cout << "Total number of paths at the end of GetPaths = " << m_paths.size() << endl;
    return m_paths;
}

template <typename MPTraits>
bool MorseQuery<MPTraits>::PerformSubQuery(const CfgType &_start, const CfgType &_goal) {
    cout << "Evaluating sub-query:" << endl
         << "\tfrom " << _start << endl
         << "\tto   " << _goal << endl;

    size_t k = 3;
    auto ccs = this->FindCCs();
    cout << "cc size:" << ccs.size() << endl;
    auto start = this->EnsureCfgInMap(_start);
    auto goal = this->EnsureCfgInMap(_goal);

    // store the previous path ids before it gets cleared
    auto prevPathIDs = this->GetPath()->VIDs();

    bool connected = false;
    for (auto cc : ccs) {
        this->ConnectToCC(start.first, cc.second);
        this->ConnectToCC(goal.first, cc.second);

        if (this->SameCC(start.first, goal.first)) {
            connected = true;
            this->GeneratePath(start.first, goal.first);
            break;
        }
    }

    cout << "connected: " << connected << endl;

    // new stuff

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

        // get k  diverse paths
        vector<Path *> m_paths;
        unordered_map<VID, VID> oldNew;
        string clockName = this->GetNameAndLabel() + "::DiverseQuery";
        this->GetStatClass()->StartClock(clockName);
        m_paths = GetPaths(start.first, goal.first, k, oldNew);
        std::sort(m_paths.begin(), m_paths.end(),
                 [](Path *path1, Path *path2) {
                     return path1->Size() < path2->Size();
                 });
        this->GetStatClass()->StopClock(clockName);

        cout << "Paths size: " << m_paths.size() << endl;

        // Get the best valid path
        clockName = this->GetNameAndLabel() + "::SocialNavigationQuery";
        this->GetStatClass()->StartClock(clockName);
        connected = GetValidPath(m_paths);
        this->GetStatClass()->StopClock(clockName);
        if(connected){
            // restore the previous path
            vector<VID> p;
            for(auto v: prevPathIDs)
                p.push_back(oldNew[v]);
            // add the new sub-path
            auto newVID = this->GetPath()->VIDs();
            p.insert(p.end(), newVID.begin(), newVID.end());
            this->GetPath()->Clear();
            *this->GetPath() += p;
        }

    }
    // end new stuf


    if (this->m_debug) {
        if (connected)
            cout << "\tSuccess: found path from start node " << start.first
                 << " to goal node " << goal.first << "." << endl;
        else
            cout << "\tFailed to connect start node " << start.first
                 << " to goal node " << goal.first << "." << endl;
    }

    if (m_deleteNodes) {
        PRMQuery<MPTraits>::RemoveTempCfg(start);
        PRMQuery<MPTraits>::RemoveTempCfg(goal);
    }

    return connected;
}

//should go through all the cfg in the segment
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

        // Construct a resolution-level path along the recreated edge.
        //vector<CfgType> edgeCfg;
        for(auto cit = inter.begin(); cit + 1 != inter.end(); ++cit) {
            vector<CfgType> edge = lp->ReconstructPath(*cit, *(cit+1),
                vector<CfgType>(), this->GetEnvironment()->GetPositionRes(), this->GetEnvironment()->GetOrientationRes());
            _edgeCfg.insert(_edgeCfg.end(), edge.begin(), edge.end());
        }
        //if(edgeCfg.size() == 0) std::cout<<"Empty intermediates"<<std::endl;
        //else std::cout<<"Intermediates size:"<<edgeCfg.size()<<std::endl;
        _edgeCfg.push_back(e);

}
template<class MPTraits>
int
MorseQuery<MPTraits>::
calculateWaitTime(vector<VID>& _segment, int startTick, vector<int>& _waitTimes) {
  vector<Cfg> edgeCfg;
  auto tick = startTick;
  for(auto it = _segment.begin(); it + 1 < _segment.end(); ++it) {
    edgeCfg.clear();
    edgeCfg.push_back(this->GetRoadmap()->GetGraph()->GetVertex(*it));//startcfg inserted
    calculateIntermediates(*it, *(it+1), edgeCfg);
    int waitTime=calculateWaitTimeCfg(edgeCfg, tick);
    if(waitTime>0) cout<< "calculated wait time in calculateWaitTime, waitTime: " << waitTime << endl;
    if(waitTime==INT_MAX) return INT_MAX;
    tick += waitTime + edgeCfg.size()-1;//
	_waitTimes.push_back(waitTime);
  }
  cout<< "finished calculating waiting time and tick is " <<tick<<endl;
  return tick;
}

template<class MPTraits>
int
MorseQuery<MPTraits>::
calculateWaitTimeCfg(vector<CfgType>& robotcfg, int startTick) {
  auto windows = GetSafeWindows(robotcfg, startTick);
  int tick = robotcfg.size();
  //tick = tick - startTick; // sine the above functon gives us the max tick for the segment, we subtract the start tick to get the tick value for iterating from zerp
  // Find minimum agent tick length to use as boundary condition
  int minAgentTick = INT_MAX;
  for (const auto& path : dynamicAgentPaths) {
    minAgentTick = std::max(minAgentTick, static_cast<int>(path.size()));
  }
  cout<< "minAgentTick:" << minAgentTick << endl;

  int colWaitTime=0;

  for (int x=0; x<tick; x++) { // For each robot tick
    int cwt = x + startTick;
    bool foundFlag = false; int time=0;
    // while the wait time does not exceed the minimum agent path length
    while (cwt - (x + startTick) <= 2*minAgentTick) { cout<<"time now: "<<time++<<endl;//remove this after debugging
        vector<int> newCwt_y;    // Vector to store new wait times for each agent
        vector<int> newColt_y;    // Vector to store next collision times for each agent
    	for(size_t ay = 0; ay < dynamicAgentPaths.size(); ay++){
          // Find the appropriate window index for this agent
          bool windowFound = false;
          for (size_t i = 0; i < windows[ay].size(); i++) {//i will be the every agents windows
              const auto& window = windows[ay][i];

              // Check if current time is in a free window for every agent time
              if (cwt >= window.first && cwt <= window.second) {
              // We're in a free window i for agent ay
                  newCwt_y.push_back(cwt);
                  newColt_y.push_back(window.second);
                  windowFound = true;
                  break;
              }
          }
          if (!windowFound) {
            // If we're not in a free window, look for the next one
            int nextWindowStart = INT_MAX;
            int nextWindowEnd = INT_MAX;
            for (const auto& window : windows[ay]) {//for every windows find the first safe window
              if (window.first > cwt) {
                if (window.first < nextWindowStart) {
                  nextWindowStart = window.first;
                  nextWindowEnd = window.second;
                  break;
                }
              }
            }
            // If no collision is found, use infinite time
            if (nextWindowStart == INT_MAX) {
                newCwt_y.push_back(cwt);
                newColt_y.push_back(INT_MAX);
            } else {
              newCwt_y.push_back(nextWindowStart);
              newColt_y.push_back(nextWindowEnd);
            }
          }
        }
        // Find maximum wait time and minimum collision time across all agents
        int maxCwt_y = *std::max_element(newCwt_y.begin(), newCwt_y.end());
        int minColt_y = *std::min_element(newColt_y.begin(), newColt_y.end());
        if (maxCwt_y < minColt_y) {
          // We found a valid waiting time where all agents are clear
          cwt = maxCwt_y;
          foundFlag = true;
          break;
        }
        // Update current wait time and continue searching
        cwt = maxCwt_y;
    }
    if (!foundFlag) {
        // If no valid waiting time was found, return infinity
//        cwt = INT_MAX;
        return INT_MAX;
    }
    cout<< "cwt :" << cwt << ", x:" << x << ",startTick:" << startTick << endl;
  	int validCwt = (cwt < INT_MAX)? (cwt - x - startTick) : INT_MAX;
  	colWaitTime = std::max(validCwt,colWaitTime);
  	cout<<"colWaitTime = "<<colWaitTime << "validCwt: " << validCwt <<endl;
  }
//  if (colWaitTime == 0) colWaitTime = INT_MAX;
  cout << "returning colWaitTime = " << colWaitTime << endl;
  return colWaitTime;
}


template<class MPTraits>
bool
MorseQuery<MPTraits>::
GetValidPath(vector<typename MPTraits::Path *> m_paths){

    bool oneSuccess = false;
    Path *bestPath = nullptr; size_t bestPathIndex;
    std::map<size_t, std::vector<std::pair<VID, double>>> waitTimes;

    for(size_t i = 0; i < m_paths.size(); ++i){
        auto pathLengthBefore = m_paths[i]->Length(); // store the length before deflection
        auto pathVIDs = m_paths[i]->VIDs();
        auto start = pathVIDs.front();
        vector<VID> segment;
        Path* newPath = new Path(this->GetRoadmap());
        bool result = true;
        int startTick = 0;
        cout << "Start: " << pathVIDs.front() << " Goal: " << pathVIDs.back() << endl;

        while (true) {
            segment.clear(); // std::cout<<"Start: "<< start << ", ";
            cout << "=====" << endl;
            cout << "Current path VIDs: " << endl;
            for (auto v: newPath->VIDs()) {
                cout << v << ", ";
            }
            cout << "=====" << endl;

            vector<pair<VID, VID>> edgeToDelete;
            auto criticals = GetNextPathRegion(start, m_paths[i], segment); // get the critical point set
            std::cout<<"Criticals "<<criticals.first<<" , "<< criticals.second<< " startTick: " << startTick << std::endl;

            cout << "Checking segment: " << endl;
            for (auto v: segment) {
                cout << v << ", ";
            }
            cout << endl;

            auto goal = segment.back(); //std::cout<<"Goal: "<<goal<<std::endl;

            // check if reached the end. If reached end, break
            if(/*segment.size() <= 1 &&*/ !newPath->VIDs().empty() && newPath->VIDs().back() == pathVIDs.back()) // reached the end
                break;

            // validate the segment
            int tempTick = startTick;
            bool validSegment = DynamicValidate(segment, tempTick, edgeToDelete);//no need
            cout << "Valid: " << validSegment << " Edge to delete size: " << edgeToDelete.size() << endl;
            if (edgeToDelete.size() > 0) {
                for (auto p: edgeToDelete) {
                    cout << "Delete: " << p.first << ", " << p.second << endl;
                }
            }
			//do not need the following if (validSegment)
            if(validSegment){ // if valid, add segment to path
                vector<VID> addVID;

                if(!newPath->VIDs().empty()) //if not the first segment, do not include the start point its' a repeat'
                    addVID.insert(addVID.end(), segment.begin()+1, segment.end());
                else
                    addVID.insert(addVID.end(), segment.begin(), segment.end());

                *newPath += addVID; // add to path

                start = goal;//do it after successfully deflected also
                startTick = tempTick; // update tick

            } else { // segment not valid
                cout << "Segment with criticals: " << criticals.first << ", " << criticals.second << " not valid. Starttick: " << startTick << endl;
                // should perform social navigation
                // initialize costs for strategies
                double deflectionCost = MAXDOUBLE, waitCost = MAXDOUBLE, diversePathCost = MAXDOUBLE;
//                int possibleWaitTick = 0,
                int possibleDeflectTick = 0;

                // case wait
				vector<int> pathWaitTimes;
                int wt = calculateWaitTime(segment, startTick, pathWaitTimes);
                waitCost = (wt == INT_MAX)? MAXDOUBLE : (wt - startTick);
//                if (wt != INT_MAX && wt <= m_maxPlanningTime) {
//
//                }



                /*auto windows = GetSafeWindows(segment, startTick);
                for (auto w: windows) {i+1
                  cout << "Windows: " << w.first << ", " << w.second << endl;
                }
                auto t1_containing_pair = std::max_element(windows.begin(), windows.end(), [](const auto& a, const auto& b) {
                    return a.first > b.first;
                });
                auto maxT1 = t1_containing_pair->first;
//                possibleWaitTick = wt;

                auto t2_containing_pair = std::min_element(windows.begin(), windows.end(), [](const auto& a, const auto& b) {
                    return a.second < b.second;
                });
                auto minT2 = t2_containing_pair->second;
                cout << "Max t1: " << maxT1 << " Min t2: " << minT2 << endl;
                if ((maxT1 < minT2) && (m_maxPlanningTime <= maxT1)) {
                  waitCost = maxT1-startTick;
                } // else waitCost is max_double initialized
*/
                // end case wait

                // case deflect:
                cout << "Before deflection, segment: " << endl;
                for (auto vvvid: segment) {
                    cout << vvvid << ", ";
                }
                cout << endl;
                // assume segment was valid and add the vids to get a length
                Path *duplicatePath_Diverse = new Path(this->GetRoadmap());
//                *duplicatePath_Diverse += newPath->VIDs(); // no need to get info of previous segments since those are valid.
                double lengthBeforeDeflection = GetSegmentLength(segment);
                possibleDeflectTick = startTick;
                //result = GenerateValidSubPath(segment, criticals, possibleDeflectTick, duplicatePath_Diverse);  // deflect
                result=false;
                cout << "Tick after generatevalidsubpath: possible: " << possibleDeflectTick << " original:" << startTick<< endl;
                if (result) {//result

                    *duplicatePath_Diverse += vector<VID>{segment.back()};

                    cout << "After deflection, segment: " << endl;
                    for (auto vvvid: duplicatePath_Diverse->VIDs()) {
                        cout << vvvid << ", ";
                    }
                    cout << endl;

                    double segmentLengthAfterDeflection = GetSegmentLength(duplicatePath_Diverse->VIDs()); // length of the deflected path till end of current segment
                    deflectionCost = (possibleDeflectTick - startTick); // 12/07/24 - chnaged cost to tick // length of segment after deflection - length of segment before deflection
                    cout << "Deflection cost: "<< deflectionCost <<
                        " segment lengthAfterDeflection:" << segmentLengthAfterDeflection <<
                        " segment lengthBeforeDeflection:"<< lengthBeforeDeflection << endl;
                    start = segment.back();//break condition needed also if start == goal
                } // if deflection fails, the deflection cost is infinite (chcek initialzation above)
                std::cout<<"Deflection success: "<<result<<std::endl;
                // case deflect end;

                //case diverse start
                if (i +1 < m_paths.size()) {
                    diversePathCost = m_paths[i+1]->Length();
                }
                // case diverse end

                auto strategy = SocialNavigation(1, deflectionCost, waitCost, diversePathCost);
                cout << "Social Navigation Strategy returned: ";
                PrettyPrintSocialNavigationStrategy(strategy);
                cout << endl;

                switch (strategy)
                {
                    case SocialNavigationStrategy::DEFLECT: {
//                        newPath->Clear(); // no need to clear since we are just getting a new deflected segment - verified Nov 29 2024.
                        vector<VID> addVID;
                        // ignore start as it's already there from previous segment
                        addVID.insert(addVID.end(), duplicatePath_Diverse->VIDs().begin()+1, duplicatePath_Diverse->VIDs().end());
                        *newPath += addVID; // add to path
//                        *newPath += duplicatePath_Diverse->VIDs();
                        startTick = possibleDeflectTick;
//                        startTick = newPath->Length();
                        break;
                    }
                    case SocialNavigationStrategy::WAIT: {
                        result = true;
                        // 12/07/2024

                        auto vids = newPath->VIDs();
                        for(size_t index = 0; index < pathWaitTimes.size(); ++index) {
                          if(pathWaitTimes[index] > 0){
                              if(waitTimes.find(i) == waitTimes.end())
                                waitTimes.insert(make_pair(i, std::vector<pair<VID, double >> (1, {vids[index], pathWaitTimes[index]})));
                              else
                        		waitTimes[i].push_back(make_pair(vids[index], pathWaitTimes[index]));
                          } else {
                            //cout << "pathWaitTimes["<<index<<"] is = " << pathWaitTimes[index] << endl;
                          }
                        }
                        std::cout << "Set wait times for " << pathWaitTimes.size() << "," << waitTimes[i].size() << " vertices " << "for path " << i<<endl;
//                        newPath->GetRoadmap()->GetGraph()->GetVertex(vid).SetStat("waittime", waitCost);
						startTick += waitCost;//+= for wait scenario
//                        startTick += (waitCost + GetSegmentTickLength(segment));
                        vector<VID> addVID;

                        if(!newPath->VIDs().empty()) //if not the first segment, do not include the start point its' a repeat'
                            addVID.insert(addVID.end(), segment.begin()+1, segment.end());
                        else
                            addVID.insert(addVID.end(), segment.begin(), segment.end());

                        *newPath += addVID; // add to path
                        break;
                    }
                    case SocialNavigationStrategy::DIVERSE:
                        result = false; // go to next path
                        break;
                    default:
                      result=false;
                      break;
                }

            }

            // result = true; // assumption
            if (result) {
                // social navigation successful for segment
                start = goal;
            } else {
                break;
            }
        }
        std::cout<<"Path "<<i<< " success: "<<result<<std::endl;

        if(result /*&& (i == m_paths.size()-1 || newPath->Length() <= m_paths[i+1]->Length())*/){
            //if(!oneSuccess) oneSuccess = true;
            for (auto vvvid: newPath->VIDs()) {
                cout << vvvid << ", ";
            }
            cout << endl;
            std::cout<<"Current path length "<<newPath->Length() <<" before deflection: "<<pathLengthBefore<<std::endl;
            //if(i < m_paths.size()-1) std::cout<<"Next path length before deform:"<<m_paths[i+1]->Length()<<std::endl;
            if(!oneSuccess){
              	cout<< "setting the best path first time for index"<<i<<endl;
                bestPath = newPath;
                oneSuccess = true; bestPathIndex = i;
            }
            else if(newPath->Length() < bestPath->Length()) {
                bestPath = newPath;
                bestPathIndex = i;
            }
            else  delete newPath;
        //return true; // early quit on success
        }
        else delete newPath;
        cout<<"================================================xxxxxx================================================"<<endl;
    }

    if(oneSuccess){
        std::cout<<"Stored best path length:"<<bestPath->Length()<<std::endl;
        std::cout<<"Deflection length: "<<GetDeflectionLength(bestPath, m_paths[bestPathIndex])<<std::endl;
       	cout << "Best path index: "<< bestPathIndex << ", " << (waitTimes.find(bestPathIndex) != waitTimes.end()) << ", " << waitTimes[bestPathIndex].size()<< endl;
        if (waitTimes.find(bestPathIndex) != waitTimes.end()) {
            for (const auto& pair : waitTimes[bestPathIndex]) {
                std::cout << "(vid: " << pair.first << ", wait: " << pair.second << ") ";
                bestPath->GetRoadmap()->GetGraph()->GetVertex(pair.first).SetStat("waittime", pair.second);
            }
            std::cout << std::endl;
        }else{
          cout<<"couldnt find best path"<<std::endl;
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

//look into this
template <typename MPTraits>
bool
MorseQuery<MPTraits>::
GenerateValidSubPath(vector<VID>& _segment, std::pair<VID, VID> _cpoints, int& _startTick, Path* _newPath){
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
  int startTick = _startTick;
//  cout << "Gets here: GenerateValidSubPath: auto start = ... " << subgraph->get_num_edges() << endl;;
  auto start = (_newPath->VIDs().empty()) ? oldNew[_segment.front()] : oldNew[_newPath->VIDs().back()];
  unordered_map<VID, vector<VID>> segments;
  vector<pair<VID, double>> vidLengths; //std::cout<<"goals trying:";
  size_t i = 0;
  while(subgraph->get_num_edges() > 0){
//    cout << "Gets here: GenerateValidSubPath: inside while " << i << endl;
    i = i+1;
    vidLengths.clear();
    segments.clear();
    vector<pair<VID, VID>> edgeToDelete;
    // Plan sub-path to every feasible critical points
    for(auto gid: feasibleVIDs){ //std::cout<<" "<<gid;
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
    }//std::cout<<std::endl;
    if(vidLengths.empty()) return false;  // No path found for any sub-goal
    // sort the segments based on the length
    std::sort(vidLengths.begin(), vidLengths.end(), [](pair<VID, double> a, pair<VID, double> b)
                                  {
                                      return a.second < b.second;
                                  });
    // Validate each segment one by one and return on first valid
    for(auto vid: vidLengths){
      // validate the segment
      if(DynamicValidate(segments[vid.first], startTick, edgeToDelete)){
        vector<VID> addVID;
        if(!_newPath->VIDs().empty()) //if not the first segment, do not include the start point its' a repeat'
          addVID.insert(addVID.end(), segments[vid.first].begin()+1, segments[vid.first].end());
        else
          addVID.insert(addVID.end(), segments[vid.first].begin(), segments[vid.first].end());
        *_newPath += addVID;
        std::cout<<"Found the local path at iteration:"<<i<<std::endl;
        return true;
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
DynamicValidate(vector<VID>& _segment, int &_startTick, vector<pair<VID, VID>>& _edgeToDelete){
  auto g = this->GetRoadmap()->GetGraph();
  // start configuration validity check
   CfgType startCfg = g->GetVertex(_segment.front());
  // validate the start _cfg
  if(!ValidCfg(startCfg, _startTick)) return false;
  int tick = _startTick - 1; // using pre-increment on ticks on validation call
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
    //if(edgeCfg.size() == 0) std::cout<<"Empty intermediates"<<std::endl;
    //else std::cout<<"Intermediates size:"<<edgeCfg.size()<<std::endl;
    edgeCfg.push_back(e);
    for(auto& c: edgeCfg)
      if(!ValidCfg(c, ++tick)) {
        _edgeToDelete.push_back(make_pair(*it, *(it+1)));
        return false;
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
        //if(edgeCfg.size() == 0) std::cout<<"Empty intermediates"<<std::endl;
        //else std::cout<<"Intermediates size:"<<edgeCfg.size()<<std::endl;
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
GetSafeWindows(vector<CfgType>& _segment, int _startTick) {//segment for robot //start cfg for DA
    //auto g = this->GetRoadmap()->GetGraph();
    // start configuration validity check
    CfgType startCfg = _segment.front();

    vector<vector<pair<int, int>>> safeWindows;

    // validate the start _cfg
    if(!ValidCfg(startCfg, _startTick)) {
      // initialize window for each dynamic agent
      vector<pair<int, int>> windows;
      for(size_t i= 0; i < dynamicAgentPaths.size(); i++) {
          windows.push_back(make_pair(-1,-1)); // tick cannot be negative. So, if negative, it is invalid. Required later.
          safeWindows.push_back(windows);
      }
      return safeWindows;
    }
    int tick = _startTick; // using pre-increment on ticks on validation call

    // Validate each edge
    for(auto it = _segment.begin(); it < _segment.end(); ++it) {

        //the below safe window calculation needs to be done outside
        for(size_t i = 0; i < dynamicAgentPaths.size(); i++) { // for each agent
          //tick = _startTick - 1; // reset tick for each dynamic agent
          vector<pair<int, int>> windows = findNonCollisionWindows(*it, tick, i);
          safeWindows.push_back(windows);
        }////it should be for one edge(cfg) at a time
        tick++;
        std::cout<<"Number of safe windows calculated: "<<safeWindows.size()<<endl;
    }
    return safeWindows;
}


template <class MPTraits>
bool
MorseQuery<MPTraits>::
ValidateCfgAgainstDynamicAgent(CfgType& _cfg, int _tick, int dynamicAgentNumber) {
    // Find the agent configurations
    //std::cout<<"Agent paths"<<m_agentPaths.size()<<std::endl;
    size_t agentTick;
    size_t sz = dynamicAgentPaths[dynamicAgentNumber].size()-1;
    size_t mult =  _tick / sz;
    size_t rem =  _tick % sz;
    if(mult % 2 == 0) agentTick = rem;
    else agentTick = sz - rem;

    //position environment for robot
    _cfg.ConfigureRobot();
    auto multiBody = _cfg.GetMultiBody();
    ActiveMultiBody* mb = dynamicAgent->GetMultiBody();
    size_t numBody = multiBody->NumFreeBody();
    size_t numOtherBody = mb->NumFreeBody();
    // For every agent _cfg
    CDInfo cdInfo;
    CollisionDetectionMethod *m_cdMethod = new Rapid();

    auto agentCfg = dynamicAgentPaths[dynamicAgentNumber][agentTick];
    mb->Configure(agentCfg); // configure the agent
    for(size_t j = 0; j < numBody; ++j) {  // for each body of robot
        for(size_t k = 0; k < numOtherBody; ++k) {   // for each body of the agent
            bool collisionFound =
              m_cdMethod->IsInCollision(multiBody->GetFreeBody(j),
                  mb->GetFreeBody(k), cdInfo);

            if(collisionFound) {  // early quit on collision
                cout<< "Collision found for robot and dynamic agent " << dynamicAgentNumber << " at " << _cfg.PrettyPrint(5) << ". Agent Cfg: " << agentCfg.PrettyPrint(5) << endl;
                return false;
            }

        }
    }
    return true;
}

template <class MPTraits>
vector<pair<int, int>>
MorseQuery<MPTraits>::
findNonCollisionWindows(CfgType& edgeCfgs, int startTick, int agentIdx) {
    vector<pair<int, int>> windows;

    int t1 = startTick;
    bool isColliding = !ValidateCfgAgainstDynamicAgent(edgeCfgs, startTick, agentIdx);

    // If starting with collision, don't set initial t1
    if(isColliding) {
        t1 = -1;  // Invalid t1 to indicate we're in collision
    }

    // Scan through all configurationsi
    for(size_t cfgIdx = 1; cfgIdx < m_maxPlanningTime; cfgIdx++) {
        int currentTick = startTick + cfgIdx;
        bool currentCollision = !ValidateCfgAgainstDynamicAgent(edgeCfgs, currentTick, agentIdx);

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
        windows.push_back({t1, std::numeric_limits<int>::max()});
    }

    return windows;
}

template <class MPTraits>
bool
MorseQuery<MPTraits>::
ValidCfg(CfgType& _cfg, int _tick){
  // Find the agent configurations
  //std::cout<<"Agent paths"<<m_agentPaths.size()<<std::endl;
  vector<size_t> agentTicks;
  for(size_t i = 0; i < dynamicAgentPaths.size(); i++){
    size_t sz = dynamicAgentPaths[i].size()-1;
    size_t mult =  _tick / sz;
    size_t rem =  _tick % sz;
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
  CollisionDetectionMethod *m_cdMethod = new Rapid();
  for(size_t i = 0; i < agentTicks.size(); i++){  // for each agent
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


#endif  