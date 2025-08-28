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
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPTraits::RoadmapType RoadmapType;
    typedef typename RoadmapType::GraphType GraphType;
    typedef typename RoadmapType::VID VID;
    typedef typename MPTraits::Path Path;
    typedef typename MPTraits::SafeIntervalHandlerType SafeIntervalHandler;
    typedef typename SafeIntervalHandler::DynamicAgentState DynamicAgentState;

    MorseQuery();
    MorseQuery(XMLNode &_node);
    virtual ~MorseQuery() = default;

    virtual void Print(ostream &_os) const override;
    virtual bool PerformSubQuery(const CfgType &_start, const CfgType &_goal) override;

protected:
    double m_maxPlanningTime = 100.0;

    enum class SocialNavigationStrategy {
        WAIT,
        DEFLECT,
        DIVERSE
    };

    vector<Path *> GetPaths(VID start, VID goal, size_t k);
    bool CheckCollision(CfgType robotCfg, CfgType dynamicAgentCfg);
    void ReadDynamicAgentPath();
    vector<CfgType> GetCfgs(const vector<VID> &_segment);
    bool ValidPath(vector<typename MPTraits::CfgType> path_segment, CfgType &collisionCfg, int previousSegmentSize);
    vector<pair<double, double>> findSafeIntervals(const vector<pair<double, double>>& collisionIntervals, 
                                                  double startTime, double endTime);
    double CalculateMinimumWaitTime(const CfgType &cfg);
    double CalculateWaitTimeForSafeMotion(const CfgType &start, const CfgType &end, double startTime);

    void PrettyPrintSocialNavigationStrategy(SocialNavigationStrategy strategy);
    SocialNavigationStrategy SocialNavigation(double congestionValue, CfgType delcetionPoint, CfgType goal, Path *newPath);
    double waitCost(double congestionvalue, double waitTime);
    double deflectionCost(double congestionValue, CfgType deflectingPoint, CfgType goal);
    double diversePathCost(double congestionValue, Path *newPath);
    CfgType findDeflectToPoint(CfgType from, int clusterNumber);

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
    m_maxPlanningTime = 100.0;
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
            in >> tempCfg;
            dynamicAgentPath.push_back(tempCfg);
        }
        
        cout << "Read " << dynamicAgentPath.size() << " for path " << i << endl;
        dynamicAgentPaths.push_back(dynamicAgentPath);
    }
    cout << "Read " << dynamicAgentPaths.size() << " paths for dynamic agent path" << endl;

    in.close();
}
template <typename MPTraits>
bool MorseQuery<MPTraits>::ValidPath(vector<typename MPTraits::CfgType> path_segment, CfgType &collisionCfg, int previousSegmentSize) {
    if (path_segment.empty()) return true;
    
    for (size_t robotIdx = 0; robotIdx < path_segment.size(); robotIdx++) {
        double currentTick = previousSegmentSize + robotIdx;
        const auto &robotCfg = path_segment[robotIdx];
        
        // Track earliest safe tick per agent
        vector<double> earliestSafeTickPerAgent;
        bool hasCollision = false;
        
        // Check collision with each dynamic agent
        for (const auto &agentPath : dynamicAgentPaths) {
            double totalTicks = agentPath.size() - 1;
            bool foundSafeInterval = false;
            double earliestSafeTick = currentTick;
            
            // Check forward in time
            for (double t = currentTick; t < currentTick + m_maxPlanningTime; t += 1.0) {
                bool isColliding = false;
                
                int direction = static_cast<int>(floor(t / totalTicks)) % 2;
                size_t agentIndex;
                
                if (direction == 0) {  // Forward
                    agentIndex = static_cast<size_t>(fmod(t, totalTicks));
                } else {  // Reverse
                    agentIndex = static_cast<size_t>(totalTicks - 1 - fmod(t, totalTicks));
                }
                
                if (agentIndex >= agentPath.size()) continue;
                
                // Check collision at this tick
                if (CheckCollision(robotCfg, agentPath[agentIndex])) {
                    isColliding = true;
                    hasCollision = true;
                    earliestSafeTick = t + 1; // Need to wait at least until after this collision
                } else if (isColliding) {
                    // We found the end of a collision interval
                    foundSafeInterval = true;
                    break;
                }
            }
            
            if (hasCollision) {
                earliestSafeTickPerAgent.push_back(earliestSafeTick);
            }
        }
        
        // If we found any collisions, calculate wait time
        if (hasCollision) {
            // Find the maximum wait time needed across all agents
            double maxEarliestSafeTick = *std::max_element(
                earliestSafeTickPerAgent.begin(), 
                earliestSafeTickPerAgent.end()
            );
            
            int waitDuration = static_cast<int>(maxEarliestSafeTick - currentTick);
            
            if (waitDuration > m_maxPlanningTime) {
                cout << "Required wait time " << waitDuration << " ticks exceeds maximum planning time" << endl;
                return false;
            }
            
            // Set collision configuration
            collisionCfg = robotCfg;
            collisionCfg.SetLabel("Collision", true);
            collisionCfg.SetStat("WaitDuration", waitDuration);
            collisionCfg.SetStat("Tick", robotIdx + previousSegmentSize);
            collisionCfg.SetStat("WaitAtVID", this->GetRoadmap()->GetGraph()->GetVID(collisionCfg));
            
            cout << "Collision found. Wait duration: " << waitDuration 
                 << " ticks at position index: " << (robotIdx + previousSegmentSize)
                 << " current tick: " << currentTick 
                 << " earliest safe tick: " << maxEarliestSafeTick << endl;
                 
            return false;
        }
    }
    
    return true;
}

template <typename MPTraits>
vector<pair<double, double>> MorseQuery<MPTraits>::findSafeIntervals(
    const vector<pair<double, double>>& collisionIntervals, 
    double startTime, 
    double endTime) 
{
    vector<pair<double, double>> safeIntervals;
    
    if (collisionIntervals.empty()) {
        safeIntervals.push_back({startTime, endTime});
        return safeIntervals;
    }
    
    double currentTime = startTime;
    
    for (const auto& collision : collisionIntervals) {
        if (currentTime < collision.first) {
            safeIntervals.push_back({currentTime, collision.first - 1});
        }
        currentTime = collision.second + 1;
    }
    
    if (currentTime < endTime) {
        safeIntervals.push_back({currentTime, endTime});
    }
    
    return safeIntervals;
}

template <typename MPTraits>
double MorseQuery<MPTraits>::CalculateMinimumWaitTime(const CfgType &cfg) {
    double currentTime = 0;
    while (currentTime < m_maxPlanningTime) {
        auto safeIntervals = m_safeIntervalHandler->findSafeIntervals(
            dynamicAgentPaths,
            cfg,
            m_maxPlanningTime);

        for (const auto &interval : safeIntervals) {
            if (interval.startTime >= currentTime) {
                return interval.startTime - currentTime;
            }
        }

        currentTime += m_safeIntervalHandler->GetTimeResolution();
    }

    return std::numeric_limits<double>::infinity();
}

template <typename MPTraits>
double MorseQuery<MPTraits>::CalculateWaitTimeForSafeMotion(
    const CfgType &start,
    const CfgType &end,
    double startTime)
{
    double waitTime = 0;
    double maxWaitTime = m_maxPlanningTime - startTime;

    while (waitTime < maxWaitTime) {
        if (m_safeIntervalHandler->isMotionSafe(
                start,
                end,
                startTime + waitTime,
                m_safeIntervalHandler->GetTimeResolution(),
                dynamicAgentPaths))
        {
            return waitTime;
        }
        waitTime += m_safeIntervalHandler->GetTimeResolution();
    }

    return std::numeric_limits<double>::infinity();
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
MorseQuery<MPTraits>::SocialNavigation(double congestionValue, CfgType collisionPoint, CfgType goal, Path *newPath) {
    double waitCostV = waitCost(congestionValue, collisionPoint.GetStat("WaitDuration"));
    double diverseCostV = diversePathCost(congestionValue, newPath);
    double deflectCostV = deflectionCost(congestionValue, collisionPoint, goal);

    map<double, SocialNavigationStrategy> costStrategymap = {
        {waitCostV, SocialNavigationStrategy::WAIT},
        {deflectCostV, SocialNavigationStrategy::DEFLECT},
        {diverseCostV, SocialNavigationStrategy::DIVERSE}
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

template <typename MPTraits>
typename MorseQuery<MPTraits>::CfgType
MorseQuery<MPTraits>::
    findDeflectToPoint(CfgType from, int clusterNumber)
{
  vector<CfgType> clusterPoints;
  cout<<""<< from.PrettyPrint(5)<<endl; 
  auto g = this->GetRoadmap()->GetGraph();
  for (auto git = g->begin(); git != g->end(); git++)
  {
    if (g->GetVertex(git).GetStat("Critical") == clusterNumber)
    {
      clusterPoints.push_back(g->GetVertex("Critical"));
    }
  }

  double minDistance = MAX_DBL;
  CfgType deflectToPoint(this->GetTask()->GetRobot());
  for (CfgType point : clusterPoints)
  {
    double distance = (from - point).Magnitude();
    if (distance < minDistance)
    {
      minDistance = distance;
      deflectToPoint = point;
    }
  }

  cout << "Deflection point is: " << deflectToPoint.PrettyPrint(5) << endl;
  return deflectToPoint;
}

template <typename MPTraits>
bool MorseQuery<MPTraits>::CheckCollision(CfgType robotCfg, CfgType dynamicAgentCfg) {
    robotCfg.ConfigureRobot();
    auto rb_multibody = robotCfg.GetMultiBody();

    auto ag_multibody = dynamicAgent->GetMultiBody();
    ag_multibody->Configure(dynamicAgentCfg);

    size_t rb_numbody = rb_multibody->NumFreeBody();
    size_t ag_numbody = ag_multibody->NumFreeBody();

    CDInfo cdInfo;
    CollisionDetectionMethod *m_cdMethod = new Rapid();
    
    for (size_t i = 0; i < rb_numbody; ++i) {
        for (size_t j = 0; j < ag_numbody; ++j) {
            if (m_cdMethod->IsInCollision(rb_multibody->GetFreeBody(i), 
                ag_multibody->GetFreeBody(j), cdInfo)) {
                cout << "Collision found for Robot Cfg: " << robotCfg.PrettyPrint(5) 
                     << " and Dynamic Agent Cfg: " << dynamicAgentCfg.PrettyPrint(5) << endl;
                delete m_cdMethod;
                return true;
            }
        }
    }
    delete m_cdMethod;
    return false;
}

template <typename MPTraits>
vector<typename MPTraits::CfgType>
MorseQuery<MPTraits>::GetCfgs(const vector<VID> &_segment) {
    if (_segment.empty())
        return std::vector<CfgType>();

    GraphType *g = this->GetRoadmap()->GetGraph();
    vector<CfgType> out = {g->GetVertex(_segment.front())};

    auto env = this->GetMPLibrary()->GetMPProblem()->GetEnvironment();

    for (auto it = _segment.begin(); it + 1 < _segment.end(); ++it) {
        typename GraphType::adj_edge_iterator ei;
        {
            typename GraphType::edge_descriptor ed(*it, *(it + 1));
            typename GraphType::vertex_iterator vi;
            g->find_edge(ed, vi, ei);
        }

        auto lp = this->GetMPLibrary()->GetLocalPlanner("sl");

        CfgType &start = g->GetVertex(*it);
        CfgType &end = g->GetVertex(*(it + 1));
        vector<CfgType> recreatedEdge = ei->property().GetIntermediates();
        recreatedEdge.insert(recreatedEdge.begin(), start);
        recreatedEdge.push_back(end);

        for (auto cit = recreatedEdge.begin(); cit + 1 != recreatedEdge.end(); ++cit) {
            vector<CfgType> edge = lp->ReconstructPath(*cit, *(cit + 1),
                                                      vector<CfgType>(), 
                                                      env->GetPositionRes(), 
                                                      env->GetOrientationRes());
            out.insert(out.end(), edge.begin(), edge.end());
        }
        out.push_back(end);
    }
    
    cout << "endcfg" << endl;
    return out;
}

template <typename MPTraits>
vector<typename MPTraits::Path *>
MorseQuery<MPTraits>::GetPaths(VID _start, VID _goal, size_t k) {
    vector<Path *> m_paths;
    unordered_map<VID, VID> oldNew;
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
        cout << "path size found line 542: " << this->GetPath()->Length() << endl;
        
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
    vector<Path *> m_paths;
    
    if (connected) {
        m_paths = GetPaths(start.first, goal.first, k);
        std::sort(m_paths.begin(), m_paths.end(), 
                 [](Path *path1, Path *path2) {
                     return path1->Size() < path2->Size();
                 });

        if (!m_inputDynamicAgentPathFile.empty()) {
            cout << "Reading dynamic agent path" << endl;
            ReadDynamicAgentPath();
        } else {
            cout << "Dynamic agent path not found" << endl;
        }

        cout << "Paths size: " << m_paths.size() << endl;

        for (size_t pathIdx = 0; pathIdx < m_paths.size(); ++pathIdx) {
            cout << "Path index = " << pathIdx << " paths size = " << m_paths.size() << endl;
            bool collisionFound = false;
            bool nextPath = false;
            bool shouldDeflect = false;

            Path *p = m_paths[pathIdx];
            vector<VID> vids = p->VIDs();
            vector<vector<VID>> segments;
            
            for (auto it = vids.begin(); it + 1 < vids.end(); ++it) {
                segments.push_back({*it, *(it + 1)});
            }
            cout << "For path: " << p << " found " << segments.size() << " edges." << endl;

            int previousEdgeSize = 0;
            for (int iter = 0; iter < segments.size(); iter++) {
                vector<VID> edge = segments[iter];
                bool shouldDoWaiting = false;
                cout << "Evaluating Edge from vertex: " << edge.front() 
                     << " to vertex:" << edge.back() << endl;

                vector<CfgType> path_cfgs = GetCfgs(edge);
                cout << "cfg size: " << path_cfgs.size() << endl;

                CfgType collisionCfg = p->GetRoadmap()->GetGraph()->GetVertex(edge.front());

                if (!ValidPath(path_cfgs, collisionCfg, previousEdgeSize)) {
                    collisionFound = true;
                    connected = false;
                    cout << "Collision found for path " << p << " at vertex: " 
                         << edge.front() << " with property: " 
                         << collisionCfg.PrettyPrint(5) << endl;
                    collisionCfg.GetAllLabelsAndStats();

                    if ((pathIdx + 1) >= m_paths.size()) {
                        cout << "Exhausted all paths.." << endl;
                        return false;
                    }

                    auto strategy = SocialNavigation(1.0, collisionCfg, _goal, m_paths[pathIdx + 1]);
                    cout << "Social Navigation Strategy returned: ";
                    PrettyPrintSocialNavigationStrategy(strategy);
                    cout << endl;

                    switch (strategy) {
                        case SocialNavigationStrategy::WAIT: {
                            int waitDuration = collisionCfg.GetStat("WaitDuration");
                            VID waitAtVID = collisionCfg.GetStat("WaitAtVID");
                            cout << "Debug print --- v:" << waitAtVID << endl;
                            p->GetRoadmap()->GetGraph()->GetVertex(waitAtVID)
                             .SetStat("WaitDuration", waitDuration);
                            p->GetRoadmap()->GetGraph()->GetVertex(waitAtVID)
                             .GetAllLabelsAndStats();
                            shouldDoWaiting = true;
                            cout << "make it wait somehow for " << waitDuration 
                                 << " ticks at vertex with VID=" << waitAtVID 
                                 << " vertex: " << edge.front() << endl;
                            break;
                        }

                        case SocialNavigationStrategy::DEFLECT: {
                            vector<VID> noIssueVIDs;
                            auto vidIter = p->VIDs().begin();
                            while (vidIter != p->VIDs().end() && *vidIter != edge.front()) {
                                noIssueVIDs.push_back(*vidIter);
                                cout << "No issue VID: " << *vidIter << endl;
                                ++vidIter;
                            }

                            int clusterNumber = p->GetRoadmap()->GetGraph()->GetVertex(edge.back()).GetStat("Critical");

                            cout << "Collision Vertex: " << edge.back() << " , " << p->GetRoadmap()->GetGraph()->GetVertex(edge.back()) << endl;
                            p->GetRoadmap()->GetGraph()->GetVertex(edge.back()).SetLabel("FeasibleCritical", false);
                            CfgType deflectedPoint = findDeflectToPoint(collisionCfg, 
                                                   clusterNumber);

                            cout<< p<<" able to get deflected point "<<this->GetRoadmap()->GetGraph()->GetVID(deflectedPoint)<<endl;
                            cout << "Trying to find path from: " << edge.front() << " to " << this->GetRoadmap()->GetGraph()->GetVID(deflectedPoint) << endl;
                            vector<Path *> collisionPt_to_deflectionPtPaths = 
                                GetPaths(edge.front(),
                                       this->GetRoadmap()->GetGraph()->GetVID(deflectedPoint), 1);
                            collisionPt_to_deflectionPtPaths[0]->Insert(0, noIssueVIDs);
                            for(auto whyy : collisionPt_to_deflectionPtPaths[0]->VIDs()) {
                                cout << "vids  1 " <<  whyy << endl;
                            }

                            vector<Path *> new_paths = 
                                GetPaths(this->GetRoadmap()->GetGraph()->GetVID(deflectedPoint),
                                       goal.first, 1);
                            *(collisionPt_to_deflectionPtPaths[0]) += *(new_paths[0]);
                            for(auto whyy : collisionPt_to_deflectionPtPaths[0]->VIDs()) {
                                cout << "vids  " <<  whyy << endl;
                            }
                            m_paths.insert((m_paths.begin() + pathIdx + 1), 
                                         collisionPt_to_deflectionPtPaths[0]);
                            shouldDeflect = true;
                            break;
                        }

                        case SocialNavigationStrategy::DIVERSE:
                            nextPath = true;
                            break;

                        default:
                            break;
                    }
                } else {
                    cout << "Edge from vertex: " << edge.front() << " to vertex:"
                         << edge.back() << "is valid." << endl;
                }

                previousEdgeSize += path_cfgs.size() - 1;

                if (nextPath || shouldDeflect || shouldDoWaiting) {
                    if (shouldDoWaiting) {
                        pathIdx -= 1;
                    }
                    break;
                }
            }

            if (!collisionFound) {
                cout << "valid this..." << p << endl;
                this->GeneratePath(p->VIDs().front(), p->VIDs().back());
                this->GetPath()->Clear();
                vector<VID> vids = p->VIDs();
                vector<VID> pathWithWait;

                for (int i = 0; i < vids.size(); i++) {
                    auto isWait = p->GetRoadmap()->GetGraph()->GetVertex(vids[i])
                                 .IsStat("WaitDuration");
                    cout << "VID = " << vids[i] << " IsStat WaitDuration = " 
                         << isWait << endl;
                    if (isWait) {
                        int waitDuration = p->GetRoadmap()->GetGraph()
                                          ->GetVertex(vids[i])
                                          .GetStat("WaitDuration");
                        cout << "WaitDuration = " << waitDuration << endl;
                        while (waitDuration > 0) {
                            pathWithWait.push_back(vids[i]);
                            waitDuration--;
                        }
                    }
                    pathWithWait.push_back(vids[i]);
                }
                cout << "With wait, path size = " << pathWithWait.size() << endl;

                *this->GetPath() += pathWithWait;
                connected = true;
                break;
            } else if (collisionFound && (pathIdx + 1) >= m_paths.size()) {
                cout << "Exhausted all possible paths." << endl;
                return false;
            }
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

    if (m_deleteNodes) {
        PRMQuery<MPTraits>::RemoveTempCfg(start);
        PRMQuery<MPTraits>::RemoveTempCfg(goal);
    }

    return connected;
}

#endif  