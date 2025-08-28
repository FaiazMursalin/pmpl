#ifndef MORSE_WAIT_QUERY_H_
#define MORSE_WAIT_QUERY_H_

#include <map>
#include <vector>
#include <numeric>
#include <unordered_set>
#include <queue>
#include "QueryMethod.h"
#include "MPLibrary/LocalPlanners/LPOutput.h"
#include "MPLibrary/LocalPlanners/StraightLine.h"
#include "Utilities/MetricUtils.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/CDInfo.h"

#include <containers/sequential/graph/algorithms/astar.h>
#include <containers/sequential/graph/algorithms/breadth_first_search.h>

template <typename MPTraits>
class MorseWaitQuery : public PRMQuery<MPTraits>
{
public:
    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::WeightType   WeightType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::GraphType GraphType;
    typedef typename RoadmapType::VID       VID;
    typedef typename MPTraits::Path         Path;

    MorseWaitQuery(); // default constructor
    MorseWaitQuery(XMLNode &_node); //constructor for xml configuration
    virtual ~MorseWaitQuery() = default; //destructor

    virtual void Print(ostream &_os) const override; //print query information
    virtual bool PerformSubQuery(const CfgType &_start, const CfgType &_goal) override; //executing the main sub query

    //initializing function to set up
    virtual void Initialize() override {
        //if query is empty go to parent class to prmquery to set the query
        if(PRMQuery<MPTraits>::GetQuery().empty())
            PRMQuery<MPTraits>::Initialize();
        // read dynamic agent path if file is specified and call Read Dynamic agent path function
        if (!m_inputDynamicAgentPathFile.empty()) {
            cout << "Reading dynamic agent path" << endl;
            ReadDynamicAgentPath();
        } else {
            cout << "Dynamic agent path not found" << endl;
        }

        InitializeCritical();//initialize the critical points and feasible critical points with stat label
    }

protected:
    size_t m_maxPlanningTime = 0; //max planning time initialized to 0 for now but later changes
	string m_vcLabel;
    enum GraphSearchAlg {DIJKSTRAS, ASTAR}; ///< The supported shortest path algorithms.
    GraphSearchAlg m_searchAlg{DIJKSTRAS};  ///< The shohrtest path algorithm to use.
    unordered_map<VID, vector<VID>> m_feasibleCritical; // map of critical point to fesible critical points
    unordered_map<VID, vector<VID>> m_clusters;  // map of critical point to region vid
    bool InitializeCritical(); //initialize critical

    //new functions declaration below
    void ReadDynamicAgentPath(); //read path of dynamic agents
    vector<Path *> GetPaths(VID start, VID goal, size_t k, unordered_map<VID, VID>& oldNew); //instead of removing 
    bool GetValidPath(vector<Path *>& m_paths); //find valid paths from k different paths
    std::pair<VID, VID> GetNextPathRegion(const VID _start, Path* _p, vector<VID>& _segment); //get next path region
    bool DynamicValidate(vector<VID>& _segment, int& _startTick); //dynamically validate path segment
    bool ValidCfg(CfgType& _cfg, int _tick); //check cfg is in collision or not or valid in _tick
    pair<int, int> calculateWaitTime(vector<VID>& _segment, int startTick, vector<int>& _waitTimes); //calculate wait time in every edge also to find the place to wait if theres any
    int calculateWaitTimeCfg(vector<CfgType>& robotcfg, int startTick, vector<int>& startWindowEndtime); // the main calculation to calculate wait time in cfg on
    void calculateIntermediates(VID start, VID goal, vector<Cfg>& _edgeCfg);
    vector<vector<pair<int, int>>> GetSafeWindows(vector<CfgType>& _segment, int _startTick);
    vector<pair<int, int>> findNonCollisionWindows(vector<CfgType>& _segment, int startTick, int agentIdx);
    void GetSafeWindows(CfgType& _cfg, int _startTick, vector<vector<pair<int, int>>>& _windows, int _max_length);  // NEW
    void findNonCollisionWindows(CfgType& _cfg, int startTick, int agentIdx, vector<pair<int, int>>& _window, int _max_length);  //NEW
    bool ValidateCfgAgainstDynamicAgent(CfgType& _cfg, int _tick, int dynamicAgentNumber);
    void GetSafeWindowsInterval(CfgType& _cfg, int _startTick, vector<pair<int, int>>& _safeWindows,  int _max_length);
    //end of new functions declarations

    vector<string> m_ncLabels{"kClosest"};//labels for nearest neighbor calculation
    vector<vector<CfgType>> dynamicAgentPaths;//paths of dynamic agents
    string m_inputDynamicAgentPathFile;//file containing dynamic agent paths
    Robot *dynamicAgent;//dynamic agent robot
    string m_dynamicAgent;//dynamic agent identifier
    string m_mapFileName;//filename of the map
    bool m_deleteNodes{false};//flag for node deletion
};
template <typename MPTraits>
MorseWaitQuery<MPTraits>::MorseWaitQuery() : PRMQuery<MPTraits>() {
    this->SetName("MorseWaitQuery");//set class name
    m_inputDynamicAgentPathFile = "";//no input file by default
    m_dynamicAgent = "";//no dynamic agent by default
    dynamicAgent = nullptr;//no dynamic agent pointer
    m_maxPlanningTime = 0;//no planning time limit
    m_mapFileName = "";//no map file
}

template <typename MPTraits>
MorseWaitQuery<MPTraits>::MorseWaitQuery(XMLNode &_node) : PRMQuery<MPTraits>(_node) {
    this->SetName("MorseWaitQuery");//initialize xml configurations
    m_inputDynamicAgentPathFile = _node.Read("agentPath", false, "",
                                           "filename of dynamic agent path");//read dynamic agent file from xml

    m_mapFileName = _node.Read("inputMap", false, "", "filename of map file");//read map file from xml

    m_dynamicAgent = _node.Read("agent", true, "", "agent robot");//read dynamic agent robot from xml

    m_deleteNodes = _node.Read("deleteNodes", false, m_deleteNodes,
                              "Whether or not to delete start and goal from roadmap");//node delettion flag from xml

    m_vcLabel = _node.Read("vcLabel", true, "", "CD Method");

    bool defaultsCleared = false;
    for (auto &child : _node) {//process node connection methods from xml
        if (child.Name() == "NodeConnectionMethod") {//clear defaults on first method
            if (!defaultsCleared) {
                defaultsCleared = true;
                m_ncLabels.clear();
            }
            m_ncLabels.push_back(child.Read("method", true, "", "Connector method"));//add the connector method to the list
        }
    }
}

template <typename MPTraits>
void MorseWaitQuery<MPTraits>::Print(ostream &_os) const {
    QueryMethod<MPTraits>::Print(_os);//print base class information from superclass query method
    _os << "\n\tDelete Nodes: " << m_deleteNodes
        << "\n\tConnectors:" << endl;//print configuration params
    for (const auto &label : m_ncLabels)
        _os << "\t\t" << label << endl;//print all connector methods
}

template <typename MPTraits>
void MorseWaitQuery<MPTraits>::ReadDynamicAgentPath() {
    m_inputDynamicAgentPathFile = MPProblem::GetPath(m_inputDynamicAgentPathFile);//get full path for dynamic agent file

    cout << "Reading dynamic agent path file \'" << m_inputDynamicAgentPathFile << endl;
    ifstream in(m_inputDynamicAgentPathFile);
    cout << "File stream ready for dynamic agent path" << endl;

    if (!in.good())//verify file can be opened
        throw ParseException(WHERE, "Can't open dynamic agent path file '" + m_inputDynamicAgentPathFile + "'.");

    if (!dynamicAgentPaths.empty())
        dynamicAgentPaths.clear();//clear existing paths if any

    //read dynamic agent robot defenition
    cout << "Reading dynamic agent from: " << m_dynamicAgent << endl;
    const string inputDynamicAgentFile(m_dynamicAgent);
    CountingStreamBuffer cbs(inputDynamicAgentFile);
    std::istream ifs(&cbs);
    //create and read multibody definition for the dynamic agents
    ActiveMultiBody *const mb = new ActiveMultiBody;
    mb->Read(ifs, cbs);

    dynamicAgent = new Robot(this->GetMPProblem(), mb, "dynamic agent");//create dynamic agent robot using the multibody above
    cout << "Gets dynamic agent" << endl;
    cout << dynamicAgent << endl;

    cout << "DOF " << dynamicAgent->GetMultiBody()->DOF() << endl;
    cout << "PosDOF " << dynamicAgent->GetMultiBody()->PosDOF() << endl;
    cout << "OrientationDOF " << dynamicAgent->GetMultiBody()->OrientationDOF() << endl;
    cout << "JointDOF " << dynamicAgent->GetMultiBody()->JointDOF() << endl;

    //read number of dynamic agents
    int numDynamicAgents;
    in >> numDynamicAgents;
    cout << "Number of dynamic agents: " << numDynamicAgents << endl;
    in.ignore();

    //read path for each dynamic agent
    vector<CfgType> dynamicAgentPath;
    for (int i = 0; i < numDynamicAgents; i++) {
        int number;
        in >> number;
        in.ignore();

        cout << "Should read lines: " << number << " for path " << i << endl;
        dynamicAgentPath.clear();

        //read each configuration in the path
        for (int j = 0; j < number; ++j) {
            CfgType tempCfg(dynamicAgent);
            double skip;
            in >> skip >> tempCfg;
            //skip the 0s and trailing
            dynamicAgentPath.push_back(tempCfg);
        }

        cout << "Read " << dynamicAgentPath.size() << " for path " << i << endl;
        m_maxPlanningTime = std::max(m_maxPlanningTime, (2*dynamicAgentPath.size()));//update max planning time based on dynamic agent max path
        dynamicAgentPaths.push_back(dynamicAgentPath);
    }
    cout << "Read " << dynamicAgentPaths.size() << " paths for dynamic agent path" << endl;
    //printing the size of each dynamic agent path
    for (int i = 0; i < dynamicAgentPaths.size(); i++) {
        cout << "Dynamic agent " << i << " has size: " << dynamicAgentPaths[i].size() << endl;
    }
    cout << "m_maxPlanningTime: " << m_maxPlanningTime << endl;
    in.close();
}



template <typename MPTraits>
bool
MorseWaitQuery<MPTraits>::
InitializeCritical() {
  cout << "InitializeCritical called" << endl;
  bool returnValue = false;
  // Initialize empty membership for each critical point in block roadmap
  auto og = this->GetBlockRoadmap()->GetGraph();
  for(auto cit = og->begin(); cit != og->end(); ++cit){
    m_feasibleCritical.insert({cit->descriptor(), vector<VID>()});
    m_clusters.insert({cit->descriptor(), vector<VID>()});
  }
  // Populate the information for every vertex in the main roadmap
  auto g = this->GetRoadmap()->GetGraph();
  for(auto vit = g->begin(); vit != g->end(); ++vit){
    auto vp = g->GetVertex(vit);
    // only if the critical point information is populated by strategy
    if(vp.IsStat("Critical")){
      returnValue = true;
      VID cVID = vp.GetStat("Critical");
      //add to cluster
      m_clusters[cVID].push_back(vit->descriptor());
      //add to feasible critical point if marked as FeasibleCritical in the label
      if(vp.IsLabel("FeasibleCritical") && vp.GetLabel("FeasibleCritical"))
        m_feasibleCritical[cVID].push_back(vit->descriptor());
    }
  }
  cout << "InitializeCritical return: " << returnValue << endl;
  return returnValue;
}

template <class MPTraits>
vector<typename MPTraits::Path *>
MorseWaitQuery<MPTraits>::GetPaths(VID _start, VID _goal, size_t k, unordered_map<VID, VID>& oldNew) {
    vector<Path *> m_paths;//vector to store the k diverse paths
    //create a new graph to work with preserving the original in the prevg
    GraphType *prevg = new GraphType();
    auto gr = this->GetRoadmap()->GetGraph();
    //copy vertices from original to new graph gr
    for (auto vit = gr->begin(); vit != gr->end(); ++vit) {
        auto newVID = prevg->AddVertex(vit->descriptor(), vit->property());
        oldNew.insert({vit->descriptor(), newVID});
    }
    //copy edges from prevg to new graph gr
    for (auto eit = gr->edges_begin(); eit != gr->edges_end(); eit++) {
        prevg->AddEdge(oldNew[eit->source()], oldNew[eit->target()], eit->property());
    }
    //store sets of critical points for path diversity
    vector<unordered_set<VID>> criticalPoints;
    unordered_set<VID> pCritical;
    //find k different paths
    while (m_paths.size() < k) {
        //clear existing paths
        this->GetPath()->Clear();
        //find a path from _start to _goal
        auto mapPassed = PRMQuery<MPTraits>::PerformSubQuery(gr->GetVertex(_start), gr->GetVertex(_goal));
        //create new path object
        Path *p = new Path(this->GetRoadmap());
        vector<VID> pVID, fVID;
        //process each vertex in the found path
        for (auto v : this->GetPath()->VIDs()) {
            //add mapped VID to path
            pVID.push_back(oldNew[v]);
            auto vp = gr->GetVertex(v);
            //collect FeasibleCritical points and
            if (vp.IsLabel("FeasibleCritical") && vp.GetLabel("FeasibleCritical")) {
                fVID.push_back(v);
            }
        }
        *p += pVID;//add vertices to path
        //collect critical points from feasible critical points
        for (auto f : fVID) {
            if (gr->GetVertex(f).IsStat("Critical"))
                pCritical.insert(gr->GetVertex(f).GetStat("Critical"));
        }
        //check if the set of critical points is unique
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
        //remove path vertices from graph except start and goal to force different paths
        for (auto v : this->GetPath()->VIDs()){
            if (v != _start && v != _goal){
                gr->DeleteVertex(v);
            }
        }
        //if path is unique store it into collections of m_paths
        if (!isSame) {
            criticalPoints.push_back(pCritical);
            m_paths.push_back(p);
        } else {
            delete p;//delete duplicate path
        }
    }
    this->GetRoadmap()->SetGraph(prevg);//restore original graph
    return m_paths;
}


template <typename MPTraits>
bool MorseWaitQuery<MPTraits>::PerformSubQuery(const CfgType &_start, const CfgType &_goal) {
  /*cout << "Perform subquery started" << endl;
  cout << "Evaluating sub-query:" << endl
       << "\tfrom " << _start << endl
       << "\tto   " << _goal << endl;*/
  // Find connected components in roadmap
  auto ccs = this->FindCCs();
  //cout << "cc size:" << ccs.size() << endl;
  // Ensure start and goal configurations are in the roadmap
  auto start = this->EnsureCfgInMap(_start);
  auto goal = this->EnsureCfgInMap(_goal);
  // store the previous path ids before it gets modifed
  auto prevPathIDs = this->GetPath()->VIDs();
  // Track if path is found
  bool connected = false;
  // Try to connect start and goal through each connected component
  for (auto cc : ccs) {
    // Attempt to connect start and goal to current component
    this->ConnectToCC(start.first, cc.second);
    this->ConnectToCC(goal.first, cc.second);
    // If start and goal are in same component
    if (this->SameCC(start.first, goal.first)) {
      connected = true;
      // Generate path between them
      this->GeneratePath(start.first, goal.first);
      break;
    }
  }
  //cout << "connected: " << connected << endl;
  // If initial path found, process critical points
  if (connected) {
    // Initialize critical point information
    InitializeCritical();
    // Find the clusters the start and goal belongs to Get roadmap graph
    auto g = this->GetRoadmap()->GetGraph();
    // Find vertices for start and goal
    auto sit = g->find_vertex(start.first);
    auto git = g->find_vertex(goal.first);
    // Find critical region for start vertex
    for(auto eit= sit->begin(); eit != sit->end(); eit++){
      // Check source vertex
      auto op = g->GetVertex(eit->source());
      if(op.IsStat("Critical")){
        // Set critical point and add to cluster
        sit->property().SetStat("Critical",op.GetStat("Critical"));
        m_clusters[op.GetStat("Critical")].push_back(sit->descriptor());
        break;
      }
      // Check target vertex
      op = g->GetVertex(eit->target());
      if(op.IsStat("Critical")){
        // Set critical point and add to cluster
        sit->property().SetStat("Critical",op.GetStat("Critical"));
        m_clusters[op.GetStat("Critical")].push_back(sit->descriptor());
        break;
      }
    }
    // Find critical region for goal vertex (same process as start)
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
    // Initialize for diverse path finding
    vector<Path *> m_paths;//contains all the diverse set of paths
    // Map for vertex ID translation
    unordered_map<VID, VID> oldNew;
    // Number of diverse paths to find
    size_t k = 4;
    // Re-ensure start and goal in map
    auto start = this->EnsureCfgInMap(_start);
    auto goal = this->EnsureCfgInMap(_goal);
    // Get k diverse paths
    m_paths = GetPaths(start.first, goal.first, k, oldNew);
    // Sort paths by size (shorter paths first)
    std::sort(m_paths.begin(), m_paths.end(),[](Path *path1, Path *path2) {
      return path1->Size() < path2->Size();
    });
    cout << "Total number of paths at the end of GetPaths = " << m_paths.size() << endl;
    /*for(int i = 0; i < m_paths.size(); i++){
    cout <<"Path "<<i<<" has size: "<< m_paths[i]->Size() << endl;
    }*/
    // Find valid path considering dynamic agents
    connected = GetValidPath(m_paths);
    if (connected){
      //restore the previous path
      vector<VID> p;
      for (auto v : prevPathIDs) {
        p.push_back(oldNew[v]);
      }
      //add the new subpath
      auto newVID = this->GetPath()->VIDs();
      p.insert(p.end(), newVID.begin(), newVID.end());
      this->GetPath()->Clear();
      *this->GetPath() += p;
    }
    if(this->m_debug){
      if (connected){
        cout << "\tSuccess: found path from start node " << start.first<< " to goal node " << goal.first << "." << endl;
      }else{
        cout << "\tFailed to connect start node " << start.first<< " to goal node " << goal.first << "." << endl;
      }
    }
    //cleanup if needed
    if(m_deleteNodes){
      PRMQuery<MPTraits>::RemoveTempCfg(start);
      PRMQuery<MPTraits>::RemoveTempCfg(goal);
    }
    return connected;
  }
}


template <class MPTraits>
bool
MorseWaitQuery<MPTraits>::
ValidateCfgAgainstDynamicAgent(CfgType& _cfg, int _tick, int dynamicAgentNumber) {// Function to check if a robot configuration collides with a dynamic agent at a specific time
    // Calculate which configuration of the dynamic agent to use based on time
    size_t agentTick;// Will store the index of agent configuration to use
    size_t sz = dynamicAgentPaths[dynamicAgentNumber].size()-1;// Get size of agent's path (minus 1 for 0-based indexing)
    size_t mult =  _tick / sz;// How many complete cycles of the path
    size_t rem =  _tick % sz;// Position within current cycle
    // Determine direction of motion (forward/backward) based on cycle number
    if(mult % 2 == 0) agentTick = rem;//even->forward
    else agentTick = sz - rem;//odd->backward motion

    //set robot configurations in environment
    _cfg.ConfigureRobot();// Configure robot with given configuration
    // Get robot's physical representation
    auto multiBody = _cfg.GetMultiBody();
    // Get dynamic agent's physical representation
    ActiveMultiBody* mb = dynamicAgent->GetMultiBody();
    // Get number of bodies to check for collisions
    size_t numBody = multiBody->NumFreeBody();
    // Number of bodies in dynamic agent
    size_t numOtherBody = mb->NumFreeBody();
    // Initialize and stores collision detection information
    CDInfo cdInfo;
    // Create collision detector using RAPID algorithm
//	CollisionDetectionMethod* m_cdMethod;
//	if (m_vcLabel == "RAPID") {
//   		m_cdMethod = new Rapid(false);
//	} else if (m_vcLabel == "RAPID_SOLID") {
//    	m_cdMethod = new Rapid(true);
//	}
    CollisionDetectionMethod *m_cdMethod = new Rapid();
    // Get and configure dynamic agent's configuration at calculated time
    auto agentCfg = dynamicAgentPaths[dynamicAgentNumber][agentTick];
    mb->Configure(agentCfg); // Set agent to this configuration
    // Check collisions between all body pairs
    for(size_t j = 0; j < numBody; ++j) {  // for each body of robot
        for(size_t k = 0; k < numOtherBody; ++k) {   // for each body of the agent
            // Check for collision between current pair of bodies
            bool collisionFound =
              m_cdMethod->IsInCollision(multiBody->GetFreeBody(j),
                  mb->GetFreeBody(k), cdInfo);
            // If collision found, configuration is invalid
            if(collisionFound) {
                return false;
            }

        }
    }
    // No collisions found - configuration is valid
    return true;
}

// Function finds time windows where configurations don't collide with a specific dynamic agent
template <class MPTraits>
vector<pair<int, int>>
MorseWaitQuery<MPTraits>::
findNonCollisionWindows(vector<CfgType>& _segment, int startTick, int agentIdx) {
    // Vector to store collision-free time windows as (start, end) pairs
    vector<pair<int, int>> windows;  // this vector stores number of pairs = number of intermediate cfgs in edge * their individual safety window
    // we need a vector of the safety window per cfg
    // Initialize first window start time with starting tick
    int t1 = startTick;
    // Iterate through each configuration in the segment
    for(auto it = _segment.begin(); it < _segment.end(); ++it) {
        // Check if initial configuration collides at start time
        bool isColliding = !ValidateCfgAgainstDynamicAgent(*it, startTick, agentIdx);
        // If starting with collision, invalidate window start time
        if(isColliding) {
            t1 = -1;  // Invalid t1 to indicate we're in collision so no valid window start time
        }
        // Initialize tick counter for checking future times
        int currentTick = 0;
        // Check each future time step up to maximum planning time
        for(size_t cfgIdx = 1; cfgIdx < m_maxPlanningTime; cfgIdx++) {
            // Calculate actual tick by adding offset to start time
            currentTick = startTick + cfgIdx;
            // Check if configuration collides at current time
            bool currentCollision = !ValidateCfgAgainstDynamicAgent(*it, currentTick, agentIdx);
            // If collision state changed (from colliding to free or vice versa)
            if(currentCollision != isColliding) {
                if(currentCollision) {
                    // transition from collsiion free to colliding so close current window with end time as previous tick
                    windows.push_back({t1, currentTick - 1});
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
            windows.push_back({t1, INT_MAX});
        }
    }
    //returns all found collision free time windows
    return windows;
}

// NEW FUNCTION
// Function finds time windows where configurations don't collide with a specific dynamic agent
template <class MPTraits>
void
MorseWaitQuery<MPTraits>::
findNonCollisionWindows(CfgType& _cfg, int startTick, int agentIdx, vector<pair<int, int>>& _windows, int  _max_length) {
    // Initialize first window start time with starting tick
    int t1;
    // Check if the configuration collides at start time
    bool isColliding = !ValidateCfgAgainstDynamicAgent(_cfg, startTick, agentIdx);
    // If starting with collision, invalidate window start time
    if(isColliding) {
       t1 = -1;  // Invalid t1 to indicate we're in collision so no valid window start time
    }
    else t1 = startTick;
    // Initialize tick counter for checking future times
    int currentTick;
    // Check each future time step up to maximum planning time
    for(size_t cfgIdx = 1; cfgIdx < _max_length; cfgIdx++) {
       // Calculate actual tick by adding offset to start time
       currentTick = startTick + cfgIdx;
       // Check if configuration collides at current time
       bool currentCollision = !ValidateCfgAgainstDynamicAgent(_cfg, currentTick, agentIdx);
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

template<class MPTraits>
bool
MorseWaitQuery<MPTraits>::
GetValidPath(vector<typename MPTraits::Path *>& m_paths){
  // static function to update a newPath with new VIDs
  auto AddToPath = [&](Path* _newPath, vector<VID>& _segment) {
     vector<VID> addVID;
      if(!_newPath->VIDs().empty()){
          // Skip first vertex if not first segment (avoid duplicates)
          addVID.insert(addVID.end(), _segment.begin()+1, _segment.end());
      }else{
      // Include all vertices for first segment
       addVID.insert(addVID.end(), _segment.begin(), _segment.end());
      }
      // Add valid segment to path
      *_newPath += addVID;

  };
  // Flag to track if at least one valid path is found
  bool oneSuccess = false;
  // Pointer to store the best path found
  Path *bestPath = nullptr;
  // Index of the best path in m_paths
  size_t bestPathIndex;
  //map to store wait times for each path ----path index -> vector of {vertex, wait time}
  std::map<size_t, std::vector<std::pair<VID, double>>> waitTimes;
  //process each candidate path
  for(size_t i = 0; i < m_paths.size(); i++){
    cout<<"======================================"<<endl;
    // Get vertex IDs for current path
    auto pathVIDs = m_paths[i]->VIDs();
    // Get starting vertex
    auto start = pathVIDs.front();
    // Vector to store path segments between critical points
    vector<VID> segment;
    // Create new path for modified version with wait times
    Path* newPath = new Path(this->GetRoadmap());
    // Track if current path is valid
    bool result = true;
    //main timer counter
    int startTick = 0;
    // Print current path info
    cout<<"For Path "<<i<<" Start: "<<pathVIDs.front()<<" Goal: "<<pathVIDs.back()
         <<" path length: "<<m_paths[i]->Length()<<endl;
    //processing path segment by segment
    while(true){
      // Clear previous segment
      segment.clear();
      // Get next segment between critical regions
      auto criticals = GetNextPathRegion(start, m_paths[i], segment);
      //break if we reached the end
      if(!newPath->VIDs().empty() && newPath->VIDs().back() == pathVIDs.back()){
        break;
      }
      int currentTick = startTick;
      //validate the segment on the current time step
      bool validSegment = DynamicValidate(segment, currentTick);
      if(validSegment){
        // If segment is valid, prepare vertices to add
        AddToPath(newPath, segment);
        // Update start point for next segment
        start = segment.back();
        //update start tick
        startTick = currentTick;
        result = true; // the segment is valid so result is true
      }else{
        //calculate required wait times if segment is invalid
        vector<int> pathWaitTimes;
        auto waitReturn = calculateWaitTime(segment, startTick, pathWaitTimes);
        int waitTime = waitReturn.first;
        cout<<"wait return second: "<<waitReturn.second<<endl;
        // Calculate wait cost to MAXDOUBLE if wait time is infinite
        double waitCost = (waitTime == INT_MAX)? MAXDOUBLE :(waitReturn.second-startTick);
        // Store wait times for vertices that needs waiting
        for(size_t index = 0; index < pathWaitTimes.size(); ++index) {
          if(pathWaitTimes[index] > 0) {
            if(waitTimes.find(i) == waitTimes.end()) {
              //create a new entry if first wait time for this path
              waitTimes.insert(make_pair(i, std::vector<pair<VID, double>>(1, {segment[index], pathWaitTimes[index]})));
            }else {
              //add to existing wait times for this path
              waitTimes[i].push_back(make_pair(segment[index], pathWaitTimes[index]));
            }
          }
        }
        //update path with wait times if valid wait times been found
        if(waitTimes[i].size() > 0 && waitCost != MAXDOUBLE) {
          result = true;
          startTick = waitReturn.second;
          AddToPath(newPath, segment);
          start = segment.back();
        }else{ // should not come here
          //if no wait times been found mark the path invalid
          result = false;
          break;
        }
      }
      //break if no valid wait time found
      if(!result)
          break;
    }
    //update best path if current path is valid
    if(result){
      if(!oneSuccess){
        //if first path is success set as best path
        bestPath = newPath;
        oneSuccess = true;
        bestPathIndex = i;
      }else if(newPath->Length() <= bestPath->Length()){
        //if better path found other than the current best update the best path
        bestPath = newPath;
        bestPathIndex = i;
      }else{
        //if not better delete the path
        delete newPath;
      }
    }else{
      //delete invalid path
      delete newPath;
    }
    cout<<"=================================================================="<<endl;
  }
  //if found atleast one valid path
  if (oneSuccess){
    cout<<"Stored best path length:"<<bestPath->Length()<<endl;
    cout << "Best path index: "<< bestPathIndex << ", " << (waitTimes.find(bestPathIndex) != waitTimes.end()) << ", " << waitTimes[bestPathIndex].size()<< endl;
    //store wait times in vertices of best path
    if(waitTimes.find(bestPathIndex) != waitTimes.end()){
      for (const auto& pair : waitTimes[bestPathIndex]) {
        cout << "(vid: " << pair.first << ", wait: " << pair.second << ") ";
        bestPath->GetRoadmap()->GetGraph()->GetVertex(pair.first).SetStat("waittime", pair.second);
      }
    }
    //set final path
    this->GetPath()->Clear();
    *this->GetPath() += bestPath->VIDs();
  }
  //return whether a valid paths been found
  return oneSuccess;
}

// Function calculates intermediate configurations between start and goal vertices
//should go through all the cfg in the segment
template<class MPTraits>
void
MorseWaitQuery<MPTraits>::
calculateIntermediates(VID start, VID goal, vector<Cfg>& _edgeCfg){
    // Get reference to the roadmap graph
    auto g= this->GetRoadmap()->GetGraph();
    // Declare iterator for edge traversal
    typename GraphType::adj_edge_iterator ei;
    {
        // Create edge descriptor for start-goal edge
        typename GraphType::edge_descriptor ed(start, goal);
        // Declare vertex iterator (needed for find_edge)
        typename GraphType::vertex_iterator vi;
        // Find the edge between start and goal vertices
        g->find_edge(ed, vi, ei);
    }
    // Declare pointer for local planner
    typename MPTraits::MPLibrary::LocalPlannerPointer lp;
    try {
        // Try to get local planner specified in edge properties
        lp = this->GetLocalPlanner(ei->property().GetLPLabel());
    }
    catch(...) {
        // If fails, fallback to straight-line planner
        lp = this->GetLocalPlanner("sl");
    }
    // Get intermediate configurations stored in edge property
    vector<CfgType> inter = ei->property().GetIntermediates();
    // Recreate this edge, including intermediates.
    inter.insert(inter.begin(), g->GetVertex(start));
    // Get reference to goal configuration
    CfgType& e = g->GetVertex(goal);
    // Add goal configuration at end of intermediates
    inter.push_back(e);

    // For each pair of consecutive configurations in intermediates
    for(auto cit = inter.begin(); cit + 1 != inter.end(); ++cit) {
        //reconstruct the detailed path between cfg using lp
        vector<CfgType> edge = lp->ReconstructPath(*cit, *(cit+1),
            vector<CfgType>(), this->GetEnvironment()->GetPositionRes(), this->GetEnvironment()->GetOrientationRes());
        //add reconstructed edge cfg to output vector
        _edgeCfg.insert(_edgeCfg.end(), edge.begin(), edge.end());
    }
    //add goal cfgs to output vector
    _edgeCfg.push_back(e);

}

// Calculates the wait time for an edge
template<class MPTraits>
int MorseWaitQuery<MPTraits>::calculateWaitTimeCfg(vector<CfgType>& robotcfgs, int startTick, vector<int>& startWindowEndtime) {
    // Get time windows where each configuration is safe from all dynamic agents
    //auto windows = GetSafeWindows(robotcfgs, startTick);
    // Need to calculate the windows for every cfg
    vector<vector<pair<int, int>>> windows;
    // Number of configurations we need to find valid times for
    int tickLimit = robotcfgs.size();

    // find shortest path length for all dynamic agents this limits the max wait time based on agent cycle lengths
   /* int minAgentTick = INT_MAX;
    for (const auto& path : dynamicAgentPaths) {
        minAgentTick = std::min(minAgentTick, static_cast<int>(path.size()));
    }*/
    // get the safety window for start vertex
    windows.push_back(vector<pair<int, int>>()); // Create a slot for start vertex
    GetSafeWindowsInterval(robotcfgs[0], startTick, windows.back(), m_maxPlanningTime);
    if(windows.back().empty()) return INT_MAX; // this would never happen except for the very start of the path
      // End of valid window for start vertex
    startWindowEndtime.push_back(windows[0].front().second);
    // Check if the start vertex is perpetually free, it is set the max wait time to m_maxPlanningTime
    int upperWaitTime = std::min(windows[0].front().second - startTick, static_cast<int>(m_maxPlanningTime));

    // Process each robot configuration to get the safety window
    for (int x = 1; x < tickLimit; x++){
       windows.push_back(vector<pair<int, int>>()); // Create a slot for cfg x
       GetSafeWindowsInterval(robotcfgs[x], startTick + x, windows.back(), upperWaitTime);
    }

    // Current time being checked
    int currentTick = startTick;
    int currentWait = 0;
    vector<size_t> indices(robotcfgs.size(), 0);
    // This loop will go over to find the minimum wait time from 0 to upperWaitTime
    while(currentWait <= upperWaitTime){
      // for every intermediate cfg we will ensure current wait time works or not
      bool safe = true;
      for (int x = 1; x < tickLimit; x++){
         // Check for the index is not out of bound
         if(indices[x] >= windows[x].size()){
           safe = safe & false;
         }
         else {
            // Get the window at indices[x]
            auto window = windows[x][indices[x]];
            // running tick for intermediate x
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
        currentWait++;
    }
    return INT_MAX;
 }
/*template<class MPTraits>
int MorseWaitQuery<MPTraits>::calculateWaitTimeCfg(vector<CfgType>& robotcfgs, int startTick, vector<int>& startWindowEndtime) {
    // Get time windows where each configuration is safe from all dynamic agents
    //auto windows = GetSafeWindows(robotcfgs, startTick);
    // NEW -- Need to calculate the windows for every cfg
    vector<vector<pair<int, int>>> windows;
    // Number of configurations we need to find valid times for
    int tickLimit = robotcfgs.size();

    // find shortest path length for all dynamic agents this limits the max wait time based on agent cycle lengths
    int minAgentTick = INT_MAX;
    for (const auto& path : dynamicAgentPaths) {
        minAgentTick = std::min(minAgentTick, static_cast<int>(path.size()));
    }
    // Total accumulated wait time
    int totalWaitTime = 0;
    // Current time being checked
    int currentTick = startTick;
    // End of valid window for start vertex
    int startVertexEndTime = INT_MAX;
    // Process each robot configuration
    for (int x = 0; x < tickLimit; x++) {
        // Get the safety windows -- //NEW
        windows.clear();
        GetSafeWindows(robotcfgs[x], currentTick, windows, minAgentTick);//added minAgentTick as a parameter
        // Track if we found a valid time window
        bool foundValidWindow = false;
        // Wait time for current configuration
        int localWaitTime = 0;
        // Trying to find valid window within twice the shortest agent path length
        while (localWaitTime < 2 * minAgentTick) {
            // Track if all agents are clear
            bool allAgentsClear = true;
            // Next potential valid time
            int nextPossibleTime = currentTick;
            // Check windows for each dynamic agent
            for (size_t ay = 0; ay < dynamicAgentPaths.size(); ay++) {
                // Track if valid window found for this agent
                bool windowFound = false;
                // Check each window for current agent
                for (const auto& window : windows[ay]) {
                  // Check if current time falls within this window
                  if (currentTick >= window.first && currentTick < window.second) {
                    windowFound = true;
                    // For first configuration, track earliest window end
                    if(x==0)
                      startVertexEndTime = std::min(startVertexEndTime, window.second);
                    break;
                  } else if (currentTick < window.first) { // If window starts in future, update next possible time
                    // belongs to the collision zone for this agent
                    // NEW
                    if(x==0) // if start cfg is in collison with any agent
                      return INT_MAX; // can't wait in start vertex
                    nextPossibleTime = std::max(nextPossibleTime, window.first);
                    break; // break becoz we found whether in collson zone for this agent
                  }
                }
                // If no window found for this agent, not all agents are clear
                if (!windowFound) {
                    allAgentsClear = false;
                   // break;  // NEW -- Why break. Need to find the nextPossibleTime against other agent collision too
                }
            }
            // If found time where all agents are clear
            if (allAgentsClear) {
                foundValidWindow = true;
                break; // breaks from while
            }

            // Calculate additional wait time needed which is nextPotentialSafetick from currentTick time
            int waitNeeded = nextPossibleTime - currentTick;
            // Add to local wait time for this configuration
            localWaitTime += waitNeeded;
            // Update current time
            currentTick = nextPossibleTime;
            // NEW -- NO NEED WHILE LOOP ALREADY CHECKS FOR IT
            // If wait time exceeds limit, configuration is impossible
            //if (localWaitTime >= 2 * minAgentTick) {
                // No valid window found within limit
               // return INT_MAX;
            //}
        }
        // If no valid window found for this configuration
        if (!foundValidWindow) {
            return INT_MAX;
        }
        // Add local wait time to total
        totalWaitTime += localWaitTime;
        // Move to next time step
        currentTick+= (localWaitTime + 1);  // NEW - account for the already computed wait time
    }
    // Check if total wait time keeps us within start vertex's valid window
    if(startTick+totalWaitTime <= startVertexEndTime ){
        cout<<"start window end time: "<< startTick+totalWaitTime <<endl;
    }else{
      cout<<"outside the start window end time: "<< startTick+totalWaitTime <<endl;
    }
    // Store end time of start vertex's window
    startWindowEndtime.push_back(startVertexEndTime);
    // Return total wait time needed
    return totalWaitTime;
}*/






// Function finds safe time windows for a edge of robot configurations
template <class MPTraits>
vector<vector<pair<int, int>>>
MorseWaitQuery<MPTraits>::
GetSafeWindows(vector<CfgType>& _segment, int _startTick) {
    // Get the first configuration from segment to check initial validity
    CfgType startCfg = _segment.front();
    // Initialize vector to store safe windows for each dynamic agent
    vector<vector<pair<int, int>>> safeWindows;

    // First check if start configuration is valid at start time
    if(!ValidCfg(startCfg, _startTick)) {
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
MorseWaitQuery<MPTraits>::
GetSafeWindowsInterval(CfgType& _cfg, int _startTick, vector<pair<int, int>>& _safeWindows,  int _max_length){
    // Function to validate across every agent
    static auto ValidateAgainstAll = [&](CfgType& _cfg, int tick){
      for(size_t i= 0; i < dynamicAgentPaths.size(); i++){
        if(!ValidateCfgAgainstDynamicAgent(_cfg, tick, i))
           return false;  // early quit if invalid against at least one agent
      }
      return true;
    };
	// Initialize first window start time with starting tick
    int t1;
    // Check if the configuration collides at start time
    bool isColliding = !ValidateAgainstAll(_cfg, _startTick);
    // If starting with collision, invalidate window start time
    if(isColliding) {
       t1 = -1;  // Invalid t1 to indicate we're in collision so no valid window start time
    }
    else t1 = _startTick;
    // Initialize tick counter for checking future times
    int currentTick;
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
            _safeWindows.push_back({t1, currentTick - 1});
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


// NEW FUNCTION
// Function finds safe time windows for a edge of robot configurations
template <class MPTraits>
void
MorseWaitQuery<MPTraits>::
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
pair<int, int>
MorseWaitQuery<MPTraits>::
calculateWaitTime(vector<VID>& _segment, int startTick, vector<int>& _waitTimes) {// Function calculates total wait time needed for a path segment
    // Vector to store configurations along edges
    vector<Cfg> edgeCfg;
    // Current time tracking
    auto tick = startTick;
    // Track total wait time for entire segment
    int totalWaitTime = 0;
    // Vectors to track window end times and start times for backtracking
    vector<int> startWindowEndtime, startTimes;
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
        int waitTime=calculateWaitTimeCfg(edgeCfg, tick, startWindowEndtime);
        if(waitTime>0 && waitTime != INT_MAX) cout<< "calculated wait time in calculateWaitTime, waitTime: " << waitTime << endl;
        // If wait time is infinite, path is impossible
        if(waitTime==INT_MAX) {
            cout<< "calculated wait time is too big" << endl;
            return {INT_MAX,INT_MAX};
        }
        // Add wait time to total
        totalWaitTime += waitTime;
        // Update current tick (wait time + time to traverse edge)
        tick += waitTime + edgeCfg.size()-1;
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
        startTimes.push_back(startTimes.back() + edgeCfg.size() -1);
    }
    cout<< "finished calculating waiting time and tick is " <<tick<<endl;
    // Return total wait time and final tick
    return {totalWaitTime,tick};
}


template <class MPTraits>
bool
MorseWaitQuery<MPTraits>::
DynamicValidate(vector<VID>& _segment, int &_startTick){
  auto g = this->GetRoadmap()->GetGraph();
  //validating start cfg
  CfgType startCfg = g->GetVertex(_segment.front());
  if (!ValidCfg(startCfg, _startTick)){
    cout<<"Invalid start configuration or cfg not valid!!"<<endl;
    return false;
  }
  int tick = _startTick-1;//using pre increment so deducting 1
  //validate each edge in the segment
  for(auto it = _segment.begin();it+1<_segment.end(); it++){

    // Get the local planner for the edge
    typename GraphType::adj_edge_iterator ei;
    {
      typename GraphType::edge_descriptor ed(*it, *(it+1));
      typename GraphType::vertex_iterator vi;
      g->find_edge(ed, vi, ei);
    }
    // Get local planner (defaulting to straight line if not specified)
    typename MPTraits::MPLibrary::LocalPlannerPointer lp;
    try {
      lp = this->GetLocalPlanner(ei->property().GetLPLabel());
    }
    catch(...) {
      lp = this->GetLocalPlanner("sl");
    }
    // Get intermediates representing each tick
    vector<CfgType> inter = ei->property().GetIntermediates();
    inter.insert(inter.begin(), g->GetVertex(*it));
    CfgType& e = g->GetVertex(*(it+1));
    inter.push_back(e);
    // Construct path along the edge at resolution level
    vector<CfgType> edgeCfg;
    for(auto cit = inter.begin(); cit + 1 != inter.end(); ++cit) {
      vector<CfgType> edge = lp->ReconstructPath(
          *cit,
          *(cit+1),
          vector<CfgType>(),
          this->GetEnvironment()->GetPositionRes(),
          this->GetEnvironment()->GetOrientationRes()
          );
          edgeCfg.insert(edgeCfg.end(), edge.begin(), edge.end());
      }
      edgeCfg.push_back(e);

      // Validate each configuration along the edge
      for(auto& c: edgeCfg) {
          if(!ValidCfg(c, ++tick)) {
              return false;
          }
      }
  }
  _startTick = tick;
  return true;
}


template <class MPTraits>
bool
MorseWaitQuery<MPTraits>::
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
//	CollisionDetectionMethod* m_cdMethod;
//	if (m_vcLabel == "Rapid") {
//    	m_cdMethod = new Rapid(false);
//	} else if (m_vcLabel == "RapidSolid") {
//    	m_cdMethod = new Rapid(true);
//	}
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


template<class MPTraits>
std::pair<typename MPTraits::RoadmapType::VID,typename MPTraits::RoadmapType::VID>
MorseWaitQuery<MPTraits>::
GetNextPathRegion(const VID _start, Path* _p, vector<VID>& _segment){// Function gets next path region between critical points
    // Get all vertex IDs from the path
    auto vids = _p->VIDs();
    // Flag to track if starting vertex is found
    bool foundStart = false;
    // Get reference to roadmap graph
    auto gr = this->GetRoadmap()->GetGraph();
    // Vector to store critical point IDs
    vector <VID> criticals;
    // Look into each vid in the path
    for(auto id : vids) {
        // Check if current vertex is the start point we're looking for
        if(!foundStart && id == _start){ // ignore vids if start is not found
            // Mark start found and add to segment
            foundStart = true;
            _segment.push_back(id);
            // If vertex is a critical point, store its critical ID
            if(gr->GetVertex(id).IsStat("Critical")){
                criticals.push_back(gr->GetVertex(id).GetStat("Critical"));
            }
        } else if(foundStart){  // Process vertices after start is found
            // Add vertex to current segment
            _segment.push_back(id);
            // Get vertex properties
            auto vp = gr->GetVertex(id);
            // Get last critical point ID found
            auto lastCritical = criticals.back();
            // Check if current vertex is a critical point
            if(vp.IsStat("Critical")){
                // If different from last critical point
                if(lastCritical != vp.GetStat("Critical")){
                    // Add new critical point and end segment
                    criticals.push_back(vp.GetStat("Critical"));
                    break;
                }
            }
            else
                std::cout<<"No critical label"<<std::endl;
            // Break if vertex is feasible critical point in different region
            if(vp.IsLabel("FeasibleCritical") && vp.GetLabel("FeasibleCritical") && criticals.back() != lastCritical)
                break;
        }
    }
    // Return empty pair if no critical points found
    if(criticals.empty()) return make_pair(VID(), VID());
    // Return first and last critical points found
    else
        return make_pair(criticals.front(), criticals.back());
}
#endif