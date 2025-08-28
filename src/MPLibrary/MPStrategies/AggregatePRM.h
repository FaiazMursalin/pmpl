#ifndef AGGREGATE_PRM_H_
#define AGGREGATE_PRM_H_

#include "MPStrategyMethod.h"
#include "AggregateHierarchy.h"
#include "Environment/BoundingBox.h"
#include "Environment/BoundingBox2D.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @brief Basic PRM approach
/// @tparam MPTraits Motion planning universe
///
/// AggregatePRM essentially combines samplers and connectors to iteratively
/// construct a roadmap until planning is "done"
///
/// \internal This strategy is configured for pausible execution.
////////////////////////////////////////////////////////////////////////////////
//using namespace aggregation;

template<class MPTraits>
class AggregatePRM : public MPStrategyMethod<MPTraits> {
  public:

    enum Start {Sampling, Connecting, ConnectingComponents, Evaluating};

    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::SamplerPointer SamplerPointer;
    typedef typename MPProblemType::ConnectorPointer ConnectorPointer;
    typedef typename MPProblemType::MapEvaluatorPointer MapEvaluatorPointer;

    AggregatePRM(
        const map<string, pair<size_t, size_t> >& _samplerLabels = map<string, pair<size_t, size_t> >(),
        const vector<string>& _connectorLabels = vector<string>(),
        const vector<string>& _componentConnectorLabels = vector<string>(),
        const vector<string>& _evaluatorLabels = vector<string>(),
        string _inputMapFilename = "",
        Start _startAt = Sampling);
    AggregatePRM(typename MPTraits::MPProblemType* _problem, XMLNode& _node);
    virtual ~AggregatePRM() {}

    virtual void ParseXML(XMLNode& _node);
    virtual void Print(ostream& _os) const;

    virtual void Initialize();
    virtual void Iterate();
    virtual void Finalize();

  protected:
    ////////////////////////////////////////////////////////////////////////////
    /// @brief Sample and add configurations to the roadmap.
    /// @tparam OutputIterator Output iterator on data structure of VIDs
    /// @param[out] _thisIterationOut Data structure of VIDs of added nodes.
    template <typename OutputIterator>
      void Sample(OutputIterator _thisIterationOut);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Connect nodes and CCs of the roadmap
    /// @tparam InputIterator Iterator on data structure of VIDs/graph nodes
    /// @param _first Begin iterator over VIDs/graph nodes
    /// @param _last End iterator over VIDs/graph nodes
    /// @param _labels Connector labels used in connection
    template<class InputIterator>
      void Connect(InputIterator _first, InputIterator _last,
          const vector<string>& _labels);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Iterate over range and check nodes to be within narrow passage
    /// @tparam InputIterator Iterator on data structure of VIDs
    /// @param _first Begin iterator over VIDs
    /// @param _last End iterator over VIDs
    template<class InputIterator>
      void CheckNarrowPassageSamples(InputIterator _first, InputIterator _last);
		
		void AggregateInitialize();

    void SetViewSimplices(const vector<CfgType>& _cfgs);

    template<class InputIterator>
		  void UpdateViewSimplices(InputIterator _first, InputIterator _last);
    template<class InputIterator>
      void FindMinDistance(CfgType& _c, CfgType& _o, InputIterator _first, InputIterator _last);

    map<string, pair<size_t, size_t> > m_samplerLabels; ///< Sampler labels with number and attempts of sampler
    vector<string> m_connectorLabels; ///< Connector labels for node-to-node
    vector<string> m_componentConnectorLabels; ///< Connector labels for cc-to-cc
    size_t m_currentIteration; ///< Current iteration of while-loop of Run function
    string m_inputMapFilename; ///< Input roadmap to initialize map
    Start m_startAt; ///< When inputting a roadmap, specifies where in algorithm to start
	private:
		vector<shared_ptr<Boundary> > m_boundingBoxes; //stores the bounding box of that level
		size_t m_level;
    size_t m_method;    //< 0-No refinement, 1-Manual Viewpoint, 3-Manual Viewsegment, 2-Automatic Viewpoint, 6-Automatic Segments 
		int m_dimension;
		double m_volume;
		double m_range;
    bool m_remains;
		shared_ptr<PRMQuery<MPTraits> > m_query;
    vector<Vector3d> m_viewPoints;
		vector<pair<Vector3d,Vector3d> > m_viewSegments;
		aggregation::AggregateHierarchy* m_hierarchy;
};

template<class MPTraits>
AggregatePRM<MPTraits>::
AggregatePRM(const map<string, pair<size_t, size_t> >& _samplerLabels,
    const vector<string>& _connectorLabels,
    const vector<string>& _componentConnectorLabels,
    const vector<string>& _evaluatorLabels,
    string _inputMapFilename, Start _startAt) :
    m_samplerLabels(_samplerLabels), m_connectorLabels(_connectorLabels),
    m_componentConnectorLabels(_componentConnectorLabels),
    m_currentIteration(0), m_inputMapFilename(_inputMapFilename),
    m_startAt(_startAt),m_level(0),m_method(0),m_dimension(2),
    m_volume(0),m_remains(false),m_query((PRMQuery<MPTraits>*)NULL),
	m_hierarchy((aggregation::AggregateHierarchy*)NULL){
  this->m_meLabels = _evaluatorLabels;
  this->SetName("AggregatePRM");
}

template<class MPTraits>
AggregatePRM<MPTraits>::
AggregatePRM(typename MPTraits::MPProblemType* _problem, XMLNode& _node) :
    MPStrategyMethod<MPTraits>(_problem, _node), m_currentIteration(0),
    m_inputMapFilename(""), m_startAt(Sampling),
    m_level(0), m_method(0), m_dimension(2), m_volume(0),m_remains(false),m_query((PRMQuery<MPTraits>*)NULL),
	m_hierarchy((aggregation::AggregateHierarchy*)NULL) {
  this->SetName("AggregatePRM");
  ParseXML(_node);
}

template<class MPTraits>
void
AggregatePRM<MPTraits>::
ParseXML(XMLNode& _node) {
  m_inputMapFilename = _node.Read("inputMap", false, "",
      "filename of roadmap to start from");
  string startAt = _node.Read("startAt", false, "sampling",
      "point of algorithm where to begin at: \
      \"sampling\" (default), \"connecting\", \
      \"connectingcomponents\", \"evaluating\"");
  if(startAt == "sampling")
    m_startAt = Sampling;
  else if(startAt == "connecting")
    m_startAt = Connecting;
  else if(startAt == "connectingcomponents")
    m_startAt = ConnectingComponents;
  else if(startAt == "evaluating")
    m_startAt = Evaluating;
  else  {
    string message = "Start at is '" + startAt +
      "'. Choices are 'sampling', 'connecting', 'connectingComponents', 'evaluating'.";
    throw ParseException(_node.Where(), message);
  }

  for(auto& child : _node) {
    if(child.Name() == "Sampler") {
      string s = child.Read("method", true, "", "Sampler Label");
      size_t num = child.Read("number", true,
          1, 0, MAX_INT, "Number of samples");
      size_t attempts = child.Read("attempts", false,
          1, 0, MAX_INT, "Number of attempts per sample");
      m_samplerLabels[s] = make_pair(num, attempts);
    }
    else if(child.Name() == "Connector")
      m_connectorLabels.push_back(
          child.Read("method", true, "", "Connector Label"));
    else if(child.Name() == "ComponentConnector")
      m_componentConnectorLabels.push_back(
          child.Read("method", true, "", "Component Connector Label"));
    else if(child.Name() == "Evaluator")
      this->m_meLabels.push_back(
          child.Read("method", true, "", "Evaluator Label"));
  }
	//optionally read in a query and create a Query object.
  m_method = _node.Read("method", false, 0, 0, MAX_INT, "Method of aggregation");
  string query = _node.Read("query", false, "", "Query Filename");
  if(query != "") {
    m_query = shared_ptr<PRMQuery<MPTraits>>(new PRMQuery<MPTraits>());
		m_query->ReadQuery(query);
    m_query->SetMPProblem(this->GetMPProblem());
    m_query->SetDebug(this->m_debug);
  }
	else
		m_query = NULL;
}

template<class MPTraits>
void
AggregatePRM<MPTraits>::
Print(ostream& _os) const {
  MPStrategyMethod<MPTraits>::Print(_os);
  _os << "\tInput Map: " << m_inputMapFilename << endl;

  _os << "\tStart At: ";
  switch(m_startAt) {
    case Sampling: _os << "sampling"; break;
    case Connecting: _os << "connecting"; break;
    case ConnectingComponents: _os << "connectingcomponents"; break;
    case Evaluating: _os << "evaluating"; break;
  }
  cout << endl;

  _os << "\tSamplers" << endl;
  for(const auto& label : m_samplerLabels)
    _os << "\t\t" << label.first
        << "\tNumber:"   << label.second.first
        << "\tAttempts:" << label.second.second
        << endl;

  _os << "\tConnectors" << endl;
  for(const auto& label : m_connectorLabels)
    _os << "\t\t" << label << endl;

  _os << "\tComponentConnectors" << endl;
  for(const auto& label : m_componentConnectorLabels)
    _os << "\t\t" << label << endl;

  _os<<"\tMapEvaluators" << endl;
  for(const auto& label : this->m_meLabels)
    _os << "\t\t" << label << endl;
}

//Initialize for aggregation
template<class MPTraits>
void
AggregatePRM<MPTraits>::
AggregateInitialize() {
	vector<vector<vector<int> > > polygonList;
	vector<vector<Vector3d> > vertexList;
	vector<pair<double,double> > environmentBox;
	vector<Vector3d> viewPoints;
	
	Environment* env = this->GetEnvironment();
	size_t numObjects = env->NumObstacles();
	m_range = env->GetRobot(0)->GetMaxAxisRange();
	
	shared_ptr<Boundary> boundingBox = env->GetBoundary();
	m_dimension = 3;
	if(boundingBox->Type().compare("Box") == 0 || boundingBox->Type().compare("Box2D") == 0)	{
		m_dimension = (boundingBox->Type().compare("Box2D") == 0)? 2:3;
		for(size_t i=0; i<m_dimension; i++)
			environmentBox.push_back(boundingBox->GetRange(i));
		m_volume = 1;
		for(size_t i=0; i<m_dimension; i++)
			m_volume *= (environmentBox[i].second - environmentBox[i].first);
	}
	cout<<"Dimension of environment "<<m_dimension<<endl;
	for(size_t i=0; i<numObjects; ++i)	 {
		shared_ptr<StaticMultiBody> obstacle = env->GetObstacle(i);
		GMSPolyhedron& obstaclePolyhedron = obstacle->GetFixedBody(0)->GetWorldPolyhedron();
		vertexList.push_back(obstaclePolyhedron.GetVertexList());
		vector<GMSPolygon>& polygons = obstaclePolyhedron.GetPolygonList();
		vector<vector<int> > obstaclePolygons;
		for(size_t j=0; j<polygons.size(); j++)
			obstaclePolygons.push_back(polygons[j].GetVertexList());
		polygonList.push_back(obstaclePolygons);
	}
	StatClass* stats = this->GetStatClass();
	string aggName = "Aggregation Hierarchy";
  	stats->StartClock(aggName);

	m_hierarchy = new aggregation::AggregateHierarchy();
  m_hierarchy->SetError((m_range*m_range),0);
	m_hierarchy->SetDimension(m_dimension);
	m_hierarchy->SetEnvBox(environmentBox);
	if(m_method>0 && m_query != NULL) {
		SetViewSimplices(m_query->GetQuery());
    if(!m_viewPoints.empty())
		  m_hierarchy->SetViewSimplices(m_viewPoints);
    if(!m_viewSegments.empty())
		  m_hierarchy->SetViewSimplices(m_viewSegments);
	}
	m_hierarchy->InitializeHierarchy(polygonList,vertexList);
	m_level = 0; //Initialize level to 0
	cout<<"Created Hierarchy"<<endl;

}

template<class MPTraits>
void
AggregatePRM<MPTraits>::SetViewSimplices(const vector<CfgType>& _cfgs) {
  if(m_method == 1 || m_method == 2) { //Viewpoints method
    for(auto&  cfg :  _cfgs)	{
			vector<double> positions = cfg.GetPosition();
			if(m_dimension == 2 && positions.size() >= 2)
				m_viewPoints.push_back(Vector3d(positions[0],positions[1],0));
			else if(m_dimension == 3 && positions.size() >= 3)
				m_viewPoints.push_back(Vector3d(positions[0],positions[1],positions[2]));
			}
  }
  if(_cfgs.size() > 1 && (m_method == 3 || m_method == 6))  { //Viewsegments method
    for(auto cfgit = _cfgs.begin(); cfgit != _cfgs.end()-1; cfgit++)  {
      vector<double> first = cfgit->GetPosition();
      vector<double> second = (cfgit+1)->GetPosition();
      if(m_dimension == 2 && first.size() >= 2 && second.size() >= 2)
				m_viewSegments.push_back(make_pair(Vector3d(first[0],first[1],0),Vector3d(second[0],second[1],0)));
			else if(m_dimension == 3 && first.size() >= 3 && second.size() >= 3)
				m_viewSegments.push_back(make_pair(Vector3d(first[0],first[1],first[2]),Vector3d(second[0],second[1],second[2])));
    }
  }
}

template<class MPTraits>
template<class InputIterator>
void
AggregatePRM<MPTraits>::UpdateViewSimplices(InputIterator _first, InputIterator _last) {
  vector<CfgType> qry = m_query->GetQuery();
  CfgType src = qry[0];
  CfgType goal = qry.back();
  GraphType* g = this->GetRoadmap()->GetGraph();
  VID sVID, gVID;
	stapl::sequential::vector_property_map<GraphType, size_t> cmap; //color map
	vector<VID> cc; //output connected component
	vector<pair<CfgType,CfgType> > temp;

	//temp.push_back(make_pair(src,src));
  
	//Find the point to in Start CC from goal 
	if(g->IsVertex(src))	{
		sVID = g->GetVID(src);
		cmap.reset();
		if(g->IsVertex(goal) && stapl::sequential::is_same_cc(*(g), cmap, sVID, g->GetVID(goal))){}
		else {
			//If goal is a vertex and is not in the same cc as the src
			cmap.reset();
			cc.clear();
			stapl::sequential::get_cc(*(g), cmap, sVID, cc);
			CfgType sGoal;
			FindMinDistance(goal, sGoal, cc.begin(), cc.end());
			if(m_method == 2)
				temp.push_back(make_pair(sGoal,src));
			else {
				if(g->IsVertex(goal)) {
					cmap.reset();
					cc.clear();
					stapl::sequential::get_cc(*(g), cmap, g->GetVID(goal), cc);
					CfgType g1;
					FindMinDistance(sGoal, g1, cc.begin(), cc.end());
					temp.push_back(make_pair(sGoal,g1));
				}
				else
					temp.push_back(make_pair(sGoal,goal));
			}
		}
	}

	if(g->IsVertex(goal))	{
		gVID = g->GetVID(goal);
		cmap.reset();
		if(g->IsVertex(src) && stapl::sequential::is_same_cc(*(g), cmap, gVID, g->GetVID(src))){} 
		else {
			//If goal is a vertex and is not in the same cc as the src
			cmap.reset();
			cc.clear();
			stapl::sequential::get_cc(*(g), cmap, gVID, cc);
			CfgType gSrc;
			FindMinDistance(src, gSrc, cc.begin(), cc.end());
			
			if(m_method == 2)
				temp.push_back(make_pair(gSrc,src));
			else {
				if(!temp.empty()) temp.push_back(make_pair(temp.back().first,gSrc));
				if(g->IsVertex(src)) {
					cmap.reset();
					cc.clear();
					stapl::sequential::get_cc(*(g), cmap, g->GetVID(src), cc);
					CfgType s1;
					FindMinDistance(gSrc, s1, cc.begin(), cc.end());
					temp.push_back(make_pair(s1,gSrc));
				}
				else
					temp.push_back(make_pair(src,gSrc));
			}
		}
	}
 // temp.push_back(make_pair(goal,1));
  
  if(m_method == 2) {
		m_viewPoints.clear();
		for(auto i=0; i<qry.size(); i++)
			temp.push_back(make_pair(qry[i],qry[i]));
		for(auto& t: temp)	{
			vector<double> pos= t.first.GetPosition();
			if(m_dimension == 2 && pos.size() >= 2)
				m_viewPoints.push_back(Vector3d(pos[0],pos[1],0));
			if(m_dimension == 3 && pos.size() >= 3)
				m_viewPoints.push_back(Vector3d(pos[0],pos[1],pos[2]));
		}
    m_hierarchy->UpdateViewSimplices(m_viewPoints);
  }
  if(m_method == 6) {
		m_viewSegments.clear();
		for(auto i=0; i<qry.size()-1; i++)
			temp.push_back(make_pair(qry[i],qry[i+1]));
		for(auto t = temp.begin(); t != temp.end(); t++) {
				vector<double> p1, p2;
				p1 = t->first.GetPosition();
				p2 = t->second.GetPosition();
				if(m_dimension == 2 && p1.size() >= 2 && p2.size() >= 2)
					m_viewSegments.push_back(make_pair(Vector3d(p1[0],p1[1],0),Vector3d(p2[0],p2[1],0)));
				if(m_dimension == 3 && p1.size() >= 3 && p2.size() >= 3)
					m_viewSegments.push_back(make_pair(Vector3d(p1[0],p1[1],p1[2]),Vector3d(p2[0],p2[1],p2[2])));
			}
		m_hierarchy->UpdateViewSimplices(m_viewSegments);
  }
}

template<class MPTraits>
template<class InputIterator>
void
AggregatePRM<MPTraits>::FindMinDistance(CfgType& _c, CfgType& _o, InputIterator _first, InputIterator _last)  {
  double minD = -1;
  //CfgType minV;
  vector<double> p = _c.GetPosition();
  
  for (InputIterator v1 = _first; v1 != _last; v1++) {
    CfgType c1 = this->GetRoadmap()->GetGraph()->GetVertex(v1);
    vector<double> pos = c1.GetPosition();
    double dist;
    if(m_dimension == 2 && pos.size() >= 2)
      dist = (Vector3d(pos[0],pos[1],0) - Vector3d(p[0],p[1],0)).norm() ;
    else if(m_dimension == 3 && pos.size() >= 3)
      dist = (Vector3d(pos[0],pos[1],pos[2]) - Vector3d(p[0],p[1],p[2])).norm();
    else
      continue;
    if(minD == -1 || minD > dist) {
      minD = dist;
      _o = c1;
    }
  }
 
}

/* -- Old automatic viewpoints
template<class MPTraits>
void
AggregatePRM<MPTraits>::UpdateViewSimplices(vector<CfgType>& _cfgs) {
  if(_cfgs.empty() || m_viewPoints.size() < 2) return;
  size_t last = m_viewPoints.size() - 1;
  double minD;
  if(last >= 2)
    minD = (m_viewPoints[last] - m_viewPoints[last-1]).norm() + (m_viewPoints[last] - m_viewPoints[0]).norm();
  else
    minD = -1;
  for(auto&  cfg :  _cfgs)	{
			vector<double> positions = cfg.GetPosition();
			if(m_dimension == 2 && positions.size() >= 2)
				m_viewPoints.push_back(Vector3d(positions[0],positions[1],0));
			else if(m_dimension == 3 && positions.size() >= 3)
				m_viewPoints.push_back(Vector3d(positions[0],positions[1],positions[2]));
      double dist = (m_viewPoints[last+1] - m_viewPoints[last-1]).norm() + (m_viewPoints[last+1] - m_viewPoints[0]).norm();
      if(minD == -1 || dist < minD) {
        if(minD != -1) {
          m_viewPoints.erase(m_viewPoints.begin()+last);
        }
        minD = dist;
      }
      else
        m_viewPoints.pop_back();
		}
  m_hierarchy->UpdateViewSimplices(m_viewPoints);
}
*/

template<class MPTraits>
void
AggregatePRM<MPTraits>::
Initialize() {
  if(this->m_debug)
    cout << this->GetNameAndLabel() << "::Initialize()" << endl;

	

  //read in and reload roadmap and evaluators
  if(!m_inputMapFilename.empty()) {
    RoadmapType* r = this->GetRoadmap();
    if(this->m_debug)
      cout << "Loading roadmap from \"" << m_inputMapFilename << "\".";

    r->Read(m_inputMapFilename.c_str());

    GraphType* g = r->GetGraph();
    for(typename GraphType::VI vi = g->begin(); vi != g->end(); ++vi)
      VDAddNode(g->GetVertex(vi));
    if(this->m_debug) {
      cout << "Roadmap has " << g->get_num_vertices() << " nodes and "
           << g->get_num_edges() << " edges." << endl;
      cout << "Resetting map evaluator states." << endl;
    }

    for(const auto& label: this->m_meLabels) {
      MapEvaluatorPointer evaluator = this->GetMapEvaluator(label);
      if(evaluator->HasState())
        evaluator->operator()();
    }
  }

   AggregateInitialize();
}


template<class MPTraits>
void
AggregatePRM<MPTraits>::
Iterate() {
  m_currentIteration++;
  vector<VID> vids;

  switch(m_startAt) {

    case Sampling:
      Sample(back_inserter(vids));

    case Connecting:
      {
        if(m_startAt == Connecting) {
          GraphType* g = this->GetRoadmap()->GetGraph();
          Connect(g->begin(), g->end(), m_connectorLabels);
          //For spark prm to grow RRT at difficult nodes
          CheckNarrowPassageSamples(g->begin(), g->end());
        }
        else {
          Connect(vids.begin(), vids.end(), m_connectorLabels);
          //For spark prm to grow RRT at difficult nodes
          CheckNarrowPassageSamples(vids.begin(), vids.end());
        }
      }

    case ConnectingComponents:
      {
        GraphType* g = this->GetRoadmap()->GetGraph();
        Connect(g->begin(), g->end(), m_componentConnectorLabels);
      }

    default:
      break;
  }
  m_startAt = Sampling;
  if(m_method > 0 && m_method%2 == 0 && m_remains && !vids.empty()) {
    GraphType* g = this->GetRoadmap()->GetGraph();
    UpdateViewSimplices(g->begin(), g->end());
  }
}

template<class MPTraits>
void
AggregatePRM<MPTraits>::
Finalize() {
  //output final map
  this->GetRoadmap()->Write(this->GetBaseFilename() + ".map",
      this->GetEnvironment());

  //output stats
  string str = this->GetBaseFilename() + ".stat";
  ofstream  osStat(str.c_str());
  StatClass* stats = this->GetStatClass();
  stats->PrintAllStats(osStat, this->GetRoadmap());
}

template<class MPTraits>
template<typename OutputIterator>
void
AggregatePRM<MPTraits>::
Sample(OutputIterator _thisIterationOut) {
  if(this->m_debug)
    cout << this->GetNameAndLabel() << "::Sample()";

  StatClass* stats = this->GetStatClass();
  string clockName = "Total Node Generation";
  stats->StartClock(clockName);

  //For each sampler generate nodes into samples
  vector<CfgType> samples;
	m_boundingBoxes.clear();
  //bool remains = false;
	//m_boundingBoxes.push_back(this->m_boundary); /*Remove this*/
	//Get the bounding boxes for the current level
	if(m_level >= m_hierarchy->GetNoLevels())	{
    if(m_hierarchy->GetMaxLevel() == m_hierarchy->GetNoLevels()) //max level reached
		  m_level = 0;
    else {
      m_remains = m_hierarchy->CreateNextLevel();
      if(m_remains) {
		
		string aggName = "Aggregation Hierarchy";
  		stats->StopClock(aggName);
	}
	else 
		cout<<"Aggregation Done"<<endl;
    }
  }
	size_t levelBoxNos = m_hierarchy->GetNumberOfBoxes(m_level);
	for(size_t i=0; i<levelBoxNos; i++)	{
		pair<double,double> xrange = m_hierarchy->GetBoxRange(m_level,i,0);
		pair<double,double> yrange = m_hierarchy->GetBoxRange(m_level,i,1);
		
		if((xrange.second- xrange.first) <= 2*m_range)	{
			xrange.first -= m_range;
			xrange.second += m_range;
		}
		if((yrange.second- yrange.first) <= 2*m_range)	{
			yrange.first -= m_range;
			yrange.second += m_range;
		}
		if(m_dimension == 2)
			m_boundingBoxes.push_back(shared_ptr<BoundingBox2D>(
				new BoundingBox2D(xrange,yrange)));
		else	{
			pair<double,double> zrange = m_hierarchy->GetBoxRange(m_level,i,2);
			if((zrange.second- zrange.first) <= 2*m_range)	{
				zrange.first -= m_range;
				zrange.second += m_range;
			}
			m_boundingBoxes.push_back(shared_ptr<BoundingBox>(
				new BoundingBox(xrange,yrange,zrange)));
		}
	}
	//calculate the volumes
	vector<double> volumes;
	for(size_t i=0; i<m_boundingBoxes.size(); i++)	{
		volumes.push_back(1);
		for(size_t j=0; j<m_dimension; j++)
			volumes[i] *= (m_boundingBoxes[i]->GetRange(j).second 
				- m_boundingBoxes[i]->GetRange(j).first);
		volumes[i] /= m_volume;
	}
	
  	for(auto&  sampler : m_samplerLabels) {
    	SamplerPointer s = this->GetSampler(sampler.first);

    	stats->StartClock(s->GetNameAndLabel());
			for(size_t i=0; i<m_boundingBoxes.size(); i++)	{
				size_t sampleNos = static_cast<size_t> (volumes[i]*sampler.second.first); 
    		if(sampleNos == 0) sampleNos = sampler.second.first;
		
		s->Sample(sampleNos, sampler.second.second,
        	m_boundingBoxes[i], back_inserter(samples));
			}

    	stats->StopClock(s->GetNameAndLabel());
  	}
		m_level++;

  if(this->m_debug && samples.empty())
    cout << "No samples generated." << endl;
 
  //add valid samples to roadmap
  GraphType* g = this->GetRoadmap()->GetGraph();
  for(auto&  sample: samples) {
    VID vid = g->AddVertex(sample);
    *_thisIterationOut++ = vid;
  }

  

  stats->StopClock(clockName);
  if(this->m_debug) {
    cout << this->GetNameAndLabel() << " has "
      << g->get_num_vertices() << " total vertices. Time: " << endl;
    this->GetStatClass()->PrintClock(clockName, cout);
  }
}

template<class MPTraits>
template<class InputIterator>
void
AggregatePRM<MPTraits>::
Connect(InputIterator _first, InputIterator _last, const vector<string>& _labels) {
  if(this->m_debug)
    cout << this->GetNameAndLabel() << "::Connect()";

  StatClass* stats = this->GetStatClass();
  string clockName = "Total Connection";
  stats->StartClock(clockName);

  typedef vector<string>::const_iterator SIT;
  for(SIT sit = _labels.begin(); sit != _labels.end(); ++sit){
    ConnectorPointer c = this->GetConnector(*sit);

    stats->StartClock(c->GetNameAndLabel());

    c->Connect(this->GetRoadmap(), _first, _last);

    stats->StopClock(c->GetNameAndLabel());
  }

  stats->StopClock(clockName);
  if(this->m_debug) {
    GraphType* g = this->GetRoadmap()->GetGraph();
    cout << this->GetNameAndLabel() << " has "
      << g->get_num_edges() << " edges and "
      << g->GetNumCCs() << " connected components. Time: " << endl;
    stats->PrintClock(clockName, cout);
  }
}

template<class MPTraits>
template<class InputIterator>
void
AggregatePRM<MPTraits>::
CheckNarrowPassageSamples(InputIterator _first, InputIterator _last) {
  if(this->m_debug)
    cout << this->GetNameAndLabel() << "::CheckNarrowPassageSamples()";
   
  for(; _first != _last; _first++) {
    VID vid = this->GetRoadmap()->GetGraph()->GetVID(_first);
    if(this->CheckNarrowPassageSample(vid))
      break;
  }
}

#endif
