#ifndef DECOMPOSITION_PRM_H_
#define DECOMPOSITION_PRM_H_

#include "MPStrategyMethod.h"
#include "Geometry/Boundaries/Boundary.h"
#include "Geometry/Boundaries/WorkspaceBoundingBox.h"
#include "Workspace/OctreeDecomposition.h"
////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @brief Basic PRM approach
///
/// DecompositionPRM essentially combines samplers and connectors to iteratively
/// construct a roadmap until planning is "done"
///
/// \internal This strategy is configured for pausible execution.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class DecompositionPRM : public MPStrategyMethod<MPTraits> {
  public:

    enum Start {Sampling, Connecting, ConnectingComponents, Evaluating};

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::GraphType GraphType;
    typedef typename RoadmapType::VID       VID;

    DecompositionPRM(
        const map<string, pair<size_t, size_t> >& _samplerLabels = map<string, pair<size_t, size_t> >(),
        const vector<string>& _connectorLabels = vector<string>(),
        const vector<string>& _componentConnectorLabels = vector<string>(),
        const vector<string>& _evaluatorLabels = vector<string>(),
        string _inputMapFilename = "",
        Start _startAt = Sampling);
    DecompositionPRM(XMLNode& _node);
    virtual ~DecompositionPRM() {}

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

    map<string, pair<size_t, size_t> > m_samplerLabels; ///< Sampler labels with number and attempts of sampler
    vector<string> m_connectorLabels; ///< Connector labels for node-to-node
    vector<string> m_componentConnectorLabels; ///< Connector labels for cc-to-cc
    size_t m_currentIteration; ///< Current iteration of while-loop of Run function
    string m_inputMapFilename; ///< Input roadmap to initialize map
    Start m_startAt; ///< When inputting a roadmap, specifies where in algorithm to start

		// Decomposition status
		size_t m_level; // store the current iteration of the level 
		double m_length; // length of the voxel map
		vector<Boundary*> m_boundingBoxes; //stores the bounding box of that level
		vector<size_t> m_levelNos; // level nos of the boxes stored in the bounding boxes vector
		void InitializeDecomposition();
};

template <typename MPTraits>
DecompositionPRM<MPTraits>::
DecompositionPRM(const map<string, pair<size_t, size_t> >& _samplerLabels,
    const vector<string>& _connectorLabels,
    const vector<string>& _componentConnectorLabels,
    const vector<string>& _evaluatorLabels,
    string _inputMapFilename, Start _startAt) :
    m_samplerLabels(_samplerLabels), m_connectorLabels(_connectorLabels),
    m_componentConnectorLabels(_componentConnectorLabels),
    m_currentIteration(0), m_inputMapFilename(_inputMapFilename),
    m_startAt(_startAt), m_level(0), m_length(0){
  this->m_meLabels = _evaluatorLabels;
  this->SetName("DecompositionPRM");
}

template <typename MPTraits>
DecompositionPRM<MPTraits>::
DecompositionPRM(XMLNode& _node) : MPStrategyMethod<MPTraits>(_node),
    m_currentIteration(0), m_inputMapFilename(""), m_startAt(Sampling) {
  this->SetName("DecompositionPRM");
  ParseXML(_node);
}

template <typename MPTraits>
void
DecompositionPRM<MPTraits>::
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
	m_length = _node.Read("length", true,
		0.1, 0.0, MAX_DBL, "Voxel length");
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
}

template <typename MPTraits>
void
DecompositionPRM<MPTraits>::
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

template <typename MPTraits>
void
DecompositionPRM<MPTraits>::
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
      auto evaluator = this->GetMapEvaluator(label);
      if(evaluator->HasState())
        evaluator->operator()();
    }
  }
  StatClass* stats = this->GetStatClass();
  string clockName = "Decomposition Generation";
  stats->StartClock(clockName);
  InitializeDecomposition();
  stats->StopClock(clockName);
}

template <typename MPTraits>
void
DecompositionPRM<MPTraits>::
InitializeDecomposition() {
	vector<pair<Point3d, size_t>> extremePoints;
  auto radius = this->GetTask()->GetRobot()->GetMultiBody()->GetFreeBody(0)->GetBoundingSphereRadius();
	OctreeDecomposition oct(this->GetEnvironment(), m_length, this->GetStatClass());
	oct.GetFreeRegions(extremePoints,radius);
  
	auto boundary = this->GetEnvironment()->GetBoundary();
	auto dimension = boundary->GetDimension();
	Point3d maxP, minP;
	for(size_t i = 0; i< dimension; ++i){
		maxP[i] = boundary->GetRange(i).max;
    minP[i] = boundary->GetRange(i).min;
  }
	auto length = oct.GetLength();

	for (auto pts : extremePoints) {
		auto bndry = new WorkspaceBoundingBox(dimension);
		double len = length / pow(2, pts.second);
    double minRange = numeric_limits<double>::max();
		for (size_t i = 0; i < dimension; ++i) {
			bndry->SetRange(i, pts.first[i], min(pts.first[i] + len, maxP[i]));
      minRange = min(minRange,min(pts.first[i] + len, maxP[i]) - pts.first[i]);
      if(minRange <= 2*radius){
        bndry->SetRange(i, max(pts.first[i]-2*radius,minP[i]), min(pts.first[i] + len + 2*radius, maxP[i]));
      }
    }
		m_boundingBoxes.push_back(bndry);
		m_levelNos.push_back(pts.second);
	}
	m_level = 0; 
}


template <typename MPTraits>
void
DecompositionPRM<MPTraits>::
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
}

template <typename MPTraits>
void
DecompositionPRM<MPTraits>::
Finalize() {
  // Output final map.
  this->GetRoadmap()->Write(this->GetBaseFilename() + ".map",
      this->GetEnvironment());

  // Output stats.
  ofstream  osStat(this->GetBaseFilename() + ".stat");
  StatClass* stats = this->GetStatClass();
  stats->PrintAllStats(osStat, this->GetRoadmap());
}

template <typename MPTraits>
template<typename OutputIterator>
void
DecompositionPRM<MPTraits>::
Sample(OutputIterator _thisIterationOut) {
  if(this->m_debug)
    cout << this->GetNameAndLabel() << "::Sample()";

  StatClass* stats = this->GetStatClass();
  string clockName = "Total Node Generation";
  stats->StartClock(clockName);

  //For each sampler generate nodes into samples
  vector<CfgType> samples;
  size_t k = 0;
  for(auto&  sampler : m_samplerLabels) {
    auto s = this->GetSampler(sampler.first);

    stats->StartClock(s->GetNameAndLabel());
    k = 0;
		for (size_t i = m_level; i < m_boundingBoxes.size() 
			   && m_levelNos[i] == m_levelNos[m_level]; i++) {
			s->Sample(sampler.second.first, sampler.second.second,
				m_boundingBoxes[i], back_inserter(samples));
			k++;
		}

    stats->StopClock(s->GetNameAndLabel());
  }
	// next level
	m_level += k;
	if(m_level >= m_boundingBoxes.size()) m_level = 0;

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

template <typename MPTraits>
template<class InputIterator>
void
DecompositionPRM<MPTraits>::
Connect(InputIterator _first, InputIterator _last, const vector<string>& _labels) {
  if(this->m_debug)
    cout << this->GetNameAndLabel() << "::Connect()";

  StatClass* stats = this->GetStatClass();
  string clockName = "Total Connection";
  stats->StartClock(clockName);

  typedef vector<string>::const_iterator SIT;
  for(SIT sit = _labels.begin(); sit != _labels.end(); ++sit){
    auto c = this->GetConnector(*sit);

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

template <typename MPTraits>
template<class InputIterator>
void
DecompositionPRM<MPTraits>::
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
