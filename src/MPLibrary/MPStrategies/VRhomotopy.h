#ifndef VRHOMOTOPY_H_
#define VRHOMOTOPY_H_

#include "MPStrategyMethod.h"
#include "Geometry/Boundaries/Boundary.h"
#include "Geometry/Boundaries/CSpaceBoundingSphere.h"
#include "MPProblem/Environment/Environment.h"

#include "TomitaAlgorithm.h"
#include "HybridAlgorithm.h"

template<class MPTraits>
class VRhomotopy : public MPStrategyMethod<MPTraits> {

  public:

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::WeightType   WeightType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename MPTraits::MPLibrary 	MPLibrary;
    typedef typename RoadmapType::GraphType GraphType;
    typedef typename RoadmapType::VID       VID;
    typedef typename MPLibrary::MapEvaluatorPointer MapEvaluatorPointer;
    typedef typename MPLibrary::SamplerPointer SamplerPointer;

	VRhomotopy(const vector<string>& _evaluatorLabels = vector<string>());
    virtual ~VRhomotopy() = default;
    VRhomotopy(XMLNode& _node);


    virtual void ParseXML(XMLNode& _node);
    virtual void Print(ostream& _os) const;

    virtual void Initialize();
    virtual void Run();
    virtual void Finalize();

	// Used by morse function method
    const map<CfgType, double>& GetDistanceSet() const { return m_distanceSet; }
	const map<CfgType, int>& GetDensitySet() const { return m_densitySet; }
	const map<CfgType, vector<VID>>& GetNeighborPts() const { return m_neighborPt; }
	const set<CfgType>& GetObstaclePt() const {return m_obstaclePts;}
	GraphType* GetParentGraph() { return this->GetRoadmap()->GetGraph();}

    void DensityAndDistanceComputations();

  protected:
	// Planning helper
	void ReadMapFile(const std::string& _filename);
	double Sample(set<Point3d>& _boundaryPt);
	void Connect();
	void GetCliques(unordered_map<VID,size_t>& _vindex, list<list<int>>& _cliques);
    void ConstructObstaclePts();

	// Geometry helper functions
    void ConvexHull(vector<Point3d>& samplePoints, vector<Point3d>& hullPoints, Point3d p, Point3d q, int dir);
    double GetHausdroffDistance(vector<Point3d>& hullPoints, set<Point3d>& boundaryPoints);
	// Finds the minimum distance of the sample points from the obstacle surface
    void MinDisttoObstacle(const vector<Point3d>& SamplePoints, const set<CfgType>& boundaryPoints);
	void DensityMethod(const set<CfgType>& boundaryPoints/*, vector<VID>& collisionPts*/);

	// Members
	string m_samplerLabel;   // Label for the underlying sampler used
	string m_connectorLabel;
	string m_inputFileName;
	size_t m_number;
	size_t m_attempts;
	vector<Point3d> m_samplePt;    // Workspace counterpart of the sampled configurations
	set<CfgType> m_obstaclePts;    // Points sampled at uniformly at the surface of the obstacle
	//GraphType* graph;            // Roadmap used by the parent class
	map<CfgType, vector<VID>> m_neighborPt;  // Map of the obstacle  points to the neighbors to find critical points and feasible critical points
	map<CfgType, int> m_densitySet;      // Map of critical point to density value
	map<CfgType, double> m_distanceSet;    // Map of distance function

	size_t m_totalnum{10000};
	size_t m_obstnum{1000};
	bool m_useWorkspaceObst{false};   // Flag to use workspace geometry obstacle instead of UOBPRM sampling
};


template<class MPTraits>
VRhomotopy<MPTraits>::
VRhomotopy(const vector<string>& _evaluatorLabels){
	this->m_meLabels = _evaluatorLabels;
	this->SetName("VRhomotopy");
}


template<class MPTraits>
VRhomotopy<MPTraits>::
VRhomotopy(XMLNode& _node)
  : MPStrategyMethod<MPTraits>(_node) {
    this->SetName("VRhomotopy");
	ParseXML(_node);
  }


template<class MPTraits>
void
VRhomotopy<MPTraits>::
ParseXML(XMLNode& _node) {
  m_totalnum = _node.Read("total", false, 10000, 0, MAX_INT,
      "total number of samples");
  m_obstnum = _node.Read("obstNum", false, 10000, 0, MAX_INT,
      "total number of obstacle samples");
  for(auto& child : _node){
	if(child.Name() =="Sampler"){
	  m_samplerLabel = child.Read("method",true,"","Node Generation Method");
	  m_number = child.Read("number", true,
          1, 0, MAX_INT, "Number of samples");
      m_attempts = child.Read("attempts", false,
          1, 0, MAX_INT, "Number of attempts per sample");
	}
	if(child.Name() =="Evaluator"){
	  this->m_meLabels.push_back(child.Read("method",true,"","Evaluation Method"));
	}
	if(child.Name() =="Connector"){
	  m_connectorLabel = child.Read("method", true, "", "Connector Label");
	}
  }
  m_inputFileName = _node.Read("inputMap", false, "", "Roadmap used to preload a roadmap");
  m_useWorkspaceObst = _node.Read("workspaceObst", false, false,
      "Use workspace geometry obstacle points as obstacle points.");
}


template<class MPTraits>
void
VRhomotopy<MPTraits>::
Print(ostream& _os) const {
	MPStrategyMethod<MPTraits>::Print(_os);
	_os<<"\tSampling method: "<<m_samplerLabel<<endl;
	_os<<"\tConnector method: "<<m_connectorLabel<<endl;

	_os<<"\tMapEvaluator :";
	for(const auto& label :this->m_meLabels)
		_os<<"\t"<<label<<endl;
}


template<class MPTraits>
void
VRhomotopy<MPTraits>::
Initialize() {
	Print(cout);
	for(const auto& label: this->m_meLabels){
		MapEvaluatorPointer evaluator = this->GetMapEvaluator(label);
		if(evaluator->HasState())
			evaluator->operator()();
	}

}

template <typename MPTraits>
void
VRhomotopy<MPTraits>::
ReadMapFile(const std::string& _filename){
  //RoadmapType* roadmap = this->GetRoadmap();
  auto robType = this->GetTask()->GetRobot()->GetMultiBody()->GetBaseType();
  WeightType::inputRobot = this->GetTask()->GetRobot();

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
     CfgType data(this->GetTask()->GetRobot());
     ifs >> vid1;
     ifs >> skip;  // skip for vizmo
     ifs >> data;

     this->GetRoadmap()->GetGraph()->AddVertex(vid1, data);  // add the vertex
	 if(robType == FreeBody::BodyType::Fixed){
		data.ConfigureRobot();
		shared_ptr<vector<Vector3d> > joints = shared_ptr<vector<Vector3d> >(new vector<Vector3d>);
		data.GetMultiBody()->PolygonalApproximation(*joints);
		auto s = (*joints)[joints->size()-1];
		m_samplePt.push_back(s);
	 }
	 else
	   m_samplePt.push_back(data.GetPoint());
     size_t adjEdges;
     ifs >> adjEdges;
     for (size_t j = 0; j < adjEdges; j++){
      WeightType weight;
      ifs >> vid2 >> weight;
      this->GetRoadmap()->GetGraph()->AddEdge(vid1, vid2, weight, true);
    }
  }
  //graph = this->GetRoadmap()->GetGraph();
}

template<class MPTraits>
void
VRhomotopy<MPTraits>::
Run() {
  if(!m_inputFileName.empty()){
	ReadMapFile(m_inputFileName);
    std::cout<<"Graph vertices: "<<this->GetRoadmap()->GetGraph()->get_num_vertices()<<std::endl;
	if(m_obstaclePts.empty()) ConstructObstaclePts();
	return;
  }
  set<Point3d> boundaryPt;
  //bool mapPassedEvaluation = this->EvaluateMap();
  double epsilon = 0.0, prevE = 0.0;
  do{
	prevE = epsilon;
    epsilon = Sample(boundaryPt);
  }while(std::fabs(prevE - epsilon) > 1e-05 || this->GetRoadmap()->GetGraph()->get_num_vertices() < m_totalnum);

  //current graph for child class
  //graph = this->GetRoadmap()->GetGraph();
  Connect();
}


template<class MPTraits>
double
VRhomotopy<MPTraits>::
Sample(set<Point3d>& _boundaryPt) {
	GraphType* g = this->GetRoadmap()->GetGraph();
	//shared_ptr<Boundary> boundary;
	auto robType = this->GetTask()->GetRobot()->GetMultiBody()->GetBaseType();
	if(robType == FreeBody::BodyType::Fixed){
		double robot_rad = this->GetTask()->GetRobot()->GetMultiBody()->GetBoundingSphereRadius();
		// Convert mathtool::Vector to std::vector<double>
        mathtool::Vector<double, 3> center = this->GetTask()->GetRobot()->GetMultiBody()->GetCenterOfMass();
        std::vector<double> cntr = {center[0], center[1], center[2]};
		shared_ptr<Boundary> b(new CSpaceBoundingSphere(cntr,robot_rad));
		this->GetEnvironment()->SetBoundary(b.get());
	}

	vector<Point3d> hullPt;
	vector<CfgType> vectorCfgs;

	StatClass* stats = this->GetStatClass();
	string clockName = "Total Node Generation";
	//while(!mapPassedEvaluation){
	SamplerPointer sampler = this->GetMPLibrary()->GetSampler(m_samplerLabel);
	stats->StartClock(clockName);
	sampler->Sample(m_number, m_attempts, this->GetEnvironment()->GetBoundary(),back_inserter(vectorCfgs));
	stats->StopClock(clockName);
	for(auto& sample: vectorCfgs){
		if(sample.IsLabel("VALID") && sample.GetLabel("VALID")){
			stats->StartClock(clockName);
			VID newVID = g->AddVertex(sample);
			stats->StopClock(clockName);
			if(this->m_debug) {
				cout<<"Node vertex ID "<< newVID <<endl;
			}
			Vector3d s;
			if(robType == FreeBody::BodyType::Fixed){
				sample.ConfigureRobot();
				shared_ptr<vector<Vector3d> > joints = shared_ptr<vector<Vector3d> >(new vector<Vector3d>);
				sample.GetMultiBody()->PolygonalApproximation(*joints);
				s = (*joints)[joints->size()-1];
				_boundaryPt.insert(this->GetEnvironment()->GetBoundary()->GetClearancePoint(s));
			}
			else{
				s = sample.GetPoint();
				_boundaryPt.insert(this->GetEnvironment()->GetBoundary()->GetClearancePoint(s));
			}
			m_samplePt.push_back(s);
		}
	}
		//mapPassedEvaluation = this->EvaluateMap();
	//}

	// Change to use CGAL to compute the convex hull
	Point3d min_p, max_p;
	if(m_samplePt.size() > 3){
		min_p = m_samplePt.front();
		max_p = m_samplePt.back();
		for (typename vector<Point3d>::iterator pt=m_samplePt.begin(); pt != m_samplePt.end();++pt){
			if(*pt[0] < min_p[0])
				min_p = *pt;

			else if(*pt[0] > max_p[0])
				max_p = *pt;
		}
	}
	ConvexHull(m_samplePt, hullPt, min_p, max_p, 1); //clockwise
	ConvexHull(m_samplePt, hullPt, min_p, max_p, 2); //counterclockwise

	if(m_obstaclePts.empty()) ConstructObstaclePts();

	string tick = "Total Computation time";
	stats->StartClock(tick);

	double epsilon = GetHausdroffDistance(hullPt, _boundaryPt);
	cout<<"Hausdroff Distance: "<< epsilon <<endl;

	stats->StopClock(tick);

	int n = (vectorCfgs.front()).DOF();
	double mu = sqrt(2*n/(n+1));
	double alpha = 2*epsilon/(2-mu);
	cout<<"alpha :"<<alpha<<endl;
	//cout<<"DOF of robot is "<<n<<endl;

	return epsilon;
}

template<class MPTraits>
void
VRhomotopy<MPTraits>::
Connect(){
  auto connector = this->GetConnector(m_connectorLabel);
  auto graph = this->GetRoadmap()->GetGraph();
  connector->Connect(this->GetRoadmap(), graph->begin(), graph->end());	  // Connect them

   // get the map of vid and count - clique computration purpose
   unordered_map<VID, size_t> VIDindex;
   vector<VID> VIDs;
   size_t count = 0;
   for(auto vit = graph->begin(); vit != graph->end(); ++vit){
	 VIDindex.insert({vit->descriptor(), count++});
	 VIDs.push_back(vit->descriptor());
   }

   // get cliques
   /*vector<vector<VID>> cliques;
   vector<pair<size_t, VID>> ccs;
   stapl::sequential::vector_property_map<GraphType, size_t> cmap;
   get_cc_stats(*graph, cmap, ccs);
   if(ccs.size() > 0){
		//Iterate through each list from cc info
		vector<VID> ccVIDs;
		for(auto& cc : ccs) {
			cmap.reset();
			ccVIDs.clear();
			get_cc(*graph, cmap, cc.second, ccVIDs);
			cliques.push_back(ccVIDs);
		}
	}*/

   list<list<int>> cliques;
   GetCliques(VIDindex, cliques);

	// Get the binary representation
	vector<bool> xorresult(count, false), andresult(count, false);
    for(auto &clique : cliques){
	  vector<bool> biNodes(count, false);
	  for(auto vc: clique){
		/*if(VIDindex.find(vc) != VIDindex.end())
		  biNodes[VIDindex[vc]] = 1;*/
		biNodes[vc] = 1;
	  }
	  // Bitwise XOR or AND?
	  for(size_t z = 0; z < count;  z++){
		xorresult[z] = xorresult[z] ^ biNodes[z];
		andresult[z] = andresult[z] & biNodes[z];
	  }
	}

	// delete the vertices for the vertices with 0 result
	size_t delete_count = 0;
    for(size_t z = 0; z < count;  z++){
	  if(xorresult[z] == true){
		graph->DeleteVertex(VIDs[z]);
		delete_count++;
	  }
	}
	std::cout<<"Number of vertices deleted:"<<delete_count<<std::endl;
}

template<class MPTraits>
void
VRhomotopy<MPTraits>::
GetCliques(unordered_map<VID,size_t>& _vindex, list<list<int>>& _cliques){
  int number = _vindex.size();
  // Create the adjacency matrix for TomitaAlgorithm
  /*char** adjacencyMatrix = new char*[number];
  for(int i=0; i < number; i++) {
	 adjacencyMatrix[i] = new char[number];
	 for(int j =0; j<number; j++)
		adjacencyMatrix[i][j] = 0;
  }*/
  // for hybrid adjacency list
  vector<list<int>> adjacencyList(number);
  set<pair<VID, VID>> already;
  // store the graph information
  auto graph = this->GetRoadmap()->GetGraph();
  for(auto vit = graph->begin(); vit != graph->end(); ++vit){
	for(auto eit = vit->begin(); eit != vit->end(); ++eit) {
	   auto srcDes = eit->source();
	   auto tarDes = eit->target();
	   if(_vindex.find(srcDes) == _vindex.end() || _vindex.find(tarDes) == _vindex.end())
		 continue;
	   if(already.find(make_pair(srcDes, tarDes)) != already.end())
		 continue;
	   adjacencyList[_vindex[srcDes]].push_back(_vindex[tarDes]);
	   adjacencyList[_vindex[tarDes]].push_back(_vindex[srcDes]);
	   already.insert({srcDes, tarDes});
	   already.insert({tarDes, srcDes});
	  /* adjacencyMatrix[_vindex[srcDes]][_vindex[tarDes]] = 1;
	   adjacencyMatrix[_vindex[tarDes]][_vindex[srcDes]] = 1;*/
	}
  }
  //auto pAlgorithm = new TomitaAlgorithm(adjacencyMatrix, number);
  auto addClique = [&](list<int> const &clique) {
       _cliques.push_back(clique);
  };
  auto pAlgorithm = new HybridAlgorithm(adjacencyList);
  pAlgorithm->AddCallBack(addClique);
  auto cliqNum = pAlgorithm->Run(_cliques);
  std::cout<<"Number of cliques"<<cliqNum<<std::endl;
  // clean up
  delete pAlgorithm;
  // delete the adjacency matrix
  /*for(int i=0; i < number; i++)
	delete [] adjacencyMatrix[i];
  delete [] adjacencyMatrix;*/
}


template<class MPTraits>
void
VRhomotopy<MPTraits>::
Finalize() {
	// Output final map.
	this->GetRoadmap()->Write(this->GetBaseFilename() + ".map", this->GetEnvironment());

	// Output stats.
	ofstream  osStat(this->GetBaseFilename() + ".stat");
	StatClass* stats = this->GetStatClass();
	stats->PrintAllStats(osStat, this->GetRoadmap());

}

template<class MPTraits>
void VRhomotopy<MPTraits>::
ConvexHull(vector<Point3d>& samplePoints, vector<Point3d>& hullPoints, Point3d p, Point3d q, int dir) {

	const static auto lineDist = [&] (Point3d& p1, Point3d& p2, Point3d& p3) -> int {
		 return abs ((p2[1] - p1[1]) * (p3[0] - p2[0]) -
                (p2[0] - p1[0]) * (p3[1] - p2[1]));
	};

	const static auto orientation = [&] (Point3d& p1, Point3d& p2, Point3d& p3) -> int {
		int val = (p2[1] - p1[1]) * (p3[0] - p2[0]) -
                (p2[0] - p1[0]) * (p3[1] - p2[1]);
    	if (val == 0) return 0;  // collinear
	    return (val > 0)? 1: 2; //clockwise or counterclockwise
	};

	Point3d ind;
	int max_dist = 0;

	for (typename vector<Point3d>::iterator S = samplePoints.begin(); S != samplePoints.end(); ++S){
		int temp = lineDist(p, q, *S);
		if(orientation(p, q, *S) == dir && temp > max_dist)
		{
			ind = *S;
			max_dist = temp;
		}
	}

	if(find(samplePoints.begin(), samplePoints.end(), ind) == samplePoints.end()){
		hullPoints.push_back(p);
		hullPoints.push_back(q);
		return;
	}

	ConvexHull(samplePoints, hullPoints, ind, p, -orientation(ind, p, q));
	ConvexHull(samplePoints, hullPoints, ind, q, -orientation(ind, q, p));
}

template<class MPTraits>
void
VRhomotopy<MPTraits>::ConstructObstaclePts(){
	if(m_obstaclePts.empty()){
	   if(m_useWorkspaceObst){
		  for(size_t i = 0; i < this->GetEnvironment()->NumObstacles(); ++i) {
		     const GMSPolyhedron& polyhedron = this->GetEnvironment()->GetObstacle(i)->GetFixedBody(0)->GetWorldPolyhedron();
		     vector<Vector3d> vertexList = polyhedron.GetVertexList();

		     for(typename vector<Vector3d>::iterator lit = vertexList.begin(); lit != vertexList.end(); ++lit) {
			    m_obstaclePts.insert(CfgType(*lit, this->GetTask()->GetRobot()));
		     }
		 }
	   }
		else{
		  vector<CfgType> obstacleCfgs;
		  auto uobprmPtr = this->GetMPLibrary()->GetSampler("UOBPRM");
          size_t attempt = 0, max_attempts = 10000;
          std::cout << " Total number requested : " <<m_obstnum << std::endl;
		  while(obstacleCfgs.size() < m_obstnum && attempt < max_attempts){
		   uobprmPtr->Sample(10, 10, this->GetEnvironment()->GetBoundary(),back_inserter(obstacleCfgs));
		   for(typename vector<CfgType>::iterator oc = obstacleCfgs.begin(); oc!=obstacleCfgs.end();++oc){
			 m_obstaclePts.insert((*oc));
           }
           attempt++;
          }
		}
		std::cout<<"Number of obstacle points: "<<m_obstaclePts.size()<<std::endl;
	}
}


template<class MPTraits>
double VRhomotopy<MPTraits>::
GetHausdroffDistance(vector<Point3d>& _hullPoints, set<Point3d>& _boundaryPoints) {
	double h = 0.0;
	double m = 0.0;
    // Find maximum of the minimum distance of hull points from boundary points
	for(typename vector<Point3d>::iterator hit = _hullPoints.begin(); hit != _hullPoints.end(); ++hit) {
		double minDist = INFINITY;
		for(typename set<Point3d>::iterator bit = _boundaryPoints.begin(); bit != _boundaryPoints.end(); ++bit) {
			double d1 = ((*hit) - (*bit)).norm();
			minDist = std::min(minDist, d1);
		}
		h = std::max(h, minDist);
	}
    // Find maximum of minimum distance of boundary points from hull points
	for(typename set<Point3d>::iterator bit = _boundaryPoints.begin(); bit != _boundaryPoints.end(); ++bit) {
		double minima = INFINITY;
		for(typename vector<Point3d>::iterator hit = _hullPoints.begin(); hit != _hullPoints.end(); ++hit) {
			double d2 = ((*bit) - (*hit)).norm();
			minima = std::min(minima,d2);
		}
		m = std::max(m, minima);
	}
	return std::max(m, h);
}


template<class MPTraits>
void
VRhomotopy<MPTraits>::
MinDisttoObstacle(const vector<Point3d>& _samplePoints, const set<CfgType>& _boundaryPoints){
	Point3d v3;
	for(auto git: _boundaryPoints) {
		double minima = INFINITY;
		for(auto it : _samplePoints) {
			const Vector3d sub = (git).GetPoint() - (it);
			double d3 = sub.norm();

			if(minima > d3){
				minima = d3;
				v3 = it;
			}
		}
		m_distanceSet.insert({git, minima});
	}
}


template<class MPTraits>
void
VRhomotopy<MPTraits>::
DensityMethod(const set<CfgType>& boundaryPoints /*,vector<VID>& collisionPts*/){
	auto nf = this->GetNeighborhoodFinder("RadiusNF");
	map<CfgType, int> density;

    auto graph = this->GetRoadmap()->GetGraph();
	for(typename set<CfgType>::iterator ait = boundaryPoints.begin(); ait != boundaryPoints.end(); ++ait) {
		//CfgType cfg(*ait, this->GetTask()->GetRobot()); //instead of size_t(-1)
		int d = 0;
		vector<pair<VID, double>> neighbors;
		// Get neighbors
		nf->FindNeighbors(this->GetRoadmap(), graph->begin(),
				graph->end(), false, *ait, back_inserter(neighbors));

		if(neighbors.empty()){
            m_densitySet.insert({*ait, 0.0});
			continue;
		}
		for(const auto& nb : neighbors){
			m_neighborPt[*ait].push_back(nb.first);
		}
		d = (m_neighborPt[*ait]).size();
		//cout<<(*ait).GetPoint()<<"density:"<<d<<endl;
		m_densitySet.insert({*ait, d});
	}
}

template<class MPTraits>
void
VRhomotopy<MPTraits>::
DensityAndDistanceComputations(){
	DensityMethod(m_obstaclePts /*, samplePt*/);
	MinDisttoObstacle(m_samplePt, m_obstaclePts);
}
#endif
