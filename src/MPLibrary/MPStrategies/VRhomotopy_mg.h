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
    map<CfgType, int> DensityMethod(set<CfgType>& boundaryPoints/*, vector<VID>& collisionPts*/);
    map<CfgType, double> GetDistanceSet();
	map<CfgType, int> GetDensitySet();
	map<CfgType, vector<VID>> GetNeighborPts();
	GraphType* GetParentGraph() { return graph;}
    set<CfgType> GetObstaclePt();

  protected:
	// Planning helper
	double Sample(set<Point3d>& _boundaryPt);
	void Connect();
	void GetCliques(unordered_map<VID,size_t>& _vindex, list<list<int>>& _cliques);

	// Geometry helper functions
    int orientation(Point3d& p, Point3d& q, Point3d& r);
	int lineDist(Point3d& p, Point3d& q, Point3d& r);
    void ConvexHull(vector<Point3d>& samplePoints, vector<Point3d>& hullPoints, Point3d p, Point3d q, int dir);
    double GetHausdroffDistance(vector<Point3d>& hullPoints, set<Point3d>& boundaryPoints);
	// Finds the minimum distance of the sample points from the obstacle surface
    map<CfgType, double> MinDisttoObstacle(vector<Point3d>& SamplePoints, set<CfgType>& boundaryPoints);

	// Members
	string m_samplerLabel;   // Label for the underlying sampler used
	size_t m_number;
	size_t m_attempts;
	vector<Point3d> samplePt;    // Workspace counterpart of the sampled configurations
	set<CfgType> ObstaclePts;    // Points sampled at uniformly at the surface of the obstacle
	GraphType* graph;            // Roadmap used by the parent class
	map<CfgType, vector<VID>> neighborPt;  // Map of the obstacle  points to the neighbors to find critical points and feasible critical points
	size_t m_totalnum{10000};
	size_t m_obstnum{1000};
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
  }
}


template<class MPTraits>
void
VRhomotopy<MPTraits>::
Print(ostream& _os) const {
	MPStrategyMethod<MPTraits>::Print(_os);
	_os<<"\tSampling method: "<<m_samplerLabel<<endl;
	
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

template<class MPTraits>
void
VRhomotopy<MPTraits>::
Run() {
  set<Point3d> boundaryPt;
  //bool mapPassedEvaluation = this->EvaluateMap();
  double epsilon = 0.0, prevE = 0.0;
  do{
	prevE = epsilon;
    epsilon = Sample(boundaryPt);
  }while(std::fabs(prevE - epsilon) > 1e-05 || this->GetRoadmap()->GetGraph()->get_num_vertices() < m_totalnum);

  //current graph for child class
  graph = this->GetRoadmap()->GetGraph();
  std::cout<<"Graph vertices: "<<graph->get_num_vertices()<<std::endl;
  Connect();
}


template<class MPTraits>
double
VRhomotopy<MPTraits>::
Sample(set<Point3d>& _boundaryPt) {
	RoadmapType* rmap = this->GetRoadmap();
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
		for(typename vector<CfgType>::iterator C = vectorCfgs.begin(); C!=vectorCfgs.end();++C){
			if(C->IsLabel("VALID") && C->GetLabel("VALID")){
				stats->StartClock(clockName);
				VID newVID = rmap->GetGraph()->AddVertex(*C);
				stats->StopClock(clockName);
				if(this->m_debug) {
					cout<<"Node vertex ID "<< newVID <<endl;
				}
				Vector3d s, b;
				if(robType == FreeBody::BodyType::Fixed){
					(*C).ConfigureRobot();
					shared_ptr<vector<Vector3d> > joints = shared_ptr<vector<Vector3d> >(new vector<Vector3d>);
					(*C).GetMultiBody()->PolygonalApproximation(*joints);
					s = (*joints)[joints->size()-1];
					b = this->GetEnvironment()->GetBoundary()->GetClearancePoint(s);
				}
				else{
					s = (*C).GetPoint();
					b = this->GetEnvironment()->GetBoundary()->GetClearancePoint(s);
				}
				samplePt.push_back(s);
				_boundaryPt.insert(b);
			}		
		}
		//mapPassedEvaluation = this->EvaluateMap();
	//}

	// Change to use CGAL to compute the convex hull
	Point3d min_p, max_p;
	if(samplePt.size() > 3){
		min_p = samplePt.front();
		max_p = samplePt.back();
		for (typename vector<Point3d>::iterator pt=samplePt.begin(); pt != samplePt.end();++pt){
			if(*pt[0] < min_p[0])
				min_p = *pt;

			else if(*pt[0] > max_p[0])
				max_p = *pt;
		}
	}
	ConvexHull(samplePt, hullPt, min_p, max_p, 1); //clockwise
	ConvexHull(samplePt, hullPt, min_p, max_p, 2); //counterclockwise
    
	ObstaclePts = GetObstaclePt();
	
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
  auto connector = this->GetConnector("Closest"); // Need to do this using the connector constructor
  connector->Connect(this->GetRoadmap(), graph->begin(), graph->end());	  // Connect them

   // get the map of vid and count
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
	vector<vector<bool>> bits;
    for(auto &clique : cliques){
	  vector<bool> biNodes(count, false);
	  for(auto vc: clique){
		/*if(VIDindex.find(vc) != VIDindex.end())
		  biNodes[VIDindex[vc]] = 1;*/
		biNodes[vc] = 1;
	  }
	  bits.push_back(biNodes);
	}
	// Bitwise XOR or AND?
	vector<bool> xorresult(count, false), andresult(count, false);
	for(auto biMap: bits){
	  for(size_t z = 0; z < count;  z++){
		xorresult[z] = xorresult[z] ^ biMap[z];
		andresult[z] = andresult[z] & biMap[z];
	  }
	}

	// delete the vertices for the vertices with 0 result
	size_t delete_count = 0;
    for(size_t z = 0; z < count;  z++){
	  //if(result[z] == false){
		//cout<<"Removing vertex "<<VIDs[z]<<std::endl;
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
  auto pAlgorithm = new HybridAlgorithm(adjacencyList);
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
int VRhomotopy<MPTraits>::
orientation(Point3d& p, Point3d& q, Point3d& r) {
	int val = (q[1] - p[1]) * (r[0] - q[0]) -
		(q[0] - p[0]) * (r[1] - q[1]);

	if (val == 0) return 0;  // collinear
	return (val > 0)? 1: 2; //clockwise or counterclockwise
}


template<class MPTraits>
int VRhomotopy<MPTraits>::
lineDist(Point3d& p, Point3d& q, Point3d& r) {
        return abs ((q[1] - p[1]) * (r[0] - q[0]) -
                (q[0] - p[0]) * (r[1] - q[1]));
}


template<class MPTraits>
void VRhomotopy<MPTraits>::
ConvexHull(vector<Point3d>& samplePoints, vector<Point3d>& hullPoints, Point3d p, Point3d q, int dir) {
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
set<typename MPTraits::CfgType>
VRhomotopy<MPTraits>::GetObstaclePt(){

	/*for(size_t i = 0; i < this->GetEnvironment()->NumObstacles(); ++i) {
		const GMSPolyhedron& polyhedron = this->GetEnvironment()->GetObstacle(i)->GetFixedBody(0)->GetWorldPolyhedron();
		vector<Vector3d> vertexList = polyhedron.GetVertexList();

		for(typename vector<Vector3d>::iterator lit = vertexList.begin(); lit != vertexList.end(); ++lit) {
			ObstaclePts.insert(*lit);
		}

	}*/
	if(ObstaclePts.empty()){
		vector<CfgType> obstacleCfgs;
		auto uobprmPtr = this->GetMPLibrary()->GetSampler("UOBPRM");
		while(obstacleCfgs.size() < m_obstnum)
		  uobprmPtr->Sample(10, 10, this->GetEnvironment()->GetBoundary(),back_inserter(obstacleCfgs));
		for(typename vector<CfgType>::iterator C = obstacleCfgs.begin(); C!=obstacleCfgs.end();++C){
			//if(C->IsLabel("VALID") && C->GetLabel("VALID")){
			//	ObstaclePts.insert((*C).GetPoint());
			//}
			ObstaclePts.insert((*C));
		}
		std::cout<<"Number of obstacle points: "<<ObstaclePts.size()<<std::endl;
	}
	return ObstaclePts;						                                         
}


template<class MPTraits>
double VRhomotopy<MPTraits>::
GetHausdroffDistance(vector<Point3d>& hullPoints, set<Point3d>& boundaryPoints) {
	double h = 0.0;
	double m = 0.0;

	for(typename vector<Point3d>::iterator hit = hullPoints.begin(); hit != hullPoints.end(); ++hit) {
		double min = INFINITY;
		for(typename set<Point3d>::iterator bit = boundaryPoints.begin(); bit != boundaryPoints.end(); ++bit) { 
			const Vector3d diff = (*hit) - (*bit);
			double d1 = diff.norm();

			if(min > d1)
				min = d1;    
		}
		if(h < min)
			h = min; 
	}
        //cout<<"h= "<<h<<endl;
	
	for(typename set<Point3d>::iterator kit = boundaryPoints.begin(); kit != boundaryPoints.end(); ++kit) {
		double minima = INFINITY;
		for(typename vector<Point3d>::iterator dit = hullPoints.begin(); dit != hullPoints.end(); ++dit) {
			const Vector3d sub = (*kit) - (*dit);
			double d2 = sub.norm();
			if(minima > d2)
				minima = d2; 
		}
		if(m < minima)
			m = minima;
	}
        //cout<<"m= "<<m<<endl;
	
	if(m > h)
		return m;
	else
	        return h;
}


template<class MPTraits>
map<typename MPTraits::CfgType, double>
VRhomotopy<MPTraits>::
MinDisttoObstacle(vector<Point3d>& SamplePoints, set<CfgType>& boundaryPoints){
	map<CfgType, double> mindist;
	Point3d v3;
	for(typename set<CfgType>::iterator git = boundaryPoints.begin(); git != boundaryPoints.end(); ++git) {
		double minima = INFINITY;
		
		for(typename vector<Point3d>::iterator it = SamplePoints.begin(); it != SamplePoints.end(); ++it) {
			const Vector3d sub = (*git).GetPoint() - (*it);
			double d3 = sub.norm();
                                              
			if(minima > d3){
				minima = d3;
				v3 = *it;
			}
		}		
		mindist.insert({*git, minima});
	}
	CfgType closest(v3, this->GetTask()->GetRobot());
	cout<<"ClosestNode\t"<<closest<<endl;

	return mindist;

}


template<class MPTraits>
map<typename MPTraits::CfgType, int>
VRhomotopy<MPTraits>::
DensityMethod(set<CfgType>& boundaryPoints /*,vector<VID>& collisionPts*/){
	auto nf = this->GetNeighborhoodFinder("RadiusNF");
	map<CfgType, int> density;
	//cout<< nf->GetRadius()<<endl;
	
	//this is just to identify pocket for biomolecule.
	/*map<Point3d, double> dist = GetDistanceSet();
	auto pocket = min_element(dist.begin(), dist.end(), 
			[](const pair<Point3d,double>& x, const pair<Point3d,double>& y)->bool{
                        return x.second < y.second; });
        cout<<"The nodes around cavity are:"<<endl;*/

	for(typename set<CfgType>::iterator ait = boundaryPoints.begin(); ait != boundaryPoints.end(); ++ait) {
		//CfgType cfg(*ait, this->GetTask()->GetRobot()); //instead of size_t(-1)
		int d = 0;

		vector<pair<VID, double>> neighbors;
		
		nf->FindNeighbors(this->GetRoadmap(), graph->begin(), 
				graph->end(), false, *ait, back_inserter(neighbors));

		if(neighbors.empty())
			continue;

		for(const auto& nb : neighbors){
			neighborPt[*ait].push_back(nb.first);
		}
		d = (neighborPt[*ait]).size();

		//cout<<(*ait).GetPoint()<<"density:"<<d<<endl;
		density.insert({*ait, d});

	}

	return density;
}

template<class MPTraits>
map<typename MPTraits::CfgType, double>
VRhomotopy<MPTraits>::GetDistanceSet(){
	return MinDisttoObstacle(samplePt, ObstaclePts);
}


template<class MPTraits>
map<typename MPTraits::CfgType, int>
VRhomotopy<MPTraits>::GetDensitySet(){
	return DensityMethod(ObstaclePts /*, samplePt*/);
}


template<class MPTraits>
map<typename MPTraits::CfgType, vector<typename VRhomotopy<MPTraits>::VID>>
VRhomotopy<MPTraits>::GetNeighborPts(){
	return neighborPt;
}



#endif