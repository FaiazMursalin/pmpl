#ifndef MORSE_FUNCTION_H_
#define MORSE_FUNCTION_H_

#include <map>
#include <vector>
#include <numeric>
#include <unordered_set>
#include <queue>
#include "MPStrategyMethod.h"
#include "VRhomotopy.h"
#include "Geometry/Boundaries/Boundary.h"
#include "Geometry/Boundaries/CSpaceBoundingSphere.h"
#include "MPProblem/Environment/Environment.h"
#include "Geometry/Bodies/Body.h"
#include "Geometry/Bodies/FixedBody.h"
#include "MPLibrary/NeighborhoodFinders/RadiusNF.h"

template<class MPTraits>
class MorseFunction : public VRhomotopy<MPTraits> {

  public:

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::WeightType   WeightType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::GraphType GraphType;
    typedef typename GraphType::ColorMap ColorMap;
    typedef typename ColorMap::property_value_type color_value;
    typedef typename RoadmapType::VID       VID;
    typedef typename MPTraits::MPLibrary 	MPLibrary;
    typedef typename MPLibrary::MapEvaluatorPointer MapEvaluatorPointer;
    typedef typename MPLibrary::SamplerPointer SamplerPointer;
	typedef typename MPTraits::Path Path;

	MorseFunction(const vector<string>& _evaluatorLabels = vector<string>());
    virtual ~MorseFunction()=default;
    MorseFunction(XMLNode& _node);

    //virtual void ParseXML(XMLNode& _node);
    virtual void Print(ostream& _os) const;

    virtual void Initialize();
    virtual void Run();
    virtual void Finalize();

	void GetClusters(const map<CfgType, vector<VID>>& samplePt);
  protected:
	void GenerateMap();
  	void AddCriticalPts(map<CfgType, double>& morsePt);
	void GetSampleNeighbors(map<CfgType, vector<VID>>& samplePt, map<CfgType, double>& morsePt);


	string m_obstNeighborFinderLabel{"RadiusNF"};
};


template<class MPTraits>
MorseFunction<MPTraits>::
MorseFunction(const vector<string>& _evaluatorLabels){
	this->m_meLabels = _evaluatorLabels;
	this->SetName("MorseFunction");
}


template<class MPTraits>
MorseFunction<MPTraits>::
MorseFunction(XMLNode& _node):
    VRhomotopy<MPTraits>(_node) {
		m_obstNeighborFinderLabel = _node.Read("obstNF",true,"","Obstacle Neighborhood Finder");
		this->SetName("MorseFunction");
}


template<class MPTraits>
void
MorseFunction<MPTraits>::
Print(ostream& _os) const {
	VRhomotopy<MPTraits>::Print(_os);
}


template<class MPTraits>
void
MorseFunction<MPTraits>::
Initialize() {
	VRhomotopy<MPTraits>::Initialize();
}


template<class MPTraits>
void
MorseFunction<MPTraits>::
Run() {
  GenerateMap();
  // Evaluate initialize only if query - to pass on the information of the roadmap
  for(auto eval : this->m_meLabels){
	if(this->GetMapEvaluator(eval)->GetName() == "MorseWaitQuery")
    	this->GetMapEvaluator(eval)->Initialize();
  }
  this->EvaluateMap();
}

template<class MPTraits>
void
MorseFunction<MPTraits>::
GenerateMap(){
	GraphType* g = new GraphType();
	this->GetRoadmap()->SetGraph(g);
	StatClass* stats = this->GetStatClass();
    // Generate the initial roadmap and the density and distance computation
	VRhomotopy<MPTraits>::Run();

	string tick = "Distance and density Computation time";
	stats->StartClock(tick);
	VRhomotopy<MPTraits>::DensityAndDistanceComputations();
	const map<CfgType, double>& dist = VRhomotopy<MPTraits>::GetDistanceSet();
	const map<CfgType, int>& dense = VRhomotopy<MPTraits>::GetDensitySet();
	const set<CfgType>& boundaryPt = VRhomotopy<MPTraits>::GetObstaclePt();
	stats->StopClock(tick);
    map<CfgType, double> morse;

	string t = "Total Calculation time";
	stats->StartClock(t);
    cout<<"start calculated morse values: "<<endl;
    // Calculate the morse function value
	for(typename set<CfgType>::iterator bit = boundaryPt.begin(); bit != boundaryPt.end(); ++bit){
		//auto obstPt = (*bit).GetPoint();
//        if(dist.find(*bit) != dist.end() && dense.find(*bit) != dense.end()){
        	double m = dist.at(*bit) * dense.at(*bit);
			morse[(*bit)] = m;
//        }
	}
    cout<<"calculated morse values: "<<endl;
	// Get obstacle critical points and feasible critical points
	AddCriticalPts(morse);
    cout<<"Added Critical Pts: "<<endl;
	//store.clear();
	stats->StopClock(t);

	const map<CfgType, vector<VID>>& sample = VRhomotopy<MPTraits>::GetNeighborPts();
	// store feasible critical points
	auto criticalGraph = this->GetBlockRoadmap()->GetGraph();
    for(auto cit = criticalGraph->begin(); cit != criticalGraph->end(); ++cit){
	   // get the feasible critical points
	   for(const auto& fp: sample.at(criticalGraph->GetVertex(cit))){
		 // store the information for query evaluator for diverse path
		 // as configuration labels
		 g->GetVertex(fp).SetLabel("FeasibleCritical", true);  // mark it as feasible critical point
		 g->GetVertex(fp).SetStat("Critical", cit->descriptor());  // store the critical point VID
	   }
	}
    cout<<"Critical graph calculated: "<<endl;
	GetClusters(sample);
    cout<<"Get cluster calculated: "<<endl;
	//GetSampleNeighbors(sample, morse);
}


template<class MPTraits>
void
MorseFunction<MPTraits>::
Finalize() {
	//Output final map
	this->GetRoadmap()->Write(this->GetBaseFilename() + ".map", this->GetEnvironment());
	this->GetBlockRoadmap()->Write(this->GetBaseFilename() + ".block.map", this->GetEnvironment());

	// Output stats.
	ofstream  osStat(this->GetBaseFilename() + ".stat");
	StatClass* stats = this->GetStatClass();
	stats->PrintAllStats(osStat, this->GetRoadmap());
}

template<class MPTraits>
void
MorseFunction<MPTraits>::
AddCriticalPts(map<CfgType, double>& morsePt){
	vector<CfgType> critPt;
	// Find the neighbors of the obstacle points
	auto radiusNf = this->GetNeighborhoodFinder(m_obstNeighborFinderLabel); // radius neighbor hood for obstacle points
	// create a new roadmap with obstacle pounts and add it in the roadmap
	auto obstRoadmap = new RoadmapType(this->GetTask()->GetRobot());
	GraphType* obstGraph = (obstRoadmap)->GetGraph();
	auto boundaryPt = VRhomotopy<MPTraits>::GetObstaclePt();
	for(auto b: boundaryPt)
		obstGraph->AddVertex(b);
	obstRoadmap->Write(this->GetBaseFilename() + ".obst.map", this->GetEnvironment());
	map<CfgType, double> avgMorseValues;
	// Find the neighbors for each boundary vertex
	for(auto b: boundaryPt){
	    vector<pair<VID, double>> neighbors;
		radiusNf->FindNeighbors(obstRoadmap, obstGraph->begin(),
				obstGraph->end(), false, b, back_inserter(neighbors));
		if(neighbors.empty())
			continue;

		std::vector<double> moreseVal;
		for(const auto& nb : neighbors){
			auto nCfg = obstGraph->GetVertex(nb.first);
			if(morsePt.find(nCfg) != morsePt.end())
			  moreseVal.push_back(morsePt[nCfg]);
		}
		if(morsePt.find(b) == morsePt.end()) continue;
        auto val = morsePt[b];
		double avg = val, maxVal = val, minVal = val;
		for(auto v: moreseVal){
		  avg += v;
		  maxVal = std::max(maxVal, v);
		  minVal = std::min(minVal, v);
		}
		avg /= (neighbors.size() +1);
		double sd = std::pow((val-avg), 2);
		for(auto v: moreseVal)
		  sd += std::pow((v-avg),2);

		sd /= (neighbors.size() + 1);
		sd = std::sqrt(sd);

		//std::cout<<"Value: "<<val <<" avg, sd: "<< avg <<" , "<<sd<<" [min, max]: "<<minVal <<" , "<<maxVal <<std::endl;
		//if(val < avg - sd || val > avg + sd){
		//std::cout<<"Max: "<<maxVal<<" min : "<<minVal<<std::endl;
		if(fabs(val - minVal) <= std::numeric_limits<float>::epsilon() || fabs(val - maxVal) <= std::numeric_limits<float>::epsilon()){
		  avgMorseValues.insert({b, avg});
		}
	}
    //std::cout<<"Number of above/below avg -sd points"<<avgMorseValues.size()<<std::endl;
	for(typename map<CfgType, double>::iterator mit = morsePt.begin(); mit != morsePt.end(); ++mit){
		if(mit->second > 0 && avgMorseValues.find(mit->first)!= avgMorseValues.end()) {
			//CfgType cfg(mit->first, this->GetTask()->GetRobot());
			critPt.push_back(mit->first);
		}
	}
    std::cout<<"Number of critical points: "<<critPt.size()<<std::endl;
	GraphType* gp = this->GetBlockRoadmap()->GetGraph();
	ColorMap color_map;

	for(typename vector<CfgType>::iterator Cp = critPt.begin(); Cp != critPt.end(); ++Cp){
		VID node = gp->AddVertex(*Cp);
		color_map.put(node, 1);
	}

}

template<class MPTraits>
void
MorseFunction<MPTraits>::
GetSampleNeighbors(map<CfgType, vector<VID>>& samplePt, map<CfgType, double>& morsePt){
	//cout<<"Inside sample neighbors"<<endl;
	GraphType* graph = this->GetRoadmap()->GetGraph();
	GraphType* prev_g = VRhomotopy<MPTraits>::GetParentGraph();

	for(typename map<CfgType, double>::iterator mit = morsePt.begin(); mit != morsePt.end(); ++mit){
		if(mit->second != 0){
			vector<VID> fcPt(samplePt[mit->first]);
			for(typename vector<VID>::iterator sit = fcPt.begin(); sit != fcPt.end(); ++sit){
				//CfgType cfg_node = prev_g->GetVertex(sit);

				VID sample_node = graph->AddVertex(prev_g->GetVertex(sit));
				if(this->m_debug)
					cout<<"Inserting node:"<<sample_node<<endl;
			}
		}
	}
}

template<class MPTraits>
void
MorseFunction<MPTraits>::
GetClusters(const map<CfgType, vector<VID>>& samplePt){
  vector<vector<VID>> clusters;
  set<pair<size_t, size_t>> clusterEdges;
  unordered_map<VID, size_t> visited;
  queue<pair<VID,size_t>> floodvertex;
  unordered_map<size_t, VID> criticalMap;
  GraphType* prevg = this->GetRoadmap()->GetGraph();

  // Initialize the clusters
  auto criticalGraph = this->GetBlockRoadmap()->GetGraph();
  for(auto cit = criticalGraph->begin(); cit != criticalGraph->end(); ++cit){
	// get the feasible critical points
	vector<VID> clusterPts(samplePt.at(criticalGraph->GetVertex(cit)));
	size_t sz = clusters.size();
	criticalMap.insert({sz, cit->descriptor()});
	for(auto fp: clusterPts){
	  visited.insert({fp,sz});
	  floodvertex.push(make_pair(fp,sz));
	}
	// Initialize the cluster with feasible critical points
	clusters.push_back(clusterPts);
  }

  // flood fill
  while(!floodvertex.empty()){
	auto vp = floodvertex.front();
	floodvertex.pop();
	auto vit=prevg->find_vertex(vp.first);
	for(auto eit = vit->begin(); eit != vit->end(); ++eit){
	  // get other vertex
	  auto oVID = (eit->source()==vp.first)?eit->target():eit->source();
	  if(visited.find(oVID)==visited.end()){
		// add to the cluster
		clusters[vp.second].push_back(oVID);
		// mark visited
		visited.insert({oVID,vp.second});
		// add to queue
		floodvertex.push(make_pair(oVID, vp.second));
		// mark the dependecy on the critical point
		prevg->GetVertex(oVID).SetStat("Critical",criticalMap[vp.second]);
	  }
	  else{
		 if(vp.second == visited[oVID])
		   continue;
		 pair<size_t, size_t> edgePair;
		 if(vp.second < visited[oVID])
		   edgePair=make_pair(vp.second, visited[oVID]);
		 else
		   edgePair=make_pair(visited[oVID], vp.second);
		  if(clusterEdges.find(edgePair)== clusterEdges.end())
			clusterEdges.insert(edgePair);
	  }
	}
  }

  // print -m_debug
 // if(m_debug)
    std::cout<<"Number of clusters"<<clusters.size()<<std::endl;
  //size_t k = 0;
  size_t sz=0;
  for(auto c: clusters){
	  sz+= c.size();
	  //std::cout<<"Cluster "<<k++<<" size "<<c.size()<<std::endl;
  }
//  if(m_debug)
    std::cout<<"Number of vertices in graph: "<<prevg->get_num_vertices()<<" "<<sz<<std::endl;
}
#endif
