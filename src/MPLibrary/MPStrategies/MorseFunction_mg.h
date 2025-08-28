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

	void GetClusters(map<CfgType, vector<VID>>& samplePt);
  protected:
  	void AddCriticalPts(map<CfgType, double>& morsePt);
	void GetSampleNeighbors(map<CfgType, vector<VID>>& samplePt, map<CfgType, double>& morsePt);
	void GetPaths(size_t k);

	// data
	unordered_map<VID, vector<VID>> m_critical; // Map of critical point vid in obstacle map to feasible critical points in roadmap
	unordered_map<VID, VID> m_feasible; // Map of feasible critical points to the critical points
	vector<Path*> m_paths;   // Diverse path set
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

	GraphType* g = new GraphType();
	this->GetRoadmap()->SetGraph(g);
	StatClass* stats = this->GetStatClass();

	VRhomotopy<MPTraits>::Run();
	
	string tick = "Distance and density Computation time";
	stats->StartClock(tick);
	map<CfgType, double> dist = VRhomotopy<MPTraits>::GetDistanceSet();
	map<CfgType, int> dense = VRhomotopy<MPTraits>::GetDensitySet();
	set<CfgType> boundaryPt = VRhomotopy<MPTraits>::GetObstaclePt();
	stats->StopClock(tick);

	map<CfgType, double> morse;
	//multiset<double> store;
	//vector<Vector3d> vertexList;

	string t = "Total Calculation time";
	stats->StartClock(t);
	
	for(typename set<CfgType>::iterator bit = boundaryPt.begin(); bit != boundaryPt.end(); ++bit){
		//auto obstPt = (*bit).GetPoint();
		double m = dist[(*bit)] * dense[(*bit)];
		//cout<<"Morse Values:"<<m<<endl;
		//store.insert(m);
		morse[(*bit)] = m;
	}
	AddCriticalPts(morse);
	//store.clear();
	stats->StopClock(t);

	map<CfgType, vector<VID>> sample = VRhomotopy<MPTraits>::GetNeighborPts();
	// store feasible critical points
	auto criticalGraph = this->GetBlockRoadmap()->GetGraph();
    for(auto cit = criticalGraph->begin(); cit != criticalGraph->end(); ++cit){
	   // get the feasible critical points
	   vector<VID> feasible(sample[criticalGraph->GetVertex(cit)]);
	   m_critical.insert({cit->descriptor(),feasible});
	   for(auto fp: feasible){
		 m_feasible.insert({fp, cit->descriptor()});
		 // store the information for query evaluator for diverse path
		 g->GetVertex(fp).SetLabel("FeasibleCritical", true);  // mark it as feasible critical point
		 g->GetVertex(fp).SetStat("Critical", cit->descriptor());  // store the critical point VID
	   }
	}
	GetClusters(sample);
	//GetSampleNeighbors(sample, morse);

	for(auto eval : this->m_meLabels){
		if(this->GetMapEvaluator(eval)->GetName() == "MorseDiverseDynamicQuery")
			this->GetMapEvaluator(eval)->Initialize();
	}
	this->EvaluateMap();
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
	auto radiusNf = /*new RadiusNF<MPTraits>("euclidean", false, 2.5, false);*/this->GetNeighborhoodFinder("RadiusNF1"); // radius neighbor hood GetNeighborhoodFinder
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
GetPaths(size_t k){
  if(!this->UsingQuery()) return;

  // Copy the graph
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

  vector<unordered_set<VID>> criticalPoints;
  // Get Diverse paths
  while(m_paths.size() < k){
	this->GetPath()->Clear(); // Clear previous path data
    auto mapPassed = this->EvaluateMap();
    if(!mapPassed) break; // no more path to be found
	//get the path and store it
    Path* p = new Path(this->GetRoadmap());
	vector<VID> pVID, fVID;
	for(auto v : this->GetPath()->VIDs()){
	  pVID.push_back(oldNew[v]);
	  // find the feasible critical points in the path
	  if(m_feasible.find(v) != m_feasible.end())
		fVID.push_back(v);
	}
	p+= pVID;
	m_paths.push_back(pVID);
    // delete the vertices in the path from the roadmap
	// get the corresponding critical point and its' feasible critical point
	unordered_set<VID> pCritical;
	for(auto f: fVID)
	  pCritical.insert(m_feasible[f]);

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
	/*for(auto f: fVID)
	  gr->DeleteVertex(f);*/

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
}


template<class MPTraits>
void
MorseFunction<MPTraits>::
GetClusters(map<CfgType, vector<VID>>& samplePt){
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
	vector<VID> clusterPts(samplePt[criticalGraph->GetVertex(cit)]);
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
  std::cout<<"Number of clusters"<<clusters.size()<<std::endl;
  //size_t k = 0;
  size_t sz=0;
  for(auto c: clusters){
	  sz+= c.size();
	  //std::cout<<"Cluster "<<k++<<" size "<<c.size()<<std::endl;
  }

  std::cout<<"Number of vertices in graph: "<<prevg->get_num_vertices()<<" "<<sz<<std::endl;
}
#endif