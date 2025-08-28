#include "SkeletonRegionMap.h"


/*------------------------- Constructors -------------------------------------*/
SkeletonRegionMap::
SkeletonRegionMap(WorkspaceDecomposition* const _wd, 
			WorkspaceSkeleton* const _ws) : m_decomposition(_wd), m_skeleton(_ws)	{
	CreateMaps();
}

SkeletonRegionMap::~SkeletonRegionMap()	{
	m_vertexToRegion.clear();
	m_edgeToRegions.clear();
	m_decomposition = nullptr;
	m_skeleton = nullptr;
}

void 
SkeletonRegionMap::
SetWorkspaceDecomposition(WorkspaceDecomposition* const _wd)	{
	m_decomposition = _wd;
	if(m_skeleton != nullptr)
		CreateMaps();
}

void 
SkeletonRegionMap::
SetWorkspaceSkeleton(WorkspaceSkeleton* const _ws)	{
	m_skeleton = _ws;
	if(m_decomposition != nullptr)
		CreateMaps();
}

/*--------------------------- Accessors ------------------------------*/

vector<WorkspaceRegion> 
SkeletonRegionMap::
GetRegions(const ED& _e)	{
	vector<WorkspaceRegion> regions;
	auto rits = m_edgeToRegions[_e];
	for(auto it = rits.begin(); it != rits.end(); ++it)
		regions.emplace_back(m_decomposition->GetRegion(*it));
	return regions;
}

const WorkspaceRegion& 
SkeletonRegionMap::
GetRegion(const ED& _e, size_t _i)	{
	return m_decomposition->GetRegion(m_edgeToRegions[_e].at(_i));
}

size_t 
SkeletonRegionMap::
GetNumRegions(const ED& _e)	{
	if(m_edgeToRegions.find(_e) == m_edgeToRegions.end())
		return 0;
	else
		return m_edgeToRegions[_e].size();
}

vector<typename SkeletonRegionMap::VD> 
SkeletonRegionMap::
GetRegionVertices(size_t _i)	{
	if(m_regionToVertices.find(_i) == m_regionToVertices.end())
		return vector<VD>();
	else
		return m_regionToVertices[_i];
}

vector<typename SkeletonRegionMap::ED> 
SkeletonRegionMap::
GetRegionEdge(size_t _i)	{
	if(m_regionToEdges.find(_i) == m_regionToEdges.end())
		return vector<ED>();
	else
		return m_regionToEdges[_i];
}

vector<Point3d> 
SkeletonRegionMap::
GetEdgePoints(size_t _i, const ED& _e)	{
	GraphType::vertex_iterator vi;
  GraphType::adj_edge_iterator ei;
	vector<Point3d> points;

	m_skeleton->GetGraph().find_edge(_e, vi, ei);
	auto paths = ei->property();
	auto key = make_pair(_i, _e);
		/*if(m_edgeRegionIntersections.find(key) == m_edgeRegionIntersections.end())
			FindEdgeRegions(m_skeleton->GetGraph().find_edge());*/
	auto vertexpairs = m_edgeRegionIntersections[key];
	for(auto v : vertexpairs)	{
		auto pit = paths.begin();
		while(pit != paths.end() && *pit != v.first)
			++pit;
		for(;pit != paths.end() && *pit != v.second; ++pit)
			points.emplace_back(*pit);
		if(pit != paths.end())
			points.emplace_back(*pit);
	}
	
	return points;
}

vector<pair<size_t,vector<Point3d>>> 
SkeletonRegionMap::
GetRegionEdgePoints(const ED& _e)	{
	vector<pair<size_t,vector<Point3d>>> discretizedEdges;
	
	if(m_edgeToRegions.find(_e) != m_edgeToRegions.end())	{
		auto regions = m_edgeToRegions[_e];
		for(auto i : regions)	
			discretizedEdges.emplace_back(make_pair(i,GetEdgePoints(i,_e)));
	}

	return discretizedEdges;	
}

vector<pair<typename SkeletonRegionMap::ED,vector<Point3d>>> 
SkeletonRegionMap::
GetRegionEdgePoints(size_t _i)	{
	vector<pair<ED,vector<Point3d>>> discretizedEdges;
	
	auto edges = GetRegionEdge(_i);
	for(auto ed : edges)	
		discretizedEdges.emplace_back(make_pair(ed,GetEdgePoints(_i,ed)));

	return discretizedEdges; 
}

/*-------------------------------- Helpers -----------------------------------*/

const size_t 
SkeletonRegionMap::
FindDecompositionRegion(const Point3d& _p)	{
	return m_decomposition->FindRegion(_p);
}

void 
SkeletonRegionMap::
CreateMaps()	{
	CreateVertexMaps();
	CreateEdgeMaps();
}


void 
SkeletonRegionMap::
CreateVertexMaps()	{
	// Reset the maps
	m_vertexToRegion.clear();
	m_regionToVertices.clear();

	// Get the graph from the skeleton
	auto skeletonGraph = m_skeleton->GetGraph();
	auto numRegions = m_decomposition->GetNumRegions();

	// Create the map from each skeleton vertex to workspace region
	for(auto vit = skeletonGraph.begin(); vit != skeletonGraph.end(); ++vit) {
		auto region = FindDecompositionRegion(vit->property());
		// if the vertex belongs to a region
		if(region > numRegions)	{
			// store in vertex to region map
			m_vertexToRegion.insert(make_pair(vit->descriptor(), region));
			// create the map from workspace region to skeleton vertices
			auto it = m_regionToVertices.find(region);
			if(it == m_regionToVertices.end())
				m_regionToVertices.insert(make_pair(region,vector<VD>(1,vit->descriptor())));
			else 
				it->second.emplace_back(vit->descriptor());
		}
	}
}

void 
SkeletonRegionMap::
CreateEdgeMaps()	{
	// Reset the maps
	m_edgeToRegions.clear();
	m_regionToEdges.clear();

	// Get the graph from the skeleton
	auto skeletonGraph = m_skeleton->GetGraph();

	// Create the map from each skeleton edge to workspace regions
	for(GraphType::const_edge_iterator eit = skeletonGraph.edges_begin(); 
			eit != skeletonGraph.edges_end(); ++eit)
		FindEdgeRegions(eit);
}


void
SkeletonRegionMap::
FindEdgeRegions(GraphType::const_edge_iterator& _ei)	{
	// Get the region where the source point reside
	size_t startRegion = numeric_limits<size_t>::max();
	auto it = m_vertexToRegion.find(_ei->source());
	if(it == m_vertexToRegion.end())
		startRegion = it->second;
	
	// Get the intersecting points 
	auto regionIntersections = m_skeleton->FindEdgeRegions(_ei, m_decomposition, startRegion);
	if(!regionIntersections.empty())	{
		vector<size_t> regions;
		for(auto rit = regionIntersections.begin(); rit != regionIntersections.end(); ++rit)	{
			regions.emplace_back(rit->first);
			// Store in region to edge map
			auto it = m_regionToEdges.find(rit->first);
			if(it == m_regionToEdges.end())
				m_regionToEdges.insert(make_pair(rit->first,vector<ED>(1,_ei->descriptor())));
			else 
				it->second.emplace_back(_ei->descriptor());
			// Store the intersection points of the region and the edge
			m_edgeRegionIntersections.insert(make_pair(make_pair(rit->first,_ei->descriptor()),rit->second));
		}
		// Store in edge to region map
		m_edgeToRegions.insert(make_pair(_ei->descriptor(),regions));
	}
}
