#ifndef NAVMESHCLASS_H
#define NAVMESHCLASS_H

#include <map>
#include <string>
#include <tuple>
#include <vector>

class BuildContext;
class InputGeom;
class dtNavMesh;
class dtNavMeshQuery;

/// Self-contained, GUI-free wrapper around RecastNavigation suitable for
/// scripting bindings. Mirrors the lifecycle of the solo-mesh demo:
///   1. init_by_obj() / init_by_raw()
///   2. set_settings() / set_partition_type()  (optional)
///   3. build_navmesh()
///   4. pathfind / raycast / distance_to_wall / save / load / ...
///
/// This class does NOT depend on Sample / Sample_SoloMesh / Tool_NavMeshTester
/// or anything that would drag in SDL2 / OpenGL / imgui.
class Navmesh
{
public:
	Navmesh();
	~Navmesh();

	Navmesh(const Navmesh&) = delete;
	Navmesh& operator=(const Navmesh&) = delete;

	bool init_by_obj(const std::string& file_path);
	bool init_by_raw(const std::vector<float>& vertices, const std::vector<int>& faces);

	bool build_navmesh();

	std::string get_log();

	std::map<std::string, float> get_settings() const;
	void set_settings(const std::map<std::string, float>& settings);

	int get_partition_type() const;
	void set_partition_type(int type);

	std::vector<float> get_bounding_box() const;

	bool save_navmesh(const std::string& file_path);
	bool load_navmesh(const std::string& file_path);

	std::tuple<std::vector<float>, std::vector<int>> get_navmesh_trianglulation() const;
	std::tuple<std::vector<float>, std::vector<int>, std::vector<int>> get_navmesh_polygonization() const;

	std::vector<float> pathfind_straight(const std::vector<float>& start,
	                                     const std::vector<float>& end,
	                                     int vertex_mode = 0) const;
	std::vector<float> pathfind_straight_batch(const std::vector<float>& coordinates,
	                                           int vertex_mode = 0) const;

	float distance_to_wall(const std::vector<float>& point) const;
	std::vector<float> raycast(const std::vector<float>& start,
	                           const std::vector<float>& end) const;
	std::vector<float> hit_mesh(const std::vector<float>& start,
	                            const std::vector<float>& end) const;

private:
	void clear();
	void clearNavMesh();
	void resetSettings();

	BuildContext* ctx = nullptr;
	InputGeom* geom = nullptr;
	dtNavMesh* navMesh = nullptr;
	dtNavMeshQuery* navQuery = nullptr;

	bool is_init = false;
	bool is_build = false;

	// Build settings (defaults match Sample::resetCommonSettings).
	float cellSize = 0.3f;
	float cellHeight = 0.2f;
	float agentHeight = 2.0f;
	float agentRadius = 0.6f;
	float agentMaxClimb = 0.9f;
	float agentMaxSlope = 45.0f;
	float regionMinSize = 8.0f;
	float regionMergeSize = 20.0f;
	float edgeMaxLen = 12.0f;
	float edgeMaxError = 1.3f;
	int vertsPerPoly = 6;
	float detailSampleDist = 6.0f;
	float detailSampleMaxError = 1.0f;
	int partitionType = 0;  // 0 WATERSHED, 1 MONOTONE, 2 LAYERS
};

#endif  // NAVMESHCLASS_H
