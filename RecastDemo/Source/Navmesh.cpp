#include "Navmesh.h"
#include "DetourCommon.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"
#include "InputGeom.h"
#include "Recast.h"
#include "SampleInterfaces.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>

// ---------------------------------------------------------------------------
// Navmesh is a self-contained wrapper for use from Python (or any other
// non-UI consumer) of RecastNavigation. It does NOT depend on the demo
// classes Sample / Sample_SoloMesh / Tool_NavMeshTester etc., so the
// resulting binary has no transitive dependency on SDL2 / OpenGL / imgui.
// The build pipeline below is a stripped-down copy of Sample_SoloMesh::build()
// (the GUI-free parts).
// ---------------------------------------------------------------------------

namespace
{

// Sample area / poly flag enum values, mirrored from Sample.h to avoid
// pulling in the whole demo header. Keep these synchronised with Sample.h.
enum SamplePolyAreas
{
	NAV_POLYAREA_GROUND = 0,
	NAV_POLYAREA_WATER = 1,
	NAV_POLYAREA_ROAD = 2,
	NAV_POLYAREA_DOOR = 3,
	NAV_POLYAREA_GRASS = 4,
	NAV_POLYAREA_JUMP = 5,
};

enum SamplePolyFlags
{
	NAV_POLYFLAGS_WALK = 1 << 0,
	NAV_POLYFLAGS_SWIM = 1 << 1,
	NAV_POLYFLAGS_DOOR = 1 << 2,
	NAV_POLYFLAGS_JUMP = 1 << 3,
	NAV_POLYFLAGS_DISABLED = 1 << 4,
	NAV_POLYFLAGS_ALL = ~0,
};

constexpr int MAX_POLYS = 256;
constexpr float DEFAULT_POLY_PICK_HALF_EXT[3] = {2.0f, 4.0f, 2.0f};
constexpr int NAVMESHSET_MAGIC = 'M' << 24 | 'S' << 16 | 'E' << 8 | 'T';
constexpr int NAVMESHSET_VERSION = 1;

struct NavMeshSetHeader
{
	int magic;
	int version;
	int numTiles;
	dtNavMeshParams params;
};

struct NavMeshTileHeader
{
	dtTileRef tileRef;
	int dataSize;
};

dtQueryFilter makeDefaultFilter()
{
	dtQueryFilter f;
	f.setIncludeFlags(NAV_POLYFLAGS_ALL ^ NAV_POLYFLAGS_DISABLED);
	f.setExcludeFlags(0);
	f.setAreaCost(NAV_POLYAREA_GROUND, 1.0f);
	f.setAreaCost(NAV_POLYAREA_WATER, 10.0f);
	f.setAreaCost(NAV_POLYAREA_ROAD, 1.0f);
	f.setAreaCost(NAV_POLYAREA_DOOR, 1.0f);
	f.setAreaCost(NAV_POLYAREA_GRASS, 2.0f);
	f.setAreaCost(NAV_POLYAREA_JUMP, 1.5f);
	return f;
}

}  // namespace

Navmesh::Navmesh()
{
	ctx = new BuildContext();
	navQuery = dtAllocNavMeshQuery();
	resetSettings();
}

Navmesh::~Navmesh()
{
	clear();
	dtFreeNavMeshQuery(navQuery);
	navQuery = nullptr;
	delete ctx;
	ctx = nullptr;
}

void Navmesh::resetSettings()
{
	cellSize = 0.3f;
	cellHeight = 0.2f;
	agentHeight = 2.0f;
	agentRadius = 0.6f;
	agentMaxClimb = 0.9f;
	agentMaxSlope = 45.0f;
	regionMinSize = 8.0f;
	regionMergeSize = 20.0f;
	edgeMaxLen = 12.0f;
	edgeMaxError = 1.3f;
	vertsPerPoly = 6;
	detailSampleDist = 6.0f;
	detailSampleMaxError = 1.0f;
	partitionType = 0;  // WATERSHED
}

void Navmesh::clearNavMesh()
{
	dtFreeNavMesh(navMesh);
	navMesh = nullptr;
	is_build = false;
}

void Navmesh::clear()
{
	clearNavMesh();
	delete geom;
	geom = nullptr;
	if (ctx)
	{
		ctx->resetLog();
	}
	is_init = false;
}

std::string Navmesh::get_log()
{
	std::string out;
	const int count = ctx->getLogCount();
	for (int i = 0; i < count; ++i)
	{
		out += ctx->getLogText(i);
		if (i + 1 < count)
		{
			out += '\n';
		}
	}
	ctx->resetLog();
	return out;
}

bool Navmesh::init_by_obj(const std::string& file_path)
{
	clear();

	geom = new InputGeom();
	if (!geom->load(ctx, file_path))
	{
		ctx->log(RC_LOG_ERROR, "Navmesh::init_by_obj: failed to load '%s'", file_path.c_str());
		clear();
		return false;
	}

	resetSettings();
	is_init = true;
	return true;
}

bool Navmesh::init_by_raw(const std::vector<float>& vertices, const std::vector<int>& faces)
{
	if (vertices.empty() || faces.empty() || vertices.size() % 3 != 0 || faces.size() % 3 != 0)
	{
		ctx->log(RC_LOG_ERROR, "Navmesh::init_by_raw: invalid vertex/face arrays.");
		return false;
	}

	clear();

	geom = new InputGeom();
	geom->mesh.verts = vertices;
	geom->mesh.tris = faces;

	const int triCount = geom->mesh.getTriCount();
	geom->mesh.normals.assign(static_cast<size_t>(triCount) * 3, 0.0f);
	for (int i = 0; i < triCount; ++i)
	{
		const float* v0 = &geom->mesh.verts[static_cast<size_t>(faces[i * 3 + 0]) * 3];
		const float* v1 = &geom->mesh.verts[static_cast<size_t>(faces[i * 3 + 1]) * 3];
		const float* v2 = &geom->mesh.verts[static_cast<size_t>(faces[i * 3 + 2]) * 3];
		const float e0[3] = {v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]};
		const float e1[3] = {v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]};
		float* n = &geom->mesh.normals[static_cast<size_t>(i) * 3];
		n[0] = e0[1] * e1[2] - e0[2] * e1[1];
		n[1] = e0[2] * e1[0] - e0[0] * e1[2];
		n[2] = e0[0] * e1[1] - e0[1] * e1[0];
		const float len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
		if (len > 0.0f)
		{
			const float s = 1.0f / len;
			n[0] *= s;
			n[1] *= s;
			n[2] *= s;
		}
	}

	rcCalcBounds(
		geom->mesh.verts.data(),
		geom->mesh.getVertCount(),
		geom->meshBoundsMin,
		geom->meshBoundsMax);

	geom->partitionedMesh = {};
	geom->partitionedMesh.PartitionMesh(
		geom->mesh.verts.data(),
		geom->mesh.tris.data(),
		geom->mesh.getTriCount(),
		256);

	resetSettings();
	is_init = true;
	return true;
}

bool Navmesh::build_navmesh()
{
	if (!is_init || !geom || geom->mesh.verts.empty())
	{
		ctx->log(RC_LOG_ERROR, "Navmesh::build_navmesh: not initialized.");
		return false;
	}

	clearNavMesh();

	// Free intermediate Recast structures via RAII-ish locals; allocate fresh.
	rcHeightfield* heightfield = nullptr;
	rcCompactHeightfield* chf = nullptr;
	rcContourSet* cset = nullptr;
	rcPolyMesh* pmesh = nullptr;
	rcPolyMeshDetail* dmesh = nullptr;
	unsigned char* triAreas = nullptr;
	bool success = false;

	const float* boundsMin = geom->getNavMeshBoundsMin();
	const float* boundsMax = geom->getNavMeshBoundsMax();
	const float* verts = geom->mesh.verts.data();
	const int numVerts = geom->mesh.getVertCount();
	const int* tris = geom->mesh.tris.data();
	const int numTris = geom->mesh.getTriCount();

	rcConfig cfg = {};
	cfg.cs = cellSize;
	cfg.ch = cellHeight;
	cfg.walkableSlopeAngle = agentMaxSlope;
	cfg.walkableHeight = static_cast<int>(std::ceil(agentHeight / cfg.ch));
	cfg.walkableClimb = static_cast<int>(std::floor(agentMaxClimb / cfg.ch));
	cfg.walkableRadius = static_cast<int>(std::ceil(agentRadius / cfg.cs));
	cfg.maxEdgeLen = static_cast<int>(edgeMaxLen / cellSize);
	cfg.maxSimplificationError = edgeMaxError;
	cfg.minRegionArea = static_cast<int>(rcSqr(regionMinSize));
	cfg.mergeRegionArea = static_cast<int>(rcSqr(regionMergeSize));
	cfg.maxVertsPerPoly = vertsPerPoly;
	cfg.detailSampleDist = detailSampleDist < 0.9f ? 0.0f : cellSize * detailSampleDist;
	cfg.detailSampleMaxError = cellHeight * detailSampleMaxError;

	rcVcopy(cfg.bmin, boundsMin);
	rcVcopy(cfg.bmax, boundsMax);
	rcCalcGridSize(cfg.bmin, cfg.bmax, cfg.cs, &cfg.width, &cfg.height);

	ctx->resetTimers();
	ctx->startTimer(RC_TIMER_TOTAL);
	ctx->log(RC_LOG_PROGRESS, "Building navigation: %d x %d cells, %d verts / %d tris",
		cfg.width, cfg.height, numVerts, numTris);

	// --- Step 2: Rasterize.
	heightfield = rcAllocHeightfield();
	if (!heightfield ||
	    !rcCreateHeightfield(ctx, *heightfield, cfg.width, cfg.height,
	                         cfg.bmin, cfg.bmax, cfg.cs, cfg.ch))
	{
		ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create heightfield.");
		goto cleanup;
	}

	triAreas = new unsigned char[numTris]();
	rcMarkWalkableTriangles(ctx, cfg.walkableSlopeAngle, verts, numVerts, tris, numTris, triAreas);
	if (!rcRasterizeTriangles(ctx, verts, numVerts, tris, triAreas, numTris,
	                          *heightfield, cfg.walkableClimb))
	{
		ctx->log(RC_LOG_ERROR, "buildNavigation: Could not rasterize triangles.");
		goto cleanup;
	}

	// --- Step 3: Filter.
	rcFilterLowHangingWalkableObstacles(ctx, cfg.walkableClimb, *heightfield);
	rcFilterLedgeSpans(ctx, cfg.walkableHeight, cfg.walkableClimb, *heightfield);
	rcFilterWalkableLowHeightSpans(ctx, cfg.walkableHeight, *heightfield);

	// --- Step 4: Partition.
	chf = rcAllocCompactHeightfield();
	if (!chf ||
	    !rcBuildCompactHeightfield(ctx, cfg.walkableHeight, cfg.walkableClimb,
	                               *heightfield, *chf))
	{
		ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
		goto cleanup;
	}
	if (!rcErodeWalkableArea(ctx, cfg.walkableRadius, *chf))
	{
		ctx->log(RC_LOG_ERROR, "buildNavigation: Could not erode walkable area.");
		goto cleanup;
	}

	for (const ConvexVolume& vol : geom->convexVolumes)
	{
		rcMarkConvexPolyArea(ctx, vol.verts, vol.nverts, vol.hmin, vol.hmax,
		                     static_cast<unsigned char>(vol.area), *chf);
	}

	if (partitionType == 0)  // WATERSHED
	{
		if (!rcBuildDistanceField(ctx, *chf) ||
		    !rcBuildRegions(ctx, *chf, 0, cfg.minRegionArea, cfg.mergeRegionArea))
		{
			ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build watershed regions.");
			goto cleanup;
		}
	}
	else if (partitionType == 1)  // MONOTONE
	{
		if (!rcBuildRegionsMonotone(ctx, *chf, 0, cfg.minRegionArea, cfg.mergeRegionArea))
		{
			ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build monotone regions.");
			goto cleanup;
		}
	}
	else  // LAYERS
	{
		if (!rcBuildLayerRegions(ctx, *chf, 0, cfg.minRegionArea))
		{
			ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build layer regions.");
			goto cleanup;
		}
	}

	// --- Step 5: Contours.
	cset = rcAllocContourSet();
	if (!cset ||
	    !rcBuildContours(ctx, *chf, cfg.maxSimplificationError, cfg.maxEdgeLen, *cset))
	{
		ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build contours.");
		goto cleanup;
	}

	// --- Step 6: Polygonize.
	pmesh = rcAllocPolyMesh();
	if (!pmesh || !rcBuildPolyMesh(ctx, *cset, cfg.maxVertsPerPoly, *pmesh))
	{
		ctx->log(RC_LOG_ERROR, "buildNavigation: Could not triangulate contours.");
		goto cleanup;
	}

	// --- Step 7: Detail mesh.
	dmesh = rcAllocPolyMeshDetail();
	if (!dmesh ||
	    !rcBuildPolyMeshDetail(ctx, *pmesh, *chf, cfg.detailSampleDist,
	                           cfg.detailSampleMaxError, *dmesh))
	{
		ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build detail mesh.");
		goto cleanup;
	}

	// --- Step 8: Build Detour navmesh.
	if (cfg.maxVertsPerPoly <= DT_VERTS_PER_POLYGON)
	{
		// Translate Recast areas to sample area / flag conventions so the
		// query filter we use below produces meaningful results.
		for (int i = 0; i < pmesh->npolys; ++i)
		{
			if (pmesh->areas[i] == RC_WALKABLE_AREA)
			{
				pmesh->areas[i] = NAV_POLYAREA_GROUND;
			}
			const unsigned char a = pmesh->areas[i];
			if (a == NAV_POLYAREA_GROUND || a == NAV_POLYAREA_GRASS || a == NAV_POLYAREA_ROAD)
			{
				pmesh->flags[i] = NAV_POLYFLAGS_WALK;
			}
			else if (a == NAV_POLYAREA_WATER)
			{
				pmesh->flags[i] = NAV_POLYFLAGS_SWIM;
			}
			else if (a == NAV_POLYAREA_DOOR)
			{
				pmesh->flags[i] = NAV_POLYFLAGS_WALK | NAV_POLYFLAGS_DOOR;
			}
		}

		dtNavMeshCreateParams params = {};
		params.verts = pmesh->verts;
		params.vertCount = pmesh->nverts;
		params.polys = pmesh->polys;
		params.polyAreas = pmesh->areas;
		params.polyFlags = pmesh->flags;
		params.polyCount = pmesh->npolys;
		params.nvp = pmesh->nvp;
		params.detailMeshes = dmesh->meshes;
		params.detailVerts = dmesh->verts;
		params.detailVertsCount = dmesh->nverts;
		params.detailTris = dmesh->tris;
		params.detailTriCount = dmesh->ntris;
		params.offMeshConVerts = geom->offmeshConnVerts.data();
		params.offMeshConRad = geom->offmeshConnRadius.data();
		params.offMeshConDir = geom->offmeshConnBidirectional.data();
		params.offMeshConAreas = geom->offmeshConnArea.data();
		params.offMeshConFlags = geom->offmeshConnFlags.data();
		params.offMeshConUserID = geom->offmeshConnId.data();
		params.offMeshConCount = static_cast<int>(geom->offmeshConnArea.size());
		params.walkableHeight = agentHeight;
		params.walkableRadius = agentRadius;
		params.walkableClimb = agentMaxClimb;
		rcVcopy(params.bmin, pmesh->bmin);
		rcVcopy(params.bmax, pmesh->bmax);
		params.cs = cfg.cs;
		params.ch = cfg.ch;
		params.buildBvTree = true;

		unsigned char* navData = nullptr;
		int navDataSize = 0;
		if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
		{
			ctx->log(RC_LOG_ERROR, "buildNavigation: dtCreateNavMeshData failed.");
			goto cleanup;
		}

		navMesh = dtAllocNavMesh();
		if (!navMesh)
		{
			dtFree(navData);
			ctx->log(RC_LOG_ERROR, "buildNavigation: dtAllocNavMesh failed.");
			goto cleanup;
		}
		if (dtStatusFailed(navMesh->init(navData, navDataSize, DT_TILE_FREE_DATA)))
		{
			dtFree(navData);
			dtFreeNavMesh(navMesh);
			navMesh = nullptr;
			ctx->log(RC_LOG_ERROR, "buildNavigation: dtNavMesh::init failed.");
			goto cleanup;
		}
		if (dtStatusFailed(navQuery->init(navMesh, 2048)))
		{
			ctx->log(RC_LOG_ERROR, "buildNavigation: dtNavMeshQuery::init failed.");
			goto cleanup;
		}
	}

	ctx->stopTimer(RC_TIMER_TOTAL);
	success = true;

cleanup:
	delete[] triAreas;
	rcFreeHeightField(heightfield);
	rcFreeCompactHeightfield(chf);
	rcFreeContourSet(cset);
	rcFreePolyMesh(pmesh);
	rcFreePolyMeshDetail(dmesh);

	is_build = success && navMesh != nullptr;
	if (!is_build)
	{
		clearNavMesh();
	}
	return is_build;
}

std::map<std::string, float> Navmesh::get_settings() const
{
	std::map<std::string, float> out;
	if (!is_init)
	{
		ctx->log(RC_LOG_ERROR, "Navmesh::get_settings: not initialized.");
		return out;
	}
	out["cellSize"] = cellSize;
	out["cellHeight"] = cellHeight;
	out["agentHeight"] = agentHeight;
	out["agentRadius"] = agentRadius;
	out["agentMaxClimb"] = agentMaxClimb;
	out["agentMaxSlope"] = agentMaxSlope;
	out["regionMinSize"] = regionMinSize;
	out["regionMergeSize"] = regionMergeSize;
	out["edgeMaxLen"] = edgeMaxLen;
	out["edgeMaxError"] = edgeMaxError;
	out["vertsPerPoly"] = static_cast<float>(vertsPerPoly);
	out["detailSampleDist"] = detailSampleDist;
	out["detailSampleMaxError"] = detailSampleMaxError;
	return out;
}

void Navmesh::set_settings(const std::map<std::string, float>& settings)
{
	if (!is_init)
	{
		ctx->log(RC_LOG_ERROR, "Navmesh::set_settings: not initialized.");
		return;
	}
	for (const auto& [k, v] : settings)
	{
		if (k == "cellSize") { cellSize = std::max(v, 0.0001f); }
		else if (k == "cellHeight") { cellHeight = std::max(v, 0.0001f); }
		else if (k == "agentHeight") { agentHeight = std::max(v, 0.0f); }
		else if (k == "agentRadius") { agentRadius = std::max(v, 0.0f); }
		else if (k == "agentMaxClimb") { agentMaxClimb = v; }
		else if (k == "agentMaxSlope") { agentMaxSlope = v; }
		else if (k == "regionMinSize") { regionMinSize = v; }
		else if (k == "regionMergeSize") { regionMergeSize = v; }
		else if (k == "edgeMaxLen") { edgeMaxLen = v; }
		else if (k == "edgeMaxError") { edgeMaxError = v; }
		else if (k == "vertsPerPoly") { vertsPerPoly = std::clamp(static_cast<int>(v), 3, 12); }
		else if (k == "detailSampleDist") { detailSampleDist = v; }
		else if (k == "detailSampleMaxError") { detailSampleMaxError = v; }
	}
}

int Navmesh::get_partition_type() const { return partitionType; }

void Navmesh::set_partition_type(int type)
{
	partitionType = std::clamp(type, 0, 2);
}

std::vector<float> Navmesh::get_bounding_box() const
{
	std::vector<float> out;
	if (!is_init)
	{
		ctx->log(RC_LOG_ERROR, "Navmesh::get_bounding_box: not initialized.");
		return out;
	}
	out = {
		geom->meshBoundsMin[0], geom->meshBoundsMin[1], geom->meshBoundsMin[2],
		geom->meshBoundsMax[0], geom->meshBoundsMax[1], geom->meshBoundsMax[2]};
	return out;
}

bool Navmesh::save_navmesh(const std::string& file_path)
{
	if (!is_build || !navMesh)
	{
		ctx->log(RC_LOG_ERROR, "Navmesh::save_navmesh: navmesh is not built.");
		return false;
	}
	std::string ext;
	const size_t pos = file_path.find_last_of('.');
	if (pos != std::string::npos)
	{
		ext = file_path.substr(pos);
		std::transform(ext.begin(), ext.end(), ext.begin(),
			[](unsigned char c) { return static_cast<char>(std::tolower(c)); });
	}
	if (ext != ".bin")
	{
		ctx->log(RC_LOG_ERROR, "Navmesh::save_navmesh: file extension must be '.bin'.");
		return false;
	}

	FILE* file = std::fopen(file_path.c_str(), "wb");
	if (!file)
	{
		ctx->log(RC_LOG_ERROR, "Navmesh::save_navmesh: could not open '%s' for writing.", file_path.c_str());
		return false;
	}

	const dtNavMesh* nm = navMesh;
	NavMeshSetHeader header = {};
	header.magic = NAVMESHSET_MAGIC;
	header.version = NAVMESHSET_VERSION;
	header.numTiles = 0;
	for (int i = 0; i < nm->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = nm->getTile(i);
		if (!tile || !tile->header || !tile->dataSize) { continue; }
		header.numTiles++;
	}
	std::memcpy(&header.params, nm->getParams(), sizeof(dtNavMeshParams));
	std::fwrite(&header, sizeof(header), 1, file);

	for (int i = 0; i < nm->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = nm->getTile(i);
		if (!tile || !tile->header || !tile->dataSize) { continue; }
		NavMeshTileHeader th;
		th.tileRef = nm->getTileRef(tile);
		th.dataSize = tile->dataSize;
		std::fwrite(&th, sizeof(th), 1, file);
		std::fwrite(tile->data, tile->dataSize, 1, file);
	}
	std::fclose(file);
	return true;
}

bool Navmesh::load_navmesh(const std::string& file_path)
{
	if (!is_init)
	{
		ctx->log(RC_LOG_ERROR, "Navmesh::load_navmesh: not initialized.");
		return false;
	}

	FILE* file = std::fopen(file_path.c_str(), "rb");
	if (!file)
	{
		ctx->log(RC_LOG_ERROR, "Navmesh::load_navmesh: could not open '%s'.", file_path.c_str());
		return false;
	}

	NavMeshSetHeader header;
	if (std::fread(&header, sizeof(header), 1, file) != 1 ||
	    header.magic != NAVMESHSET_MAGIC ||
	    header.version != NAVMESHSET_VERSION)
	{
		std::fclose(file);
		ctx->log(RC_LOG_ERROR, "Navmesh::load_navmesh: '%s' is not a valid navmesh file.", file_path.c_str());
		return false;
	}

	dtNavMesh* loaded = dtAllocNavMesh();
	if (!loaded || dtStatusFailed(loaded->init(&header.params)))
	{
		std::fclose(file);
		dtFreeNavMesh(loaded);
		ctx->log(RC_LOG_ERROR, "Navmesh::load_navmesh: dtNavMesh::init failed.");
		return false;
	}

	for (int i = 0; i < header.numTiles; ++i)
	{
		NavMeshTileHeader th;
		if (std::fread(&th, sizeof(th), 1, file) != 1)
		{
			std::fclose(file);
			dtFreeNavMesh(loaded);
			ctx->log(RC_LOG_ERROR, "Navmesh::load_navmesh: tile header read failed.");
			return false;
		}
		if (!th.tileRef || !th.dataSize) { break; }

		auto* data = static_cast<unsigned char*>(dtAlloc(th.dataSize, DT_ALLOC_PERM));
		if (!data) { break; }
		std::memset(data, 0, th.dataSize);
		if (std::fread(data, th.dataSize, 1, file) != 1)
		{
			dtFree(data);
			std::fclose(file);
			dtFreeNavMesh(loaded);
			ctx->log(RC_LOG_ERROR, "Navmesh::load_navmesh: tile data read failed.");
			return false;
		}
		loaded->addTile(data, th.dataSize, DT_TILE_FREE_DATA, th.tileRef, nullptr);
	}
	std::fclose(file);

	clearNavMesh();
	navMesh = loaded;
	if (dtStatusFailed(navQuery->init(navMesh, 2048)))
	{
		ctx->log(RC_LOG_ERROR, "Navmesh::load_navmesh: dtNavMeshQuery::init failed.");
		clearNavMesh();
		return false;
	}
	is_build = true;
	return true;
}

std::tuple<std::vector<float>, std::vector<int>> Navmesh::get_navmesh_trianglulation() const
{
	std::vector<float> verts;
	std::vector<int> tris;
	if (!is_build || !navMesh)
	{
		ctx->log(RC_LOG_ERROR, "Navmesh::get_navmesh_trianglulation: navmesh is not built.");
		return {std::move(verts), std::move(tris)};
	}
	const dtNavMesh* nm = navMesh;
	const int maxTiles = nm->getMaxTiles();
	int baseIndex = 0;
	for (int i = 0; i < maxTiles; ++i)
	{
		const dtMeshTile* tile = nm->getTile(i);
		if (!tile || !tile->header) { continue; }
		for (int j = 0; j < tile->header->vertCount; ++j)
		{
			verts.push_back(tile->verts[j * 3 + 0]);
			verts.push_back(tile->verts[j * 3 + 1]);
			verts.push_back(tile->verts[j * 3 + 2]);
		}
		for (int j = 0; j < tile->header->polyCount; ++j)
		{
			const dtPoly* p = &tile->polys[j];
			if (p->getType() == DT_POLYTYPE_OFFMESH_CONNECTION) { continue; }
			const dtPolyDetail* pd = &tile->detailMeshes[j];
			for (int k = 0; k < pd->triCount; ++k)
			{
				const unsigned char* t = &tile->detailTris[(pd->triBase + k) * 4];
				tris.push_back(static_cast<int>(p->verts[t[0]]) + baseIndex);
				tris.push_back(static_cast<int>(p->verts[t[1]]) + baseIndex);
				tris.push_back(static_cast<int>(p->verts[t[2]]) + baseIndex);
			}
		}
		baseIndex += tile->header->vertCount;
	}
	return {std::move(verts), std::move(tris)};
}

std::tuple<std::vector<float>, std::vector<int>, std::vector<int>> Navmesh::get_navmesh_polygonization() const
{
	std::vector<float> verts;
	std::vector<int> polys;
	std::vector<int> sizes;
	if (!is_build || !navMesh)
	{
		ctx->log(RC_LOG_ERROR, "Navmesh::get_navmesh_polygonization: navmesh is not built.");
		return {std::move(verts), std::move(polys), std::move(sizes)};
	}
	const dtNavMesh* nm = navMesh;
	const int maxTiles = nm->getMaxTiles();
	int baseIndex = 0;
	for (int i = 0; i < maxTiles; ++i)
	{
		const dtMeshTile* tile = nm->getTile(i);
		if (!tile || !tile->header) { continue; }
		for (int j = 0; j < tile->header->vertCount; ++j)
		{
			verts.push_back(tile->verts[j * 3 + 0]);
			verts.push_back(tile->verts[j * 3 + 1]);
			verts.push_back(tile->verts[j * 3 + 2]);
		}
		for (int j = 0; j < tile->header->polyCount; ++j)
		{
			const dtPoly* p = &tile->polys[j];
			if (p->getType() == DT_POLYTYPE_OFFMESH_CONNECTION) { continue; }
			for (int k = 0; k < p->vertCount; ++k)
			{
				polys.push_back(static_cast<int>(p->verts[k]) + baseIndex);
			}
			sizes.push_back(p->vertCount);
		}
		baseIndex += tile->header->vertCount;
	}
	return {std::move(verts), std::move(polys), std::move(sizes)};
}

std::vector<float> Navmesh::pathfind_straight(
	const std::vector<float>& start,
	const std::vector<float>& end,
	int vertex_mode) const
{
	std::vector<float> out;
	if (!is_build || !navQuery || !navMesh)
	{
		ctx->log(RC_LOG_ERROR, "Navmesh::pathfind_straight: navmesh is not built.");
		return out;
	}
	if (start.size() != 3 || end.size() != 3)
	{
		ctx->log(RC_LOG_ERROR, "Navmesh::pathfind_straight: start/end must have 3 elements.");
		return out;
	}

	const dtQueryFilter filter = makeDefaultFilter();
	dtPolyRef startRef = 0;
	dtPolyRef endRef = 0;
	float startNearest[3] = {};
	float endNearest[3] = {};
	navQuery->findNearestPoly(start.data(), DEFAULT_POLY_PICK_HALF_EXT, &filter, &startRef, startNearest);
	navQuery->findNearestPoly(end.data(), DEFAULT_POLY_PICK_HALF_EXT, &filter, &endRef, endNearest);
	if (!startRef || !endRef)
	{
		ctx->log(RC_LOG_ERROR, "Navmesh::pathfind_straight: failed to find nearest poly.");
		return out;
	}

	dtPolyRef polys[MAX_POLYS];
	int npolys = 0;
	if (dtStatusFailed(navQuery->findPath(
			startRef, endRef, startNearest, endNearest, &filter, polys, &npolys, MAX_POLYS)))
	{
		ctx->log(RC_LOG_ERROR, "Navmesh::pathfind_straight: findPath failed.");
		return out;
	}
	if (!npolys) { return out; }

	float clampedEnd[3];
	dtVcopy(clampedEnd, endNearest);
	if (polys[npolys - 1] != endRef)
	{
		navQuery->closestPointOnPoly(polys[npolys - 1], endNearest, clampedEnd, nullptr);
	}

	int options = 0;
	if (vertex_mode == 1) { options = DT_STRAIGHTPATH_AREA_CROSSINGS; }
	else if (vertex_mode == 2) { options = DT_STRAIGHTPATH_ALL_CROSSINGS; }

	float straightPath[MAX_POLYS * 3];
	unsigned char straightPathFlags[MAX_POLYS];
	dtPolyRef straightPathPolys[MAX_POLYS];
	int nstraight = 0;
	if (dtStatusFailed(navQuery->findStraightPath(
			startNearest, clampedEnd, polys, npolys,
			straightPath, straightPathFlags, straightPathPolys,
			&nstraight, MAX_POLYS, options)))
	{
		ctx->log(RC_LOG_ERROR, "Navmesh::pathfind_straight: findStraightPath failed.");
		return out;
	}

	out.reserve(static_cast<size_t>(nstraight) * 3);
	for (int i = 0; i < nstraight; ++i)
	{
		out.push_back(straightPath[i * 3 + 0]);
		out.push_back(straightPath[i * 3 + 1]);
		out.push_back(straightPath[i * 3 + 2]);
	}
	return out;
}

std::vector<float> Navmesh::pathfind_straight_batch(const std::vector<float>& coordinates, int vertex_mode) const
{
	std::vector<float> out;
	if (!is_build)
	{
		ctx->log(RC_LOG_ERROR, "Navmesh::pathfind_straight_batch: navmesh is not built.");
		return out;
	}
	if (coordinates.size() % 6 != 0)
	{
		ctx->log(RC_LOG_ERROR, "Navmesh::pathfind_straight_batch: input length must be a multiple of 6.");
		return out;
	}
	const size_t pairs = coordinates.size() / 6;
	for (size_t i = 0; i < pairs; ++i)
	{
		const std::vector<float> s{coordinates[i * 6 + 0], coordinates[i * 6 + 1], coordinates[i * 6 + 2]};
		const std::vector<float> e{coordinates[i * 6 + 3], coordinates[i * 6 + 4], coordinates[i * 6 + 5]};
		std::vector<float> path = pathfind_straight(s, e, vertex_mode);
		out.push_back(static_cast<float>(path.size() / 3));
		out.insert(out.end(), path.begin(), path.end());
	}
	return out;
}

float Navmesh::distance_to_wall(const std::vector<float>& point) const
{
	if (!is_build || !navQuery || !navMesh)
	{
		ctx->log(RC_LOG_ERROR, "Navmesh::distance_to_wall: navmesh is not built.");
		return 0.0f;
	}
	if (point.size() != 3)
	{
		ctx->log(RC_LOG_ERROR, "Navmesh::distance_to_wall: point must have 3 elements.");
		return 0.0f;
	}
	const dtQueryFilter filter = makeDefaultFilter();
	dtPolyRef startRef = 0;
	float nearest[3] = {};
	navQuery->findNearestPoly(point.data(), DEFAULT_POLY_PICK_HALF_EXT, &filter, &startRef, nearest);
	if (!startRef) { return 0.0f; }
	float distance = 0.0f;
	float hitPos[3] = {};
	float hitNormal[3] = {};
	navQuery->findDistanceToWall(startRef, nearest, 100.0f, &filter, &distance, hitPos, hitNormal);
	return distance;
}

std::vector<float> Navmesh::raycast(const std::vector<float>& start, const std::vector<float>& end) const
{
	std::vector<float> out;
	if (!is_build || !navQuery || !navMesh)
	{
		ctx->log(RC_LOG_ERROR, "Navmesh::raycast: navmesh is not built.");
		return out;
	}
	if (start.size() != 3 || end.size() != 3)
	{
		ctx->log(RC_LOG_ERROR, "Navmesh::raycast: start/end must have 3 elements.");
		return out;
	}
	const dtQueryFilter filter = makeDefaultFilter();
	dtPolyRef startRef = 0;
	float startNearest[3] = {};
	navQuery->findNearestPoly(start.data(), DEFAULT_POLY_PICK_HALF_EXT, &filter, &startRef, startNearest);
	if (!startRef) { return out; }

	float t = 0.0f;
	dtPolyRef polys[MAX_POLYS];
	int npolys = 0;
	float hitNormal[3] = {};
	if (dtStatusFailed(navQuery->raycast(
			startRef, startNearest, end.data(), &filter, &t, hitNormal, polys, &npolys, MAX_POLYS)))
	{
		ctx->log(RC_LOG_ERROR, "Navmesh::raycast: dtNavMeshQuery::raycast failed.");
		return out;
	}

	float hitPos[3];
	if (t > 1.0f)
	{
		hitPos[0] = end[0]; hitPos[1] = end[1]; hitPos[2] = end[2];
	}
	else
	{
		hitPos[0] = startNearest[0] + (end[0] - startNearest[0]) * t;
		hitPos[1] = startNearest[1] + (end[1] - startNearest[1]) * t;
		hitPos[2] = startNearest[2] + (end[2] - startNearest[2]) * t;
	}
	if (npolys > 0)
	{
		float h = 0.0f;
		if (dtStatusSucceed(navQuery->getPolyHeight(polys[npolys - 1], hitPos, &h)))
		{
			hitPos[1] = h;
		}
	}

	out = {startNearest[0], startNearest[1], startNearest[2], hitPos[0], hitPos[1], hitPos[2]};
	return out;
}

std::vector<float> Navmesh::hit_mesh(const std::vector<float>& start, const std::vector<float>& end) const
{
	std::vector<float> out;
	if (!is_init || !geom)
	{
		ctx->log(RC_LOG_ERROR, "Navmesh::hit_mesh: not initialized.");
		return out;
	}
	if (start.size() != 3 || end.size() != 3)
	{
		ctx->log(RC_LOG_ERROR, "Navmesh::hit_mesh: start/end must have 3 elements.");
		return out;
	}
	float src[3] = {start[0], start[1], start[2]};
	float dst[3] = {end[0], end[1], end[2]};
	float t = 1.0f;
	if (geom->raycastMesh(src, dst, t))
	{
		out = {start[0] + (end[0] - start[0]) * t,
		       start[1] + (end[1] - start[1]) * t,
		       start[2] + (end[2] - start[2]) * t};
	}
	else
	{
		out = end;
	}
	return out;
}
