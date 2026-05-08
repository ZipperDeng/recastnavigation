#include "Navmesh.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(PyNavmesh, m)
{
	m.doc() =
		"Python binding for RecastNavigation's solo-mesh sample. Build a navmesh "
		"from an .obj file (or raw vertex/face arrays), then run pathfinding, "
		"raycasts, distance-to-wall queries, etc. See "
		"https://github.com/recastnavigation/recastnavigation for the underlying "
		"library.";

	py::class_<Navmesh>(m, "Navmesh")
		.def(py::init<>())

		// --- initialisation ----------------------------------------------------
		.def("init_by_obj", &Navmesh::init_by_obj, py::arg("file_path"),
		     "Load geometry from an .obj or .gset file. Returns True on success.")
		.def("init_by_raw", &Navmesh::init_by_raw, py::arg("vertices"), py::arg("faces"),
		     "Load geometry from flat arrays: vertices[3*N] and faces[3*M]. Returns True on success.")
		.def("build_navmesh", &Navmesh::build_navmesh,
		     "Build the navmesh using the current settings. Returns True on success.")
		.def("get_log", &Navmesh::get_log,
		     "Return all queued context log lines and clear them.")

		// --- settings ----------------------------------------------------------
		.def("get_settings", &Navmesh::get_settings,
		     "Return a dict of build settings (cellSize, agentRadius, ...).")
		.def("set_settings", &Navmesh::set_settings, py::arg("settings"),
		     "Override one or more build settings using a dict.")
		.def("get_partition_type", &Navmesh::get_partition_type,
		     "0 = WATERSHED, 1 = MONOTONE, 2 = LAYERS.")
		.def("set_partition_type", &Navmesh::set_partition_type, py::arg("type"))

		// --- introspection -----------------------------------------------------
		.def("get_bounding_box", &Navmesh::get_bounding_box,
		     "Return [minX, minY, minZ, maxX, maxY, maxZ] of the input geometry.")
		.def("get_navmesh_trianglulation", &Navmesh::get_navmesh_trianglulation,
		     "Return (vertices, triIndices) of the built navmesh.")
		.def("get_navmesh_polygonization", &Navmesh::get_navmesh_polygonization,
		     "Return (vertices, polyIndices, polySizes) of the built navmesh.")

		// --- persistence -------------------------------------------------------
		.def("save_navmesh", &Navmesh::save_navmesh, py::arg("file_path"),
		     "Save the built navmesh to a .bin file.")
		.def("load_navmesh", &Navmesh::load_navmesh, py::arg("file_path"),
		     "Load a previously-saved .bin navmesh. Requires init_by_*.")

		// --- queries -----------------------------------------------------------
		.def("pathfind_straight", &Navmesh::pathfind_straight,
		     py::arg("start"), py::arg("end"), py::arg("vertex_mode") = 0,
		     "Compute a straight (funnel) path. vertex_mode: 0 corners only, "
		     "1 add area-crossings, 2 add all-crossings.")
		.def("pathfind_straight_batch", &Navmesh::pathfind_straight_batch,
		     py::arg("coordinates"), py::arg("vertex_mode") = 0,
		     "Run pathfind_straight on N (start,end) pairs packed in a flat array of length 6*N.")
		.def("distance_to_wall", &Navmesh::distance_to_wall, py::arg("point"))
		.def("raycast", &Navmesh::raycast, py::arg("start"), py::arg("end"),
		     "Raycast across the navmesh. Returns [startX,startY,startZ, hitX,hitY,hitZ].")
		.def("hit_mesh", &Navmesh::hit_mesh, py::arg("start"), py::arg("end"),
		     "Raycast against the input geometry (not the navmesh).");
}
