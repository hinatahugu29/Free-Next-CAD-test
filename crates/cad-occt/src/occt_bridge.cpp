#include "cad-occt/include/occt_bridge.h"
#include "cad-occt/src/lib.rs.h"

#include <cmath>
#include <stdexcept>
#include <string>

#include <Standard_Failure.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>

#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>

#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_Transform.hxx>

#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Common.hxx>

#include <BRepOffsetAPI_MakeThickSolid.hxx>
#include <BRepOffsetAPI_DraftAngle.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <BRepOffsetAPI_MakePipe.hxx>
#include <TopTools_ListOfShape.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <ShapeFix_Shape.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>

#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <Poly_Triangulation.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <GCPnts_TangentialDeflection.hxx>

#include <STEPControl_Writer.hxx>
#include <STEPControl_Reader.hxx>
#include <IFSelect_ReturnStatus.hxx>

#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <gp_Dir.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Trsf.hxx>

namespace ferrocad {

namespace {

constexpr double kPi = 3.14159265358979323846;

[[noreturn]] void rethrow_occt_exception(const Standard_Failure& e) {
    const char* msg = e.GetMessageString();
    std::string text = (msg && msg[0] != '\0') ? std::string(msg) : std::string("OCCT Standard_Failure");
    throw std::runtime_error(text);
}

void ensure_valid_positive(double val, const char* name) {
    if (!std::isfinite(val) || val <= 1e-9) {
        throw std::invalid_argument(std::string("Invalid non-positive dimension for ") + name);
    }
}

} // namespace

std::unique_ptr<OcctShapeHandle> clone_shape(const OcctShapeHandle& handle) {
    try {
        BRepBuilderAPI_Copy copier(handle.shape());
        return std::make_unique<OcctShapeHandle>(copier.Shape());
    } catch (const Standard_Failure& e) {
        rethrow_occt_exception(e);
    }
}

std::unique_ptr<OcctShapeHandle> make_box(double dx, double dy, double dz) {
    ensure_valid_positive(dx, "dx");
    ensure_valid_positive(dy, "dy");
    ensure_valid_positive(dz, "dz");
    try {
        BRepPrimAPI_MakeBox maker(dx, dy, dz);
        maker.Build();
        if (!maker.IsDone()) {
            throw std::runtime_error("BRepPrimAPI_MakeBox failed");
        }
        return std::make_unique<OcctShapeHandle>(maker.Shape());
    } catch (const Standard_Failure& e) {
        rethrow_occt_exception(e);
    }
}

std::unique_ptr<OcctShapeHandle> make_cylinder(double radius, double height) {
    ensure_valid_positive(radius, "radius");
    ensure_valid_positive(height, "height");
    try {
        BRepPrimAPI_MakeCylinder maker(radius, height);
        maker.Build();
        if (!maker.IsDone()) {
            throw std::runtime_error("BRepPrimAPI_MakeCylinder failed");
        }
        return std::make_unique<OcctShapeHandle>(maker.Shape());
    } catch (const Standard_Failure& e) {
        rethrow_occt_exception(e);
    }
}

std::unique_ptr<OcctShapeHandle> make_sphere(double radius) {
    ensure_valid_positive(radius, "radius");
    try {
        BRepPrimAPI_MakeSphere maker(radius);
        maker.Build();
        if (!maker.IsDone()) {
            throw std::runtime_error("BRepPrimAPI_MakeSphere failed");
        }
        return std::make_unique<OcctShapeHandle>(maker.Shape());
    } catch (const Standard_Failure& e) {
        rethrow_occt_exception(e);
    }
}

std::unique_ptr<OcctShapeHandle> make_cone(double radius1, double radius2, double height) {
    if (!std::isfinite(radius1) || radius1 < 0.0 || !std::isfinite(radius2) || radius2 < 0.0) {
        throw std::invalid_argument("Cone radii must be non-negative");
    }
    if (std::abs(radius1) <= 1e-9 && std::abs(radius2) <= 1e-9) {
        throw std::invalid_argument("At least one cone radius must be positive");
    }
    ensure_valid_positive(height, "height");
    try {
        BRepPrimAPI_MakeCone maker(radius1, radius2, height);
        maker.Build();
        if (!maker.IsDone()) {
            throw std::runtime_error("BRepPrimAPI_MakeCone failed");
        }
        return std::make_unique<OcctShapeHandle>(maker.Shape());
    } catch (const Standard_Failure& e) {
        rethrow_occt_exception(e);
    }
}

std::unique_ptr<OcctShapeHandle> make_extrude_polygon(
    rust::Slice<const double> coords_xy,
    double depth
) {
    ensure_valid_positive(depth, "depth");
    if (coords_xy.size() < 6 || coords_xy.size() % 2 != 0) {
        throw std::invalid_argument("Polygon requires at least 3 (x, y) vertices");
    }
    try {
        BRepBuilderAPI_MakePolygon poly;
        for (size_t i = 0; i < coords_xy.size(); i += 2) {
            poly.Add(gp_Pnt(coords_xy[i], coords_xy[i + 1], 0.0));
        }
        poly.Close();
        if (!poly.IsDone()) {
            throw std::runtime_error("Failed to construct closed polygon wire");
        }
        BRepBuilderAPI_MakeFace face_maker(poly.Wire(), Standard_True);
        if (!face_maker.IsDone()) {
            throw std::runtime_error("Failed to construct planar face from wire");
        }
        BRepPrimAPI_MakePrism prism(face_maker.Face(), gp_Vec(0.0, 0.0, depth));
        prism.Build();
        if (!prism.IsDone()) {
            throw std::runtime_error("BRepPrimAPI_MakePrism failed");
        }
        return std::make_unique<OcctShapeHandle>(prism.Shape());
    } catch (const Standard_Failure& e) {
        rethrow_occt_exception(e);
    }
}

namespace {
// Builds a closed wire from encoded segments, in the XY plane (or XZ for revolve).
// Encoding per segment: 0 = line (x0, y0, x1, y1); 1 = CCW arc (cx, cy, r, a0, a1) with
// a1 > a0 in radians; 2 = full circle (cx, cy, r).
TopoDS_Wire build_profile_wire(rust::Slice<const double> d, bool xz) {
    auto pnt = [xz](double u, double v) { return xz ? gp_Pnt(u, 0.0, v) : gp_Pnt(u, v, 0.0); };
    // Axis so that the arc parameter runs from +u toward +v.
    auto ax = [xz](double cu, double cv) {
        return xz ? gp_Ax2(gp_Pnt(cu, 0.0, cv), gp_Dir(0.0, -1.0, 0.0), gp_Dir(1.0, 0.0, 0.0))
                  : gp_Ax2(gp_Pnt(cu, cv, 0.0), gp_Dir(0.0, 0.0, 1.0), gp_Dir(1.0, 0.0, 0.0));
    };
    BRepBuilderAPI_MakeWire wire;
    size_t i = 0;
    while (i < d.size()) {
        int kind = static_cast<int>(d[i++]);
        if (kind == 0 && i + 4 <= d.size()) {
            gp_Pnt a = pnt(d[i], d[i + 1]);
            gp_Pnt b = pnt(d[i + 2], d[i + 3]);
            i += 4;
            if (a.Distance(b) < 1e-9) {
                continue;
            }
            wire.Add(BRepBuilderAPI_MakeEdge(a, b).Edge());
        } else if (kind == 1 && i + 5 <= d.size()) {
            gp_Circ circ(ax(d[i], d[i + 1]), d[i + 2]);
            double a0 = d[i + 3];
            double a1 = d[i + 4];
            i += 5;
            wire.Add(BRepBuilderAPI_MakeEdge(circ, a0, a1).Edge());
        } else if (kind == 2 && i + 3 <= d.size()) {
            gp_Circ circ(ax(d[i], d[i + 1]), d[i + 2]);
            i += 3;
            wire.Add(BRepBuilderAPI_MakeEdge(circ).Edge());
        } else {
            throw std::invalid_argument("Malformed profile segment data");
        }
        if (!wire.IsDone()) {
            throw std::runtime_error("Profile segments do not connect into a wire");
        }
    }
    if (!wire.IsDone() || !wire.Wire().Closed()) {
        throw std::runtime_error("Profile is not a closed loop");
    }
    return wire.Wire();
}
}  // namespace

std::unique_ptr<OcctShapeHandle> make_profile_solid(
    rust::Slice<const double> segments,
    double amount,
    bool revolve
) {
    ensure_valid_positive(amount, revolve ? "revolve angle" : "depth");
    try {
        TopoDS_Wire wire = build_profile_wire(segments, revolve);
        BRepBuilderAPI_MakeFace face_maker(wire, Standard_True);
        if (!face_maker.IsDone()) {
            throw std::runtime_error("Failed to construct planar face from profile");
        }
        if (revolve) {
            if (amount > 360.0) {
                throw std::invalid_argument("Revolve angle must be in (0, 360] degrees");
            }
            gp_Ax1 axis(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0));
            BRepPrimAPI_MakeRevol revol(face_maker.Face(), axis, amount * (kPi / 180.0));
            revol.Build();
            if (!revol.IsDone()) {
                throw std::runtime_error("BRepPrimAPI_MakeRevol failed");
            }
            return std::make_unique<OcctShapeHandle>(revol.Shape());
        }
        BRepPrimAPI_MakePrism prism(face_maker.Face(), gp_Vec(0.0, 0.0, amount));
        prism.Build();
        if (!prism.IsDone()) {
            throw std::runtime_error("BRepPrimAPI_MakePrism failed");
        }
        return std::make_unique<OcctShapeHandle>(prism.Shape());
    } catch (const Standard_Failure& e) {
        rethrow_occt_exception(e);
    }
}

std::unique_ptr<OcctShapeHandle> make_loft_profiles(
    rust::Slice<const double> first_segments,
    rust::Slice<const double> second_segments,
    double second_z
) {
    if (!std::isfinite(second_z) || std::abs(second_z) < 1e-9)
        throw std::invalid_argument("Loft section distance must be non-zero");
    try {
        TopoDS_Wire first = build_profile_wire(first_segments, false);
        TopoDS_Wire second = build_profile_wire(second_segments, false);
        gp_Trsf move; move.SetTranslation(gp_Vec(0.0, 0.0, second_z));
        BRepBuilderAPI_Transform transformed(second, move, true);
        BRepOffsetAPI_ThruSections loft(Standard_True, Standard_False);
        loft.AddWire(first);
        loft.AddWire(TopoDS::Wire(transformed.Shape()));
        loft.Build();
        if (!loft.IsDone()) throw std::runtime_error("BRepOffsetAPI_ThruSections failed");
        return std::make_unique<OcctShapeHandle>(loft.Shape());
    } catch (const Standard_Failure& e) { rethrow_occt_exception(e); }
}

std::unique_ptr<OcctShapeHandle> make_extrude_circle(double radius, double depth) {
    ensure_valid_positive(radius, "radius");
    ensure_valid_positive(depth, "depth");
    try {
        gp_Circ circ(gp_Ax2(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)), radius);
        TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(circ).Edge();
        TopoDS_Wire wire = BRepBuilderAPI_MakeWire(edge).Wire();
        TopoDS_Face face = BRepBuilderAPI_MakeFace(wire, Standard_True).Face();
        BRepPrimAPI_MakePrism prism(face, gp_Vec(0.0, 0.0, depth));
        prism.Build();
        if (!prism.IsDone()) {
            throw std::runtime_error("Circle extrude failed");
        }
        return std::make_unique<OcctShapeHandle>(prism.Shape());
    } catch (const Standard_Failure& e) {
        rethrow_occt_exception(e);
    }
}

std::unique_ptr<OcctShapeHandle> make_revolve_polygon(
    rust::Slice<const double> coords_xz,
    double angle_deg
) {
    if (!std::isfinite(angle_deg) || angle_deg <= 1e-6 || angle_deg > 360.0) {
        throw std::invalid_argument("Revolve angle must be in (0, 360] degrees");
    }
    if (coords_xz.size() < 6 || coords_xz.size() % 2 != 0) {
        throw std::invalid_argument("Revolve profile requires at least 3 (x, z) vertices");
    }
    try {
        BRepBuilderAPI_MakePolygon poly;
        for (size_t i = 0; i < coords_xz.size(); i += 2) {
            poly.Add(gp_Pnt(coords_xz[i], 0.0, coords_xz[i + 1]));
        }
        poly.Close();
        if (!poly.IsDone()) {
            throw std::runtime_error("Failed to construct revolve polygon wire");
        }
        BRepBuilderAPI_MakeFace face_maker(poly.Wire(), Standard_True);
        if (!face_maker.IsDone()) {
            throw std::runtime_error("Failed to construct planar face for revolve");
        }
        gp_Ax1 axis(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0));
        double angle_rad = angle_deg * (kPi / 180.0);
        BRepPrimAPI_MakeRevol revol(face_maker.Face(), axis, angle_rad);
        revol.Build();
        if (!revol.IsDone()) {
            throw std::runtime_error("BRepPrimAPI_MakeRevol failed");
        }
        return std::make_unique<OcctShapeHandle>(revol.Shape());
    } catch (const Standard_Failure& e) {
        rethrow_occt_exception(e);
    }
}

std::unique_ptr<OcctShapeHandle> boolean_fuse(
    const OcctShapeHandle& a,
    const OcctShapeHandle& b
) {
    try {
        BRepAlgoAPI_Fuse op(a.shape(), b.shape());
        op.Build();
        if (!op.IsDone() || op.HasErrors()) {
            throw std::runtime_error("BRepAlgoAPI_Fuse failed");
        }
        return std::make_unique<OcctShapeHandle>(op.Shape());
    } catch (const Standard_Failure& e) {
        rethrow_occt_exception(e);
    }
}

std::unique_ptr<OcctShapeHandle> boolean_cut(
    const OcctShapeHandle& a,
    const OcctShapeHandle& b
) {
    try {
        BRepAlgoAPI_Cut op(a.shape(), b.shape());
        op.Build();
        if (!op.IsDone() || op.HasErrors()) {
            throw std::runtime_error("BRepAlgoAPI_Cut failed");
        }
        return std::make_unique<OcctShapeHandle>(op.Shape());
    } catch (const Standard_Failure& e) {
        rethrow_occt_exception(e);
    }
}

std::unique_ptr<OcctShapeHandle> boolean_common(
    const OcctShapeHandle& a,
    const OcctShapeHandle& b
) {
    try {
        BRepAlgoAPI_Common op(a.shape(), b.shape());
        op.Build();
        if (!op.IsDone() || op.HasErrors()) {
            throw std::runtime_error("BRepAlgoAPI_Common failed");
        }
        return std::make_unique<OcctShapeHandle>(op.Shape());
    } catch (const Standard_Failure& e) {
        rethrow_occt_exception(e);
    }
}

std::unique_ptr<OcctShapeHandle> translate_shape(
    const OcctShapeHandle& handle,
    double dx,
    double dy,
    double dz
) {
    if (!std::isfinite(dx) || !std::isfinite(dy) || !std::isfinite(dz)) {
        throw std::invalid_argument("Translation vector contains non-finite values");
    }
    try {
        gp_Trsf trsf;
        trsf.SetTranslation(gp_Vec(dx, dy, dz));
        BRepBuilderAPI_Transform xform(handle.shape(), trsf, Standard_True);
        if (!xform.IsDone()) {
            throw std::runtime_error("BRepBuilderAPI_Transform (translate) failed");
        }
        return std::make_unique<OcctShapeHandle>(xform.Shape());
    } catch (const Standard_Failure& e) {
        rethrow_occt_exception(e);
    }
}

std::unique_ptr<OcctShapeHandle> rotate_shape(
    const OcctShapeHandle& handle,
    double ox,
    double oy,
    double oz,
    double ax,
    double ay,
    double az,
    double angle_deg
) {
    double axis_len = std::sqrt(ax * ax + ay * ay + az * az);
    if (!std::isfinite(axis_len) || axis_len <= 1e-9 || !std::isfinite(angle_deg)) {
        throw std::invalid_argument("Invalid rotation axis or angle");
    }
    try {
        gp_Ax1 axis(gp_Pnt(ox, oy, oz), gp_Dir(ax / axis_len, ay / axis_len, az / axis_len));
        gp_Trsf trsf;
        trsf.SetRotation(axis, angle_deg * (kPi / 180.0));
        BRepBuilderAPI_Transform xform(handle.shape(), trsf, Standard_True);
        if (!xform.IsDone()) {
            throw std::runtime_error("BRepBuilderAPI_Transform (rotate) failed");
        }
        return std::make_unique<OcctShapeHandle>(xform.Shape());
    } catch (const Standard_Failure& e) {
        rethrow_occt_exception(e);
    }
}

std::unique_ptr<OcctShapeHandle> scale_shape(
    const OcctShapeHandle& handle,
    double cx,
    double cy,
    double cz,
    double factor
) {
    ensure_valid_positive(factor, "scale factor");
    try {
        gp_Trsf trsf;
        trsf.SetScale(gp_Pnt(cx, cy, cz), factor);
        BRepBuilderAPI_Transform xform(handle.shape(), trsf, Standard_True);
        if (!xform.IsDone()) {
            throw std::runtime_error("BRepBuilderAPI_Transform (scale) failed");
        }
        return std::make_unique<OcctShapeHandle>(xform.Shape());
    } catch (const Standard_Failure& e) {
        rethrow_occt_exception(e);
    }
}

std::unique_ptr<OcctShapeHandle> mirror_shape(
    const OcctShapeHandle& handle,
    double ox,
    double oy,
    double oz,
    double nx,
    double ny,
    double nz
) {
    double len = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (!std::isfinite(len) || len <= 1e-9) {
        throw std::invalid_argument("Invalid mirror plane normal");
    }
    try {
        gp_Trsf trsf;
        trsf.SetMirror(gp_Ax2(gp_Pnt(ox, oy, oz), gp_Dir(nx / len, ny / len, nz / len)));
        BRepBuilderAPI_Transform xform(handle.shape(), trsf, Standard_True);
        if (!xform.IsDone()) {
            throw std::runtime_error("BRepBuilderAPI_Transform (mirror) failed");
        }
        return std::make_unique<OcctShapeHandle>(xform.Shape());
    } catch (const Standard_Failure& e) {
        rethrow_occt_exception(e);
    }
}

std::unique_ptr<OcctShapeHandle> make_thick_solid(
    const OcctShapeHandle& handle,
    rust::Slice<const uint32_t> removed_face_indices,
    double thickness
) {
    ensure_valid_positive(thickness, "shell thickness");
    try {
        TopTools_IndexedMapOfShape face_map;
        TopExp::MapShapes(handle.shape(), TopAbs_FACE, face_map);
        TopTools_ListOfShape removed;
        for (uint32_t idx : removed_face_indices) {
            int one_based = static_cast<int>(idx) + 1;
            if (one_based < 1 || one_based > face_map.Extent()) {
                throw std::out_of_range("Face index out of range for shell");
            }
            removed.Append(face_map(one_based));
        }
        BRepOffsetAPI_MakeThickSolid maker;
        // Negative offset hollows the solid inward, keeping the outer dimensions.
        maker.MakeThickSolidByJoin(handle.shape(), removed, -thickness, 1.0e-3);
        maker.Build();
        if (!maker.IsDone()) {
            throw std::runtime_error("BRepOffsetAPI_MakeThickSolid failed (wall may be too thick for this shape)");
        }
        return std::make_unique<OcctShapeHandle>(maker.Shape());
    } catch (const Standard_Failure& e) {
        rethrow_occt_exception(e);
    }
}

std::unique_ptr<OcctShapeHandle> fillet_all_edges(
    const OcctShapeHandle& handle,
    double radius
) {
    ensure_valid_positive(radius, "fillet radius");
    try {
        BRepFilletAPI_MakeFillet fillet(handle.shape());
        TopTools_IndexedMapOfShape edge_map;
        TopExp::MapShapes(handle.shape(), TopAbs_EDGE, edge_map);
        if (edge_map.Extent() == 0) {
            throw std::runtime_error("Shape has no edges to fillet");
        }
        for (int i = 1; i <= edge_map.Extent(); ++i) {
            fillet.Add(radius, TopoDS::Edge(edge_map(i)));
        }
        fillet.Build();
        if (!fillet.IsDone()) {
            throw std::runtime_error("BRepFilletAPI_MakeFillet failed for given radius");
        }
        return std::make_unique<OcctShapeHandle>(fillet.Shape());
    } catch (const Standard_Failure& e) {
        rethrow_occt_exception(e);
    }
}

std::unique_ptr<OcctShapeHandle> fillet_selected_edges(
    const OcctShapeHandle& handle,
    rust::Slice<const uint32_t> edge_indices,
    double radius
) {
    ensure_valid_positive(radius, "fillet radius");
    if (edge_indices.empty()) {
        throw std::invalid_argument("No edge indices specified for fillet");
    }
    try {
        BRepFilletAPI_MakeFillet fillet(handle.shape());
        TopTools_IndexedMapOfShape edge_map;
        TopExp::MapShapes(handle.shape(), TopAbs_EDGE, edge_map);
        for (uint32_t idx : edge_indices) {
            int one_based = static_cast<int>(idx) + 1;
            if (one_based < 1 || one_based > edge_map.Extent()) {
                throw std::out_of_range("Edge index out of range for fillet");
            }
            fillet.Add(radius, TopoDS::Edge(edge_map(one_based)));
        }
        fillet.Build();
        if (!fillet.IsDone()) {
            throw std::runtime_error("BRepFilletAPI_MakeFillet failed on selected edges");
        }
        return std::make_unique<OcctShapeHandle>(fillet.Shape());
    } catch (const Standard_Failure& e) {
        rethrow_occt_exception(e);
    }
}

std::unique_ptr<OcctShapeHandle> chamfer_all_edges(
    const OcctShapeHandle& handle,
    double distance
) {
    ensure_valid_positive(distance, "chamfer distance");
    try {
        BRepFilletAPI_MakeChamfer chamfer(handle.shape());
        TopTools_IndexedDataMapOfShapeListOfShape edge_face_map;
        TopExp::MapShapesAndAncestors(handle.shape(), TopAbs_EDGE, TopAbs_FACE, edge_face_map);
        if (edge_face_map.Extent() == 0) {
            throw std::runtime_error("Shape has no edges to chamfer");
        }
        for (int i = 1; i <= edge_face_map.Extent(); ++i) {
            const TopoDS_Edge& edge = TopoDS::Edge(edge_face_map.FindKey(i));
            const TopTools_ListOfShape& faces = edge_face_map.FindFromIndex(i);
            if (!faces.IsEmpty()) {
                chamfer.Add(distance, distance, edge, TopoDS::Face(faces.First()));
            }
        }
        chamfer.Build();
        if (!chamfer.IsDone()) {
            throw std::runtime_error("BRepFilletAPI_MakeChamfer failed for given distance");
        }
        return std::make_unique<OcctShapeHandle>(chamfer.Shape());
    } catch (const Standard_Failure& e) {
        rethrow_occt_exception(e);
    }
}

std::unique_ptr<OcctShapeHandle> chamfer_selected_edges(
    const OcctShapeHandle& handle,
    rust::Slice<const uint32_t> edge_indices,
    double distance
) {
    ensure_valid_positive(distance, "chamfer distance");
    if (edge_indices.empty()) {
        throw std::invalid_argument("No edge indices specified for chamfer");
    }
    try {
        BRepFilletAPI_MakeChamfer chamfer(handle.shape());
        TopTools_IndexedDataMapOfShapeListOfShape edge_face_map;
        TopExp::MapShapesAndAncestors(handle.shape(), TopAbs_EDGE, TopAbs_FACE, edge_face_map);
        for (uint32_t idx : edge_indices) {
            int one_based = static_cast<int>(idx) + 1;
            if (one_based < 1 || one_based > edge_face_map.Extent()) {
                throw std::out_of_range("Edge index out of range for chamfer");
            }
            const TopoDS_Edge& edge = TopoDS::Edge(edge_face_map.FindKey(one_based));
            const TopTools_ListOfShape& faces = edge_face_map.FindFromIndex(one_based);
            if (faces.IsEmpty()) {
                throw std::runtime_error("Edge has no adjacent face for chamfer");
            }
            chamfer.Add(distance, distance, edge, TopoDS::Face(faces.First()));
        }
        chamfer.Build();
        if (!chamfer.IsDone()) {
            throw std::runtime_error("BRepFilletAPI_MakeChamfer failed on selected edges");
        }
        return std::make_unique<OcctShapeHandle>(chamfer.Shape());
    } catch (const Standard_Failure& e) {
        rethrow_occt_exception(e);
    }
}

std::unique_ptr<OcctShapeHandle> draft_faces(
    const OcctShapeHandle& handle,
    rust::Slice<const uint32_t> face_indices,
    uint32_t neutral_face_index,
    double angle_deg,
    bool reversed
) {
    if (face_indices.empty()) throw std::invalid_argument("No faces specified for draft");
    if (!std::isfinite(angle_deg) || std::abs(angle_deg) < 1e-6 || std::abs(angle_deg) >= 89.0)
        throw std::invalid_argument("Draft angle must be between 0 and 89 degrees");
    try {
        TopTools_IndexedMapOfShape faces;
        TopExp::MapShapes(handle.shape(), TopAbs_FACE, faces);
        if (neutral_face_index >= static_cast<uint32_t>(faces.Extent()))
            throw std::out_of_range("Neutral face index out of range for draft");
        TopoDS_Face neutral = TopoDS::Face(faces(neutral_face_index + 1));
        BRepAdaptor_Surface neutral_surface(neutral);
        if (neutral_surface.GetType() != GeomAbs_Plane)
            throw std::invalid_argument("Draft neutral face must be planar");
        const gp_Pln neutral_plane = neutral_surface.Plane();
        BRepOffsetAPI_DraftAngle draft(handle.shape());
        const double radians = (reversed ? -1.0 : 1.0) * angle_deg * kPi / 180.0;
        for (uint32_t index : face_indices) {
            if (index >= static_cast<uint32_t>(faces.Extent()))
                throw std::out_of_range("Draft face index out of range");
            TopoDS_Face face = TopoDS::Face(faces(index + 1));
            if (face.IsSame(neutral)) throw std::invalid_argument("Draft face cannot also be the neutral face");
            draft.Add(face, gp_Dir(0, 0, 1), radians, neutral_plane);
        }
        draft.Build();
        if (!draft.IsDone()) throw std::runtime_error("BRepOffsetAPI_DraftAngle failed");
        return std::make_unique<OcctShapeHandle>(draft.Shape());
    } catch (const Standard_Failure& e) { rethrow_occt_exception(e); }
}

std::unique_ptr<OcctShapeHandle> push_pull_face(
    const OcctShapeHandle& handle, uint32_t face_index, double distance
) {
    if (!std::isfinite(distance) || std::abs(distance) < 1e-9)
        throw std::invalid_argument("Push/Pull distance must be non-zero");
    try {
        TopTools_IndexedMapOfShape faces;
        TopExp::MapShapes(handle.shape(), TopAbs_FACE, faces);
        if (face_index >= static_cast<uint32_t>(faces.Extent())) throw std::out_of_range("Face index out of range");
        TopoDS_Face face = TopoDS::Face(faces(face_index + 1));
        BRepAdaptor_Surface surface(face);
        if (surface.GetType() != GeomAbs_Plane) throw std::invalid_argument("Push/Pull supports planar faces only");
        gp_Dir normal = surface.Plane().Axis().Direction();
        BRepPrimAPI_MakePrism prism(face, gp_Vec(normal) * distance);
        prism.Build();
        if (!prism.IsDone()) throw std::runtime_error("Push/Pull prism failed");
        if (distance > 0.0) { BRepAlgoAPI_Fuse fuse(handle.shape(), prism.Shape()); fuse.Build(); if (!fuse.IsDone()) throw std::runtime_error("Push/Pull fuse failed"); return std::make_unique<OcctShapeHandle>(fuse.Shape()); }
        BRepAlgoAPI_Cut cut(handle.shape(), prism.Shape()); cut.Build(); if (!cut.IsDone()) throw std::runtime_error("Push/Pull cut failed"); return std::make_unique<OcctShapeHandle>(cut.Shape());
    } catch (const Standard_Failure& e) { rethrow_occt_exception(e); }
}

std::unique_ptr<OcctShapeHandle> make_sweep_circle(double radius, double dx, double dy, double dz) {
    ensure_valid_positive(radius, "sweep radius");
    gp_Vec path(dx, dy, dz);
    if (path.SquareMagnitude() < 1e-12) throw std::invalid_argument("Sweep path must have length");
    try {
        gp_Dir tangent(path);
        gp_Ax2 axis(gp_Pnt(0, 0, 0), tangent);
        TopoDS_Edge profile_edge = BRepBuilderAPI_MakeEdge(gp_Circ(axis, radius)).Edge();
        TopoDS_Wire profile = BRepBuilderAPI_MakeWire(profile_edge).Wire();
        TopoDS_Edge path_edge = BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, 0), gp_Pnt(dx, dy, dz)).Edge();
        TopoDS_Wire spine = BRepBuilderAPI_MakeWire(path_edge).Wire();
        BRepOffsetAPI_MakePipe pipe(spine, profile);
        pipe.Build();
        if (!pipe.IsDone()) throw std::runtime_error("BRepOffsetAPI_MakePipe failed");
        return std::make_unique<OcctShapeHandle>(pipe.Shape());
    } catch (const Standard_Failure& e) { rethrow_occt_exception(e); }
}

double compute_volume(const OcctShapeHandle& handle) {
    try {
        GProp_GProps props;
        BRepGProp::VolumeProperties(handle.shape(), props);
        return props.Mass();
    } catch (const Standard_Failure& e) {
        rethrow_occt_exception(e);
    }
}

double compute_surface_area(const OcctShapeHandle& handle) {
    try {
        GProp_GProps props;
        BRepGProp::SurfaceProperties(handle.shape(), props);
        return props.Mass();
    } catch (const Standard_Failure& e) {
        rethrow_occt_exception(e);
    }
}

RawBBox compute_bounding_box(const OcctShapeHandle& handle) {
    try {
        Bnd_Box box;
        BRepBndLib::Add(handle.shape(), box);
        if (box.IsVoid()) {
            throw std::runtime_error("BoundingBox is void");
        }
        Standard_Real xmin = 0, ymin = 0, zmin = 0, xmax = 0, ymax = 0, zmax = 0;
        box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
        return RawBBox{xmin, ymin, zmin, xmax, ymax, zmax};
    } catch (const Standard_Failure& e) {
        rethrow_occt_exception(e);
    }
}

bool check_shape_valid(const OcctShapeHandle& handle) {
    try {
        if (handle.shape().IsNull()) {
            return false;
        }
        BRepCheck_Analyzer analyzer(handle.shape());
        return analyzer.IsValid() == Standard_True;
    } catch (const Standard_Failure& e) {
        rethrow_occt_exception(e);
    }
}

std::unique_ptr<OcctShapeHandle> heal_shape(const OcctShapeHandle& handle) {
    try {
        ShapeFix_Shape fixer(handle.shape());
        fixer.Perform();
        TopoDS_Shape healed = fixer.Shape();
        if (healed.IsNull()) throw std::runtime_error("Shape healing returned an empty shape");
        return std::make_unique<OcctShapeHandle>(healed);
    } catch (const Standard_Failure& e) { rethrow_occt_exception(e); }
}

std::unique_ptr<OcctShapeHandle> refine_shape(const OcctShapeHandle& handle) {
    try {
        ShapeUpgrade_UnifySameDomain unifier(handle.shape(), Standard_True, Standard_True, Standard_False);
        unifier.Build();
        TopoDS_Shape refined = unifier.Shape();
        if (refined.IsNull()) throw std::runtime_error("Refine returned an empty shape");
        return std::make_unique<OcctShapeHandle>(refined);
    } catch (const Standard_Failure& e) { rethrow_occt_exception(e); }
}

RawTopoCounts get_topology_counts(const OcctShapeHandle& handle) {
    try {
        TopTools_IndexedMapOfShape solids, faces, edges, vertices;
        TopExp::MapShapes(handle.shape(), TopAbs_SOLID, solids);
        TopExp::MapShapes(handle.shape(), TopAbs_FACE, faces);
        TopExp::MapShapes(handle.shape(), TopAbs_EDGE, edges);
        TopExp::MapShapes(handle.shape(), TopAbs_VERTEX, vertices);
        return RawTopoCounts{
            static_cast<uint32_t>(solids.Extent()),
            static_cast<uint32_t>(faces.Extent()),
            static_cast<uint32_t>(edges.Extent()),
            static_cast<uint32_t>(vertices.Extent())
        };
    } catch (const Standard_Failure& e) {
        rethrow_occt_exception(e);
    }
}

rust::Vec<RawFaceInfo> inspect_faces(const OcctShapeHandle& handle) {
    try {
        rust::Vec<RawFaceInfo> result;
        TopTools_IndexedMapOfShape face_map;
        TopExp::MapShapes(handle.shape(), TopAbs_FACE, face_map);
        for (int i = 1; i <= face_map.Extent(); ++i) {
            const TopoDS_Face& face = TopoDS::Face(face_map(i));
            GProp_GProps props;
            BRepGProp::SurfaceProperties(face, props);
            gp_Pnt centre = props.CentreOfMass();

            BRepAdaptor_Surface surf(face, Standard_True);
            double u_mid = 0.5 * (surf.FirstUParameter() + surf.LastUParameter());
            double v_mid = 0.5 * (surf.FirstVParameter() + surf.LastVParameter());
            gp_Pnt p;
            gp_Vec du, dv;
            surf.D1(u_mid, v_mid, p, du, dv);
            gp_Vec n = du.Crossed(dv);
            double nx = 0.0, ny = 0.0, nz = 1.0;
            if (n.Magnitude() > 1e-9) {
                n.Normalize();
                if (face.Orientation() == TopAbs_REVERSED) {
                    n.Reverse();
                }
                nx = n.X();
                ny = n.Y();
                nz = n.Z();
            }
            result.push_back(RawFaceInfo{
                static_cast<uint32_t>(i - 1),
                props.Mass(),
                centre.X(),
                centre.Y(),
                centre.Z(),
                nx,
                ny,
                nz
            });
        }
        return result;
    } catch (const Standard_Failure& e) {
        rethrow_occt_exception(e);
    }
}

rust::Vec<RawEdgeInfo> inspect_edges(const OcctShapeHandle& handle) {
    try {
        rust::Vec<RawEdgeInfo> result;
        TopTools_IndexedMapOfShape edge_map;
        TopExp::MapShapes(handle.shape(), TopAbs_EDGE, edge_map);
        for (int i = 1; i <= edge_map.Extent(); ++i) {
            const TopoDS_Edge& edge = TopoDS::Edge(edge_map(i));
            GProp_GProps props;
            BRepGProp::LinearProperties(edge, props);
            gp_Pnt centre = props.CentreOfMass();
            result.push_back(RawEdgeInfo{
                static_cast<uint32_t>(i - 1),
                props.Mass(),
                centre.X(),
                centre.Y(),
                centre.Z()
            });
        }
        return result;
    } catch (const Standard_Failure& e) {
        rethrow_occt_exception(e);
    }
}

void write_step_file(const OcctShapeHandle& handle, rust::Str file_path) {
    std::string path_str(file_path.data(), file_path.size());
    try {
        STEPControl_Writer writer;
        IFSelect_ReturnStatus transfer_status =
            writer.Transfer(handle.shape(), STEPControl_AsIs);
        if (transfer_status != IFSelect_RetDone) {
            throw std::runtime_error("STEPControl_Writer::Transfer failed");
        }
        IFSelect_ReturnStatus write_status = writer.Write(path_str.c_str());
        if (write_status != IFSelect_RetDone) {
            throw std::runtime_error("STEPControl_Writer::Write failed: " + path_str);
        }
    } catch (const Standard_Failure& e) {
        rethrow_occt_exception(e);
    }
}

std::unique_ptr<OcctShapeHandle> read_step_file(rust::Str file_path) {
    std::string path_str(file_path.data(), file_path.size());
    try {
        STEPControl_Reader reader;
        IFSelect_ReturnStatus read_status = reader.ReadFile(path_str.c_str());
        if (read_status != IFSelect_RetDone) {
            throw std::runtime_error("STEPControl_Reader::ReadFile failed: " + path_str);
        }
        reader.TransferRoots();
        TopoDS_Shape shape = reader.OneShape();
        if (shape.IsNull()) {
            throw std::runtime_error("STEPControl_Reader produced null shape");
        }
        return std::make_unique<OcctShapeHandle>(shape);
    } catch (const Standard_Failure& e) {
        rethrow_occt_exception(e);
    }
}

RawMeshResult tessellate_shape(
    const OcctShapeHandle& handle,
    double linear_deflection,
    double angular_deflection
) {
    ensure_valid_positive(linear_deflection, "linear_deflection");
    ensure_valid_positive(angular_deflection, "angular_deflection");
    try {
        BRepMesh_IncrementalMesh mesher(
            handle.shape(),
            linear_deflection,
            Standard_False,
            angular_deflection,
            Standard_True
        );

        RawMeshResult out;
        TopTools_IndexedMapOfShape face_map;
        TopExp::MapShapes(handle.shape(), TopAbs_FACE, face_map);

        for (int face_idx = 1; face_idx <= face_map.Extent(); ++face_idx) {
            const TopoDS_Face& face = TopoDS::Face(face_map(face_idx));
            TopLoc_Location loc;
            Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
            if (tri.IsNull()) {
                continue;
            }

            const gp_Trsf trsf = loc.Transformation();
            const bool reversed = (face.Orientation() == TopAbs_REVERSED);
            const uint32_t base_vertex = static_cast<uint32_t>(out.positions.size() / 3);
            const int nb_nodes = tri->NbNodes();
            const int nb_tris = tri->NbTriangles();

            // Accumulate per-node normals from triangles
            std::vector<gp_Vec> node_normals(static_cast<size_t>(nb_nodes), gp_Vec(0.0, 0.0, 0.0));
            std::vector<gp_Pnt> transformed_nodes(static_cast<size_t>(nb_nodes));

            for (int n = 1; n <= nb_nodes; ++n) {
                transformed_nodes[static_cast<size_t>(n - 1)] = tri->Node(n).Transformed(trsf);
            }

            for (int t = 1; t <= nb_tris; ++t) {
                int i1 = 0, i2 = 0, i3 = 0;
                tri->Triangle(t).Get(i1, i2, i3);
                if (reversed) {
                    std::swap(i2, i3);
                }
                const gp_Pnt& p1 = transformed_nodes[static_cast<size_t>(i1 - 1)];
                const gp_Pnt& p2 = transformed_nodes[static_cast<size_t>(i2 - 1)];
                const gp_Pnt& p3 = transformed_nodes[static_cast<size_t>(i3 - 1)];
                gp_Vec v12(p1, p2);
                gp_Vec v13(p1, p3);
                gp_Vec fn = v12.Crossed(v13);
                node_normals[static_cast<size_t>(i1 - 1)] += fn;
                node_normals[static_cast<size_t>(i2 - 1)] += fn;
                node_normals[static_cast<size_t>(i3 - 1)] += fn;

                out.indices.push_back(base_vertex + static_cast<uint32_t>(i1 - 1));
                out.indices.push_back(base_vertex + static_cast<uint32_t>(i2 - 1));
                out.indices.push_back(base_vertex + static_cast<uint32_t>(i3 - 1));
                out.triangle_face_ids.push_back(static_cast<uint32_t>(face_idx - 1));
            }

            for (int n = 0; n < nb_nodes; ++n) {
                const gp_Pnt& p = transformed_nodes[static_cast<size_t>(n)];
                out.positions.push_back(static_cast<float>(p.X()));
                out.positions.push_back(static_cast<float>(p.Y()));
                out.positions.push_back(static_cast<float>(p.Z()));

                gp_Vec norm = node_normals[static_cast<size_t>(n)];
                if (norm.Magnitude() > 1e-12) {
                    norm.Normalize();
                } else {
                    norm = gp_Vec(0.0, 0.0, 1.0);
                }
                out.normals.push_back(static_cast<float>(norm.X()));
                out.normals.push_back(static_cast<float>(norm.Y()));
                out.normals.push_back(static_cast<float>(norm.Z()));
            }
        }

        // Discretize B-Rep topological edges for crisp CAD wireframe/boundary display & edge picking
        TopTools_IndexedMapOfShape edge_map;
        TopExp::MapShapes(handle.shape(), TopAbs_EDGE, edge_map);
        for (int edge_idx = 1; edge_idx <= edge_map.Extent(); ++edge_idx) {
            const TopoDS_Edge& edge = TopoDS::Edge(edge_map(edge_idx));
            if (BRep_Tool::Degenerated(edge)) {
                continue;
            }
            try {
                BRepAdaptor_Curve curve(edge);
                GCPnts_TangentialDeflection discretizer(
                    curve,
                    angular_deflection,
                    linear_deflection
                );
                int npts = discretizer.NbPoints();
                for (int k = 1; k < npts; ++k) {
                    gp_Pnt a = discretizer.Value(k);
                    gp_Pnt b = discretizer.Value(k + 1);
                    out.edge_positions.push_back(static_cast<float>(a.X()));
                    out.edge_positions.push_back(static_cast<float>(a.Y()));
                    out.edge_positions.push_back(static_cast<float>(a.Z()));
                    out.edge_positions.push_back(static_cast<float>(b.X()));
                    out.edge_positions.push_back(static_cast<float>(b.Y()));
                    out.edge_positions.push_back(static_cast<float>(b.Z()));
                    out.edge_segment_ids.push_back(static_cast<uint32_t>(edge_idx - 1));
                }
            } catch (...) {
                // Skip non-discretizable edge gracefully
            }
        }

        return out;
    } catch (const Standard_Failure& e) {
        rethrow_occt_exception(e);
    }
}

} // namespace ferrocad
