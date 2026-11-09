FeatureScript 3112;
import(path : "onshape/std/common.fs", version : "3112.0");

// Plate over selected hole rims; rebuilds when the holes move.
export function buildHoleBracket(context is Context, id is Id, holes is Query, thickness is ValueWithUnits,
    margin is ValueWithUnits, clearance is ValueWithUnits)
{
    const edges = evaluateQuery(context, holes);
    if (size(edges) < 2)
        throw regenError("Select at least two hole edges", ["holes"]);
    var circles = [];
    for (var e in edges)
        circles = append(circles, evCurveDefinition(context, { "edge" : e }));
    const face = qGeometry(qAdjacent(edges[0], AdjacencyType.EDGE, EntityType.FACE), GeometryType.PLANE);
    if (size(evaluateQuery(context, face)) == 0)
        throw regenError("First hole edge must lie on a planar face", ["holes"]);
    const n = evPlane(context, { "face" : face }).normal; // outward: plate grows away from the part
    const o = circles[0].coordSystem.origin;
    var x = circles[1].coordSystem.origin - o;
    x = normalize(x - dot(x, n) * n);
    const pl = plane(o, n, x);
    var lo = worldToPlane(pl, o);
    var hi = lo;
    var rmax = 0 * meter;
    var pts = [];
    for (var c in circles)
    {
        const p = worldToPlane(pl, c.coordSystem.origin);
        pts = append(pts, p);
        lo = vector(min(lo[0], p[0]), min(lo[1], p[1]));
        hi = vector(max(hi[0], p[0]), max(hi[1], p[1]));
        rmax = max(rmax, c.radius);
    }
    const pad = rmax + clearance + margin;
    const sk = newSketchOnPlane(context, id + "sketch", { "sketchPlane" : pl });
    skRectangle(sk, "plate", { "firstCorner" : lo - vector(pad, pad), "secondCorner" : hi + vector(pad, pad) });
    for (var i = 0; i < size(pts); i += 1)
        skCircle(sk, "hole" ~ i, { "center" : pts[i], "radius" : circles[i].radius + clearance });
    skSolve(sk);
    opExtrude(context, id + "extrude", {
                "entities" : qSketchRegion(id + "sketch", true),
                "direction" : n,
                "endBound" : BoundingType.BLIND,
                "endDepth" : thickness });
    opDeleteBodies(context, id + "deleteSketch", { "entities" : qCreatedBy(id + "sketch", EntityType.BODY) });
}

annotation { "Feature Type Name" : "Hole Bracket", "Feature Type Description" : "Plate over selected hole rims, linked to them" }
export const holeBracket = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        annotation { "Name" : "Hole edges", "Filter" : EntityType.EDGE && GeometryType.CIRCLE }
        definition.holes is Query;
        annotation { "Name" : "Thickness" }
        isLength(definition.thickness, { (millimeter) : [0.5, 3, 50] } as LengthBoundSpec);
        annotation { "Name" : "Margin" }
        isLength(definition.margin, { (millimeter) : [0, 4, 50] } as LengthBoundSpec);
        annotation { "Name" : "Clearance" }
        isLength(definition.clearance, { (millimeter) : [0, 0.2, 5] } as LengthBoundSpec);
    }
    {
        buildHoleBracket(context, id, definition.holes, definition.thickness, definition.margin, definition.clearance);
    }, { "holes" : qNothing(), "thickness" : 3 * millimeter, "margin" : 4 * millimeter, "clearance" : 0.2 * millimeter });
