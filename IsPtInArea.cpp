// Point-in-polygon test on the XY plane.
// Returns true if pt lies inside or on the boundary of the polygon defined by pt3dArr.
// The polygon is given by a sequence of vertices in pt3dArr (not necessarily closed).
// Implementation:
//   1) Quick reject with an axis-aligned bounding box.
//   2) Cast a ray to the +X direction and count intersections with polygon edges.
//      An odd number of intersections => inside; even => outside.
//   3) Uses AcGeTol to handle floating-point tolerances and boundary cases.

bool IsPtInArea(AcGePoint3d pt, AcGePoint3dArray& pt3dArr)
{
    int iLen = pt3dArr.length();
    if (iLen < 3)
        return false;

    // Build an axis-aligned bounding box and get min/max X/Y
    int i;
    double dblMaxX = pt3dArr[0].x;
    double dblMinX = pt3dArr[0].x;
    double dblMaxY = pt3dArr[0].y;
    double dblMinY = pt3dArr[0].y;
    for (i = 1; i < iLen; i++)
    {
        if (pt3dArr[i].x > dblMaxX)
            dblMaxX = pt3dArr[i].x;
        if (pt3dArr[i].x < dblMinX)
            dblMinX = pt3dArr[i].x;
        if (pt3dArr[i].y > dblMaxY)
            dblMaxY = pt3dArr[i].y;
        if (pt3dArr[i].y < dblMinY)
            dblMinY = pt3dArr[i].y;
    }

    // If the point is outside the bounding box, it cannot be inside the polygon
    if (Compfloat(pt.x, dblMaxX) > 0 || Compfloat(pt.x, dblMinX) < 0
        || Compfloat(pt.y, dblMaxY) > 0 || Compfloat(pt.y, dblMinY) < 0)
    {
        return false;
    }

    // Cast a ray from pt towards +X direction on the XY plane
    pt.z = 0.0;
    AcGePoint3d xpt = pt;
    xpt.x = dblMaxX + 10.0;   // any point guaranteed to be outside the bbox is fine
    //xpt.x = 1.7e208;
    AcGeLineSeg3d lineseg3d(pt, xpt);

    AcGePoint3d p1 = pt3dArr.first();
    p1.z = 0.0;

    AcGeTol tol;
    tol.setEqualVector(0.01);
    tol.setEqualPoint(0.01);

    // Ensure the polygon is closed by appending the first point if needed
    bool bAdd = false;
    if (!pt3dArr[iLen - 1].isEqualTo(p1, tol))
    {
        pt3dArr.append(p1);
        iLen++;
        bAdd = true;
    }

    AcGePoint3d p2;
    int nCount = 0;

    for (i = 1; i < iLen; i++)
    {
        p2 = pt3dArr[i];
        p2.z = 0.0;

        // If the test point coincides with a vertex, treat it as inside
        if (pt.isEqualTo(p1, tol))
        {
            if (bAdd)
                pt3dArr.removeLast();
            return true;
        }

        AcGeLineSeg3d xlineseg3d(p1, p2);

        // If the test point lies exactly on an edge, treat it as inside
        if (xlineseg3d.isOn(pt, tol) == Adesk::kTrue)
        {
            if (bAdd)
                pt3dArr.removeLast();
            return true;
        }

        AcGePoint3d ipt;

        // Count intersections between the ray and this edge
        if (lineseg3d.intersectWith(xlineseg3d, ipt, tol) == Adesk::kTrue)
        {
            // Handle vertex intersection cases carefully:
            // only count the intersection when the edge crosses the ray
            if (ipt.isEqualTo(p1, tol))
            {
                if (p2.y > ipt.y)
                    nCount++;
            }
            else if (ipt.isEqualTo(p2, tol))
            {
                if (p1.y > ipt.y)
                    nCount++;
            }
            else
            {
                nCount++;
            }
        }

        p1 = p2;
    }

    if (bAdd)
        pt3dArr.removeLast();

    // Odd number of intersections => inside; even => outside
    if ((nCount % 2) == 0)
        return false;
    else
        return true;
}
