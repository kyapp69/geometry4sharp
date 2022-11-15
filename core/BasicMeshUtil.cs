using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace g4.core
{
    internal class BasicMeshUtil
    {
        public static bool ReduceMesh(DMesh3 mesh, float edgeLength, int triangleCount)
        {
            Reducer r = new Reducer(mesh);

            DMeshAABBTree3 tree = new DMeshAABBTree3(new DMesh3(mesh));
            tree.Build();


            //MeshConstraints cons = new MeshConstraints();
            //EdgeRefineFlags useFlags = EdgeRefineFlags.NoFlip;
            //foreach (int eid in mesh.EdgeIndices())
            //{
            //    double fAngle = MeshUtil.OpeningAngleD(mesh, eid);
            //    if (fAngle > contraintAngle)
            //    {
            //        cons.SetOrUpdateEdgeConstraint(eid, new EdgeConstraint(useFlags));
            //        Index2i ev = mesh.GetEdgeV(eid);
            //        int nSetID0 = (mesh.GetVertex(ev[0]).y > 1) ? 1 : 2;
            //        int nSetID1 = (mesh.GetVertex(ev[1]).y > 1) ? 1 : 2;
            //        cons.SetOrUpdateVertexConstraint(ev[0], new VertexConstraint(true, nSetID0));
            //        cons.SetOrUpdateVertexConstraint(ev[1], new VertexConstraint(true, nSetID1));
            //    }
            //}
            

            if (triangleCount > 0)
            {
                r.ReduceToTriangleCount(triangleCount);
            }
            else
            {
                r.ReduceToEdgeLength(edgeLength);
            }


            return true;
        }

        private class MeshNode
        {

            public int meshIndex;
            public int index;
            public g4.Frame3f frame;
            public g4.Index3i neighbors_index;
            public g4.Index3i vertex_index;
            public List<MeshNode> neighbors = new List<MeshNode>();
            public bool locked = false;

            internal MeshNode(int i, int fi, g4.Frame3f f, g4.Index3i neighbors_index, g4.Index3i vertex_index)
            {
                frame = f;
                this.neighbors_index = neighbors_index;
                this.vertex_index = vertex_index;
                meshIndex = fi;
                index = i;
            }

            internal bool UsesVertex(int vi)
            {
                return vertex_index[0] == vi || vertex_index[1] == vi || vertex_index[2] == vi;

            }

            internal bool Randomize(g4.DMesh3 mesh, DMeshAABBTree3 tree, Random r, double max, double moveTries, double average)
            {
                bool result = false;

                for (int i = 0; i < moveTries; i++)
                {
                    result |= this.RandomAdjust(mesh, tree, r, max, moveTries, average);

                    //foreach (var n in neighbors)
                    //    result |= n.RandomAdjust(mesh, tree, r, max, moveTries, average);
                }

                return result;
            }

            internal double CompuateAverageArea(g4.DMesh3 mesh)
            {
                double area = this.TriangleArea(mesh);

                foreach (var n in neighbors)
                    area += n.TriangleArea(mesh);

                area /= (neighbors.Count + 1);

                return area;
            }

            internal double HowCloseToTargetArea(g4.DMesh3 mesh, double targetArea, int depth)
            {
                double area = Math.Pow(Math.Abs(this.TriangleArea(mesh) - targetArea) / targetArea, 3) * 3;

                if (depth == 0)
                    return area;

                foreach (var n in neighbors)
                    area += n.HowCloseToTargetArea(mesh, targetArea, depth - 1);

                return area / (neighbors.Count + 1);
            }

            internal double TriangleArea(g4.DMesh3 mesh)
            {
                return mesh.GetTriArea(meshIndex);
            }

            internal double GetTriangleAnglesQuality(g4.DMesh3 mesh)
            {
                Vector3d v0 = Vector3d.Zero, v1 = Vector3d.Zero, v2 = Vector3d.Zero;
                mesh.GetTriVertices(meshIndex, ref v0, ref v1, ref v2);

                Vector3d anglesD = Vector3d.Zero;

                Vector3d e00 = (v1 - v0);
                e00.Normalize();
                Vector3d e01 = (v2 - v0);
                e01.Normalize();
                anglesD.x = Vector3d.AngleD(e00, e01);

                Vector3d e10 = (v0 - v1);
                e10.Normalize();
                Vector3d e11 = (v2 - v1);
                e11.Normalize();
                anglesD.y = Vector3d.AngleD(e10, e11);

                anglesD.z = 180 - anglesD.x - anglesD.y;

                double resultA = Math.Min(Math.Min(Math.Abs(anglesD.x - 90) / 10.0, Math.Abs(anglesD.y - 90) / 10.0), Math.Abs(anglesD.z - 90) / 10.0);

                double resultB = Math.Abs(anglesD.x - 60) / 30.0 + Math.Abs(anglesD.y - 60) / 30.0 + Math.Abs(anglesD.z - 60) / 30.0;

                double result = Math.Min(resultA, resultB);

                return Math.Pow(result, 3);
            }

            internal double GetTriangleTotalAnglesQualityHelper(g4.DMesh3 mesh, int depth)
            {
                double total = GetTriangleAnglesQuality(mesh);

                if (depth == 0)
                    return total;

                foreach (var n in neighbors)
                    total += GetTriangleTotalAnglesQualityHelper(mesh, depth - 1);

                return total / (neighbors.Count + 1);
            }

            double GetNormalQuality(g4.DMesh3 mesh, g4.Vector3d target, int depth)
            {
                if (depth == 0)
                    return 1 - (mesh.GetTriNormal(meshIndex).Dot(target));

                double amount = GetNormalQuality(mesh, target, 0);

                foreach (var n in neighbors)
                    amount += GetNormalQuality(mesh, target, depth - 1);

                return amount;
            }

            internal bool RandomAdjust(g4.DMesh3 mesh, DMeshAABBTree3 tree, Random r, double max, double moveTries, double targetArea)
            {
                bool moved = false;

                if (this.locked)
                    return false;

                for (int i = 0; i < moveTries; i++)
                {
                    var v0 = mesh.GetVertex(vertex_index.a);
                    var v1 = mesh.GetVertex(vertex_index.b);
                    var v2 = mesh.GetVertex(vertex_index.c);

                    var v0_old = mesh.GetVertex(vertex_index.a);
                    var v1_old = mesh.GetVertex(vertex_index.b);
                    var v2_old = mesh.GetVertex(vertex_index.c);

                    v0.x += (r.NextDouble() * max * 2 - max);
                    v0.y += (r.NextDouble() * max * 2 - max);
                    v0.z += (r.NextDouble() * max * 2 - max);

                    v1.x += (r.NextDouble() * max * 2 - max);
                    v1.y += (r.NextDouble() * max * 2 - max);
                    v1.z += (r.NextDouble() * max * 2 - max);

                    v2.x += (r.NextDouble() * max * 2 - max);
                    v2.y += (r.NextDouble() * max * 2 - max);
                    v2.z += (r.NextDouble() * max * 2 - max);

                    int tNearestID = tree.FindNearestTriangle(v0);
                    DistPoint3Triangle3 q = MeshQueries.TriangleDistance(tree.Mesh, tNearestID, v0);
                    v0 = q.TriangleClosest;

                    tNearestID = tree.FindNearestTriangle(v1);
                    q = MeshQueries.TriangleDistance(tree.Mesh, tNearestID, v1);
                    v1 = q.TriangleClosest;

                    tNearestID = tree.FindNearestTriangle(v2);
                    q = MeshQueries.TriangleDistance(tree.Mesh, tNearestID, v2);
                    v2 = q.TriangleClosest;

                    double oldArea = (HowCloseToTargetArea(mesh, targetArea, 2) / targetArea) * 3;

                    double oldAngleQuality = GetTriangleTotalAnglesQualityHelper(mesh, 2);

                    var n = mesh.GetTriNormal(meshIndex);

                    double oldNormalQuality = GetNormalQuality(mesh, n, 2) * 6;

                    mesh.SetVertex(vertex_index.a, v0);
                    mesh.SetVertex(vertex_index.b, v1);
                    mesh.SetVertex(vertex_index.c, v2);

                    double newArea = (HowCloseToTargetArea(mesh, targetArea, 2) / targetArea) * 3;
                    double newAngleQuality = GetTriangleTotalAnglesQualityHelper(mesh, 2);
                    double newNormalQuality = GetNormalQuality(mesh, n, 2) * 6;

                    if ((oldArea + oldAngleQuality + oldNormalQuality) < (newArea + newAngleQuality + newNormalQuality))
                    {
                        mesh.SetVertex(vertex_index.a, v0_old);
                        mesh.SetVertex(vertex_index.b, v1_old);
                        mesh.SetVertex(vertex_index.c, v2_old);
                    }
                    else
                    {
                        moved = true;

                    }
                }

                return moved;
            }
        }

        public static bool RandomizeMesh(g4.DMesh3 mesh, out g4.DMesh3 outputMesh, double amount, double moveTries)
        {
            System.Collections.Generic.SortedDictionary<int, MeshNode> faces = new System.Collections.Generic.SortedDictionary<int, MeshNode>();

            int index = 0;
            foreach (var meshFaceIndex in mesh.TriangleIndices())
            {
                var frame = mesh.GetTriFrame(meshFaceIndex);

                g4.Index3i neighbors = mesh.GetTriNeighbourTris(meshFaceIndex);
                g4.Index3i vertex_index = mesh.GetTriangle(meshFaceIndex);

                faces.Add(meshFaceIndex, new MeshNode(index++, meshFaceIndex, frame, neighbors, vertex_index));
            }

            foreach (var f in faces)
            {
                f.Value.neighbors.Clear();
                f.Value.neighbors.Capacity = 3;
                for (int i = 0; i < 3; ++i)
                {
                    int fn = f.Value.neighbors_index[i];
                    if (fn >= 0)
                        f.Value.neighbors.Add(faces[fn]);
                }

                if (f.Value.neighbors.Count < 3)
                {
                    f.Value.locked = true;

                    foreach (var n in f.Value.neighbors)
                        n.locked = true;
                }
            }

            DMesh3 projectMeshCopy = new DMesh3(mesh);
            outputMesh = new DMesh3(mesh);

            if (faces.Count == 0)
                return false;


            DMeshAABBTree3 treeProject = new DMeshAABBTree3(projectMeshCopy);
            treeProject.Build();

            Random r = new Random();

            bool result = false;

            double faceArea = 0;

            foreach (var f in faces)
            {
                faceArea += f.Value.TriangleArea(outputMesh);
            }

            faceArea /= faces.Count;

            foreach (var f in faces)
            {
                result |= f.Value.Randomize(outputMesh, treeProject, r, amount, moveTries, faceArea);
            }

            double newFaceArea = 0;

            foreach (var f in faces)
            {
                newFaceArea += f.Value.TriangleArea(outputMesh);
            }

            newFaceArea /= faces.Count;

            return result;
        }

        public static void VoronoiMesh(g4.DMesh3 mesh, out g4.DMesh3 outputMesh, out List<g4.Line3d> listLines, out List<g4.PolyLine3d> listPolylines)
        {
            System.Collections.Generic.SortedDictionary<int, MeshNode> faces = new System.Collections.Generic.SortedDictionary<int, MeshNode>();

            int index = 0;
            foreach (var meshFaceIndex in mesh.TriangleIndices())
            {
                var frame = mesh.GetTriFrame(meshFaceIndex);

                g4.Index3i neighbors = mesh.GetTriNeighbourTris(meshFaceIndex);
                g4.Index3i vertex_index = mesh.GetTriangle(meshFaceIndex);

                faces.Add(meshFaceIndex, new MeshNode(index++, meshFaceIndex, frame, neighbors, vertex_index));
            }


            foreach (var f in faces)
            {
                f.Value.neighbors.Clear();
                f.Value.neighbors.Capacity = 3;
                for (int i = 0; i < 3; ++i)
                {
                    int fn = f.Value.neighbors_index[i];
                    if (fn >= 0)
                        f.Value.neighbors.Add(faces[fn]);
                }

                if (f.Value.neighbors.Count < 3)
                {
                    f.Value.locked = true;

                    foreach (var n in f.Value.neighbors)
                        n.locked = true;
                }
            }

            outputMesh = new g4.DMesh3(g4.MeshComponents.None);
            listLines = new List<g4.Line3d>();
            listPolylines = new List<g4.PolyLine3d>();
            foreach (var f in faces)
            {
                outputMesh.AppendVertex(f.Value.frame.Origin);
            }

            HashSet<int> processedPoints = new HashSet<int>();

            foreach (var f in faces)
            {
                for (int i = 0; i < 3; i++)
                {
                    List<int> outputLine = new List<int>();

                    if (processedPoints.Contains(f.Value.vertex_index[i]))
                        continue;

                    int checkVertex = f.Value.vertex_index[i];

                    MeshNode currentFaces = f.Value;
                    MeshNode prevFace = null;

                    bool fullLoop = false;

                    while (true)
                    {
                        for (int j = 0; j < currentFaces.neighbors.Count; j++)
                        {

                            var neighbor = currentFaces.neighbors[j];
                            if (neighbor.UsesVertex(checkVertex))
                            {

                                if (neighbor == prevFace)
                                    continue;

                                if (neighbor == f.Value)
                                {
                                    fullLoop = true;
                                    break; // Found full loop
                                }

                                outputLine.Add(neighbor.index);

                                prevFace = currentFaces;
                                currentFaces = neighbor;
                                j = -1;
                            }
                        }

                        break;
                    }

                    if (fullLoop)
                    {
                        processedPoints.Add(checkVertex);

                        var polyline = new g4.PolyLine3d();

                        if (outputLine.Count > 2)
                        {
                            g4.Vector3d centerPoint = f.Value.frame.Origin;

                            foreach (var p in outputLine)
                                centerPoint += outputMesh.GetVertex(p);

                            centerPoint /= (outputLine.Count + 1);

                            int center = outputMesh.AppendVertex(centerPoint);

                            var pS = outputMesh.GetVertex(f.Value.index);
                            var p0 = outputMesh.GetVertex(outputLine[0]);
                            var pE = outputMesh.GetVertex(outputLine[outputLine.Count - 1]);

                            var normal = mesh.GetTriNormal(f.Value.meshIndex);

                            polyline.AppendVertex(pS);
                            polyline.AppendVertex(p0);

                            listLines.Add(new g4.Line3d(pS, p0 - pS));

                            var n = MathUtil.Normal(centerPoint, pS, p0);

                            bool reverseTri = n.Dot(normal) < 0;

                            if (!reverseTri)
                                outputMesh.AppendTriangle(center, f.Value.index, outputLine[0]);
                            else
                                outputMesh.AppendTriangle(center, outputLine[0], f.Value.index);

                            for (int j = 0; j < outputLine.Count - 1; j++)
                            {
                                var p1 = outputMesh.GetVertex(outputLine[j]);
                                var p2 = outputMesh.GetVertex(outputLine[j + 1]);

                                listLines.Add(new g4.Line3d(p1, p2 - p1));
                                polyline.AppendVertex(p2);

                                if (!reverseTri)
                                    outputMesh.AppendTriangle(center, outputLine[j], outputLine[j + 1]);
                                else
                                    outputMesh.AppendTriangle(center, outputLine[j + 1], outputLine[j]);
                            }

                            polyline.AppendVertex(pS);
                            listLines.Add(new g4.Line3d(pE, pS - pE));

                            listPolylines.Add(polyline);

                            if (!reverseTri)
                                outputMesh.AppendTriangle(center, outputLine[outputLine.Count - 1], f.Value.index);
                            else
                                outputMesh.AppendTriangle(center, f.Value.index, outputLine[outputLine.Count - 1]);
                        }
                    }
                }

            }
        }
    }
}
