using System;
using Utility.Logger;
using System.Collections.Generic;
using Utility.Logger; 


namespace NMGen
{
    public class DetailMeshBuilder
    {


#region HeightPatch 

        private class HeightPatch
        {
            public static int UNSET = int.MaxValue; 
            
            public int minWidthIndex ;
            public int minDepthIndex;

            public int width;
            public int depth;

            public int[] data; 

            public int getData(int globalWidthIndex, int globalDepthIndex)
            {
                int idx = Math.Min(Math.Max(globalWidthIndex - minWidthIndex, 0), width - 1) * depth
                    + Math.Min(Math.Max(globalDepthIndex - minDepthIndex, 0), depth - 1);

                return data[idx]; 
            }

            public bool isInPatch(int globalWidthIndex,int globalDepthIndex)
            {
                return (globalWidthIndex >= minWidthIndex
                    && globalDepthIndex >= minDepthIndex
                    && globalWidthIndex < minWidthIndex + width 
                    && globalDepthIndex < minDepthIndex + depth ); 
            }
            
            public void resetData()
            {
                if( null == data )
                {
                    return; 
                }

                for(int i = 0; i < data.Length; ++i)
                {
                    data[i] = UNSET; 
                }
            }


            public void setData(int globalWidthInex,int globalDepthIndex,int value )
            {
                data[(globalWidthInex - minWidthIndex) * depth + globalDepthIndex - minDepthIndex] = value;  
            }
        } // HeightPatch 

#endregion

        private static int UNDEFINED = -1;
        private static int HULL = -2;
        private static int MAX_VERTS = 256;
        private static int MAX_EDGES = 64;

        private float mContourSampleDistance;
        private float mContourMaxDeviation;

        public DetailMeshBuilder(float contourSampleDistance ,float contourMaxDeviation )
        {
            mContourSampleDistance = Math.Max(0, contourSampleDistance);
            mContourMaxDeviation = Math.Max(0, contourMaxDeviation);
        }

        public TriangleMesh build(PolyMeshField sourceMesh ,OpenHeightfield heightField)
        {
            if( null == sourceMesh 
                || 0 == sourceMesh.vertCount()
                || 0 == sourceMesh.polyCount() )
            {
                Logger.LogError("[DetailMeshBuilder][build]Param Error"); 
                return null; 
            }

            TriangleMesh mesh = new TriangleMesh();
            int sourcePolyCount = sourceMesh.polyCount();

            float cellSize = sourceMesh.cellSize();
            float cellHeight = sourceMesh.cellHeight();
            float[] minBounds = sourceMesh.boundsMin();
            int maxVertsPerPoly = sourceMesh.maxVertsPerPoly();
            int[] sourceVerts = sourceMesh.verts;
            int[] sourcePolys = sourceMesh.polys;

            //(xmin,xmax,zmin,zmax)，记录每个多边形的Bounds
            int[] polyXZBounds = new int[sourcePolyCount * 4];
            int totalPolyVertCount = 0;

            int maxPolyWidth = 0;
            int maxPolyDepth = 0; 

            for(int iPoly = 0; iPoly < sourcePolyCount; ++iPoly)
            {
                //在SourcePolyMesh里面，每个Poly的是两组maxVertsPoly的数组，一组是自己的顶点，一组和别的Poly相连的顶点
                int pPoly = iPoly * maxVertsPerPoly * 2;
                int pxmin = iPoly * 4;
                int pxmax = iPoly * 4 + 1;
                int pzmin = iPoly * 4 + 2;
                int pzmax = iPoly * 4 + 3;

                polyXZBounds[pxmin] = heightField.width();
                polyXZBounds[pxmax] = 0 ;
                polyXZBounds[pzmin] = heightField.depth();
                polyXZBounds[pzmax] = 0 ;


                //开始遍历多边形里面的顶点，找出Bounds
                for(int vertOffset = 0; vertOffset < maxVertsPerPoly; ++vertOffset)
                {
                    if( PolyMeshField.NULL_INDEX == sourcePolys[pPoly+vertOffset] )
                    {
                        //找到多边形的顶点结尾了，跳出循环
                        break; 
                    }

                    //获取多边形的顶点索引，再去取真正的顶点
                    int pVert = sourcePolys[pPoly + vertOffset] * 3;
                    polyXZBounds[pxmin] = Math.Min(polyXZBounds[pxmin], sourceVerts[pVert]);
                    polyXZBounds[pxmax] = Math.Max(polyXZBounds[pxmax], sourceVerts[pVert]);
                    polyXZBounds[pzmin] = Math.Min(polyXZBounds[pzmin], sourceVerts[pVert]);
                    polyXZBounds[pzmax] = Math.Max(polyXZBounds[pzmax], sourceVerts[pVert]);

                    totalPolyVertCount++; 
                } // for maxVertsPerPoly 

                //clamp
                polyXZBounds[pxmin] = Math.Max(0, polyXZBounds[pxmin] - 1);
                polyXZBounds[pxmax] = Math.Min(heightField.width(),polyXZBounds[pxmax] + 1) ;
                polyXZBounds[pzmin] = Math.Max(0, polyXZBounds[pzmin] - 1);
                polyXZBounds[pzmax] = Math.Min(heightField.depth(), polyXZBounds[pzmax] + 1); 

                if( polyXZBounds[pxmin] >= polyXZBounds[pxmax]
                    || polyXZBounds[pzmin] >= polyXZBounds[pzmax] )
                {
                    continue; 
                }

                //获取高度Patch最大的长宽
                maxPolyWidth = Math.Max(maxPolyWidth, polyXZBounds[pxmax] - polyXZBounds[pxmin]);
                maxPolyDepth = Math.Max(maxPolyDepth, polyXZBounds[pzmax] - polyXZBounds[pzmin]); 

            } // for sourcePolyCount

            //(x,y,z)为一组，这里存的是真正顶点的值 
            float[] poly = new float[maxVertsPerPoly * 3];
            int polyVertCount = 0;

            List<int> polyTriangles = new List<int>(maxVertsPerPoly * 2 * 3);

            float[] polyTriangleVerts = new float[MAX_VERTS * 3];
            int polyTriangleVertCount = 0;  

            HeightPatch hfPatch = new HeightPatch(); 
            if( mContourSampleDistance > 0 )
            {
                //这个范围内的Grid才是有navmesh数据的
                hfPatch.data = new int[maxPolyWidth * maxPolyDepth]; 
            } // if mContourSampleDistance < 0 

            Stack<int> workingStack = new Stack<int>(256);
            Stack<OpenHeightSpan> workingSpanStack = new Stack<OpenHeightSpan>(128);
            List<int> workingEdges = new List<int>(MAX_EDGES * 4);
            List<int> workingSamples = new List<int>(512);

            int[] workingWidthDepth = new int[2];

            //每组 (x,y,z)
            List<float> globalVerts = new List<float>(totalPolyVertCount * 2 * 3);

            //(vertAIdx,vertBIdx,vertCIdx，regionID)
            List<int> globalTriangles = new List<int>(totalPolyVertCount * 2 * 4); 

            for(int iPoly = 0; iPoly < sourcePolyCount; ++iPoly)
            {
                int pPoly = iPoly * maxVertsPerPoly * 2;

                polyVertCount = 0;  
                for(int vertOffset = 0; vertOffset < maxVertsPerPoly; ++ vertOffset)
                {
                    if( sourcePolys[pPoly+vertOffset] == PolyMeshField.NULL_INDEX )
                    {
                        break; 
                    }

                    //以xyz为一组数据,所以取出来的索引需要 * 3 
                    int pVert = sourcePolys[pPoly + vertOffset] * 3;

                    //查看生成轮廓那部分的代码，特别是生成Raw轮廓那里，就知道里面的顶点x z 方向的顶点索引，都是按照
                    //Span的索引算出来的，然后Span就是按照cellSize来划分的，所以pVert指向的顶点的索引，就需要乘以
                    //cellSize来还原，这里面的位置都是相对的。
                    poly[vertOffset * 3 + 0] = sourceVerts[pVert] * cellSize;
                    poly[vertOffset * 3 + 1] = sourceVerts[pVert + 1] * cellHeight;  
                    poly[vertOffset * 3 + 2] = sourceVerts[pVert + 2] * cellSize;

                    polyVertCount++; 
                } // for maxVertsPerPoly

                if( mContourSampleDistance > 0 )
                {
                    //这里数据都对于当前的Poly来说的,限制了当前hfPatch的范围
                    hfPatch.minWidthIndex = polyXZBounds[iPoly * 4];
                    hfPatch.minDepthIndex = polyXZBounds[iPoly * 4 + 2];
                    hfPatch.width = polyXZBounds[iPoly * 4 + 1] - polyXZBounds[iPoly * 4 + 0];
                    hfPatch.depth = polyXZBounds[iPoly * 4 + 3] - polyXZBounds[iPoly * 4 + 2];

                    loadHeightPatch(pPoly,polyVertCount,sourcePolys,sourceVerts,heightField,hfPatch,workingStack,workingSpanStack,ref workingWidthDepth); 

                } // if mContourSampleDistance

                //所以多边形的高度数据之后 ，开始做真正的细化处理
                polyTriangleVertCount = buildPolyDetail(poly, polyVertCount,
                    heightField, hfPatch,
                    polyTriangleVerts, polyTriangles,
                    workingEdges, workingSamples); 

            } // all Polygons 

            return null; 

        } //build 

        private int buildPolyDetail(float[] sourcePoly,int sourceVertCount,
            OpenHeightfield heightField,HeightPatch patch,float[] outVerts,
            List<int> outTriangles,List<int> workingEdges,List<int> workingSamples
            )
        {
            //分段的，真正的顶点坐标
            float[] workingVerts = new float[(MAX_EDGES + 1) * 3];
            //分段顶点对应的顶点索引
            int[] workingIndices = new int[MAX_EDGES];
            int workingIndicesCount = 0;
            int[] hullIndices = new int[MAX_VERTS];
            int hullIndicesCount = 0;

            float cellSize = heightField.cellSize();
            float inverseCellSize = 1.0f / heightField.cellSize();

            Array.Copy(sourcePoly, 0, outVerts, 0, sourceVertCount * 3);
            int outVertCount = sourceVertCount;

            float heightPathLimit = HeightPatch.UNSET * heightField.cellHeight(); 
            if( mContourSampleDistance > 0 )
            {
                //因为都是A -> B 来分割的，所以当B等于0的时候，A就是绕回去的 sourceVertCount - 1 
                for(int iSourceVertB = 0 , iSourceVertA = sourceVertCount -1; 
                    iSourceVertB < sourceVertCount;
                    iSourceVertA = iSourceVertB++)
                {
                    int pSourceVertA = iSourceVertA * 3;
                    int pSourceVertB = iSourceVertB * 3;
                    bool swapped = false;  

                    //字典序,先对比 x的，再对比z ，反正就是坐标较小那个为先
                    //TODO 不明白为啥要搞这个？保护顶点的处理顺序一致？
                    //两个点的x坐标非常相近？
                    if( Math.Abs(sourcePoly[pSourceVertA] - sourcePoly[pSourceVertB]) < float.MinValue )  
                    {
                        //A的Z坐标稍微大点？
                        if( sourcePoly[pSourceVertA+2] > sourcePoly[pSourceVertB+2] )
                        {
                            pSourceVertA = iSourceVertB * 3;
                            pSourceVertB = iSourceVertA * 3;
                            swapped = true; 
                        }
                    }
                    else if( sourcePoly[pSourceVertA] > sourcePoly[pSourceVertB] )
                    {
                        pSourceVertA = iSourceVertB * 3;
                        pSourceVertB = iSourceVertA * 3;
                        swapped = true; 
                    }

                    //deltaX只会大于等于0，因为前面的顺序保证了这一点
                    float deltaX = sourcePoly[pSourceVertB] - sourcePoly[pSourceVertA];
                    //deltaZ可能会有负的，当deltaX等于0的时候，deltaZ就一定大于等于0
                    float deltaZ = sourcePoly[pSourceVertB + 2] - sourcePoly[pSourceVertA + 2];

                    float edgeXZLength = (float)Math.Sqrt(deltaX * deltaX + deltaZ * deltaZ);

                    //可以分成多少份来采样
                    int iMaxEdge = 1 + (int)Math.Floor(edgeXZLength / mContourSampleDistance);
                    iMaxEdge = Math.Min(iMaxEdge, MAX_EDGES);
                    if( iMaxEdge + outVertCount >= MAX_VERTS)
                    {
                        iMaxEdge = MAX_VERTS - 1 - outVertCount;  
                    }

                    //这条边是从 A->B 的
                    //将A->B 分割成几份
                    for(int iEdgeVert = 0; iEdgeVert <= iMaxEdge; ++iEdgeVert)
                    {
                        float percentOffset = (float)iEdgeVert / iMaxEdge;
                        int pEdge = iEdgeVert * 3;
                        workingVerts[pEdge] = sourcePoly[pSourceVertA] + (deltaX * percentOffset);
                        workingVerts[pEdge + 2] = sourcePoly[pSourceVertA + 2] + (deltaZ * percentOffset);

                        workingVerts[pEdge + 1] = getHeightWithinField(workingVerts[pEdge], workingVerts[pEdge + 2], cellSize, inverseCellSize, patch) * heightField.cellHeight(); 
                        
                    } // for iMaxEdge

                    //就是和生成SimpleContour同一套路的，一个开始点，一个终点，不断地插入点
                    workingIndices[0] = 0;
                    workingIndices[1] = iMaxEdge; 
                    workingIndicesCount = 2; 

                    //然后 A->B 这条边，还可以更加细分出来吗？
                    for(int iWorkingIndex = 0; iWorkingIndex < workingIndicesCount - 1; )
                    {
                        //iWorkingVertA和iWorkingVertB对应的都是分段顶点的顶点索引
                        int iWorkingVertA = workingIndices[iWorkingIndex];
                        int iWorkingVertB = workingIndices[iWorkingIndex + 1];
                        int pWorkingVertA = iWorkingVertA * 3;
                        int pWorkingVertB = iWorkingVertB * 3;

                        float maxDistanceSq = 0;
                        int iMaxDistanceVert = -1;

                        //遍历采样点
                        for(int iTestVert = iWorkingVertA + 1; iTestVert < iWorkingVertB; iTestVert++)
                        {
                            int iTestVertBase = iTestVert * 3;
                            //iTestVertBase + 1 对应的是y坐标
                            if ( workingVerts[iTestVertBase+1] >= heightPathLimit )  //异常的高度
                            {
                                Logger.LogWarning("[DetailMeshBuilder][buildPolyDetail]Potential Loss Height|{0}|{1}",workingVerts[iTestVertBase],workingVerts[iTestVertBase+2]);
                                continue; 
                            } // heightPathLimit

                            //注意，和生成轮廓那里不一样的是，这里算的是三维的距离，算上高度的
                            //所以这里其实是找出高度偏离得特别厉害的采样点，这样的点，就需要插入到新的边
                            float distanceSq = Geometry.getPointSegmentDistanceSq(
                                workingVerts[iTestVertBase],
                                workingVerts[iTestVertBase+1],
                                workingVerts[iTestVertBase+2],
                                workingVerts[pWorkingVertA],
                                workingVerts[pWorkingVertA+1],
                                workingVerts[pWorkingVertA+2],
                                workingVerts[pWorkingVertB],
                                workingVerts[pWorkingVertB+1],
                                workingVerts[pWorkingVertB+2]
                                ); 

                            if( distanceSq > maxDistanceSq )
                            {
                                maxDistanceSq = distanceSq;
                                iMaxDistanceVert = iTestVert;  
                            }
                        }  //for  iWorkingVertB 

                        if( iMaxDistanceVert != -1
                            && maxDistanceSq > mContourMaxDeviation * mContourMaxDeviation )
                        {
                            //插到iWorkingIndex后面
                            for(int i = workingIndicesCount; i > iWorkingIndex; --i)
                            {
                                workingIndices[i] = workingIndices[i - 1];
                            }

                            workingIndices[iWorkingIndex + 1] = iMaxDistanceVert;
                            workingIndicesCount++; 
                        } // if maxDistanceSq
                        else
                        {
                            iWorkingIndex++; 
                        }
                    }  // for workingIndicesCount

                    //多边形外壳的索引,iSourceVertA的索引是相对于outVerts和sourcePoly来说的
                    hullIndices[hullIndicesCount++] = iSourceVertA; 
                    if( swapped )
                    {
                        //因为先加了A？
                        //TODO 为什么要反过来插入，而且为什么是要减2呢 ？
                        for(int iWorkingIndex = workingIndicesCount - 2; iWorkingIndex > 0; iWorkingIndex--)
                        {
                            //outVertCount原来的值是sourceVertCount
                            int outVertCountBase = outVertCount * 3; 
                            
                            //outVerts本来就原本的顶点数据了，在前面的ArrayCopy。
                            //但是这里为什么要直接插在后面呢？因为后面是使用hullIndices数组来索引的，所以顺序没有关系了。
                            // hullIndices里面存的是相对于outVerts顶点数组中的索引
                            outVerts[outVertCountBase] = workingVerts[workingIndices[iWorkingIndex] * 3];
                            outVerts[outVertCountBase+1] = workingVerts[workingIndices[iWorkingIndex] * 3 + 1];
                            outVerts[outVertCountBase+2] = workingVerts[workingIndices[iWorkingIndex] * 3 + 2];
                            hullIndices[hullIndicesCount++] = outVertCount;  //outVertCount其实指向的是索引
                            outVertCount++; 
                        } // for iWorkingIndex
                    }  // if swapped
                    else
                    {
                        for(int iWorkingIndex = 1; iWorkingIndex < workingIndicesCount-1; ++iWorkingIndex)
                        {
                            int outVertCountBase = outVertCount * 3;
                            outVerts[outVertCountBase] = workingVerts[workingIndices[iWorkingIndex] * 3];
                            outVerts[outVertCountBase + 1] = workingVerts[workingIndices[iWorkingIndex] * 3 + 1];
                            outVerts[outVertCountBase + 2] = workingVerts[workingIndices[iWorkingIndex] * 3 + 2];
                            hullIndices[hullIndicesCount++] = outVertCount;
                            outVertCount++; 
                        }
                    } // if swapped

                }  //for SourceVertCount , 遍历边
            } // if mContourSampleDistance
            else
            {
                //如果不用细分边界的话，那就外壳的索引数组，就是对应outVerts数组的
                for(int i = 0; i < outVertCount; ++i)
                {
                    hullIndices[i] = i; 
                }
                hullIndicesCount = outVertCount;  
            } // if - else mContourSampleDistance

            if( outVertCount > 3 )
            {
                performDelaunayTriangulation(
                    outVerts,outVertCount,
                    hullIndices,hullIndicesCount,
                    workingEdges,outTriangles); 
            }  //if outVertCount > 3 
            else if( 3 == outVertCount )
            {
                outTriangles.Clear();
                outTriangles.Add(0);
                outTriangles.Add(1);
                outTriangles.Add(2); 
            }
            else
            {
                outTriangles.Clear();
                return 0; 
            }

            int badIndicesCount = getInvalidIndicesCount(outTriangles, outVertCount); 
            if( badIndicesCount > 0 )
            {
                Logger.LogError("[DetailMeshBuilder][build]Invalid indices|{0}",badIndicesCount);
                outTriangles.Clear();
                return 0;
            }

            if( mContourSampleDistance > 0 )
            {
                float minX = sourcePoly[0];
                float minZ = sourcePoly[2];
                float maxX = minX;
                float maxZ = minZ;  

                //获取 sourcePoly 的Bounds ，基于真正的坐标值 
                for(int iVert = 1; iVert < sourceVertCount; ++iVert )
                {
                    int pVert = iVert * 3;
                    minX = Math.Min(minX, sourcePoly[pVert]);
                    minZ = Math.Min(minZ, sourcePoly[pVert + 2]);
                    maxX = Math.Min(maxX, sourcePoly[pVert]);
                    maxZ = Math.Min(maxZ, sourcePoly[pVert + 2]);
                }  // for sourceVertCount 

                //TODO ，转换成基于mContourSampleDistance的一个Grid
                int x0 = (int)Math.Floor(minX / mContourSampleDistance);
                int z0 = (int)Math.Floor(minZ / mContourSampleDistance);
                int x1 = (int)Math.Floor(maxX / mContourSampleDistance);
                int z1 = (int)Math.Floor(minZ / mContourSampleDistance);

                workingSamples.Clear(); 

                for(int z = z0; z < z1; ++z)
                {
                    for( int x = x0; x < x1; ++x )
                    {
                        float vx = x * mContourSampleDistance;
                        float vz = z * mContourSampleDistance; 

                        if(getSignedDistanceToPolygonSq(vx,vz,sourcePoly,sourceVertCount) > -mContourSampleDistance/2 )
                        {
                            continue; 
                        }

                        workingSamples.Add(x);
                        workingSamples.Add(getHeightWithinField(vx,vz,cellSize,inverseCellSize,patch) );
                        workingSamples.Add(z);  
                    } // for x -> x1
                } // for z -> z1 


                int sampleCount = workingSamples.Count / 3; 
                for(int iterationCount = 0; iterationCount < sampleCount; ++iterationCount )
                {
                    float selectedX = 0;
                    float selectedY = 0;
                    float selectedZ = 0;

                    float maxDistance = 0; 


                    for(int iSampleVert = 0; iSampleVert < sampleCount; ++iSampleVert)
                    {
                        int iSampleVertBaseIndex = iSampleVert * 3;
                        float sampleX = workingSamples[iSampleVertBaseIndex];
                        float sampleY = workingSamples[iSampleVertBaseIndex + 1];
                        float sampleZ = workingSamples[iSampleVertBaseIndex + 2]; 

                        float sampleDistance = =getInte()
                    } // for iSampleVert -> sampleCount
                }  //for iterationCount -> sampleCount 


            }  // if mContourSampleDistance > 0 

            return 0; 
        }

        private static float getInternalDistanceToMesh(float px,float py,float pz,
            float[] verts, List<int> indices)
        {
            float minDistance = float.MaxValue; 
            int triangelCount = indices.Count / 3; 

            for(int iTriangle = 0; iTriangle < triangelCount; ++iTriangle)
            {
                int iTriangleBaseIndex = iTriangle * 3;
                int pVertA = indices[iTriangleBaseIndex] * 3;
                int pVertB = indices[iTriangleBaseIndex + 1] * 3;
                int pVertC = indices[iTriangleBaseIndex + 2] * 3;

                float distance = float.MaxValue;

                float deltaACx = verts[pVertC] - verts[pVertA];
                float deltaACy = verts[pVertC + 1] - verts[pVertA + 1];
                float deltaACz = verts[pVertC + 2] - verts[pVertA + 2];

                float deltaABx = verts[pVertB] - verts[pVertA];
                float deltaABy = verts[pVertB + 1] - verts[pVertA + 1];
                float deltaABz = verts[pVertB + 2] - verts[pVertA + 2];

                float deltaAPx = px - verts[pVertA];
                float deltaAPz = pz - verts[pVertA + 2]; 


            }  // for iTriangle -> triangleCount 
        }


        /// <summary>
        /// 
        /// </summary>
        /// <param name="x"></param>
        /// <param name="z"></param>
        /// <param name="verts"></param>
        /// <param name="vertCount"></param>
        /// <returns></returns>
        private static float getSignedDistanceToPolygonSq(float x, float z,float[] verts , int vertCount)
        {
            float minDistance = float.MaxValue;
            int iVertB;
            int iVertA;
            bool isInside = false; 

            //按边来轮循
            for( iVertB = 0 , iVertA = vertCount - 1; iVertB < vertCount; iVertA = iVertB++ )
            {
                //以A-B为边
                int pVertB = iVertB * 3;
                int pVertA = iVertA * 3;

                /*
                 *  https://www.codeproject.com/tips/84226/is-a-point-inside-a-polygon
                 *  https://wrf.ecse.rpi.edu//Research/Short_Notes/pnpoly.html
                 *  First of all, notice that each iteration considers two adjacent points and the target point. Then the if statement evaluates two conditions:Y-value of our target point is within the range [verty[j], verty[i]).
                 *  X-value of our target point is below the linear line connecting the point j and i.
                 */

                //条件1就是y轴投影要在多边形的范围内
                bool cond1 = (verts[pVertB + 2] > z) != (verts[pVertA + 2] > z);
                //条件2用两点式代入即可，将小于号看成等号，好理解些.
                //其实就是点与直线之间的关系 ，在直线哪一侧 ，自己用一条横线横过三角形试试
                bool cond2 = (x < (verts[pVertA] - verts[pVertB]) * (z - verts[pVertB + 2]) / (verts[pVertA + 2] - verts[pVertB + 2]) + verts[pVertB])  ;  
                if( cond1 && cond2 )
                {
                    //作者的代码有误，这里没用对
                    isInside = !isInside ; 
                } //

                minDistance = Math.Min(minDistance, 
                    Geometry.getPointSegmentDistanceSq(x,z,
                    verts[pVertA],verts[pVertA+2],
                    verts[pVertB],verts[pVertB+2])
                    ); 
            } // for iVertB -> vertCount

            return isInside ? -minDistance : minDistance; 
        }

        private static int getInvalidIndicesCount( List<int> indices ,int vertCount)
        {
            int badIndicesCount = 0; 
            for(int i = 0; i < indices.Count; ++i)
            {
                int index = indices[i]; 
                if( index < 0 || index >= vertCount  )
                {
                    badIndicesCount++; 
                }
            }
            return badIndicesCount;  
        }

        private static bool overlapsExistingEdge(int iVertA ,int iVertB ,float[] verts,List<int> edges )
        {
            for(int pEdge = 0 ; pEdge < edges.Count; pEdge += 4 )
            {
                int iEdgeVertA = edges[pEdge];
                int iEdgeVertB = edges[pEdge + 1]; 

                if( iEdgeVertA == iVertA
                    || iEdgeVertA == iVertB 
                    || iEdgeVertB == iVertA
                    || iEdgeVertB == iVertB )
                {
                    continue; 
                }

                //和现在任意的一条边都有可能重叠 
                if( Geometry.segmentsOverlap(verts[iEdgeVertA*3],verts[iEdgeVertA*3+2],
                                    verts[iEdgeVertB*3],verts[iEdgeVertB*3+2],
                                    verts[iVertA*3],verts[iVertA*3+2],
                                    verts[iVertB*3],verts[iVertB*3+2])
                                    )
                {
                    return true; 
                }

            } // for pEdge < edges.Count

            return false;  
        }

        /// <summary>
        /// http://mathworld.wolfram.com/Circumcircle.html
        /// </summary>
        /// <param name="ax"></param>
        /// <param name="ay"></param>
        /// <param name="bx"></param>
        /// <param name="by"></param>
        /// <param name="cx"></param>
        /// <param name="cb"></param>
        /// <param name="triangleAreaX2"></param>
        /// <param name="outCircle"></param>
        /// <returns></returns>
        private static bool buildCircumcircle(float ax , float ay ,
            float bx,float by,
            float cx,float cy,
            float triangleAreaX2,float[] outCircle )
        {
            float epsilon = 1e-6f; 

            if( Math.Abs(triangleAreaX2) > epsilon )
            {
                //Graphic Gems 1 Page 22
                float aLenSq = ax * ax + ay * ay;
                float bLenSq = bx * bx + by * by;
                float cLenSq = cx * cx + cy * cy;

                outCircle[0] = (aLenSq * (by - cy) + bLenSq * (cy - ay) + cLenSq * (ay - by)) / (2 * triangleAreaX2);
                outCircle[1] = (aLenSq * (cx - bx) + bLenSq * (ax - cx) + cLenSq * (bx - ax)) / (2 * triangleAreaX2);

                outCircle[2] = (float)Math.Sqrt(Geometry.getDistanceSq(outCircle[0], outCircle[1], ax, ay));

                return true; 
            } //if Math.Abs

            outCircle[0] = 0;
            outCircle[1] = 0;
            outCircle[2] = UNDEFINED;
            return false; 
        }

        //VertA和VertB的谁先谁后没关系，因为目的是找到半边对应的边，所以只有有一组半边对应上就好了
        private static int getEdgeIndex( List<int> edges,int vertAIndex,int vertBIndex )
        {
            int edgeCount = edges.Count / 4; 
            for(int i = 0; i < edgeCount; ++i)
            {
                int u = edges[i * 4];
                int v = edges[i * 4 + 1]; 
                //一组边，当两个半边来存了
                if( (u == vertAIndex && v == vertBIndex) 
                    || (u == vertBIndex && v == vertAIndex))
                {
                    return i; 
                }
            }

            return UNDEFINED;  
        }

        private static void updateLeftFace(int iEdge,int iStartVert , int faceValue,List<int> edges)
        {
            int pEdge = iEdge * 4; 
            if( edges[pEdge] == iStartVert 
                && edges[pEdge+2] == UNDEFINED )
            {
                edges[pEdge + 2] = faceValue; 
            }
            else if( edges[pEdge+1] == iStartVert 
                && edges[pEdge+3] == UNDEFINED )
            {
                edges[pEdge + 3] = faceValue; 
            }
        }

        private static int completeTriangle(int iEdge,
            float[] verts , int vertCount,
            int currentTriangleCount , List<int> edges )
        {
            int iVertA;
            int iVertB; 

            //这里确保的都是半边的左侧
            if( edges[iEdge * 4 + 2] == UNDEFINED )
            {
                iVertA = edges[iEdge * 4];
                iVertB = edges[iEdge * 4 + 1];
            }
            else if( edges[iEdge * 4 + 3] == UNDEFINED )
            {
                iVertA = edges[iEdge * 4 + 1];
                iVertB = edges[iEdge * 4];  
            }
            else
            {
                return currentTriangleCount; 
            }

            int pVertA = iVertA * 3;
            int pVertB = iVertB * 3;

            int iSelectedVert = UNDEFINED;

            //(x,z,r)
            float[] selectedCircle = { 0,0,-1};
            float tolerance = 0.001f;
            float epsilon = 1e-5f; 

            for(int iPotentialVert = 0; iPotentialVert < vertCount; ++iPotentialVert)
            {
                if( iPotentialVert == iVertA 
                    || iPotentialVert == iVertB )
                {
                    continue; 
                }

                int pPotentialVert = iPotentialVert * 3;
                float area = Geometry.getSignedAreaX2(verts[pVertA],verts[pVertA+2],
                    verts[pVertB],verts[pVertB+2],
                    verts[pPotentialVert],verts[pPotentialVert+2] ); 

                //TODO 不是应该使用绝对值么？因为如果这个有符号的面积大于0，首先可以知道
                //点是在半边的左侧。如果面积是小于0的话，那边证明点是在半边的右侧，不符合
                if( area > epsilon )
                {
                    //证明还没有算过
                    if( selectedCircle[2] < 0  )
                    { 
                        if( overlapsExistingEdge(iVertA,iPotentialVert,verts,edges)
                            || overlapsExistingEdge(iVertB,iPotentialVert,verts,edges) ) 
                        {
                            continue; 
                        }

                        //找到一个点可以组成外接圆
                        if( buildCircumcircle(verts[pVertA],verts[pVertA+2],
                            verts[pVertB],verts[pVertB+2],
                            verts[pPotentialVert],verts[pPotentialVert+2],
                            area,selectedCircle) )
                        {
                            iSelectedVert = iPotentialVert;  
                        } // if buildCircumcircle 

                        continue;  
                    }  // if selectedCircle[2] < 0

                    //前面已经算过外接圆了，
                    float distanceToOrgin = (float)Math.Sqrt( Geometry.getDistanceSq(selectedCircle[0],selectedCircle[1],verts[pPotentialVert],verts[pPotentialVert+2])); 
                    if( distanceToOrgin > selectedCircle[2] * (1+tolerance) ) 
                    {
                        //Delaunay condition：http://www.dma.fi.upm.es/personal/mabellanas/tfcs/flips/Intercambios/html/teoria/teoria_del_ing.htm
                        //An edge is 'legal' when starting from the edge in question and the two triangles which it belongs to, the circumscribed circumference to one of the triangles does not contain the remaining point that belongs to the other triangle, and vice versa.
                        //因为这个点在圆外面，所以不用考虑了
                        continue; 
                    } // distanceToOrgin
                    else
                    {
                        //为了排除浮点错误，二次排除？
                        //因为要取最小的外接圆，为了满足，空圆属性
                        if( overlapsExistingEdge(iVertA,iPotentialVert,verts,edges) 
                            || overlapsExistingEdge(iVertB,iPotentialVert,verts,edges) )
                        {
                            continue; 
                        }

                        if( buildCircumcircle(verts[pVertA],verts[pVertA+2],
                            verts[pVertB],verts[pVertB+2],
                            verts[pPotentialVert],verts[pPotentialVert+2],
                            area,
                            selectedCircle) )
                        {
                            //一个更小的外接圆
                            iSelectedVert = iPotentialVert;  
                        }
                    }
                } //if area > epsilon 

            } //  for iPotentialVert

            if( iSelectedVert != UNDEFINED )
            {
                //currentTriangleCount是索引
                updateLeftFace(iEdge, iVertA, currentTriangleCount, edges);

                //TODO：注意selectedVert 和 VertA 、VertB的关系。这里是先从 selectVert -> A -> B -> selectVert的
                //顶点顺序。但是看这个函数开头的地方，A和B可能是已经换过位置了。所以确保了face Value始终在半边的左侧。

                //selectedVert -> vertA
                int iSelectedEdge = getEdgeIndex(edges, iSelectedVert, iVertA);  
                if( iSelectedEdge == UNDEFINED )
                {
                    edges.Add(iSelectedVert);
                    edges.Add(iVertA);
                    edges.Add(currentTriangleCount);
                    edges.Add(UNDEFINED); 
                }
                else
                {
                    updateLeftFace(iSelectedEdge, iSelectedVert, currentTriangleCount, edges); 
                }

                //vertB -> selectedVert 
                iSelectedEdge = getEdgeIndex(edges, iVertB, iSelectedVert);  
                if(iSelectedEdge == UNDEFINED)
                {
                    edges.Add(iVertB);
                    edges.Add(iSelectedVert);
                    edges.Add(currentTriangleCount);
                    edges.Add(UNDEFINED);
                }
                else
                {
                    updateLeftFace(iSelectedEdge, iVertB, currentTriangleCount, edges); 
                }

                currentTriangleCount++;  
            } // if iSelectedVert != UNDEFINED 
            else
            {
                updateLeftFace(iEdge, iVertA, HULL, edges); 
            }

            return currentTriangleCount; 
        }

        /// <summary>
        /// 默认多边形顶点顺序是顺时针的。
        /// 这里对边的定义，感觉有点像半边数据，然后判断的都是半边左侧
        /// </summary>
        /// <param name="verts"></param>
        /// <param name="vertCount"></param>
        /// <param name="immutableHull"></param>
        /// <param name="hullEdgeCount"></param>
        /// <param name="workingEdges"></param>
        /// <param name="outTriangles"></param>
        private static void performDelaunayTriangulation(float[] verts,int vertCount,
            int[] immutableHull,int hullEdgeCount,
            List<int> workingEdges,List<int> outTriangles)
        {
            int triangleCount = 0;
            //(vertA,vertB,valueA,valueB)
            workingEdges.Clear(); 

            //hullEdgeCount 叫这个名字不太合适吧。。。
            for(int iHullVertB = 0 , iHullVertA = hullEdgeCount - 1; 
                iHullVertB < hullEdgeCount;
                iHullVertA = iHullVertB++ )
            {
                workingEdges.Add(immutableHull[iHullVertA]);
                workingEdges.Add(immutableHull[iHullVertB]);

                workingEdges.Add(HULL);  //默认是顺时针，所以边的左边是壳，也就是外界
                workingEdges.Add(UNDEFINED);  //边的右边未知

            } // for hullEdgeCount

            int iCurrentEdge = 0; 
            while( iCurrentEdge * 4 < workingEdges.Count )
            {
                if( workingEdges[iCurrentEdge * 4 + 2] == UNDEFINED 
                    || workingEdges[iCurrentEdge* 4 + 3] == UNDEFINED )
                {
                    //注意，这里会增加workingEdges的边数
                    triangleCount = completeTriangle(iCurrentEdge, verts, vertCount, triangleCount, workingEdges);
                    iCurrentEdge++; 
                }
            } // while iCurrentEdge

            outTriangles.Clear(); 
            for(int i = 0; i < triangleCount * 3; ++i)
            {
                outTriangles.Add(UNDEFINED);
            }


            for(int pEdge = 0; pEdge < workingEdges.Count; pEdge +=4 )
            {
                if( workingEdges[pEdge+3] != HULL )
                {
                    //pEdge + 3 取的是 顺时针半边的右值？存的就是三角形的索引
                    int pTriangle = workingEdges[pEdge + 3] * 3; 
                    if( outTriangles[pTriangle] == UNDEFINED )  //如果到这里，还有一个未定义的话，就是一种错误 
                    {
                        //先给两点，三角形的
                        outTriangles[pTriangle] = workingEdges[pEdge];
                        outTriangles[pTriangle + 1] = workingEdges[pEdge + 1]; 
                    }
                    else if( outTriangles[pTriangle+2] == UNDEFINED )
                    {
                        //这个三角形前两个顶点都确定了
                        if( workingEdges[pEdge] == outTriangles[pTriangle]
                            || workingEdges[pEdge] == outTriangles[pTriangle+1])
                        {
                            //某条边的第一个顶点在这个三角形里，而且，这个三角形的前两个点都已经确认了。
                            //所以这条边的第二个顶点，也就是三角形的第三个顶点
                            outTriangles[pTriangle + 2] = workingEdges[pEdge + 1]; 
                        }
                        else
                        {
                            outTriangles[pTriangle + 2] = workingEdges[pEdge]; 
                        } 

                    } // else if  outTriangles[pTriangle+2] == UNDEFINED
                } // workingEdges[pEdge+3] != HULL

                if( workingEdges[pEdge+2] != HULL  )
                {
                    //A->B 这个半边的左侧有关联的三角形，那么 B -> A 才是顺时针的顺序

                    int pTriangle = workingEdges[pEdge + 2] * 3; 
                    if( outTriangles[pTriangle] == UNDEFINED  )
                    {
                        //注意这里， B -> A 才是顺时针
                        outTriangles[pTriangle] = workingEdges[pEdge + 1];
                        outTriangles[pTriangle + 1] = workingEdges[pEdge]; 
                    } // if outTriangles[pTriangle] == UNDEFINED
                    else if( outTriangles[pTriangle+2] == UNDEFINED )
                    {
                        if (workingEdges[pEdge] == outTriangles[pTriangle]
                        || workingEdges[pEdge] == outTriangles[pTriangle + 1])
                        {
                            //操作同上
                            outTriangles[pTriangle + 2] = workingEdges[pEdge + 1];
                        } 
                        else
                        {
                            outTriangles[pTriangle + 2] = workingEdges[pEdge]; 
                        }
                    } // else if outTriangles[pTriangle+2] == UNDEFINED


                } // if workingEdges[pEdge+2] != HULL  
            } // for workingEdges
        }

        private static int getHeightWithinField(float x ,float z ,float cellSize,float inverseCellSize,HeightPatch patch)
        {
            int widthIndex = (int)Math.Floor(x * inverseCellSize + 0.01f);
            int depthIndex = (int)Math.Floor(z * inverseCellSize + 0.01f);

            int height = patch.getData(widthIndex, depthIndex);  
            if( height == HeightPatch.UNSET )
            {
                //使用8邻居来找该端点对应的高度
                int[] neighborOffset = { -1, 0, -1, -1, 0, -1, 1, -1, 1, 0, 1, 1, 0, 1, -1, 1 };
                float minNeighborDistanceSq = float.MaxValue; 
                for(int p = 0; p < 16; p +=2)
                {
                    int nWidthIndex = widthIndex + neighborOffset[p];
                    int nDepthIndex = depthIndex + neighborOffset[p + 1]; 

                    if( !patch.isInPatch(nWidthIndex,nDepthIndex)) 
                    {
                        continue; 
                    }

                    int nNeighborHeight = patch.getData(nWidthIndex, nDepthIndex); 
                    if( HeightPatch.UNSET == nNeighborHeight)
                    {
                        continue; 
                    }

                    //0.5是为了取整
                    //因为 nWidthIndex和nDepthIndex都已经偏移了原来的x/z了，所以选一个最近距离的作为高度
                    float deltaWidth = (nWidthIndex + 0.5f) * cellSize - x;
                    float deltaDepth = (nDepthIndex + 0.5f) * cellSize - z;

                    float neighborDistanceSq = deltaWidth * deltaWidth + deltaDepth * deltaDepth; 
                    if( neighborDistanceSq < minNeighborDistanceSq )
                    {
                        height = nNeighborHeight;
                        minNeighborDistanceSq = neighborDistanceSq; 
                    }
                    
                } // for neighbor
            } // if no height

            return height; 
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="polyPointer"></param>
        /// <param name="vertCount"></param>
        /// <param name="indices">存的是多边形的顶点索引</param>
        /// <param name="verts"></param>
        /// <param name="heightField"></param>
        /// <param name="inoutPatch"></param>
        /// <param name="gridIndexStack"></param>
        /// <param name="spanStack"></param>
        /// <param name="widthDepth"></param>
        private static void loadHeightPatch(int polyPointer,int vertCount,int[] indices,int[] verts,
            OpenHeightfield heightField,
            HeightPatch inoutPatch,
            Stack<int> gridIndexStack,
            Stack<OpenHeightSpan> spanStack,
            ref int[] widthDepth)
        {
            inoutPatch.resetData();
            gridIndexStack.Clear();
            spanStack.Clear(); 


            for(int vertOffset = 0; vertOffset < vertCount; ++vertOffset)
            {
                //看这里，这就是前面为什么要乘以  cellSize的原因
                int baseIdx = indices[polyPointer + vertOffset] * 3; 
                int vertX = verts[baseIdx+0];
                int vertY = verts[baseIdx + 1];
                int vertZ = verts[baseIdx + 2];

                OpenHeightSpan selectedSpan = getBestSpan(vertX, vertY, vertZ, heightField,ref widthDepth);  
                if( selectedSpan != null )
                {
                    gridIndexStack.Push(widthDepth[0]);
                    gridIndexStack.Push(widthDepth[1]);
                    spanStack.Push(selectedSpan); 
                }

            } // for vertCount

            while( spanStack.Count > 0  )
            {
                int depthIndex = gridIndexStack.Pop();
                int widthIndex = gridIndexStack.Pop();
                OpenHeightSpan span = spanStack.Pop(); 

                if( inoutPatch.getData(widthIndex,depthIndex) != HeightPatch.UNSET )
                {
                    continue; 
                }

                if( inoutPatch.isInPatch(widthIndex,depthIndex) )
                {
                    inoutPatch.setData(widthIndex, depthIndex, span.floor()); 
                } // isInPatch

                //四个方向广度优先一下，也存一下它们的高度
                for(int dir = 0; dir < 4; ++dir)
                {
                    OpenHeightSpan nSpan = span.getNeighbor(dir); 
                    if( null == nSpan )
                    {
                        continue; 
                    }

                    int nWidthIndex = widthIndex + BoundeField.getDirOffsetWidth(dir);
                    int nDepthIndex = depthIndex + BoundeField.getDirOffsetDepth(dir); 

                    if( !inoutPatch.isInPatch(nWidthIndex,nDepthIndex) )
                    {
                        continue; 
                    }

                    if( inoutPatch.getData(nWidthIndex,nDepthIndex) != HeightPatch.UNSET )
                    {
                        continue; 
                    }

                    gridIndexStack.Push(nWidthIndex);
                    gridIndexStack.Push(nDepthIndex);
                    spanStack.Push(nSpan); 
                } // for dir

            }  //while spanStack 

        } // load


        private static OpenHeightSpan getBestSpan(int vertX ,int vertY,int vertZ,
            OpenHeightfield heightField ,
            ref int[] outWidthDepth)
        {
            int[] targetOffset = { 0, 0, -1, 0, 0, -1, -1, -1, 1, -1, -1, 1, 1, 0, 1, 1, 0, 1 };
            OpenHeightSpan resultSpan = null;
            int minDistance = int.MaxValue; 

            //why 17？因为总共8对
            for(int p = 0; p < 17; p +=2  )
            {
                int widthIndex = vertX + targetOffset[p];
                int depthIndex = vertZ + targetOffset[p + 1]; 

                if( !heightField.isInBounds(widthIndex,depthIndex) )
                {
                    continue; 
                }

                OpenHeightSpan span = heightField.getData(widthIndex, depthIndex);
                span = getBestSpan(span, vertY); 
                if( null == span )
                {
                    continue; 
                }
                else
                {
                    int distance = Math.Abs(vertY - span.floor()); 
                    if( p == 0   //p 等于0 就是自己，没有Offset
                        && (distance <= heightField.cellHeight() ))
                    {
                        outWidthDepth[0] = widthIndex;
                        outWidthDepth[1] = depthIndex;
                        return span; 
                    }
                    else if( distance < minDistance )
                    {
                        resultSpan = span;
                        outWidthDepth[0] = widthIndex;
                        outWidthDepth[1] = depthIndex;
                        minDistance = distance; 
                    }
                }
            } // for

            return resultSpan;  
        }  // getBestSpan


        private static OpenHeightSpan getBestSpan(OpenHeightSpan baseSpan ,int targetHeight)
        {
            int minDistance = int.MaxValue;
            OpenHeightSpan result = null; 

            for(OpenHeightSpan span = baseSpan; span != null; span = span.next() )
            {
                int distance = Math.Abs(targetHeight - span.floor()); 
                if( distance < minDistance )
                {
                    result = span;
                    minDistance = distance; 
                }
            }
            return result;  
        }


    } // builder
}// namespace
