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
                //在SourcePolyMesh里面，每个Poly的是两组maxVertsPoly的数组，一组是自己的顶点，一组和别的
                //Poly相连的顶点
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
                        break; 
                    }

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

            List<float> globalVerts = new List<float>(totalPolyVertCount * 2 * 3);
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

                    //以xyz为一组数据
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
                    //这里数据都对于当前的Poly来说的
                    hfPatch.minWidthIndex = polyXZBounds[iPoly * 4];
                    hfPatch.minDepthIndex = polyXZBounds[iPoly * 4 + 2];
                    hfPatch.width = polyXZBounds[iPoly * 4 + 1] - polyXZBounds[iPoly * 4 + 0];
                    hfPatch.depth = polyXZBounds[iPoly * 4 + 3] - polyXZBounds[iPoly * 4 + 2];

                    loadHeightPatch(pPoly,polyVertCount,sourcePolys,sourceVerts,heightField,hfPatch,workingStack,workingSpanStack,ref workingWidthDepth); 

                } // if mContourSampleDistance

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
            float[] workingVerts = new float[(MAX_EDGES + 1) * 3];
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
                    for(int iEdgeVert = 0; iEdgeVert <= iMaxEdge; ++iEdgeVert)
                    {
                        float percentOffset = (float)iEdgeVert / iMaxEdge;
                        int pEdge = iEdgeVert * 3;
                        workingVerts[pEdge] = sourcePoly[pSourceVertA] + (deltaX * percentOffset);
                        workingVerts[pEdge + 2] = sourcePoly[pSourceVertA + 2] + (deltaZ * percentOffset);

                        workingVerts[pEdge + 1] = getHeightWithinField(workingVerts[pEdge], workingVerts[pEdge + 2], cellSize, inverseCellSize, patch) * heightField.cellHeight(); 
                        
                    } // for iMaxEdge

                    workingIndices[0] = 0;
                    //就是和生成SimpleContour同一套路的，一个开始点，一个终点，不断地插入点
                    //???为什么定义的是最后一个顶点呢？
                    workingIndices[1] = iMaxEdge; 
                    workingIndicesCount = 2; 

                    for(int iWorkingIndex = 0; iWorkingIndex < workingIndicesCount - 1; )
                    {
                        int iWorkingVertA = workingIndices[iWorkingIndex];
                        int iWorkingVertB = workingIndices[iWorkingIndex + 1];
                        int pWorkingVertA = iWorkingVertA * 3;
                        int pWorkingVertB = iWorkingVertB * 3;

                        float maxDistanceSq = 0;
                        int iMaxDistanceVert = -1;
                        for(int iTestVert = iWorkingVertA + 1; iTestVert < iWorkingVertB; iTestVert++)
                        {
                            int iTestVertBase = iTestVert * 3; 
                            if( workingVerts[iTestVertBase+1] >= heightPathLimit )  //异常的高度
                            {
                                Logger.LogWarning("[DetailMeshBuilder][buildPolyDetail]Potential Loss Height|{0}|{1}",workingVerts[iTestVertBase],workingVerts[iTestVertBase+2]);
                                continue; 
                            } // heightPathLimit

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
                            //找到iWorkingIndex后面
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

                }  //for SourceVertCount , 遍历边
            } // if mContourSampleDistance


            return 0; 
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
