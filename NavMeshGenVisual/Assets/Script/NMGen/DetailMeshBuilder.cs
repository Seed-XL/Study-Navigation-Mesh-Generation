﻿using System;
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

            //(xmin,xmax,zmin,zmax)
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

                polyXZBounds[pxmin] = Math.Max(0, polyXZBounds[pxmin] - 1);
                polyXZBounds[pxmax] = Math.Min(heightField.width(),polyXZBounds[pxmax] + 1) ;
                polyXZBounds[pzmin] = Math.Max(0, polyXZBounds[pzmin] - 1);
                polyXZBounds[pzmax] = Math.Min(heightField.depth(), polyXZBounds[pzmax] + 1); 

                if( polyXZBounds[pxmin] >= polyXZBounds[pxmax]
                    || polyXZBounds[pzmin] >= polyXZBounds[pzmax] )
                {
                    continue; 
                }

                maxPolyWidth = Math.Max(maxPolyWidth, polyXZBounds[pxmax] - polyXZBounds[pxmin]);
                maxPolyDepth = Math.Max(maxPolyDepth, polyXZBounds[pzmax] - polyXZBounds[pzmin]); 

            } // for sourcePolyCount


            float[] poly = new float[maxVertsPerPoly * 3];
            int polyVertCount = 0;

            List<int> polyTriangles = new List<int>(maxVertsPerPoly * 2 * 3);

            float[] polyTriangleVerts = new float[MAX_VERTS * 3];

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

                    int pVert = sourcePolys[pPoly + vertOffset] * 3;

                    //查看生成轮廓那部分的代码，特别是生成Raw轮廓那里，就知道里面的顶点x z 方向的顶点索引，都是按照
                    //Span的索引算出来的，然后Span就是按照cellSize来划分的，所以pVert指向的顶点的索引，就需要乘以
                    //cellSize来还原，这里面的位置都是相对的。
                    poly[vertOffset * 3 + 0] = sourceVerts[pVert] * cellSize;
                    poly[vertOffset * 3 + 1] = sourceVerts[pVert + 1] * cellHeight;  
                    poly[vertOffset * 3 + 2] = sourceVerts[pVert+2] * cellSize;

                    polyVertCount++; 
                } // for maxVertsPerPoly

                if( mContourSampleDistance > 0 )
                {
                    hfPatch.minWidthIndex = polyXZBounds[iPoly * 4];
                    hfPatch.minDepthIndex = polyXZBounds[iPoly * 4 + 2];
                    hfPatch.width = polyXZBounds[iPoly * 4 + 1] - polyXZBounds[iPoly * 4 + 0];
                    hfPatch.depth = polyXZBounds[iPoly * 4 + 3] - polyXZBounds[iPoly * 4 + 2];

                    loadHeightPatch(); 

                } // if mContourSampleDistance

            } // all Polygons 

        } //build 

        private static void loadHeightPatch(int polyPointer,int vertCount,int[] indices,int[] verts,
            OpenHeightfield heightField,
            HeightPatch inoutPath,
            Stack<int> gridIndexStack,
            Stack<OpenHeightSpan> spanStack,
            int[] widthDepth)
        {
            inoutPath.resetData();
            gridIndexStack.Clear();
            spanStack.Clear(); 


            for(int vertOffset = 0; vertOffset < vertCount; ++vertOffset)
            {
                //看这里，这就是前面为什么要乘以  cellSize的原因
                int baseIdx = indices[polyPointer + vertOffset] * 3; 
                int vertX = verts[baseIdx+0];
                int vertY = verts[baseIdx + 1];
                int vertZ = verts[baseIdx + 2];

                OpenHeightSpan selectedSpan = getBestSpan(vertX, vertY, vertZ, heightField, widthDepth);  

            } // for vertCount
        }

    } // builder
}// namespace
