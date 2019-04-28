using System;
using Utility.Logger;
using System.Collections.Generic;


namespace NMGen
{
    public class PolyMeshFieldBuilder
    {
        private readonly static int FLAG = 0x8000000;  // 20 000 000 000
        private readonly static int DEFLAG = 0xFFFFFFF;  //1777 777 777

        private int mMaxVertsPerPoly; 

        public PolyMeshFieldBuilder(int maxVertsPerPoly)
        {
            mMaxVertsPerPoly = maxVertsPerPoly; 
        }

        public PolyMeshField build(ContourSet contours)
        {
            if( null == contours
                || 0 == contours.size() )
            {
                Logger.LogError("[PolyMeshFieldBuild][build]Param Error");  
                return null;
            }

            PolyMeshField result = new PolyMeshField(contours.boundsMin(),
                   contours.boundsMax(),
                   contours.cellSize(),
                   contours.cellHeight(),
                   mMaxVertsPerPoly);

            int sourceVertCount = 0;
            int maxPossiblePolygons = 0;
            int maxVertsPerContour = 0;  

            for(int contourIndex = 0; contourIndex < contours.size(); ++contourIndex)
            {
                int count = contours.get(contourIndex).vertCount;
                sourceVertCount += count;
                //过 n 边形的一个顶点，能把n边形最多分成 n - 2 三角形
                maxPossiblePolygons += count - 2;  
                maxVertsPerContour = Math.Max(maxVertsPerContour, count);  
            }

            if( sourceVertCount - 1 > DEFLAG  )
            {
                Logger.LogError("[PolyMeshFieldBuilder][build]sourceVertCount out of Range|{0}|{1}",DEFLAG,sourceVertCount);
                return null; 
            }

            int[] globalVerts = new int[sourceVertCount * 3];
            int globalVertCount = 0;

            int[] globalPolys = new int[maxPossiblePolygons * mMaxVertsPerPoly]; 
            for(int i = 0; i < globalPolys.Length; ++i)
            {
                globalPolys[i] = PolyMeshField.NULL_INDEX; 
            }

            int[] globalRegions = new int[maxPossiblePolygons];
            int globalPolyCount = 0;

            int[] contourToGlobalIndicesMap = new int[maxVertsPerContour];
            Dictionary<int, int> vertIndices = new Dictionary<int, int>();

            List<int> workingIndices = new List<int>(maxVertsPerContour);
            List<int> workingTriangles = new List<int>(maxVertsPerContour);

            //TODO 为什么要+1？
            int[] workingPolys = new int[(maxVertsPerContour + 1) * mMaxVertsPerPoly];
            int workingPolyCount = 0;
            int[] mergeInfo = new int[3];
            int[] mergedPoly = new int[mMaxVertsPerPoly];  

            for(int contourIndex = 0; contourIndex < contours.size(); ++contourIndex)
            {
                Contour contour = contours.get(contourIndex); 
                //4个数据一组，(x,y,z,regionID)，最少需要三个顶点
                if( contour.verts.Length < 3 * 4 ) 
                {
                    Logger.LogError("[PolyMeshFieldBuilder][build]Bad Contour|{0}", contour.regionID);
                    continue; 
                }

                workingIndices.Clear(); 
                for(int i = 0; i < contour.vertCount; ++i)
                {
                    workingIndices.Add(i);  
                } // for contour.vert

                //三角剖分
                int triangleCount = triangulate(contour.verts, ref workingIndices,ref  workingTriangles); 

            } // for contours
        } // build 

        private static int triangulate(int[] verts,
            ref List<int> inoutIndices ,
            ref List<int> outTriangles )
        {
            outTriangles.Clear();  
            for(int i = 0; i < inoutIndices.Count; ++i)
            {
                int iPlus1 = getNextIndex(i, inoutIndices.Count);
                int iPlus2 = getNextIndex(iPlus1, inoutIndices.Count); 
                if( isValidPartition(i,iPlus2,verts,inoutIndices) )
                {
                    inoutIndices[iPlus1] = inoutIndices[iPlus1] | FLAG; 
                }

            } // for inoutIndices
        } // triangulate

        private static bool isValidPartition(int indexA,int indexB,
            int[] verts,
            List<int> indices)
        {
            return liesWithinInternalAngle(indexA, indexB, verts, indices)
                && !hasIllegalEdgeIntersection(indexA, indexB, verts, indices); 
        }

        private static bool liesWithinInternalAngle(int indexA,
            int indexB,
            int[] verts ,
            List<int> indices)
        {
            int pVertA = ((indices[indexA] & DEFLAG) * 4);
            int pVertB = ((indices[indexB] & DEFLAG) * 4);

        }

        //为什么超出之后不是环绕呢？
        private static int getNextIndex(int i,int n)
        {
            return i + 1 < n ? i + 1 : 0; 
        }

    } // class
}
