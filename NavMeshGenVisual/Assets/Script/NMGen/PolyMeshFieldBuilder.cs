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

                //初始情况下，顶点索引对应的就是对应的顶点
                /*
                 *   Indices : 0 1 2 3 4 5 6  ....
                 *             0 1 2 3 4 5 6  ....
                 */
                workingIndices.Clear(); 
                for(int i = 0; i < contour.vertCount; ++i)  //这个顶点数是按四个一组的 
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
            //这个DEFLAG就是一个全集吧
            int pVertA = ((indices[indexA] & DEFLAG) * 4);  //因为是按四个一组的，所以需要 *4
            int pVertB = ((indices[indexB] & DEFLAG) * 4);

            int prevIndexA = getPreviousIndex(indexA, indices.Count);
            int nextIndexA = getNextIndex(indexA, indices.Count); 
            int pVertAMinus = ((indices[prevIndexA] & DEFLAG) * 4 );
            int pVertAPlus = ((indices[nextIndexA] & DEFLAG) * 4 );

            //这里内角大于180度的情况
            //点A在 AMinu->APlus的左边 ,坑爹啊，这里是以 APlus为起点来看的
            if ( isLeftOrCollinear( verts[pVertA],verts[pVertA+2],
                verts[pVertAMinus],verts[pVertAMinus+2],
                verts[pVertAPlus],verts[pVertAPlus+2] ) )
            {
                // B 在 A->AMinus的左边 ,以Minus为起点来看
                return isLeft( verts[pVertB],verts[pVertB+2],
                    verts[pVertA],verts[pVertA+2],
                    verts[pVertAMinus],verts[pVertAMinus+2] )
                // B 在 A->APlus的右边 ，以APlus为起点来看
                && isRight(verts[pVertB], verts[pVertB + 2],
                    verts[pVertA], verts[pVertA + 2],
                    verts[pVertAPlus], verts[pVertAPlus + 2]);
            }


            //内角小于180的情况，如果是位于外角，反转便是了
            return !(
                //判断一下
                isLeftOrCollinear(verts[pVertB], verts[pVertB + 2],
                verts[pVertA], verts[pVertA + 2],
                verts[pVertAPlus], verts[pVertAPlus + 2])

                && isLeftOrCollinear(verts[pVertB], verts[pVertB + 2],
                verts[pVertA], verts[pVertA + 2],
                verts[pVertAMinus], verts[pVertAMinus + 2]) ) ; 

                //源码好像有误，先注释掉
                //&& isRightOrCollinear( verts[pVertB],verts[pVertB+2],
                //verts[pVertA],verts[pVertA+2],
                //verts[pVertAMinus],verts[pVertAMinus+2] ) 
                //); 


        }

        private static bool isRightOrCollinear(int px,int py,
            int ax,int ay,
            int bx,int by)
        {
            return getSignedAreaX2(ax, ay, px, py, bx, by) >= 0;  
        }

        private static bool isRight(int px,int py,
            int ax,int ay,
            int bx,int by)
        {
            return getSignedAreaX2(ax, ay, px, py, bx, by) > 0; 
        }

        private static bool isLeft(int px,int py,
            int ax,int ay,
            int bx,int by)
        {
            return getSignedAreaX2(ax, ay, px, py, bx, by) < 0;  
        }

        private static bool isLeftOrCollinear(int px,int py,
            int ax,int ay,
            int bx,int by)
        {
            return getSignedAreaX2(ax, ay, px, py, bx, by) <= 0;    
        }

        /// <summary>
        /// http://www.mathopenref.com/coordpolygonarea.html
        /// http://mathworld.wolfram.com/TriangleArea.html
        /// http://www.everyinch.net/index.php/computergeometry1/  在现代三角形面积计算方法那里
        /// https://qiita.com/fujii-kotaro/items/a411f2a45627ed2f156e  割耳法
        /// </summary>
        /// <param name="ax"></param>
        /// <param name="ay"></param>
        /// <param name="bx"></param>
        /// <param name="by"></param>
        /// <param name="cx"></param>
        /// <param name="cy"></param>
        /// <returns></returns>
        private static int getSignedAreaX2(int ax,int ay,
            int bx,int by,
            int cx,int cy)
        {
            /*  
             *              v2
             *             / \ 
             *            /   \
             *           /     \
             *        v0 - - -- v1
             *            
             *  
             *      A = 1/2 | V X W|
             *        = 1/2 | (v1-v0) x (v2-v0)|
             * 
             * =>  2A = (x1-x0)(y2-y0) - (x2-x0)(y1-y0)
             * 
             *   v0 = a
             *   v1 = b
             *   v2 = c
             * 
             *   用右手定则，如果 v1-v0 x  v2-v0  小于 < 0 则，法向量指向下，证明v1 在 v2-v0左边
             */


            return (bx - ax) * (cy - ay) - (cx - ax) * (by - ay); 
        }


        private static int getPreviousIndex(int i,int n)
        {
            return i - 1 >= 0 ? i - 1 : n - 1; 
        }

        //为什么超出之后不是环绕呢？
        private static int getNextIndex(int i,int n)
        {
            return i + 1 < n ? i + 1 : 0; 
        }

    } // class
}
