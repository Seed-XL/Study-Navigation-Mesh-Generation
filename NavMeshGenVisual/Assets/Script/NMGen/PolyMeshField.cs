using System;
using Utility.Logger;
using System.Collections.Generic;


namespace NMGen
{
    public class PolyMeshField : BoundeField
    {
        public readonly static int NULL_INDEX = -1;
        private int mMaxVertsPerPoly;
        public int[] verts = null;

        //每个poly的数据占 2 * mMaxVertsPerPoly。
        //第1份包含的是多边形顶点的索引
        //第2份包含的指向相邻多边形顶点的索引
        public int[] polys = null;


        public int[] polyRegions = null;  

        public PolyMeshField( float[] gridBoundsMin ,
            float[] gridBoundsMax,
            float cellSize, 
            float cellHeight,
            int maxVertsPerPoly):base(gridBoundsMin,gridBoundsMax,cellSize,cellHeight)
        {
            mMaxVertsPerPoly = Math.Max(maxVertsPerPoly, 3); 
        }


        public int getPolyRegion(int polyIndex)
        {
            if( polyIndex < 0 
                || polyIndex >= polyRegions.Length )
            {
                return -1; 
            }
            return polyRegions[polyIndex]; 
        }

        private int GetPolyPointer(int polyIndex)
        {
            return polyIndex * mMaxVertsPerPoly * 2; 
        }

        public int[] getPolyVerts( int polyIndex )
        {
            //TODO 为啥是乘以2 ？
            int pPoly = GetPolyPointer(polyIndex) ; 
            if( polyIndex < 0 
                || pPoly >= polys.Length )
            {
                return null; 
            }

            int polyVertCount = getPolyVertCount(pPoly, polys, mMaxVertsPerPoly);
            int[] result = new int[polyVertCount * 3]; //乘以3，是因为 xyz 三个数据 

            for(int i = 0; i < polyVertCount; ++i)
            {
                int pVert = polys[pPoly + i] * 3;  //3个数据为一级
                result[i * 3] = verts[pVert]; //x
                result[i * 3 + 1] = verts[pVert + 1];  //y
                result[i * 3 + 1] = verts[pVert + 2];  //z
            }

            return result; 
        }

        public int maxVertsPerPoly()
        {
            return mMaxVertsPerPoly; 
        }

        public int polyCount()
        {
            if( null == polys )
            {
                return 0; 
            }
            return polys.Length / ( 2 * mMaxVertsPerPoly ); 
        }

        public int vertCount()
        {
            if( null == verts )
            {
                return 0; 
            }
            return verts.Length / 3; 
        }

        /// <summary>
        /// 以polyPointer为Base，开始遍历直到遇到NULL_INDEX 统计多边形有多少个顶点
        /// </summary>
        /// <param name="polyPointer"></param>
        /// <param name="polys"></param>
        /// <param name="maxVertsPerPoly"></param>
        /// <returns></returns>
        private static int getPolyVertCount(int polyPointer ,
            int[] polys,
            int maxVertsPerPoly )
        {
            for(int i = 0; i < maxVertsPerPoly; ++i)
            {
                //遇到第一个结束符，就是顶点的数目了
                if( NULL_INDEX == polys[polyPointer+i] )
                {
                    return i; 
                }
            }
            return maxVertsPerPoly;  
        }
    }
}
