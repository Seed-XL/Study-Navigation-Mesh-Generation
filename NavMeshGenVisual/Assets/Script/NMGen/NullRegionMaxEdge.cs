using System;
using System.Collections;
using System.Collections.Generic;
using Utility.Logger;

namespace NMGen
{
    public class NullRegionMaxEdge : IContourAlgorithm 
    {
        private static int NULL_REGION = OpenHeightSpan.NULL_REGION;
        private int mMaxEdgeLength;  

        public NullRegionMaxEdge(int maxEdgeLength)
        {
            mMaxEdgeLength = Math.Max(maxEdgeLength, 0);
        }

        public void apply(List<int> sourceVerts ,
            List<int> resultVerts)
        {
            if( mMaxEdgeLength <= 0 )
            {
                Logger.LogWarning("[NullRegionMaxEdge][apply]Feature Disable"); 
                return; 
            }

            int sourceVertCount = sourceVerts.Count / 4;
            int resultVertCount = resultVerts.Count / 4;
            int iVertA = 0; 

            while( iVertA < resultVertCount )
            {
                int iVertB = (iVertA + 1) % resultVertCount;
                int iVertAIdxBase = iVertA * 4; 
                int ax = resultVerts[iVertAIdxBase];
                int az = resultVerts[iVertAIdxBase + 2];
                int iVertASource = resultVerts[iVertAIdxBase + 3];

                int iVertBIdxBase = iVertB * 4;
                int bx = resultVerts[iVertBIdxBase];
                int bz = resultVerts[iVertBIdxBase + 2];
                int iVertBSource = resultVerts[iVertBIdxBase + 3];

                int iNewVert = -1;
                int iTestVert = (iVertASource + 1) % sourceVertCount;
                //注意，检查的是iVertASource下一个顶点是否连接到NULL_REGION
                if ( NULL_REGION  == sourceVerts[iTestVert * 4 + 3] ) 
                {
                    int dx = bx - ax;
                    int dz = bz - az; 

                    if( dx * dx + dz * dz > mMaxEdgeLength * mMaxEdgeLength )
                    {
                        //VertB 因为环绕，有可能会比 VertA前面
                        int indexDistance = iVertBSource < iVertASource
                            ? (iVertBSource + (sourceVertCount - iVertASource))
                            : (iVertBSource - iVertASource);
                        //在两个顶点之间再插一个新的顶点
                        iNewVert = (iVertASource + indexDistance / 2) % sourceVertCount; 
                    } // Compare 
                } // if NULL_REGION 

                if( iNewVert != -1 )
                {
                    //直接在A的下一个顶点插入新的顶点
                    int iNewInsertToResultBase = (iVertA + 1) * 4;
                    int iNewVertSourceIdxBase = iNewVert * 4; 

                    resultVerts.Insert(iNewInsertToResultBase,iNewVertSourceIdxBase);
                    resultVerts.Insert(iNewInsertToResultBase+1, iNewVertSourceIdxBase + 1);
                    resultVerts.Insert(iNewInsertToResultBase+2, iNewVertSourceIdxBase + 2);
                    resultVerts.Insert(iNewInsertToResultBase+3, iNewVert );  //在原来的顶点列表中的索引

                    resultVertCount = resultVerts.Count / 4;  
                }
                else
                {
                    iVertA++; 
                }
            }

        }

    }
}
