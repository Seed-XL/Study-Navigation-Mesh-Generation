using System;
using System.Collections;
using System.Collections.Generic;
using Utility.Logger;

namespace NMGen
{
    public class MatchNullRegionEdges : IContourAlgorithm 
    {
        private static int NULL_REGION = OpenHeightSpan.NULL_REGION;
        private float mThreshold; 

        public MatchNullRegionEdges(float threshold)
        {
            this.mThreshold = Math.Max(threshold, 0); 
        }

        public void apply( List<int> sourceVerts ,List<int> inoutResultVerts )
        {
            if( null == sourceVerts
                || null == inoutResultVerts )
            {
                Logger.LogError("[MatchNullRegionEdges][apply]invalid param");
                return; 
            }

            int sourceVertCount = sourceVerts.Count / 4;
            int simplifiedVertCount = inoutResultVerts.Count / 4;
            int iResultVertA = 0;  

            while( iResultVertA < simplifiedVertCount )
            {
                int iResultVertB = (iResultVertA + 1) % simplifiedVertCount ; //下一个顶点

                int ax = inoutResultVerts[iResultVertA * 4];
                int az = inoutResultVerts[iResultVertA * 4 + 2];
                int iVertASource = inoutResultVerts[iResultVertA * 4 + 3];  //原来顶点所在的索引

                int bx = inoutResultVerts[iResultVertB * 4];
                int bz = inoutResultVerts[iResultVertB * 4 + 2];
                int iVertBSource = inoutResultVerts[iResultVertB * 4 + 3];

                int iTestVert = (iVertASource + 1) % sourceVertCount;
                float maxDeviation = 0;

                int iVertToInsert = -1; 
                if( NULL_REGION == sourceVerts[iTestVert*4+3] )
                {
                    while( iTestVert != iVertBSource  )
                    {

                    }
                }

            } // while 
        }
    }
}
