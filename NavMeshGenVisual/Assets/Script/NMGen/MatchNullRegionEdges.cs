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


        /// <summary>
        /// 
        /// </summary>
        /// <param name="sourceVerts">这个原始Region的顶点</param>
        /// <param name="inoutResultVerts">这个是上一步得到的强变化顶点</param>
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

            //从简化列表的第一个点开始
            while( iResultVertA < simplifiedVertCount )
            {
                //简化顶点的第二个
                int iResultVertB = (iResultVertA + 1) % simplifiedVertCount ; //下一个顶点

                //ay = inoutResultVerts[iResultVertA * 4 + 1] ;
                int ax = inoutResultVerts[iResultVertA * 4];
                int az = inoutResultVerts[iResultVertA * 4 + 2];
                int iVertASource = inoutResultVerts[iResultVertA * 4 + 3];  //原来顶点所在的索引

                int bx = inoutResultVerts[iResultVertB * 4];
                int bz = inoutResultVerts[iResultVertB * 4 + 2];
                //简化顶点集的第二个在原始顶点的索引
                int iVertBSource = inoutResultVerts[iResultVertB * 4 + 3];

                //从a-b边开始逼近


                // iTestVert 其实就是 原始点中 iVertASource 和 iVertBSource中的某个点。
                // 它在简化顶点集里面可能并不存在。
                int iTestVert = (iVertASource + 1) % sourceVertCount;

                float maxDeviation = 0;
                int iVertToInsert = -1; 

                //如果 AB 边之间的某个顶点对应的是NULL_REGION，那么其中就是连接NULL_REGION 
                if( NULL_REGION == sourceVerts[iTestVert*4+3] )
                {
                    //找到在 iVertASource 和 iVertBSource 之间，离对应Simple边最远的顶点
                    while( iTestVert != iVertBSource  )
                    {
                        //计算原始点，距离对应Simple边的距离是否大于阈值
                        float deviation = Geometry.getPointSegmentDistanceSq(
                            sourceVerts[iTestVert * 4], //x
                            sourceVerts[iTestVert * 4 + 2],  //z 
                            ax,
                            az,
                            bx,
                            bz
                            );

                        //找到原始点中离Simple边最远的点，找到之后 ，就找回到
                        //iVertASource和iVertBSouce中。
                        if( deviation > maxDeviation )
                        {
                            maxDeviation = deviation;
                            iVertToInsert = iTestVert;  
                        }

                        iTestVert = (iTestVert + 1) % sourceVertCount; 
                    } // while 

                    if( iVertToInsert != -1 
                        &&  maxDeviation > ( mThreshold * mThreshold ))
                    {
                        //在iResultVertA的下一个位置插入，在原来顶点集中的iVertToInsert
                        int iInsertResultVert = iResultVertA + 1;
                        int iInsertResultVertBase = iInsertResultVert * 4; 
                        int iVertToInsertBase = iVertToInsert * 4; 
                        inoutResultVerts.Insert(iInsertResultVertBase, sourceVerts[iVertToInsertBase]);
                        inoutResultVerts.Insert(iInsertResultVertBase + 1, sourceVerts[iVertToInsertBase + 1]);
                        inoutResultVerts.Insert(iInsertResultVertBase + 2, sourceVerts[iVertToInsertBase + 2]);
                        inoutResultVerts.Insert(iInsertResultVertBase + 3, iVertToInsert);

                        simplifiedVertCount = inoutResultVerts.Count / 4; 
                    }
                    else
                    {
                        iResultVertA++; 
                    }  //
                }

            } // while 
        }
    }
}
