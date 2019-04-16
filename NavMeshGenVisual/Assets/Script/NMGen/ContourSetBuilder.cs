using System;
using Utility.Logger;
using System.Collections.Generic;


namespace NMGen
{
    public class ContourSetBuilder
    {
        private static readonly int NULL_REGION = OpenHeightSpan.NULL_REGION;
        private List<IContourAlgorithm> mAlgorithms = new List<IContourAlgorithm>(); 

        public ContourSetBuilder(List<IContourAlgorithm> algorithms)
        {
            if( algorithms == null )
            {
                Logger.LogWarning("[ContourSetBuilder][Ctor]null"); 
                return; 
            }

            this.mAlgorithms.AddRange(algorithms); 
        }


        public ContourSet build(OpenHeightfield sourceField)
        {
            if( null == sourceField
                || 0 == sourceField.regionCount() )
            {
                Logger.LogError("[ContourSetBuilder][build]sourceField Invalid"); 
                return null; 
            }

            ContourSet result = new ContourSet(sourceField.boundsMin(),
                sourceField.boundsMax(),
                sourceField.cellSize(),
                sourceField.cellHeight(),
                sourceField.regionCount());

            int discardedContours = 0;

            //TODO
            /*
            *  If a span has no connections to external regions or is
            *  completely surrounded by other regions (a single span island),
            *  its flag will be zero.
            * 
            *  If a span is connected to one or more external regions then the
            *  flag will be a 4 bit value where connections are recorded as
            *  follows:
            *      bit1 = neighbor0
            *      bit2 = neighbor1
            *      bit3 = neighbor2
            *      bit4 = neighbor3
            *  With the meaning of the bits as follows:
            *      0 = neighbor in same region.
            *      1 = neighbor not in same region. (Neighbor may be the null
            *      region or a real region.)
            */
            OpenHeightfield.OpenHeightFieldIterator iter = sourceField.GetEnumerator(); 
            while( iter.MoveNext() )
            {
                OpenHeightSpan span = iter.Current;
                span.flags = 0;   //默认没有与任何外部Region相连
                if( NULL_REGION == span.regionID() )
                {
                    continue; 
                }

                for( int dir = 0; dir < 4; ++dir)
                {
                    int nRegionID = NULL_REGION;
                    OpenHeightSpan nSpan = span.getNeighbor(dir); 
                    if( nSpan != null )
                    {
                        nRegionID = nSpan.regionID(); 
                    }
                    //这里是反常操作，先将相同的当作是1，然后统一位反
                    if( nRegionID == span.regionID() )
                    {
                        span.flags |= 1 << dir; 
                    }
                } // for 

                //1111111
                span.flags ^= 0xf; 
                if( span.flags == 0xf )   //证明四个邻居都不在同一个Region？？？
                {
                    //重置这个位置
                    span.flags = 0;
                    discardedContours++;
                    Logger.LogWarning("[ContourSetBuilder][apply]Island Span|{0}",span.regionID()); 
                }

            }  //while iter  

            List<int> workingRawVerts = new List<int>(256);
            List<int> workingSimplifiedVerts = new List<int>(64);

            iter.Reset(); 
            while( iter.MoveNext() )
            {
                OpenHeightSpan span = iter.Current; 
                if( NULL_REGION == span.regionID()
                    || 0 == span.flags )  //flag等于0的话，是前面那些孤岛Span
                {
                    continue; 
                }

                workingRawVerts.Clear();
                workingSimplifiedVerts.Clear();

                int startDir = 0; 
                while( isSameRegion(span,startDir) )
                {
                    //不等于0，证明是另外一个Region
                    startDir++; 
                }

                buildRawContours(span,
                    iter.widthIndex(),
                    iter.depthIndex(),
                    startDir,
                    ref workingRawVerts
                    );

                generateSimplifiedContour(span.regionID(),
                    workingRawVerts,
                    ref workingSimplifiedVerts); 

            } // while iter 


            return null; 
        } // build 

        private void generateSimplifiedContour(int regionID,
            List<int> sourceVerts ,
            ref List<int> outVerts )
        {
            bool noConnections = true;

            for (int pVert = 0; pVert < sourceVerts.Count; pVert += 4 )
            {
               //TODO
            }
        }


        private static bool isSameRegion(OpenHeightSpan span , int dir)
        {
            if( span == null  )
            {
                return false ; 
            }

            return (span.flags & (1 << dir)) == 0;
        }


        private void buildRawContours(OpenHeightSpan startSpan,
            int startWidthIndex,
            int startDepthIndex,
            int startDirection ,
            ref List<int> outContourVerts 
            )
        {
            OpenHeightSpan span = startSpan;
            int dir = startDirection;
            int spanX = startWidthIndex;
            int spanZ = startDepthIndex;
            int loopCount = 0; 

            while( ++loopCount < ushort.MaxValue  )
            {
                if( !isSameRegion(span,dir) )
                {
                    //span
                    int px = spanX;
                    int py = getCornerHeight(span,dir);
                    int pz = spanZ;

                    /* 
                     * 这里取的是点，所以需要做这些偏移来记录点，而不是记录对应的Span
                     * 这里需要结合 ：  方向 + 某个点 ，来理解 为什么要加这个偏移
                     * 
                     *
                     *     * --- *
                     *     |  S  |
                     *     ----—-*     
                     *  
                     *    dir = 0 的时候，需要取左上角的点
                     *    dir = 1 的时候，需要取右上角的点   
                     *    dir = 2 的时候，需要取右下角的点
                     *    dir = 3 的时候，取的就是参考点
                     *    
                     *    以左下角的点为参考点，就是 dir方向对应的顺时针的点
                     * 
                     */


                    /*
                     * Update the px and pz values based on current direction.
                     * The update is such that the corner being represented is
                     * clockwise from the edge the direction is currently pointing
                     * toward.
                     */

                    switch(dir)
                    {
                        case 0: pz++; break;
                        case 1: px++; pz++; break;
                        case 2: px++; break; 
                    }

                    int regionThisDirection = NULL_REGION;
                    OpenHeightSpan nSpan = span.getNeighbor(dir); 
                    if( nSpan != null )
                    {
                        regionThisDirection = nSpan.regionID(); 
                    }

                    //这是轮廓的点
                    outContourVerts.Add(px);
                    outContourVerts.Add(py);
                    outContourVerts.Add(pz);
                    outContourVerts.Add(regionThisDirection);

                    span.flags &= ~(1 << dir);  //清除dir对应的位
                    dir = NMGenUtility.ClockwiseRotateDir(dir);  

                } //isSameRegion
                else
                {
                    //这段就是步进到下一个同Region的Span，很好理解。
                    span = span.getNeighbor(dir); 
                    switch(dir)
                    {
                        case 0: spanX--;break;
                        case 1: spanZ++;break;
                        case 2: spanX++;break;
                        case 3: spanZ--;break;
                    }
                    dir = NMGenUtility.CClockwiseRotateDir(dir);  

                } // no the SameRegion

                if( span == startSpan 
                    && dir == startDirection )
                {
                    break;  
                }

            }  //while
        } //buildRawContour


        /*
         *  以Span为s ， direction 为 0 举例，这个函数要寻找的就是s,ds,ns,里面，floor最大值 
         *      ds      
         *      ns  s   
         * 
         *  如果direction为1的话
         *          ns ds
         *          s 
         * 
         */
        private static int getCornerHeight(OpenHeightSpan span, int direction )
        {
            //OpenHeightSpan的floor 就真的是floor了，而不是
            //像 SolidHeightSpan那样对应的其实是顶部
            int maxFloor = span.floor();  
            OpenHeightSpan dSpan = null;
            //顺时针
            int directionOffset =  NMGenUtility.ClockwiseRotateDir(direction) ;
            OpenHeightSpan nSpan = span.getNeighbor(direction);
            if( nSpan != null )
            {
                maxFloor = Math.Max(maxFloor, nSpan.floor());
                dSpan = nSpan.getNeighbor(directionOffset); 
            }

            nSpan = span.getNeighbor(directionOffset);
            if( nSpan != null )
            {
                maxFloor = Math.Max(maxFloor, nSpan.floor()); 
                if( null == dSpan )
                {
                    dSpan = nSpan.getNeighbor(direction); 
                }
            }

            if( dSpan != null )
            {
                maxFloor = Math.Max(maxFloor, dSpan.floor()); 
            }

            return maxFloor;  
        } 
    }
}
