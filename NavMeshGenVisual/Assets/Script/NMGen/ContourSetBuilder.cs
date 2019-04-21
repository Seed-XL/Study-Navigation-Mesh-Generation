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
                if( span.flags == 0xf)   //证明四个邻居都不在同一个Region或者是一个孤岛Span
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

                //找到第一个不是同一个Region的Span
                int startDir = 0; 
                while( isSameRegion(span,startDir) )
                {
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
                //第四个字段就RegionID 对应的索引就是3
                if( sourceVerts[pVert+3] != NULL_REGION )
                {
                    noConnections = false;
                    break; 
                }
            } //for 

            if( noConnections )
            {
                //一个四周都是NULL_Region的Region

                // ll => lowr left
                // ur => upper right
                int llx = sourceVerts[0];
                int lly = sourceVerts[1];
                int llz = sourceVerts[2];
                int lli = 0;

                int urx = sourceVerts[0];
                int ury = sourceVerts[1];
                int urz = sourceVerts[2];
                int uri = 0;

                for (int pVert = 0; pVert < sourceVerts.Count; pVert += 4)
                {
                    int x = sourceVerts[pVert];
                    int y = sourceVerts[pVert + 1];
                    int z = sourceVerts[pVert + 2];

                    if( x < llx 
                        || ( x == llx && z < llz )) 
                    {
                        llx = x;
                        lly = y;
                        llz = z;
                        lli = pVert / 4;  
                    }
                    
                    if( x >= urx 
                        || ( x == urx && z > urz))
                    {
                        urx = x;
                        ury = y;
                        urz = z;
                        uri = pVert / 4; 
                    }
                } // for 

                //最低点和最高点，连在一起
                outVerts.Add(llx);
                outVerts.Add(lly);
                outVerts.Add(llz);
                outVerts.Add(lli);

                outVerts.Add(urx);
                outVerts.Add(ury);
                outVerts.Add(urz);
                outVerts.Add(uri);

            } // if noConnections
            else
            {
                for( int iVert = 0 , vCount = sourceVerts.Count /4; 
                    iVert < vCount;  
                    ++iVert )
                {
                    //当前顶点与下一个顶点属于不同的Region，所以是一个突变点？
                    if( !(sourceVerts[iVert*4+3] == sourceVerts[((iVert+1)%vCount)*4+3] ) )
                    {
                        outVerts.Add(sourceVerts[iVert * 4]);
                        outVerts.Add(sourceVerts[iVert * 4 + 1]);
                        outVerts.Add(sourceVerts[iVert * 4 + 2]);
                        outVerts.Add(iVert);  
                    }
                }
            }  // else noConnections


            //1、多边形逼近的算法
            //2、找到连接Null-Region的超长边
            foreach(IContourAlgorithm algorithm in mAlgorithms)
            {
                algorithm.apply(sourceVerts, outVerts); 
            }

            int sourceVertCount = sourceVerts.Count / 4;
            if ( outVerts.Count < 12 )  //Simple Contour 小于三个点
            {
                int iSelected = -1;
                float maxDistance = 0;
                int ax = outVerts[0];
                int az = outVerts[2];
                int bx = outVerts[4];
                int bz = outVerts[6]; 

                //最长去到 8

                for(int iVert = 0; iVert < sourceVertCount; ++iVert )
                {
                    //找到原来顶点中，离这两点组成的线段最远的点
                    float dist = Geometry.getPointSegmentDistanceSq(
                        sourceVerts[iVert * 4 + 0],
                        sourceVerts[iVert * 4 + 2],
                        ax, az,
                        bx, bz
                        );

                    if( dist > maxDistance )
                    {
                        maxDistance = dist;
                        iSelected = iVert; 
                    }
                } // for

                //如果这个选中的顶点 Simple Vert 中 顶点A 在 Source Verts 中的索引还要早
                if( iSelected < outVerts[3] )
                {
                    //bx 向后挪一位
                    outVerts.Add(bx);
                    outVerts.Add(outVerts[5]);
                    outVerts.Add(bz);
                    outVerts.Add(outVerts[7]);

                    //ax 向后挪一位
                    outVerts[4] = ax;
                    outVerts[5] = outVerts[1];
                    outVerts[6] = az;
                    outVerts[7] = outVerts[3];

                    //直接插到最前
                    int iSelectedBase = iSelected * 4;
                    outVerts[0] = sourceVerts[iSelectedBase];
                    outVerts[1] = sourceVerts[iSelectedBase + 1];
                    outVerts[2] = sourceVerts[iSelectedBase + 2];
                    outVerts[3] = sourceVerts[iSelected];
                }
                else if( iSelected < outVerts[7])  //在B之前 
                {
                    //bx 向后挪一位
                    outVerts.Add(bx);
                    outVerts.Add(outVerts[5]);
                    outVerts.Add(bz);
                    outVerts.Add(outVerts[7]);

                    //插到B原来的位置
                    int iSelectedBase = iSelected * 4;
                    outVerts[4] = sourceVerts[iSelectedBase];
                    outVerts[5] = sourceVerts[iSelectedBase + 1];
                    outVerts[6] = sourceVerts[iSelectedBase + 2];
                    outVerts[7] = sourceVerts[iSelected];
                }
                else  //直接插到最后面
                {
                    int iSelectedBase = iSelected * 4;
                    outVerts.Add(iSelectedBase);
                    outVerts.Add(iSelectedBase + 1 );
                    outVerts.Add(iSelectedBase + 2);
                    outVerts.Add(iSelected);
                }
            }  // outVerts.Count < 12 

            sourceVertCount = sourceVerts.Count / 4;
            int simplifiedVertCount = outVerts.Count / 4; 

            //TODO 为什么用的是下一个顶点的Region ？
            for(int iVert = 0; iVert < simplifiedVertCount; ++iVert )
            {
                //TODO  RegionID从源顶点对应的顶点的下一个中取出来？为啥不取自己的？
                //  这里不是明显有问题么？因为有可能是不同的Region
                int iVertSourceIdx = iVert * 4 + 3;
                //这个我先注释掉
                //int sourceVertIndex = ( outVerts[iVertSourceIdx] + 1 ) % sourceVertCount ;
                int sourceVertIndex = (outVerts[iVertSourceIdx]) % sourceVertCount;
                outVerts[iVertSourceIdx] = sourceVerts[sourceVertIndex * 4 + 3];  //将输出Idx连接的Region设置为下一个顶点的Region
            }

            removeVerticalSegments(regionID, outVerts);
            removeIntersectingSegments(regionID, outVerts); 
        }  // func end 

        private static void removeIntersectingSegments(int regionID,List<int> verts)
        {
            int startSize = verts.Count;
            int vCount = startSize / 4; 
            for(int iVert = 0; iVert < vCount; ++iVert)
            {
                int iVertNext = (iVert + 1) % vCount; 
                if( NULL_REGION != verts[iVertNext*4+3] )
                {
                    iVert += removeIntersectingSegments()
                }
            }
        }

        private static void removeVerticalSegments(int regionID ,List<int> verts)
        {
            for( int pVert = 0; pVert < verts.Count; )
            {
                int pNextVert = (pVert + 4) % verts.Count; 
                if( verts[pVert] == verts[pNextVert]   //x坐标
                    && verts[pVert+2] == verts[pNextVert+2] )  //z坐标
                {
                    verts.Remove(pNextVert);
                    verts.Remove(pNextVert); //+1
                    verts.Remove(pNextVert); //+2
                    verts.Remove(pNextVert); //+3

                    Logger.LogWarning("[ContourSetBuilder][removeVerticalSegments]Contour Detail lost|{0}", regionID); 
                }
                else
                {
                    pVert += 4; 
                }
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
                //这是一个突变点
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
