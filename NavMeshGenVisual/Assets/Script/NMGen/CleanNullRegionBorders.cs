using System;
using System.Collections;
using System.Collections.Generic;
using Utility.Logger;

namespace NMGen
{
    public class CleanNullRegionBorders : IOpenHeightFieldAlgorithm
    {
        private static readonly int NULL_REGION = OpenHeightSpan.NULL_REGION;
        private bool mUseOnlyNullSpans;

        private Stack<OpenHeightSpan> mwOpenSpans = new Stack<OpenHeightSpan>();
        private Stack<int> mwBorderDistance = new Stack<int>();
        private int[] mwNeighborRegions = new int[8]; 


        public CleanNullRegionBorders(bool useOnlyNullRegionSpans)
        {
            mUseOnlyNullSpans = useOnlyNullRegionSpans; 
        }

        public void apply(OpenHeightfield field)
        {
            if( null == field )
            {
                Logger.LogError("[CLeanNullRegionBorders][apply]field null"); 
                return; 
            }

            int nextRegionID = field.regionCount();
            OpenHeightfield.OpenHeightFieldIterator iter = field.GetEnumerator(); 
            while( iter.MoveNext() )
            {
                OpenHeightSpan span = iter.Current; 
                if( span.flags != 0  )
                {
                    continue; 
                }

                span.flags = 1;

                OpenHeightSpan workingSpan = null;
                int edgeDirection = -1; 

                if( NULL_REGION == span.regionID() )
                {
                    //找到Border Span第一个非空的邻居
                    edgeDirection = getNonNullBorderDrection(span); 
                    if( -1 == edgeDirection )
                    {
                        continue; 
                    }

                    //起点Span为有所属Region的Span
                    //而行走方向为 null Region所在的方向 
                    workingSpan = span.getNeighbor(edgeDirection);
                    //转180度
                    edgeDirection = NMGenUtility.AntiDir(edgeDirection) ;
                }
                else if ( !mUseOnlyNullSpans ) 
                {
                    //起点Span为 null Region的Span
                    //而行走方向即为有所属 Region 的 Span 

                    edgeDirection = getNullBorderDrection(span); 
                    if( -1 == edgeDirection )
                    {
                        continue; 
                    }
                    workingSpan = span; 
                }
                else
                {
                    continue; 
                }

                //上面两个分支都会保证workingSpan是有所属Region的
                //而Dir 即是 Region 为 Null Region的Dir 
                bool isEncompassedNullRegion = processNullRegion(workingSpan, edgeDirection); 
                if( isEncompassedNullRegion )  //确定以这个Span为起点的Region，是一个单一Region，并且内含了一个Null Region
                {
                    //如果是完全包含了不可走的NullRegion ，就将这个Region分割成两个Region
                    partialFloodRegion(workingSpan, edgeDirection, nextRegionID);
                    nextRegionID++; 
                }
            }

            field.setRegionCount(nextRegionID);

            iter.Reset(); 
            while(iter.MoveNext())
            {
                iter.Current.flags = 0; 
            }
            
        }      


        /// <summary>
        /// </summary>
        /// <param name="startSpan"></param>
        /// <param name="borderDirection"></param>
        /// <param name="newRegionID"></param>
        private void partialFloodRegion(OpenHeightSpan startSpan,
            int borderDirection ,
            int newRegionID )
        {
            int antiBorderDirection = NMGenUtility.AntiDir(borderDirection);
            int regionID = startSpan.regionID();

            startSpan.setRegionID(newRegionID);
            startSpan.setDistanceToRegionCore(0);  //???所以这个值 没啥卵用啊，一直都是0的
            mwOpenSpans.Push(startSpan);
            mwBorderDistance.Push(0);

            while( mwOpenSpans.Count > 0 )
            {
                OpenHeightSpan span = mwOpenSpans.Pop();
                int distance = mwBorderDistance.Pop(); 

                for(int i = 0; i < 4; ++i)
                {
                    OpenHeightSpan nSpan = span.getNeighbor(i); 
                    if( null == nSpan
                        || nSpan.regionID() != regionID )
                    {
                        continue; 
                    }

                    int nDistance = distance; 
                    if( i == borderDirection )  //null region所在的方向 
                    {
                        //这里是不是应该小于等于0呢？
                        // 以这个距离Border为0作为边界，将大于0那一侧的Span全部变成
                        // 新的Region。然后小于等于0那一侧的作为旧的Region保留下来。
                        if ( 0 == distance )  
                        {
                            continue;  
                        }
                        nDistance--;  
                    }
                    else if( i == antiBorderDirection)
                    {
                        nDistance++; 
                    }

                    //注意上面的if-else，如果都不是这两个方向的Span
                    //会直接被重新设置为新的Region


                    nSpan.setRegionID(newRegionID);
                    nSpan.setDistanceToRegionCore(0);

                    mwOpenSpans.Push(nSpan);
                    mwBorderDistance.Push(nDistance); 
                }
            }
        }

        /// <summary>
        /// 为了防止 Null Region 内含在有效Region内，这阻碍了凸包的生成
        /// </summary>
        /// <param name="startSpan"></param>
        /// <param name="startDirection"></param>
        /// <returns></returns>
        private bool processNullRegion(OpenHeightSpan startSpan ,int startDirection )
        {
            /*
             *  这段直接翻译源码的：
             *  这个算法遍历这个轮廓。正如它所做的，这个算法检测并且修复一些危险的
             *  Span Configurations。
             *  
             *  遍历轮廓：一个很好的可视化方向就是，想像一个机器人面向一堵墙，并坐
             *  在地板上。它会采取以下措施来绕过这堵墙：
             *  1. 如果有一堵墙位于它的前面，顺时针转90度，直到他前面不再是墙。
             *  2. 向前一步。
             *  3. 逆时针转向90度
             *  4. 重复从1开始的步骤 ，直到它发现自己位于原来的起点，还有原来的朝向。
             *   
             */

             /*
              * 算法在遍历的同时，检测锐角(90度) 和 钝角(270)拐点。如果
              * 一个完整的轮廓被检测完整，并且 钝角拐点比锐角拐点多。那
              * 么 null Region 就在这个轮廓里面。否则就在轮廓外面。
              * 
              */


            //环绕null region 走一圈，最后走回自己的起点
            int borderRegionID = startSpan.regionID();

            OpenHeightSpan span = startSpan;
            OpenHeightSpan nSpan = null;
            int dir = startDirection;

            int loopCount = 0;
            int acuteCornerCount = 0;
            int obtuseCornerCount = 0;
            int stepsWithoutBorder = 0;
            bool borderSeenLastLoop = false;
            bool isBorder = true;

            bool hasSingleConnection = true;  

            while( ++loopCount < int.MaxValue  )
            {
                //初始方向就是面向的Null Region，所以一开始是肯定是isBorder的
                nSpan = span.getNeighbor(dir);  
                if( null == nSpan )  
                {
                    isBorder = true;  
                }
                else
                {
                    nSpan.flags = 1;
                    if( NULL_REGION == nSpan.regionID() )
                    {
                        isBorder = true; 
                    }
                    else
                    {
                        isBorder = false; 
                        if( nSpan.regionID() != borderRegionID ) 
                        {
                            hasSingleConnection = false;      
                        }
                    }
                } // else

                
                if( isBorder )
                {
                    if( borderSeenLastLoop )
                    {
                        /*
                         *  a x
                         *  x x
                         */

                        //其实这个应用用inner来描述更加准确 ，表明a被x包围了
                        acuteCornerCount++; 
                    }
                    else if( stepsWithoutBorder > 1 )  
                    {
                        /*
                         *  a a
                         *  a x
                         *  
                         */
                        obtuseCornerCount++;  //相对地，我觉得这个应该用outer来描述更加准确，表明a正在包围x
                        stepsWithoutBorder = 0; 

                        //处理钝角的各种异常情况 
                        if( processOuterCorner(span,dir) )
                        {
                            hasSingleConnection = false; 
                        }
                    }

                    dir = NMGenUtility.ClockwiseRotateDir(dir); 
                    borderSeenLastLoop = true;
                    stepsWithoutBorder = 0;
                }
                else   //注意，不是边界，才会进行移动，如果是边界的话，只会进行转向
                {
                    span = nSpan;
                    dir = NMGenUtility.CClockwiseRotateDir(dir);  //逆时针转向一下
                    borderSeenLastLoop = false;
                    stepsWithoutBorder++; 
                } // else

                //回到原方位了
                if(  startSpan == span 
                    && startDirection == dir )
                {
                    return (hasSingleConnection
                        && obtuseCornerCount > acuteCornerCount ); 
                }
            } // while 

            return false;
        }

        /// <summary>
        /// 就是Reference所属的Region分成两部分来看。分别是backTwo和Reference。
        /// 然后再分别和backOne所属的Region来检测连接。
        /// </summary>
        /// <param name="referenceSpan"></param>
        /// <param name="borderDirection"></param>
        /// <returns></returns>
        private bool processOuterCorner( OpenHeightSpan referenceSpan , int borderDirection )
        {
            bool hasMultiRegions = false;
            //比如 borderDirection 是 方向1的话 ，那么 backOne 就是 0 
            /*
             *    r 是 referenceSpan , x 是 border span，1、2就是对应的backOne和backTwo
             *    2 x
             *    1 r
             */

            //backOne 和 backTwo 沿着borderDir方向的两个Span
            int backOneDirection = (borderDirection + 3) & 0x3;
            int antiBorderDirection = (borderDirection + 2) & 0x3; 
            OpenHeightSpan backOne = referenceSpan.getNeighbor(backOneDirection);  
            OpenHeightSpan backTwo = backOne.getNeighbor(borderDirection);
            OpenHeightSpan testSpan; 
            
            if( backOne.regionID() != referenceSpan.regionID() 
                && backTwo.regionID() == referenceSpan.regionID() )
            {
                /*
                 *     Example:
                 * 
                 *     a a x x x a
                 *     a a x x a a
                 *     b b a a a a
                 *     b b a a a a
                 * 
                 */

                /*
                 * 有问题的布局
                 *    a  x
                 *    b  a
                 * 
                 * 需要转换成下面两种
                 *  b x      a x
                 *  b a      b b 
                 * 
                 */

                hasMultiRegions = true;

                /*
                 *    2 x           2 x
                 *    1 r   =>    t 1 r 
                 * 
                 */
                testSpan = backOne.getNeighbor(backOneDirection);
                //检查 backTwo 和 backOne 有多少个连接
                int backTwoConnections = 0; 
                if( testSpan != null 
                    && testSpan.regionID() == backOne.regionID() )
                {
                    // b 是 backOne ， a 是 backTwo 
                    /*
                     *      a x   
                     *    t b a
                     *       
                     *       ||
                     *       
                     *      a x   
                     *    b b a
                     * 
                     */
                    backTwoConnections++;

                    testSpan = testSpan.getNeighbor(borderDirection); 
                    if( testSpan != null 
                        && testSpan.regionID() == backOne.regionID() )
                    {
                        /*
                         * 
                         *    t a x
                         *    b b a
                         *      
                         *      ||
                         *      
                         *    b a x
                         *    b b a
                         * 
                         */
                        backTwoConnections++;  
                    }
                }  // backTwo - backOne

                //检查 reference span 和 backOne 有多少连接
                int referenceConnections = 0;
                testSpan = backOne.getNeighbor(antiBorderDirection);
                if (testSpan != null
                    && testSpan.regionID() == backOne.regionID())
                {
                    /*
                     *      a x
                     *      b a
                     *      t
                     *      
                     *      ||
                     *      
                     *      a x
                     *      b a
                     *      b
                     */

                    referenceConnections++;
                    //TODO  这个方向我修改过,应该是转向才对,感觉原作者是笔误
                    testSpan = testSpan.getNeighbor((borderDirection + 1) & 0x3);
                    if (testSpan != null
                        && testSpan.regionID() == backOne.regionID())
                    {
                        /*
                         *      a x
                         *      b a
                         *      b t
                         *      
                         *      
                         *      ||
                         *      
                         *      a x
                         *      b a
                         *      b b
                         *      
                         */
                        if (testSpan != null
                           && testSpan.regionID() == backOne.regionID())
                        {
                            referenceConnections++;
                        }
                    }
                } // Reference-backOne


                //变成这个拐点
                if (referenceConnections > backTwoConnections)
                {
                    referenceSpan.setRegionID(backOne.regionID());
                }
                else
                {
                    backTwo.setRegionID(backOne.regionID());
                }
            }
            else if( backOne.regionID() == referenceSpan.regionID()
                && backTwo.regionID() == referenceSpan.regionID() )
            {
                //源码本来的注释
                /*
                 * Potential dangerous short wrap.
                 * 
                 *  a x
                 *  a a
                 * 
                 *  Example of actual problem configuration:
                 * 
                 *  b b x x
                 *  b a x x <- Short wrap.
                 *  b a a a
                 * 
                 *  In the above case, the short wrap around the corner of the
                 *  null region has been demonstrated to cause self-intersecting
                 *  polygons during polygon formation.
                 * 
                 *  This algorithm detects whether or not one (and only one)
                 *  of the axis neighbors of the corner should be re-assigned to
                 *  a more appropriate region.
                 * 
                 *  In the above example, the following configuration is more
                 *  appropriate:
                 * 
                 *  b b x x
                 *  b b x x <- Change to this row.
                 *  b a a a
                 */

                /*
                 *    2 x
                 *    1 r 
                 *    
                 *    ||
                 *    
                 *    a x
                 *    a a
                 *    
                 * 
                 */

                //以1为borderDirection的话，那么相对于backTwo来说
                //borderDirection 为 2 ，所以+1 ，CornerDirection 为3，所以+2，
                int selectedRegion = selectedRegionID(backTwo,
                    (borderDirection + 1) & 0x3,
                    (borderDirection + 2) & 0x3
                    );   

                if( backTwo.regionID() == selectedRegion )
                {
                    //backTwo不用改变，尝试改变一下reference
                    selectedRegion = selectedRegionID(referenceSpan,
                        borderDirection,
                        (borderDirection+3) & 0x3); //
                    if( selectedRegion != referenceSpan.regionID() )
                    {
                        referenceSpan.setRegionID(selectedRegion);
                        hasMultiRegions = true; 
                    }
                }
                else
                {
                    backTwo.setRegionID(selectedRegion);
                    hasMultiRegions = true; 
                }

            }
            else
            {
                hasMultiRegions = true; 
            }

            return hasMultiRegions; 
        }

        /// <summary>
        /// 重新设置对应的Region，根据拐点的情况
        /// </summary>
        /// <param name="referenceSpan"></param>
        /// <param name="borderDirection"></param>
        /// <param name="cornerDirection"></param>
        /// <returns></returns>
        private int selectedRegionID(OpenHeightSpan referenceSpan,
            int borderDirection ,
            int cornerDirection )
        {
            referenceSpan.getDetailedRegionMap(ref mwNeighborRegions, 0);

            /*
             * Initial example state:
             * 
             * a - Known region.
             * x - Null region.
             * u - Unknown, not checked yet.
             * 
             *     u u u
             *     u a x
             *     u a a
             */

            int antiBorderDirection = NMGenUtility.AntiDir(borderDirection) ;
            int antiCornerDirection = NMGenUtility.AntiDir(cornerDirection) ; 
            int regionID = mwNeighborRegions[antiBorderDirection];
            if( regionID == referenceSpan.regionID()
                || NULL_REGION == regionID )
            {
                /*
                 * The region away from the border is either a null region
                 * or the same region.  So we keep the current region.
                 * 
                 *     u u u      u u u
                 *     a a x  or  x a x  <-- Potentially bad, but stuck with it.
                 *     u a a      u a a
                 */
                return referenceSpan.regionID(); 
            }

            int potentialRegion = regionID;
            regionID = mwNeighborRegions[antiCornerDirection];
            if( regionID == referenceSpan.regionID()
                || NULL_REGION == regionID )
            {
                /*
                 * The region opposite from the corner direction is
                 * either a null region or the same region.  So we
                 * keep the current region.
                 * 
                 *     u a u      u x u
                 *     b a x  or  b a x
                 *     u a a      u a a
                 */
                return referenceSpan.regionID(); 
            }

            int potentialCount = 0;
            int currentCount = 0; 
            
            for(int i = 0; i < 8; ++i )
            {
                if( mwNeighborRegions[i] == referenceSpan.regionID() )
                {
                    currentCount++; 
                }
                else if( mwNeighborRegions[i] == potentialRegion)
                {
                    potentialCount++; 
                }
            }

            return potentialCount < currentCount
                ? referenceSpan.regionID()
                : potentialRegion; 
        }

        private static int getNullBorderDrection(OpenHeightSpan span)
        {
            for(int dir = 0; dir < 4; ++dir)
            {
                OpenHeightSpan nSpan = span.getNeighbor(dir); 
                if( null == nSpan 
                    || nSpan.regionID() == NULL_REGION )
                {
                    return dir; 
                }
            }
            return -1; 
        }

        private static int getNonNullBorderDrection(OpenHeightSpan span)
        {
            for(int dir = 0; dir < 4; ++dir)
            {
                OpenHeightSpan nSpan = span.getNeighbor(dir); 
                if(nSpan != null 
                    && nSpan.regionID() != NULL_REGION )
                {
                    return dir; 
                }
            }
            return -1; 
        }

    }
}
