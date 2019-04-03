using System;
using System.Collections;
using System.Collections.Generic;
using Utility.Logger;

namespace NMGen
{
    public class OpenHeightfieldBuilder
    {
        private static readonly int NULL_REGION = OpenHeightSpan.NULL_REGION;

        private static readonly int BORDER = 0;

        private static readonly int NEEDS_INIT = int.MaxValue;

        private int mMinTraversableHeight;
        private int mMaxTraversableStep;
        private int mSmoothingThreshold;
        private int mTraversableAreaBorderSize;
        private int mFilterFlags;
        private bool mUseConservativeExpansion; 

        private List<IOpenHeightFieldAlgorithm> mRegionAlgorithms = new List<IOpenHeightFieldAlgorithm>() ; 

        public OpenHeightfieldBuilder(int minTraversableHeight ,
            int maxTraversablestep ,
            int traversableAreaBorderSize,
            int smoothingThreshold,
            int filterFlags,
            bool useConservativeExpansion ,
            List<IOpenHeightFieldAlgorithm> regionAlgorithms)
        {
            mMaxTraversableStep = Math.Max(0, maxTraversablestep);
            mMinTraversableHeight = Math.Min(1, mMinTraversableHeight);
            mTraversableAreaBorderSize = Math.Max(0, traversableAreaBorderSize);
            mFilterFlags = filterFlags;
            mSmoothingThreshold = Math.Min(4, Math.Max(0, smoothingThreshold));
            mUseConservativeExpansion = useConservativeExpansion;
            if( regionAlgorithms != null )
            {
                mRegionAlgorithms.AddRange(regionAlgorithms);
            }
        }

        public OpenHeightfield build(SolidHeightfield sourceField ,
            bool performFullGeneration )
        {
            if( sourceField == null )
            {
                Logger.LogError("[OpenHeightfieldBuilder][build] sourceField null "); 
                return null; 
            }

            OpenHeightfield result = new OpenHeightfield(
                sourceField.boundsMin(),
                sourceField.boundsMax(),
                sourceField.cellSize(),
                sourceField.cellHeight()
                );
            
            for(int depthIndex = 0; depthIndex < sourceField.depth(); depthIndex++)
            {
                for(int widthIndex = 0; widthIndex < sourceField.width(); widthIndex++)
                {
                    OpenHeightSpan baseSpan = null;
                    OpenHeightSpan previousSpan = null;

                    for (HeightSpan span = sourceField.getData(widthIndex, depthIndex);
                         span != null;
                         span = span.next()
                        )
                    {
                        if ( span.flags() != mFilterFlags )
                        {
                            continue; 
                        }

                        //当前Solid Span的max对应的是对应OpenSpan的floor
                        int floor = span.max();  
                        //下一个Next Solid Span的min对应当前OpenSpan的Ceil。
                        int ceiling = (span.next() != null
                            ? span.next().min()
                            : int.MaxValue) ;

                        //对应的Open Span
                        OpenHeightSpan oSpan = new OpenHeightSpan(floor,
                            (ceiling - floor )
                            ); 
                        if( baseSpan == null  )
                        {
                            baseSpan = oSpan; 
                        }
                        if( previousSpan != null )
                        {
                            previousSpan.setNext(oSpan); 
                        }
                        previousSpan = oSpan;
                        result.incrementSpanCount(); 
                    } //for 
                    if( baseSpan != null )
                    {
                        result.addData(widthIndex, depthIndex, baseSpan); 
                    }
                }//for
            } //for

            if( performFullGeneration )
            {
                generateNeighborLinks(result);
                generateDistanceField(result);
                blurDistanceField(result);
                generateRegions(result);  
            }

            return result; 
        }

        /// <summary>
        /// TODO 解决那两个地方为啥是加2
        /// </summary>
        /// <param name="inoutSpans"></param>
        /// <param name="maxIterations"></param>
        private void expandRegions(List<OpenHeightSpan> inoutSpans ,int maxIterations)
        {
            if( inoutSpans.Count == 0 )
            {
                return; 
            }

            int iterCount = 0; 
            while(true)
            {
                int skipped = 0; 

                for(int iSpan = 0; iSpan < inoutSpans.Count; ++iSpan)
                {
                    OpenHeightSpan span = inoutSpans[iSpan]; 
                    if( null == span )
                    {
                        skipped++;
                        continue; 
                    }

                    int spanRegion = NULL_REGION;
                    int regionCenterDist = int.MaxValue; 

                    //看看邻居的RegionID情况
                    for(int dir = 0; dir < 4; ++dir)
                    {
                        OpenHeightSpan nSpan = span.getNeighbor(dir); 
                        if( null == nSpan )
                        {
                            continue; 
                        }

                        if( nSpan.regionID() > NULL_REGION  )
                        {
                            //TODO 为啥是+2
                            if( nSpan.distanceToRegionCore() + 2 < regionCenterDist  )
                            {
                                int sameRegionCount = 0; 
                                if( mUseConservativeExpansion )
                                {
                                    for( int ndir = 0; ndir < 4; ++ndir )
                                    {
                                        OpenHeightSpan nnSpan = nSpan.getNeighbor(ndir); 
                                        if( null == nnSpan )
                                        {
                                            continue; 
                                        }

                                        if( nnSpan.regionID() == nSpan.regionID() )
                                        {
                                            sameRegionCount++; 
                                        }
                                    }     
                                }

                                //如果轴对齐邻居的邻居的Region相同，也就是多于1个体素就可以了
                                if( !mUseConservativeExpansion
                                    || sameRegionCount > 1)
                                {
                                    //TODO 为啥要加2
                                    spanRegion = nSpan.regionID();
                                    regionCenterDist = nSpan.distanceToRegionCore() + 2; 
                                }
                            }
                        }
                    } // for 

                    if( spanRegion != NULL_REGION )
                    {
                        inoutSpans[iSpan] = null;
                        span.setRegionID(spanRegion); 
                    }
                    else
                    {
                        skipped++; 
                    }
                } // for 

                if( skipped == inoutSpans.Count )
                {
                    break; 
                }

                if( maxIterations != -1)
                {
                    iterCount++; 
                    if( iterCount > maxIterations )
                    {
                        break; 
                    }
                }
            }
        }

        public void generateRegions( OpenHeightfield field )
        {
            if (null == field)
            {
                return;
            }

            //这个距离，控制生成的网络有多贴近实际的模型
            int minDist = mTraversableAreaBorderSize + field.minBorderDistance();
            int expandIterations = 4 + (mTraversableAreaBorderSize * 2); //TODO emmm

            //排除奇数
            int dist = (field.maxBorderDistance() - 1) & ~1 ;

            List<OpenHeightSpan> floodedSpans = new List<OpenHeightSpan>(1024);
            Stack<OpenHeightSpan> workingStack = new Stack<OpenHeightSpan>(1024);

            OpenHeightfield.OpenHeightFieldIterator iter = field.GetEnumerator();

            int nextRegionID = 1; 
            while( dist > minDist )  //高于这个距离的体素都得生成Regions
            {
                iter.Reset();
                floodedSpans.Clear(); 

                while( iter.MoveNext() )
                {
                    OpenHeightSpan span = iter.Current; 
                    if( span.regionID() == NULL_REGION
                        && span.distanceToBorder() >= dist )
                    {
                        floodedSpans.Add(span); 
                    }
                }

                if( nextRegionID > 1 )
                {
                    //大于1表示已经至少存在1个region，先去尝试一下合并
                    if( dist > 0 )
                    {
                        expandRegions(floodedSpans, expandIterations);     
                    }
                    else
                    {
                        expandRegions(floodedSpans, -1); 
                    }
                }

                //剩下的可能要生成新的Region
                foreach( OpenHeightSpan span in floodedSpans )
                {
                    if( null == span 
                        || span.regionID() != NULL_REGION )
                    {
                        continue; 
                    }

                    //TODO ????
                    int fillTo = Math.Max(dist - 2, minDist); 
                    if( floodNewRegion(span,fillTo,nextRegionID,workingStack ))
                    {
                        nextRegionID++; 
                    }
                }

                //更新深度
                dist = Math.Max(dist - 2, 0); 
            }  //while

            //最后一篇循环
            iter.Reset();
            floodedSpans.Clear(); 
            while( iter.MoveNext() )
            {
                OpenHeightSpan span = iter.Current; 
                if( span.distanceToBorder() >= minDist
                    && span.regionID() == NULL_REGION )
                {
                    floodedSpans.Add(span); 
                }
            }

            if( minDist > 0 )
            {
                expandRegions(floodedSpans, expandIterations * 8); 
            }
            else
            {
                expandRegions(floodedSpans, -1); 
            }
            field.setRegionCount(nextRegionID);

            foreach( IOpenHeightFieldAlgorithm algorithm in mRegionAlgorithms )
            {
                algorithm.apply(field);  
            }
        }

        private static bool floodNewRegion( OpenHeightSpan rootSpan,
            int fillToDist ,
            int regionID,
            Stack<OpenHeightSpan> workingStack )
        {
            workingStack.Clear();

            List<OpenHeightSpan> workingList = new List<OpenHeightSpan>();

            workingStack.Push(rootSpan);
            workingList.Add(rootSpan);
            rootSpan.setRegionID(regionID);
            rootSpan.setDistanceToRegionCore(0);

            int regionSize = 0; 

            //广度优先搜索
            while( workingStack.Count > 0 )
            {
                OpenHeightSpan span = workingStack.Pop();

                bool isOnRegionBorder = false; 
                for( int dir = 0; dir < 4; ++dir )
                {
                    OpenHeightSpan nSpan = span.getNeighbor(dir); 
                    if( null == nSpan )
                    {
                        continue; 
                    }

                    if( nSpan.regionID() != NULL_REGION
                        && nSpan.regionID() != regionID )
                    {
                        isOnRegionBorder = true;
                        break;
                    }

                    //对角线邻居
                    nSpan = nSpan.getNeighbor((dir + 1) & 0x3); 
                    if( nSpan != null 
                        && nSpan.regionID() != NULL_REGION 
                        && nSpan.regionID() != regionID )
                    {
                        isOnRegionBorder = true;
                        break; 
                    }

                }  //for 

                if( isOnRegionBorder )
                {
                    span.setRegionID(NULL_REGION);
                    continue; 
                }

                //到这里，表明要新增加一个Region了
                regionSize++; 

                for( int dir = 0;  dir < 4;++dir )
                {
                    OpenHeightSpan nSpan = span.getNeighbor(dir); 
                    if( nSpan != null 
                        && nSpan.distanceToBorder() >= fillToDist  //
                        && nSpan.regionID() == NULL_REGION )
                    {
                        nSpan.setRegionID(regionID);
                        nSpan.setDistanceToRegionCore(0);
                        workingStack.Push(nSpan);
                        workingList.Add(nSpan);   //这个是要被返回的
                    }
                }

            } // while


            return regionSize > 0; 
        }


        public void blurDistanceField(OpenHeightfield field)
        {
            if( null == field )
            {
                return; 
            }

            if( mSmoothingThreshold <= 0 )
            {
                return; 
            }

            //Span => Distance
            Dictionary<OpenHeightSpan, int> blurResults = new Dictionary<OpenHeightSpan, int>();

            OpenHeightfield.OpenHeightFieldIterator iter = field.GetEnumerator(); 
            while( iter.MoveNext() )
            {
                OpenHeightSpan span = iter.Current;
                int origDist = span.distanceToBorder(); 
                if( origDist <= mSmoothingThreshold )  //这里也会Border引进去
                {
                    blurResults.Add(span, mSmoothingThreshold);
                    continue; 
                }

                int workingDist = origDist; 
                //将9个格子的距离加起来
                for(int dir = 0; dir < 4; ++dir)
                {
                    OpenHeightSpan nSpan = span.getNeighbor(dir); 
                    if( null == nSpan )
                    {
                        //这是不知道能不能看作自己占的比较增加了？但是为啥是 * 2 呢？
                        workingDist += origDist * 2; 
                    }
                    else
                    {
                        workingDist += nSpan.distanceToBorder();
                        nSpan = nSpan.getNeighbor( (dir+1) & 0x3 );   //对角线的
                        if( null == nSpan )
                        {
                            workingDist += origDist; 
                        }
                        else
                        {
                            workingDist += nSpan.distanceToBorder();  
                        }
                    }
                }  //for 

                if( blurResults.ContainsKey(span) )
                {
                    //除以9是平均呢，但是加上五就真的是不知道为什么了
                    blurResults[span] = (workingDist + 5) / 9; 
                }

            }  //while

            //更新一下距离值 
            foreach( var blurIter in blurResults )
            {
                blurIter.Key.setDistanceToBorder(blurIter.Value);   
            }
        }

        /* 参考 ：http://www.cnblogs.com/wantnon/p/4947067.html 
        *       http://fab.cba.mit.edu/classes/S62.12/docs/Meijster_distance.pdf
        * 具体算法名：Saito算法
        * 因为初始化值都是未知的，通过一个大约估计的初始化，来重复来回算一个大致准确的距离
        * 值 ？
        */
        public void generateDistanceField(OpenHeightfield field)
        {
            if (field == null)
            {
                Logger.LogError("[OpenHeightfieldBuilder][generateNeighborLinks]field Empty");
                return;
            }

          

            /*
             * 先将OpenHeightField的Span数据转换成0和1的二值图。如果是边缘Span
             * 那么就是0，否则就是NEEDS_INIT。
             */


            OpenHeightfield.OpenHeightFieldIterator iter = field.GetEnumerator(); 
            while( iter.MoveNext() )
            {
                OpenHeightSpan span = iter.Current;
                bool isBorder = false; 

                for(int dir = 0; dir < 4; ++dir)
                {
                    OpenHeightSpan nSpan = span.getNeighbor(dir); 
                    if( null == nSpan
                        || nSpan.getNeighbor(dir == 3 ? 0 : dir + 1) == null) 
                    {
                        //如果8个邻居有任何一个缺失的话，那么就是边界Border
                        isBorder = true;
                        break;
                    }
                }

                if( isBorder )
                {
                    //自己就是边界Border
                    span.setDistanceToBorder(BORDER); 
                }
                else
                {
                    //需要再次计算
                    span.setDistanceToBorder(NEEDS_INIT);
                }
            } //while


            /*
             * 逆时针访问？
             *     
             *    (-1,1)  (0,1)  (1,1)
             *    (-1,0)    x    (1,0)
             *    (-1,-1) (0,-1) (1,-1)
             */

            //Pass 1
            //顺序访问 (-1,0) (-1,-1) (0,-1) (1,-1)
            iter.Reset(); 
            while( iter.MoveNext() )
            {
                OpenHeightSpan span = iter.Current;
                int selfDist = span.distanceToBorder(); 

                if( selfDist == BORDER)
                {
                    continue; 
                }

                //(-1,0)
                OpenHeightSpan nSpan = span.getNeighbor(0);
                selfDist = calcMiniDistanceToBorder(selfDist, nSpan, true);

                //(-1,-1),左下角的领域
                nSpan = nSpan.getNeighbor(3); //领域0的领域3，也就是原Span的左下角
                selfDist = calcMiniDistanceToBorder(selfDist, nSpan, false);

                //(0,-1)
                nSpan = span.getNeighbor(3);
                selfDist = calcMiniDistanceToBorder(selfDist, nSpan, true);

                //(1,-1) 
                nSpan = nSpan.getNeighbor(2);
                selfDist = calcMiniDistanceToBorder(selfDist, nSpan, true);
                
                span.setDistanceToBorder(selfDist); 

            } // while


            //Pass 2
            //顺序访问 (1,0) (1,1) (0,1) (-1,1)
            //注意这个是反向遍历
            iter.ReverseReset(); 
            while( iter.MoveNext() )
            {
                OpenHeightSpan span = iter.Current;
                int selfDist = span.distanceToBorder();
                if( selfDist == BORDER )
                {
                    continue; 
                }

                /* 
                 * 因为经过Pass1之后 ，所有的Span要么就是Border
                 * 要么就是有个大概值的，不会等于NEED_INIT的。
                 * 所以直接按照按照上面的流程跑
                 */

                //(1,0)
                OpenHeightSpan nSpan = span.getNeighbor(2);
                selfDist = calcMiniDistanceToBorder(selfDist, nSpan, true);

                //(1,1)
                nSpan = nSpan.getNeighbor(1);
                selfDist = calcMiniDistanceToBorder(selfDist, nSpan, false);

                //(0,1)
                nSpan = span.getNeighbor(1);
                selfDist = calcMiniDistanceToBorder(selfDist, nSpan, true);

                //(-1,1)
                nSpan = nSpan.getNeighbor(0);
                selfDist = calcMiniDistanceToBorder(selfDist, nSpan, false);

                span.setDistanceToBorder(selfDist); 
            }

            field.clearBorderDistanceBounds(); 
        }

        /// <summary>
        /// 必须nSpan是存在的，如果不存在的话，span2Border的值自然就0了
        /// </summary>
        /// <param name="span2Border"></param>
        /// <param name="nSpan"></param>
        /// <param name="isAxisNeighbor"></param>
        /// <returns></returns>
        private int calcMiniDistanceToBorder( int span2Border, OpenHeightSpan nSpan , bool isAxisNeighbor)
        {
            if( nSpan != null  )
            {
                int nDist = nSpan.distanceToBorder(); 
                if( nDist == NEEDS_INIT )
                {
                    nDist = isAxisNeighbor ? 1 : 2; 
                }
                else
                {
                    nDist = isAxisNeighbor ? nDist + 2 : nDist + 3; 
                }
                span2Border = Math.Min(span2Border, nDist); 
            }
            else
            {
                Logger.LogWarning("[OpenHeightfieldBuilder][calcMiniDistanceToBorder]nSpan null"); 
            }

            return span2Border; 
        }

        public void generateNeighborLinks(OpenHeightfield field)
        {
            if( field == null )
            {
                Logger.LogError("[OpenHeightfieldBuilder][generateNeighborLinks]field Empty"); 
                return; 
            }

            OpenHeightfield.OpenHeightFieldIterator iter = field.GetEnumerator(); 
            while( iter.MoveNext() )
            {
                OpenHeightSpan span = iter.Current; 
                for(int dir = 0; dir < 4; ++dir)
                {
                    //邻居的GirdIndex
                    int nWidthIndex = (iter.widthIndex() + BoundeField.getDirOffsetWidth(dir));
                    int nDepthIndex = (iter.depthIndex() + BoundeField.getDirOffsetDepth(dir)); 
                    
                    for(OpenHeightSpan nSpan = field.getData(nWidthIndex,nDepthIndex);
                        nSpan != null;
                        nSpan = nSpan.next())
                    {
                        int maxFloor = Math.Max(span.floor(), nSpan.floor());
                        int minCeling = Math.Min(span.ceiling(), nSpan.ceiling()); 

                        
                        if( (minCeling - maxFloor) >= mMinTraversableHeight   //邻居之间的通道足够高，可以通过 
                            && Math.Abs(nSpan.floor() - span.floor()) <= mMaxTraversableStep )  //两邻居之间的落差足够小
                        {
                            span.setNeighbor(dir, nSpan);
                            break; 
                        }
                    }

                }
            }
        }

    }
}
