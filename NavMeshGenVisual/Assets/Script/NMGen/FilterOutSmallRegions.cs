using System;
using System.Collections;
using System.Collections.Generic;
using Utility.Logger;

namespace NMGen
{
    public class FilterOutSmallRegions : IOpenHeightFieldAlgorithm
    {
        private static int NULL_REGION = OpenHeightSpan.NULL_REGION;
        private int mMinUnconnectedRegionSize;
        private int mMergeRegionSize; 

        public FilterOutSmallRegions(int minUnconnectedRegionSize ,
            int mergeRegionSize)
        {
            mMinUnconnectedRegionSize = minUnconnectedRegionSize;
            mMergeRegionSize = Math.Max(0, mergeRegionSize); 
        }

        public void apply( OpenHeightfield field )
        {
            if (null == field)
            {
                Logger.LogError("[FilterOutSmallRegions][apply]field null");
                return;
            }

            if( field.regionCount() < 2)
            {
                Logger.LogError("[FilterOutSmallRegions][apply]RegionCnt|{0}",field.regionCount());
                return; 
            }

            //PS：索引即是对应的ID
            Region[] regions = new Region[field.regionCount()]; 
            for(int i = 0; i < field.regionCount();++i)
            {
                regions[i] = new Region(i);  
            }

            #region 收集邻接信息
            OpenHeightfield.OpenHeightFieldIterator iter = field.GetEnumerator(); 
            while( iter.MoveNext() )
            {
                OpenHeightSpan span = iter.Current; 
                if( span.regionID() <= NULL_REGION)
                {
                    continue; 
                }

                //索引即RegionID
                Region region = regions[span.regionID()];
                region.spanCount++;  

                for(OpenHeightSpan nextHigherSpan = span.next(); 
                    nextHigherSpan != null;
                    nextHigherSpan = nextHigherSpan.next()  
                    )
                {
                    int nextHigherSpanRegionID = nextHigherSpan.regionID(); 
                    if (nextHigherSpanRegionID <= NULL_REGION )
                    {
                        continue; 
                    }
                    //因为是同属一个Grid的，所以肯定是重叠的
                    if( !region.overlappingRegions.Contains(nextHigherSpanRegionID))
                    {
                        region.overlappingRegions.Add(nextHigherSpanRegionID); 
                    }
                } //for 

                if( region.connections.Count > 0 )
                {
                    continue; 
                }

                int edgeDirection = getRegionEdgeDirection(span); 
                if( edgeDirection != -1 )
                {
                    findRegionConnections(span,edgeDirection,ref region.connections); 
                }

            } // while 

            #endregion

            #region 清理孤岛Region
            for (int regionID = 1; regionID < field.regionCount(); ++regionID )
            {
                Region region = regions[regionID];  
                if( 0 == region.spanCount )
                {
                    continue; 
                }

                // 有且仅有一个 Null Region 邻居
                if( region.connections.Count == 1 
                    && region.connections[0] == NULL_REGION  )
                {
                    if( region.spanCount < mMinUnconnectedRegionSize )
                    {
                        region.resetWithID(0); 
                    }
                }
            }  //for 

            #endregion

            #region 合并小的Region
            int mergeCount;
            do
            {
                mergeCount = 0; 

                foreach(Region region in regions )
                {
                    if( region.id <= NULL_REGION 
                        || region.spanCount == 0 )
                    {
                        continue; 
                    }

                    if( region.spanCount > mMergeRegionSize )
                    {
                        continue; 
                    }

                    Region targetMergeRegion = null;
                    int smallestSizeFound = int.MaxValue; 

                    foreach( int nRegionID in region.connections)
                    {
                        if( nRegionID <= 0 )
                        {
                            continue; 
                        }

                        Region nRegion = regions[nRegionID]; 
                        if( nRegion.spanCount < smallestSizeFound 
                            && canMerge(region,nRegion) )
                        {
                            targetMergeRegion = nRegion;
                            smallestSizeFound = nRegion.spanCount; 
                        }

                    } // foreach nRegionID 

                    if( targetMergeRegion != null 
                        && mergeRegions(targetMergeRegion,region) )  //为啥是反过来Merge。。。
                    {
                        int oldRegionID = region.id;
                        region.resetWithID(targetMergeRegion.id);  

                        foreach(Region r in regions )
                        {
                            if( r.id <= NULL_REGION )
                            {
                                continue; 
                            }
                            if( r.id == oldRegionID )
                            {
                                r.id = targetMergeRegion.id; 
                            }
                            else
                            {
                                replaceNeighborRegionID(r,oldRegionID,targetMergeRegion.id); 
                            }
                        } // foreach regions

                        mergeCount++; 
                    } // if mergerRegion 

                } // foreach region


            } while (mergeCount > 0);

            #endregion

            #region  re-map 区域ID，保持ID连接
            foreach(Region region in regions)
            {
                if( region.id >= NULL_REGION )
                {
                    region.remap = true; 
                }
            }

            int currRegionID = NULL_REGION ; 
            foreach(Region region in regions )
            {
                if(!region.remap )
                {
                    continue; 
                }
                currRegionID++;
                int oldID = region.id; 

                foreach(Region r in regions )
                {
                    if( r.id == oldID )
                    {
                        r.id = currRegionID;
                        r.remap = false; 
                    }
                }  //foreach 
            } //foreach 

            field.setRegionCount(currRegionID + 1);

            iter.Reset(); 
            while( iter.MoveNext() )
            {
                OpenHeightSpan span = iter.Current; 
                if( NULL_REGION == span.regionID() )
                {
                    continue; 
                }
                else
                {
                    //真正re-map一下
                    span.setRegionID(regions[span.regionID()].id);
                }
               
            }

            #endregion


        }  //apply


        private static void replaceNeighborRegionID(Region region , int oldID, int newID )
        {
            bool connectionsChanged = false; 
            for(int i = 0; i < region.connections.Count; ++i )
            {
                if( region.connections[i] == oldID  )
                {
                    region.connections[i] = newID;
                    connectionsChanged = true; 
                }
            } // for

            for(int i = 0; i < region.overlappingRegions.Count; ++i)
            {
                if( region.overlappingRegions[i] == oldID )
                {
                    region.overlappingRegions[i] = newID; 
                }
            } // for 

            //可能会有重复连接的节点 
            if( connectionsChanged )
            {
                removeAdjacentDuplicateConnections(region);  
            }
        }
        
        private static bool mergeRegions(Region target ,Region candidate)
        {
            int connectionPointOnTargetIndex =
                target.connections.IndexOf(candidate.id); 
            if( connectionPointOnTargetIndex == -1)
            {
                return false; 
            }

            int connectionPointOnCandidateIndex =
                candidate.connections.IndexOf(target.id);  
            if(connectionPointOnCandidateIndex == -1)
            {
                return false;
            }

            List<int> targetConns = new List<int>(target.connections);

            //第一步：重建target的connections 
            /*
             *  Merge connection information.
             * 
             *  Step 1: Rebuild the target connections.
             * 
             *  Start from the point just past where the candidate connection
             *  exists and loop back until just before the candidate connection.
             *  Scenario:
             *      Original target connections are 0, 2, 3, 4, 5
             *      Merging with region 2.
             *  Then:
             *      Rebuild starting at index 2 and stop building at
             *      index 0 to get: 3, 4, 5, 0.
             */

            //剔掉和candate连接的索引
            target.connections.Clear();
            int workingSize = targetConns.Count; 
            for(int i = 0; i < workingSize -1; ++i)  //PS 减了1的
            {
                int newIdx = (connectionPointOnTargetIndex + 1 + i) % workingSize;  
                target.connections.Add( targetConns[newIdx] );  
            }

            /*
             * Step 2: Insert candidate connections into target connections at
             * their mutual connection point.
             * 
             * Same extraction process as for step one, but inserting data
             * into target connection data.
             * 
             * Scenario:
             *         Target connections after step 1 are 3, 4, 5, 0
             *      Candidate connections are 3, 1, 0
             *      Target region id is 1.
             *  Then:
             *      The loop will insert 0, 3 from the candidate at the end of
             *      the target connections.
             *      The final target connections:  3, 4, 5, 0, 0, 3
             *  Note that this process can result in adjacent duplicate
             *  connections which will be fixed later.
             */

            workingSize = candidate.connections.Count; 
            for(int i = 0; i < workingSize - 1 ; ++i)
            {
                //这个newIdx会剔掉属于targetRegion的RegionID索引，然后将candate自己连接的Region加到target里面去
                int newIdx = (connectionPointOnCandidateIndex + 1 + i) % workingSize;
                target.connections.Add( candidate.connections[newIdx] );  
            }

            /*
             * Step 3: Get rid of any adjacent duplicate connections that may
             * have been created.
             */
            removeAdjacentDuplicateConnections(target); 

            //重叠的Region也给target了
            foreach( int i in candidate.overlappingRegions )
            {
                if( !target.overlappingRegions.Contains(i))
                {
                    target.overlappingRegions.Add(i);  
                }
            } // foreach overlappingRegions

            target.spanCount += candidate.spanCount;
            return true;
        }

        /// <summary>
        /// 清理合并后，重复连接的点
        /// </summary>
        /// <param name="region"></param>
        private static void removeAdjacentDuplicateConnections(Region region)
        {
            int iConnection = 0; 
            while( iConnection < region.connections.Count
                && region.connections.Count > 1 )
            {
                int iNextConnection = iConnection + 1; 
                if( iNextConnection >= region.connections.Count )
                {
                    iNextConnection = 0; //回绕
                }
                if( region.connections[iConnection] == region.connections[iNextConnection] )
                {
                    //在遍历中删除真的大丈夫？
                    region.connections.RemoveAt(iNextConnection); 
                }
                else
                {
                    iConnection++;  
                }
            }
        }

        private static bool canMerge(Region regionA ,Region regionB )
        {
            int connectionsAB = 0; 
            foreach( int connectionID in regionA.connections)
            {
                if( connectionID == regionB.id )
                {
                    connectionsAB++; 
                }
            } // foreach connectionID 

            if( connectionsAB != 1 )
            {
                //如果连接点少1，证明不是邻接。
                //如果大于1，则合并后可能是凹边形
                return false;
            }

            //同一个Gird也不行
            if( regionA.overlappingRegions.Contains(regionB.id) )
            {
                return false; 
            }
            if( regionB.overlappingRegions.Contains(regionA.id))
            {
                return false;
            }

            return true;  
        }

        //遍历轮廓，找邻接的Region
        private static void findRegionConnections(OpenHeightSpan startSpan ,
            int startDirection ,
            ref List<int> outConnections )
        {
            OpenHeightSpan span = startSpan;
            int dir = startDirection;
            int lastEdgeRegionID = NULL_REGION;

            OpenHeightSpan nSpan = span.getNeighbor(dir); 
            if( nSpan != null )
            {
                lastEdgeRegionID = nSpan.regionID();  
            }

            //连接的Region，至少存在一个NULL_REGION
            outConnections.Add(lastEdgeRegionID);

            int loopCount = 0; 
            while( ++loopCount < ushort.MaxValue )
            {
                nSpan = span.getNeighbor(dir);
                int currEdgeRegionID = NULL_REGION;   //默认nSpan是null的
                if( null == nSpan
                    || nSpan.regionID() != span.regionID() )
                {
                    if( nSpan != null )
                    {
                        currEdgeRegionID = nSpan.regionID(); 
                    }
                    if( currEdgeRegionID != lastEdgeRegionID )
                    {
                        outConnections.Add(currEdgeRegionID);
                        lastEdgeRegionID = currEdgeRegionID;
                    }

                     //顺时针转向下一个
                    dir = NMGenUtility.ClockwiseRotateDir(dir); 
                }
                else
                {
                    //这个分支代表 Region是相同的
                    span = nSpan;
                    //逆时针转一下
                    dir = NMGenUtility.CClockwiseRotateDir(dir); 
                }

                if( startSpan == span 
                    && startDirection == dir )
                {
                    break; 
                }

            } //while 

            //TODO 为啥会存在首尾相同呢？因为退出条件是原来的那个点么
            int connectionsCnt = outConnections.Count; 
            if ( connectionsCnt > 1 
                && outConnections[0] == outConnections[connectionsCnt - 1])
            {
                outConnections.RemoveAt(connectionsCnt - 1);
            }

        }

        private static int getRegionEdgeDirection(OpenHeightSpan span)
        {
            for(int dir = 0; dir < 4; ++dir )
            {
                OpenHeightSpan nSpan = span.getNeighbor(dir); 
                if( null == nSpan
                    || nSpan.regionID() != span.regionID() )
                {
                    return dir; 
                }
            }
            return -1; 
        }
    }
}
