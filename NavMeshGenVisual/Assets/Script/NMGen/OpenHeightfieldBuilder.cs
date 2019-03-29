using System;
using System.Collections;
using System.Collections.Generic;
using Utility.Logger;

namespace NMGen
{
    public class OpenHeightfieldBuilder
    {
        private static readonly int NULL_REGION = OpenHeightSpan.NULL_REGION;

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

            int BORDER = 0;

            int NEEDS_INIT = int.MaxValue;

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

                    }
                }
            }

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
