using System;
using System.Collections;
using System.Collections.Generic;
using Utility.Logger;  


namespace NMGen
{
    //代表AABB中不可行走的区域
    public class SolidHeightfield : BoundeField,IEnumerable
    {
        //记录x-z平面中的Grid对应的y方向上Span列表中，最矮的那一个Span
        private Dictionary<int, HeightSpan> mSpans = new Dictionary<int, HeightSpan>();

        public SolidHeightfield(float cellSize,float cellHeight) : base(cellSize, cellHeight)
        {

        }

        /// <summary>
        /// 获取Grid对应的最矮Span
        /// </summary>
        /// <param name="widthIndex"></param>
        /// <param name="depthIndex"></param>
        /// <returns></returns>
        public HeightSpan getData(int widthIndex,int depthIndex)
        {
            HeightSpan retSpan = null;
            mSpans.TryGetValue(GetGridIndex(widthIndex, depthIndex), out retSpan); 
            return retSpan; 
        }

        /// <summary>
        /// 注意这个添加Span的参数，heightIndexMin和heightIndexMax貌似
        /// 都变成了Span的闭区间
        /// </summary>
        /// <param name="widthIndex"></param>
        /// <param name="depthIndex"></param>
        /// <param name="heightIndexMin"></param>
        /// <param name="heightIndexMax"></param>
        /// <param name="flags"></param>
        /// <returns></returns>
        public bool addData( int widthIndex,
            int depthIndex,
            int heightIndexMin,
            int heightIndexMax,
            int flags
            )
        {
            if( widthIndex < 0 
                || widthIndex >= width() 
                || depthIndex < 0
                || depthIndex >= depth())
            {
                Logger.LogWarning("[SolidHeightfield][addData]width|depth|{0}|{1}|{2}|{3}"); 
                return false; 
            }


            if( heightIndexMin < 0 
                || heightIndexMax < 0 
                || heightIndexMin > heightIndexMax)
            {
                Logger.LogWarning("[SolidHeightfield][addData]heightMin|heightMax|{0}|{1}|{2}|{3}");
                return false; 
            }


            if( mSpans == null  ) 
            {
                return false;
            }

            int gridIndex = GetGridIndex(widthIndex, depthIndex);
            HeightSpan currentSpan; 
            if(!mSpans.TryGetValue(gridIndex,out currentSpan))
            {
                mSpans.Add(gridIndex,
                    new HeightSpan(heightIndexMin, heightIndexMax, flags));

                return true;  
            }

            //各种情况的合并
            HeightSpan previousSpan = null; 
            while( currentSpan != null  )
            {
                //最小比最大还大，不重叠
                //
                /*
                 *  
                 *     -  currentMin
                 *     -  heigtMax
                 *     -
                 *     -  
                 *     -
                 *     -  heightMin 
                 *  
                 */
                if( currentSpan.min() > heightIndexMax + 1 )  //加1的理由看上图，低闭高开
                {
                    HeightSpan newSpan = new HeightSpan(heightIndexMin, heightIndexMax, flags);
                    newSpan.setNext(currentSpan);   //newSpan更加矮

                    //没有更加矮的了
                    if( previousSpan == null )
                    {
                        //更新最矮的Span
                        mSpans[gridIndex] = newSpan; 
                    }
                    else
                    {
                        previousSpan.setNext(newSpan);  
                    }
                    return true; 
                }
                //当前Gird对应的最高，比新的最矮还要矮
                else if( currentSpan.max() < heightIndexMin - 1 )
                {
                    //这一列只有一个Span
                    if( currentSpan.next() == null  )
                    {
                        currentSpan.setNext(
                            new HeightSpan(heightIndexMin, heightIndexMax, flags));
                        return true; 
                    }

                    //这一列还有其它Span，所以要找到插入的点
                    previousSpan = currentSpan;
                    currentSpan = currentSpan.next(); 
                }
                //重叠或者刚好邻接
                else
                {
                    /* Case 1: 
                     * 新的比当前矮，更新最矮点
                     * 最高点相同，合并可走的标志位
                    */
                    if ( heightIndexMin < currentSpan.min() )
                    {
                        currentSpan.setMin(heightIndexMin); 
                    }
                    
                    if( heightIndexMax == currentSpan.max() )
                    {
                        currentSpan.setFlags( currentSpan.flags() | flags );
                        return true;  
                    }

                    /* Case 2:
                     * 最高点没有更新
                     * 
                     */
                    if (currentSpan.max() > heightIndexMax)
                    {
                        return true; 
                    }

                    //最高点更高了
                    HeightSpan nextSpan = currentSpan.next(); 
                    while( true )
                    {
                        //找到第一个刚好比新的最高点高的
                        if( nextSpan == null 
                            || nextSpan.min() > heightIndexMax + 1 )  
                        {
                            currentSpan.setMax(heightIndexMax);
                            currentSpan.setFlags(flags); 

                            if( nextSpan == null )
                            {
                                currentSpan.setNext(null); 
                            }
                            else
                            {
                                currentSpan.setNext(nextSpan);  
                            }

                            return true; 
                        }

                        //刚好邻接，或者旧的高度更加高
                        if( nextSpan.min() == heightIndexMax + 1 
                            || heightIndexMax <= nextSpan.max() )
                        {
                            //吞并旧的
                            currentSpan.setMax(nextSpan.max());
                            currentSpan.setNext(nextSpan.next());
                            currentSpan.setFlags(nextSpan.flags()); 

                            if( heightIndexMax == currentSpan.max() )
                            {
                                currentSpan.setFlags(currentSpan.flags() | flags);
                                return true; 
                            }
                            return true;  
                        }

                        nextSpan = nextSpan.next(); 
                    }
                }
            }

            return false; 
        }


        IEnumerator IEnumerable.GetEnumerator()
        {
            return GetEnumerator();
        }

        public SolidHeightFieldIterator GetEnumerator()
        {
            return new SolidHeightFieldIterator(this); 
        }

        #region iterator define

        public class SolidHeightFieldIterator : IEnumerator
        {
            private int mNextWidth = 0;
            private int mNextDepth = 0;
            private HeightSpan mNext = null;

            private int mLastWidth = 0;
            private int mLastDepth = 0;

            private SolidHeightfield mSoldHeightfield = null;

            object IEnumerator.Current
            {
                get
                {
                    return Current;
                }
            }

            public HeightSpan Current
            {
                get
                {
                    //may be null 
                    return mNext;
                }
            }

            public SolidHeightFieldIterator(SolidHeightfield field)
            {
                if (field == null)
                {
                    Logger.LogError("[SolidHeightFieldIterator][ctor]filed Empty");
                    return;
                }

                mSoldHeightfield = field;

                Reset();
            }

            public int depthIndex()
            {
                return mLastDepth;
            }

            public int widthIndex()
            {
                return mLastWidth;
            }


            public void Reset()
            {
                mNextWidth = 0;
                mNextDepth = 0;
                mNext = null;
                mLastWidth = 0;
                mLastDepth = 0;
            }

            public bool MoveNext()
            {
                if (mNext != null)
                {
                    //下一个的下一个
                    if (mNext.next() != null)
                    {
                        mNext = mNext.next();
                        return true;
                    }
                    else
                    {
                        //当前列已经没有下一个，移动到下一个Grid
                        mNextWidth++;  //寻找x方向的下一个
                    }
                }

                if (mSoldHeightfield != null)
                {
                    for (int depthIndex = mNextDepth; depthIndex < mSoldHeightfield.depth(); depthIndex++)
                    {
                        for (int widthIndex = mNextWidth; widthIndex < mSoldHeightfield.width(); widthIndex++)
                        {
                            int gridIndex = mSoldHeightfield.GetGridIndex(widthIndex, depthIndex);
                            HeightSpan span = mSoldHeightfield.mSpans[gridIndex];

                            if (span != null)
                            {
                                mNext = span;
                                mNextWidth = widthIndex;
                                mNextDepth = depthIndex;
                                return true;
                            }

                        }
                        mNextWidth = 0;
                    }
                }

                mNext = null;
                mNextDepth = -1;
                mNextWidth = -1;

                return false;
            }
        }

        #endregion

    }
}
