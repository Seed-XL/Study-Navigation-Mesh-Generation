using System;
using System.Collections;
using System.Collections.Generic;


namespace NMGen
{
    //代表AABB中不可行走的区域
    public class SolidHeightfield : BoundeField,IEnumerable
    {
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

            public SolidHeightFieldIterator( SolidHeightfield field )
            {
                if( field == null )
                {
                    throw new Exception("[SolidHeightfield][SolidHeightFieldIterator]field Empty"); 
                }

                mSoldHeightfield = field;  

                MoveNext();      
            } 
            
            public int depthIndex()
            {
                return mLastDepth; 
            }

            public bool hasNext()
            {
                return mNext != null; 
            }

            public HeightSpan next()
            {
                if( mNext == null )
                {
                    throw new Exception("[SolidHeightfield][SolidHeightFieldIterator][next]Empty"); 
                }

                HeightSpan next = mNext;
                mLastWidth = mNextWidth;
                mLastDepth = mNextDepth;
                MoveNext();

                return next;
            }

         
            public void Reset()
            {
                mNextWidth = 0;
                mNextDepth = 0;
                mNext = null;
                mLastWidth = 0;
                mLastDepth = 0;
                MoveNext(); 
            }

            public bool MoveNext()
            {
                if( mNext != null )
                {
                    //下一个的下一个
                    if( mNext.next() != null )
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

                if( mSoldHeightfield != null )
                {
                    for (int depthIndex = mNextDepth; depthIndex < mSoldHeightfield.depth(); depthIndex++)
                    {
                        for( int widthIndex = mNextWidth; widthIndex < mSoldHeightfield.width(); widthIndex++ )
                        {
                            int gridIndex = mSoldHeightfield.GetGridIndex(widthIndex, depthIndex);
                            HeightSpan span = mSoldHeightfield.mSpans[gridIndex]; 

                            if( span != null )
                            {
                                mNext = span;
                                mNextWidth = widthIndex;
                                mNextDepth = depthIndex;
                                return true ; 
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

        //记录每个Grid对应的列中，最矮的那一个Span
        private Dictionary<int, HeightSpan> mSpans = new Dictionary<int, HeightSpan>();

        public SolidHeightfield(float cellSize,float cellHeight) : base(cellSize, cellHeight)
        {

        }

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
                return false; 
            }


            if( heightIndexMin < 0 
                || heightIndexMax < 0 
                || heightIndexMin > heightIndexMax)
            {
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
                if( currentSpan.min() > heightIndexMax + 1 )
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
            return (IEnumerator)GetEnumerator();
        }

        public SolidHeightFieldIterator GetEnumerator()
        {
            return new SolidHeightFieldIterator(this); 
        }

    }
}
