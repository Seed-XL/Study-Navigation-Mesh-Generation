using System;
using System.Collections;
using System.Collections.Generic;
using Utility.Logger; 

namespace NMGen
{
    //代表AABB中可以行走的区域
    public class OpenHeightfield : BoundeField, IEnumerable
    {
        #region iterator define

        public class OpenHeightFieldIterator : IEnumerator , IHeightFieldIterInterface
        {
            private int mNextWidth;
            private int mNextDepth;

            private OpenHeightSpan mNext;

            private OpenHeightfield mOpenHeightfield = null;

            private bool mIsReverseIter = false; 

            public OpenHeightFieldIterator( OpenHeightfield field ,bool isInitReverseIter )
            {
                if (field == null)
                {
                    Logger.LogError("[SolidHeightfield][OpenHeightFieldIterator]field Empty");
                    return; 
                }

                mOpenHeightfield = field;

                if( isInitReverseIter )
                {
                    ReverseReset();
                }
                else
                {
                    Reset();
                }
            }

            object IEnumerator.Current
            {
                get
                {
                    return Current;
                }
            }

            public OpenHeightSpan Current
            {
                get
                {
                    //may be null 
                    return mNext;
                }
            }

            public int depthIndex()
            {
                return mNextDepth;
            }


            public int widthIndex()
            {
                return mNextWidth; 
            }


            public bool MoveNext()
            {
                if( mOpenHeightfield == null  )
                {
                    Logger.LogError("[OpenHeightfield][OpenHeightFieldIterator][MoveNext]Null");
                    return false ; 
                }

                //公有逻辑
                if (mNext != null)
                {
                    if (mNext.next() != null)
                    {
                        mNext = mNext.next();
                        return true;
                    }
                    else
                    {
                        if( mIsReverseIter )
                        {
                            mNextWidth--;
                        }
                        else
                        {
                            mNextWidth++; 
                        }
                       
                    }
                }

                if ( mIsReverseIter )
                {
                    #region 反向遍历 
                    for (int depthIndex = mNextDepth ; depthIndex >= 0 ; --depthIndex)
                    {
                        for (int widthIndex = mNextWidth ; widthIndex >= 0 ; --widthIndex )
                        {
                            OpenHeightSpan span = mOpenHeightfield.getData(widthIndex, depthIndex);
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

                    mNext = null;
                    mNextDepth = -1;
                    mNextWidth = -1;

                    return false;

                    #endregion 
                }
                else
                {
                    #region 正向遍历 
                    for (int depthIndex = mNextDepth;depthIndex < mOpenHeightfield.depth();++depthIndex)
                    {
                        for (int widthIndex = mNextWidth;widthIndex < mOpenHeightfield.width();widthIndex++)
                        {
                            OpenHeightSpan span = mOpenHeightfield.getData(widthIndex, depthIndex);
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

                    mNext = null;
                    mNextDepth = -1;
                    mNextWidth = -1;

                    return false;

                    #endregion 
                } // if-else
            } // MoveNext

          
            public void Reset()
            {
                mNextWidth = 0;
                mNextDepth = 0;
                mNext = null;
                mIsReverseIter = false; 
            }

            public void ReverseReset()
            {
                if(mOpenHeightfield!= null)
                {
                    mNextWidth = mOpenHeightfield.width() ;
                    mNextDepth = mOpenHeightfield.depth() ;
                }

                mNext = null;
                mIsReverseIter = true;
            }

        }


        #endregion

        IEnumerator IEnumerable.GetEnumerator()
        {
            return GetEnumerator();
        }

        public OpenHeightFieldIterator GetEnumerator()
        {
            return new OpenHeightFieldIterator(this,false);
        }

        public OpenHeightFieldIterator GetReverseEnumerator()
        {
            return new OpenHeightFieldIterator(this, true); 
        }

        private static readonly int UNKNOWN = -1;

        private int mSpanCount = 0;
        private int mRegionCount = 0;

        private int mMaxBorderDistance = UNKNOWN;
        private int mMinBorderDistance = UNKNOWN;


        //记录x-z平面中的Grid对应的y方向上Span列表中，最矮的那一个Span
        private Dictionary<int, OpenHeightSpan> mSpans = new Dictionary<int, OpenHeightSpan>();


        public OpenHeightfield(float[]  gridBoundsMin,
            float[] gridBoundsMax ,
            float cellSize,
            float cellHeight):base(gridBoundsMin,gridBoundsMax,cellSize,cellHeight)
        {
   
        }

        public bool addData(int widthIndex,int depthIndex,OpenHeightSpan span)
        {
            if (widthIndex < 0
                  || widthIndex >= width()
                  || depthIndex < 0
                  || depthIndex >= depth() )
            {
                Logger.LogWarning("[OpenHeightfield][addData]width|depth|{0}|{1}|{2}|{3}",widthIndex,depthIndex,width(),depth());
                return false;
            }

            if ( mSpans == null || span == null )
            {
                Logger.LogWarning("[OpenHeightfield][addData]mSpan or Span null|{0}|{1}|{2}|{3}", widthIndex, depthIndex, width(), depth());
                return false;
            }

            int gridIndex = GetGridIndex(widthIndex, depthIndex);
            OpenHeightSpan currentSpan;
            if (!mSpans.TryGetValue(gridIndex, out currentSpan))
            {
                mSpans.Add(gridIndex,span);
                return true;
            }
            else
            {
                Logger.LogWarning("[OpenHeightfield][addData]Span already in|{0}|{1}|{2}|{3}", widthIndex, depthIndex, width(), depth());
                return false;
            }
        }

        public void clearBorderDistanceBounds()
        {
            mMaxBorderDistance = UNKNOWN;
            mMinBorderDistance = UNKNOWN; 
        }

        public OpenHeightSpan getData(int widthIndex,int depthIndex)
        {
            OpenHeightSpan retSpan = null;
            if( mSpans != null )
            {
                mSpans.TryGetValue(GetGridIndex(widthIndex, depthIndex), out retSpan); 
            }
            return retSpan; 
        }

        public int incrementSpanCount()
        {
            return ++mSpanCount;
        }
      

        public int maxBorderDistance()
        {
            if(mMaxBorderDistance == UNKNOWN)
            {
                calcBorderDistanceBounds(); 
            }
            return mMaxBorderDistance; 
        }

        public void printDistanceField()
        {
            Logger.Log("[OpenHeightfield][printDistanceField]Log Start");
            Logger.Log("[OpenHeightfield][printDistanceField]Distance Field Spans|{0}",mSpanCount);
           
            int depth = -1;
            Logger.Log("\t"); 
            for (int width = 0; width < this.width(); ++width)
            {
                Logger.Log("{0}\t", width);
            }

            OpenHeightFieldIterator iter = GetEnumerator();
            while( iter.MoveNext() )
            {
                OpenHeightSpan span = iter.Current; 
                if( iter.depthIndex() != depth )
                {
                    Logger.LogWarning("\n{0}\t", ++depth);
                }
                Logger.Log("{0}\t", span.distanceToBorder());
            }

            Logger.Log("[OpenHeightfield][printDistanceField]Log End");  
        }

        public int regionCount()
        {
            return mRegionCount; 
        }

        public void setRegionCount(int value)
        {
            mRegionCount = value; 
        }

        public int spanCount()
        {
            return mSpanCount; 
        }

        private void calcBorderDistanceBounds()
        {
            if( 0 == mSpanCount )
            {
                return; 
            }

            mMinBorderDistance = int.MaxValue;
            mMaxBorderDistance = UNKNOWN;

            OpenHeightFieldIterator iter = GetEnumerator(); 
            while( iter.MoveNext()  )
            {
                OpenHeightSpan span = iter.Current;
                mMinBorderDistance = Math.Min(mMinBorderDistance, span.distanceToBorder());
                mMaxBorderDistance = Math.Max(mMaxBorderDistance, span.distanceToBorder()); 
            }

            if( mMinBorderDistance == int.MaxValue )
            {
                mMinBorderDistance = UNKNOWN;  
            }
        }
    }
}
