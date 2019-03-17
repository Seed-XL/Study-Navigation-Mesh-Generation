using System;
using System.Collections;
using System.Collections.Generic;


namespace NMGen
{
    //代表AABB中可以行走的区域
    public class OpenHeightfield : BoundeField, IEnumerable
    {
        #region iterator define

        public class OpenHeightFieldIterator : IEnumerator
        {
            private int mNextWidth;
            private int mNextDepth;

            private OpenHeightSpan mNext;

            private int mLastWidth;
            private int mLastDepth;

            private OpenHeightfield mOpenHeightfield = null;


            private OpenHeightFieldIterator( OpenHeightfield field )
            {
                if (field == null)
                {
                    throw new Exception("[SolidHeightfield][OpenHeightFieldIterator]field Empty");
                }

                mOpenHeightfield = field;

                Reset(); 
            }



            public void MoveNext()
            {
                if( mNext != null  )
                {
                    if( mNext.next() != null  )
                    {
                        mNext = mNext.next();
                        return; 
                    }
                    else
                    {
                        mNextWidth++;
                    }
                }

                for( int depthIndex = mNextDepth;
                    depthIndex < mOpenHeightfield.depth();
                    ++depthIndex)
                {
                    for(int widthIndex = mNextWidth; 
                        widthIndex < mOpenHeightfield.width(); 
                        widthIndex++)
                    {
                        //OpenHeightSpan span = mOpenHeightfield.
                    }
                }
               
            }

            public OpenHeightSpan next()
            {
                if( mNext == null )
                {
                    throw new Exception("[OpenHeightfield][OpenHeightFieldIterator][next]Empty");
                }

                OpenHeightSpan next = mNext;
                mLastWidth = mNextWidth;
                mLastDepth = mNextDepth;

                //moveToNext();
                return next; 
            }

            public bool hasNext()
            {
                return mNext != null; 
            }
      

            public int depthIndex()
            {
                return mLastDepth; 
            }

            public void Reset()
            {
                mNextWidth = 0;
                mNextDepth = 0;
                mNext = null;
                mLastWidth = 0;
                mLastDepth = 0;   
            }

        }


        #endregion

    }
}
