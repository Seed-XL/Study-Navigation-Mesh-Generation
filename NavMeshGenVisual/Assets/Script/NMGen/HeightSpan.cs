using System;
using Utility.Logger; 

namespace NMGen
{
    /// <summary>
    /// 代表在y轴方向上某一列连续的体素，准备来说是给封闭的高度域使用的
    /// </summary>
    public class HeightSpan
    {
        //以体素坐标系为参考。
        //分别代表Span里，在y方向上，最底那个Span属于的索引，最高那个Span属于的索引。
        //因为需要向上取整，所以左右都是闭区间，也就是[mMinmum,mMaxmum]
        private int mMinmum;   
        private int mMaxmum;   
        private int mFlags = 0; 
        private HeightSpan mNext = null;   //同一列的，下一个更高的Span


        public HeightSpan(int min,int max ,int flags)
        {
            if( min > max )
            {
                Logger.LogError("[HeightSpan][ctor]Minmum is greater than or equal to the maximum|{0}|{1}",min,max); 
                return;  
            }

            mMinmum = min;
            mMaxmum = max;
            mFlags = flags; 
        }

        public int flags()
        {
            return mFlags; 
        }

        public int max()
        {
            return mMaxmum; 
        }

        public int min()
        {
            return mMinmum; 
        }


        public HeightSpan next()
        {
            return mNext; 
        }


        public void setFlags(int value)
        {
            mFlags = value; 
        }

        public void setMax(int value)
        {
            if( value <= mMinmum )
            {
                mMaxmum = mMinmum + 1; 
            }
            else
            {
                mMaxmum = value; 
            }
        }

        public void setMin(int value)
        {
            if( value >= mMaxmum )
            {
                mMinmum = mMaxmum - 1; 
            }
            else
            {
                mMinmum = value; 
            }
        }


        public void setNext( HeightSpan value )
        {
            mNext = value; 
        }
        
        public string toString()
        {
            return string.Format("|{0}->{1}|Flags|{2}", mMinmum, mMaxmum, mFlags);
        }

    }
}
