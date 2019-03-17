using System;
using Utility.Logger; 

namespace NMGen
{
    //代表在y轴方向上某一列连续的体素
    public class HeightSpan
    {
        //从图上，这两个值有点像Bound Min 和 Max ，并不是指索引，有点像 [ )，就是低闭，高开区间
        // min指的是Span最低点的索引
        // max指的是与Span紧接着的下一个voxel的索引
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
