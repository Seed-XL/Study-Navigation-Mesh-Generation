using System;

namespace NMGen
{
    //代表在y轴方向上某一列连续的体素
    public class HeightSpan
    {
        private int mMinmum;   //对应Y方向上最小的索引
        private int mMaxmum;   //相对就最大的索引
        private int mFlags = 0; 
        private HeightSpan mNext = null;   //同一列的，下一个更高的Span


        public HeightSpan(int min,int max ,int flags)
        {
            if( min > max )
            {
                throw new Exception("Minmum is greater than or equal to the maximum"); 
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
