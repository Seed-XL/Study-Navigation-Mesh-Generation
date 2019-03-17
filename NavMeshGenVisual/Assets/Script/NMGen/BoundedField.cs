using System;
using Utility.Logger; 

namespace NMGen
{
    //AABB
    public class BoundeField
    {

        //x方向 cell的最大个数 
        private int mWidth;
        //z方向 cell的最大个数
        private int mDepth;
        //数组三个元素分别对标：x,y,z
        private float[] mBoundsMin = new float[3];
        private float[] mBoundsMax = new float[3];

        //体素x-z平面的边长
        private float mCellSize; 
        //体素y方向上的长度
        private float mCellHeight; 

    
        public BoundeField()
        {
            resetBounds();
            resetCellInfo(); 
        }

        public BoundeField(float cellSize,float cellHeight)
        {
            mCellSize = Math.Max(cellSize, float.MinValue);
            mCellHeight = Math.Max(cellHeight, float.MinValue);
            calculateWidthDepth();
        }

        
        public BoundeField( float[] gridBoundsMin,
            float[] gridBoundsMax,
            float cellSize ,
            float cellHeight
            )
        {
            if( !isBoundsValid(gridBoundsMin,gridBoundsMax) )
            {
                Logger.LogError("[BoundedField][ctor]one or both bounds invalid");
                return;
            }

            Array.Copy(gridBoundsMin,0, mBoundsMin,0, 3);
            Array.Copy(gridBoundsMax,0, mBoundsMax,0, 3);

            mCellSize = Math.Max(cellSize, float.MinValue);
            mCellHeight = Math.Max(cellHeight, float.MinValue);

            calculateWidthDepth(); 
        }

        public float[] boundsMax()
        {
            return mBoundsMax; 
        }

        public float[] boundsMin()
        {
            return mBoundsMin; 
        }

        public float cellHeight()
        {
            return mCellHeight; 
        }

        public float cellSize()
        {
            return mCellSize; 
        }

        public int depth()
        {
            return mDepth; 
        }

        public int width()
        {
            return mWidth; 
        }


        protected bool isBoundsValid(float[] boundsMin,float[] boundsMax)
        {
            return !(boundsMax == null
               || boundsMin == null
               || boundsMax.Length != 3
               || boundsMin.Length != 3); 
        }

        public bool isInBounds(int widthIndex, int depthIndex)
        {
            return ( widthIndex >= 0
                && depthIndex >= 0 
                && widthIndex < mWidth
                && depthIndex < mDepth
                ); 
        }

        public bool overlaps( float[] boundsMin,float[] boundsMax )
        {
            bool overlaps = true;
            if( !isBoundsValid(boundsMin,boundsMax) )
            {
                return false; 
            }

            //三个轴分别检测是否重叠
            overlaps = (mBoundsMin[0] > boundsMax[0] || mBoundsMax[0] < boundsMin[0])
                ? false : overlaps;
            overlaps = (mBoundsMin[1] > boundsMax[1] || mBoundsMax[1] < boundsMin[1])
               ? false : overlaps;
            overlaps = (mBoundsMin[2] > boundsMax[2] || mBoundsMax[2] < boundsMin[2])
               ? false : overlaps;

            return overlaps; 
        }

        protected int GetGridIndex( int widthIndex,int depthIndex )
        {
            if(widthIndex < 0
                || depthIndex < 0 
                || widthIndex >= mWidth 
                || depthIndex >= mDepth )
            {
                return -1; 
            }

            return widthIndex * mDepth + depthIndex; 
        }


        protected void resetBounds()
        {
            mBoundsMin[0] = 0;
            mBoundsMin[1] = 0;
            mBoundsMin[2] = 0;

            mBoundsMin[0] = 1;
            mBoundsMin[1] = 1;
            mBoundsMin[2] = 1;
        }

        protected void resetCellInfo()
        {
            mCellSize = 0.1f;
            mCellHeight = 0.1f;
            calculateWidthDepth(); 
        }

        public void setBounds(float xmin,float ymin,float zmin,
            float xmax,float ymax,float zmax
            )
        {
            mBoundsMin[0] = xmin;
            mBoundsMin[1] = ymin;
            mBoundsMin[2] = zmin;

            mBoundsMax[0] = xmax;
            mBoundsMax[1] = ymax;
            mBoundsMax[2] = zmax;

            calculateWidthDepth();
        }

        protected void setBounds( float[] min,float[] max)
        {
            if( !isBoundsValid(min,max) )
            {
                return; 
            }

            setBounds(min[0], min[1], min[2],
                max[0],max[1],max[2]
                ); 
        }

        protected void setBoundsMax( float[] value )
        {
            if( null == value 
                || value.Length != 3 )
            {
                return; 
            }

            Array.Copy(value, 0, mBoundsMax, 0, 3);
            calculateWidthDepth(); 
        }

        protected void setBoundsMin(float[] value)
        {
            if (null == value
                || value.Length != 3)
            {
                return;
            }

            Array.Copy(value, 0, mBoundsMin, 0, 3);
            calculateWidthDepth();
        }


        protected void setCellSize(float value)
        {
            mCellSize = Math.Max(value, float.MinValue);
            calculateWidthDepth(); 
        }

        /// <summary>
        /// 分别对应y轴四个方向上的值
        /// </summary>
        ///             |
        ///             |
        ///           (x,1)
        ///             |  
        ///---(x,0)-----|------(x,0)---
        ///             |
        ///             |
        ///           (x,-1)
        /// 
        /// <param name="dir"></param>
        /// <returns></returns>
        public static int getDirOffsetDepth(int dir)
        {
            int[] offset = new int[] { 0,1,0,-1};
            return offset[dir & 0x03]; 
        }

        /// <summary>
        ///  分别对应x轴四个方向上的值
        /// </summary>
        ///             |
        ///             |
        ///           (0,y)
        ///             |  
        ///---(-1,y)----|------(1,y)---
        ///             |
        ///             |
        ///           (0,y)
        /// <param name="dir"></param>
        /// <returns></returns>
        public static int getDirOffsetWidth(int dir)
        {
            int[] offset = new int[] { -1,0,1,0};
            return offset[dir & 0x03]; 
        }

        /// <summary>
        /// max-min得到的是Bounds的width，然后再除以cellSize，
        /// 可以得到Bound对应边的Cell的个数。再加上0.5f，是为了向上取整。补齐个数
        /// mWidth和mDepth实际上说的是Cell的个数
        /// </summary>
        private void calculateWidthDepth()
        {
            mWidth = (int)((mBoundsMax[0] - mBoundsMin[0]) / mCellSize + 0.5f);
        }

    }
}