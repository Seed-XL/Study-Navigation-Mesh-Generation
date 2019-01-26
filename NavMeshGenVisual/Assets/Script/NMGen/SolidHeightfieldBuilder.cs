using System;
using System.Collections;
using System.Collections.Generic;

namespace NMGen
{
    //体素化Builder
    public class SolidHeightfieldBuilder
    {
        private bool mClipLedges;

        //Span之间允许的最小高度,以体素为单位 
        private int mMinTraversableHeight;

        //Span之间允许的最大落差，以体素为单位 
        private int mMaxTraversableStep;

        //二面角的余弦值 
        private float mMinNormalY;

        private float mCellSize;

        private float mCellHeight;

        public SolidHeightfieldBuilder( float cellSize,
            float cellHeight,
            int minTraversableHeight ,
            int maxTraversableStep,
            int maxTraversableSlope,
            bool clipLedges
            )
        {
            mMinTraversableHeight = Math.Max(1, minTraversableHeight);
            mMaxTraversableStep = Math.Max(0, maxTraversableStep);
            //注意，传进来的值被改变了
            maxTraversableSlope = Math.Min(85, Math.Max(0, maxTraversableSlope));

            mClipLedges = clipLedges;
            mCellSize = cellSize;
            mCellHeight = cellHeight; 
        }


    }
}
