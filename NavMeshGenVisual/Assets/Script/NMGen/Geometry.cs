using System;
using Utility.Logger;
using System.Collections.Generic;


namespace NMGen
{
    public static class Geometry
    {
        public static float getPointSegmentDistanceSq(
            int px,int py,   //点P
            int ax,int ay,   //端点 a
            int bx,int by    //端点 b 
            )
        {
            float deltaABx = bx - ax;
            float deltaABy = by - ay;
            float deltaAPx = px - ax;
            float deltaAPy = py - ay;

            //线段长度的平方
            float segmentABLengthSq = deltaABx * deltaABx + deltaABy * deltaABy; 
            if( 0 == segmentABLengthSq )
            {
                //不是线段，直接返回两点的距离 
                return deltaAPx * deltaAPx + deltaAPy * deltaAPy; 
            }


            //向量ap 点乘  向量 ab，根据两者的夹角、以及投影来判断 p 是位于a左，b右，还是中间
            /*
             *   p
             *    \
             *     \
             *      \
             *      a -----------------  b
             */
            //u 实则就是 ap在ab上的投影比例
            float u = (deltaAPx * deltaABx + deltaAPy * deltaABy) / segmentABLengthSq ; 
            if( u < 0 )
            {
                return deltaAPx * deltaAPx + deltaAPy * deltaAPy; 
            }

            /*
             * 
             *                              p
             *                             /
             *                            /
             *                           / 
             *      a -----------------  b
             */
            else if ( u > 1 )
            {
                return (px-bx) * (px-bx) + (py-by)*(py-by); 
            }

            //如果是在中间，那么计算投影就好
            /*
             * 
             *           p
             *          /|                
             *         / |               
             *        /  |               
             *      a -----------------  b
             */

            float deltaX = (ax + u * deltaABx) - px ;
            float deltaY = (ay + u * deltaABy) - py ;
            return deltaX * deltaX + deltaY * deltaY;  
           
        } 


    }
}
