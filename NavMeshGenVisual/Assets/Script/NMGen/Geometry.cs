using System;
using Utility.Logger;
using System.Collections.Generic;


namespace NMGen
{
    public static class Geometry
    {
        public static bool segmentsIntersect(int ax,int ay,
            int bx,int by,
            int cx,int cy,
            int dx,int dy)
        {
            int deltaABx = bx - ax;
            int deltaABy = by - ay;

            int deltaCAx = ax - cx;
            int deltaCAy = ay - cy;

            int deltaCDx = dx - cx;
            int deltaCDy = dy - cy;

            int numerator = (deltaCAy * deltaCDx) - (deltaCAx * deltaCDy);  //分子
            int denominator = (deltaABx * deltaCDy) - (deltaABy * deltaCDx);  //分母
            
            if( 0 == denominator 
                && numerator != 0 )
            {
                return false; 
            }

            float factorAB = numerator / (float)denominator;
            float factorCD = ( (deltaCAy * deltaABx) - (deltaCAx * deltaABy)) / (float)denominator ; 

            if( (factorAB >= 0.0f)
                && (factorAB <= 1.0f)
                && (factorCD >= 0.0f)
                && (factorCD <= 1.0f))
            {
                return true; 
            }

            return false;
        }

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

            //u 实则就是 ap在ab上的投影比例
            float u = (deltaAPx * deltaABx + deltaAPy * deltaABy) / segmentABLengthSq ;

            //向量ap 点乘  向量 ab，根据两者的夹角、以及投影来判断 p 是位于a左，b右，还是中间
            /*
             *   p
             *    \
             *     \
             *      \
             *      a -----------------  b
             */
            if ( u < 0 )
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
