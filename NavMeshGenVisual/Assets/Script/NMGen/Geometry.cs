using System;
using Utility.Logger;
using System.Collections.Generic;


namespace NMGen
{
    public static class Geometry
    {

        public static float getPointSegmentDistanceSq(float px,float py,float pz,
            float ax,float ay,float az,
            float bx,float by,float bz)
        {
            float deltaABx = bx - ax;
            float deltaABy = by - ay;
            float deltaABz = bz - az;

            float deltaAPx = px - ax;
            float deltaAPy = py - ay;
            float deltaAPz = pz - az;

            float segmentABDistSq = deltaABx * deltaABx
                + deltaABy * deltaABy
                + deltaABz * deltaABz;  

            if( segmentABDistSq == 0 )
            {
                return deltaAPx * deltaAPx
                    + deltaAPy * deltaAPy
                    + deltaAPz * deltaAPz; 
            }

            float u = (deltaABx * deltaAPx  + deltaABy * deltaAPy + deltaABz * deltaAPz) / segmentABDistSq ;
            if( u < 0 )
            {
                return deltaAPx * deltaAPx + deltaAPy * deltaAPy + deltaAPz * deltaAPz; 
            }
            else if( u > 1 )
            {
                return (px - bx) * (px - bx) + (py - by) * (py - by) + (pz - bz) * (pz - bz); 
            }

            float deltaX = (ax + u * deltaABx) - px;
            float deltaY = (ay + u * deltaABy) - py;
            float deltaZ = (az + u * deltaABz) - pz;

            return deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ;  
        }

        /// <summary>
        /// http://mathworld.wolfram.com/Line-LineIntersection.html
        /// </summary>
        /// <param name="ax"></param>
        /// <param name="ay"></param>
        /// <param name="bx"></param>
        /// <param name="by"></param>
        /// <param name="cx"></param>
        /// <param name="cy"></param>
        /// <param name="dx"></param>
        /// <param name="dy"></param>
        /// <returns></returns>
        public static bool segmentsIntersect(int ax,int ay,
            int bx,int by,
            int cx,int cy,
            int dx,int dy)
        {
            //线段AB
            int deltaABx = bx - ax;
            int deltaABy = by - ay;

            //线段CA
            int deltaCAx = ax - cx;
            int deltaCAy = ay - cy;

            //线段CD
            int deltaCDx = dx - cx;
            int deltaCDy = dy - cy;


            /*
             * 
             *          d
             *          |
             *          |
             *  a  -----|------------ b
             *          |
             *          c
             */

            //分子， CD X CA
            int numerator = (deltaCAy * deltaCDx) - (deltaCAx * deltaCDy);  //分子
            //分母  ，AB X CD
            int denominator = (deltaABx * deltaCDy) - (deltaABy * deltaCDx);   

            //看跨立那一组说明
            //https://dev.gameres.com/Program/Abstract/Geometry.htm#%E5%88%A4%E6%96%AD%E4%B8%A4%E7%BA%BF%E6%AE%B5%E6%98%AF%E5%90%A6%E7%9B%B8%E4%BA%A4
            if ( 0 == denominator   //叉乘等于0，平行，所以不会相交，sin值等于0
                && numerator != 0 )  //看上面链接，如果这个等于0的话，三点共线了。
            {
                return false; 
            }

            //参考这个公式推导：https://blog.csdn.net/wcl0617/article/details/78654944
            float factorAB = numerator / (float)denominator;
            //AB X CA
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

        public static float getPointSegmentDistanceSq(float px,float py,
            float ax,float ay,
            float bx,float by)
        {
            float deltaABx = bx - ax;
            float deltaABy = by - ay;
            float deltaAPx = px - ax;
            float deltaAPy = py - ay;

            float segmentABLengthSq = deltaABx * deltaABx + deltaABy * deltaABy; 
            if( 0 == segmentABLengthSq )
            {
                return deltaAPx * deltaAPx + deltaAPy * deltaAPy; 
            }

            float u = (deltaAPx * deltaABx + deltaAPy * deltaABy) / segmentABLengthSq  ; 
            if( u < 0 )
            {
                return deltaAPx * deltaAPx + deltaAPy * deltaAPy; 
            }
            else if ( u > 1 )
            {
                return (px - bx) * (px - bx)  + (py - by) * (py -by) ; 
            }

            float deltaX = (ax + u * deltaABx) - px;
            float deltaY = (ay + u * deltaABy) - py;

            return deltaX * deltaX + deltaY * deltaY; 
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
                //u < 0 夹角大于180 ，投影为负
                return deltaAPx * deltaAPx + deltaAPy * deltaAPy; 
            }

            /*
             *                      / p
             *                     /
             *                    /     
             *                   /                 
             *                  /                  
             *                 /                   
             *              a ----  b
             */
            else if ( u > 1 )
            {
                //u > 0 夹角小于180，并且投影比大于，即已经超过线段的长了
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


        public static float getSignedAreaX2(float ax ,float ay,
            float bx,float by,
            float cx,float cy)
        {
            //AB X AC 
            return (bx - ax) * (cy - ay) - (cx - ax) * (by - ay);  
        }

        public static bool segmentsOverlap(float ax,float ay,
            float bx,float by,
            float cx,float cy,
            float dx,float dy)
        {
            float deltaABx = bx - ax;
            float deltaABy = by - ay;

            float deltaCDx = dx - cx;
            float deltaCDy = dy - cy;

            float deltaCAx = ax - cx;
            float deltaCAy = ay - cy;

            float numerator = (deltaCAy * deltaCDx) - (deltaCAx * deltaCDy);
            float denominator = (deltaABx * deltaCDy) - (deltaABy * deltaCDx);

            float tolerance = 0.001f;  

            //平行
            if( denominator == 0 )
            {
                //且不共线
                if( numerator != 0 )
                {
                    return false;  
                }

                //如果是共线的话，还需要判断一下是否重叠了
                //如果是垂直到于x轴
                if( Math.Abs(cx-dx) < tolerance  )
                {
                    //判断一下在y轴上面的投影有没重叠
                    if( Math.Max(cy,dy) < Math.Min(ay,by) 
                        || Math.Max(ay,by) < Math.Min(cy,dy) )
                    {
                        return false; 
                    }
                    else
                    {
                        return true; 
                    }
                }
                else
                {
                    //只有检测一个轴就行了，如果一个轴中没有重叠的话，那么必然不重叠 
                    if( Math.Max(cx,dx) < Math.Min(ax,bx)
                        || Math.Max(ax,bx) < Math.Min(cx,dx) )
                    {
                        return false; 
                    }
                    else
                    {
                        return true;  
                    }
                }
            } // if denominator

            //在某个交点相交，参考上面的segmentIntersect
            float factorAB = numerator / denominator;
            float factorCD = ( (deltaCAy * deltaABx) - (deltaCAx * deltaABy)) / denominator ; 

            if( ( factorAB >= 0.0f)
                && (factorAB <= 1.0f)
                && (factorCD >= 0.0f)
                && (factorCD <= 1.0f))
            {
                return true; 
            }

            return false;  
        }

        public static float getDistanceSq(float ax, float ay,float bx,float by)
        {
            float deltaX = (ax - bx);
            float deltaY = (ay - by);
            return deltaX * deltaX + deltaY * deltaY;  
        }

    }
}
