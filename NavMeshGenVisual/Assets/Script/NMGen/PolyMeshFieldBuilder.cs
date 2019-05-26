using System;
using Utility.Logger;
using System.Collections.Generic;


namespace NMGen
{
    public class PolyMeshFieldBuilder
    {
        private readonly static int FLAG = 0x8000000; //最高位用于标记Center 顶点  
        private readonly static int DEFLAG = 0x0FFFFFF;  //最高位空出来为了消除标记，同时还原顶点。

        private int mMaxVertsPerPoly; 

        public PolyMeshFieldBuilder(int maxVertsPerPoly)
        {
            mMaxVertsPerPoly = maxVertsPerPoly; 
        }

        public PolyMeshField build(ContourSet contours)
        {
            if( null == contours
                || 0 == contours.size() )
            {
                Logger.LogError("[PolyMeshFieldBuild][build]Param Error");  
                return null;
            }

            PolyMeshField result = new PolyMeshField(contours.boundsMin(),
                   contours.boundsMax(),
                   contours.cellSize(),
                   contours.cellHeight(),
                   mMaxVertsPerPoly);

            int sourceVertCount = 0;
            int maxPossiblePolygons = 0;
            int maxVertsPerContour = 0;  

            for(int contourIndex = 0; contourIndex < contours.size(); ++contourIndex)
            {
                int count = contours.get(contourIndex).vertCount;
                sourceVertCount += count;
                //过 n 边形的一个顶点，能把n边形最多分成 n - 2 三角形
                maxPossiblePolygons += count - 2;  
                maxVertsPerContour = Math.Max(maxVertsPerContour, count);  
            }

            if( sourceVertCount - 1 > DEFLAG  )
            {
                Logger.LogError("[PolyMeshFieldBuilder][build]sourceVertCount out of Range|{0}|{1}",DEFLAG,sourceVertCount);
                return null; 
            }

            int[] globalVerts = new int[sourceVertCount * 3];  //存的是索引，存的是整个轮廓的
            int globalVertCount = 0;

            int[] globalPolys = new int[maxPossiblePolygons * mMaxVertsPerPoly];  //存的也是索引
            for(int i = 0; i < globalPolys.Length; ++i)
            {
                globalPolys[i] = PolyMeshField.NULL_INDEX; 
            }

            int[] globalRegions = new int[maxPossiblePolygons];
            int globalPolyCount = 0;

            int[] contourToGlobalIndicesMap = new int[maxVertsPerContour]; //存的也是索引
            Dictionary<int, int> vertIndices = new Dictionary<int, int>();

            List<int> workingIndices = new List<int>(maxVertsPerContour);
            List<int> workingTriangles = new List<int>(maxVertsPerContour);

            //TODO 为什么要+1？
            int[] workingPolys = new int[(maxVertsPerContour + 1) * mMaxVertsPerPoly];
            int workingPolyCount = 0;
            int[] mergeInfo = new int[3];
            int[] mergedPoly = new int[mMaxVertsPerPoly];  

            for(int contourIndex = 0; contourIndex < contours.size(); ++contourIndex)
            {
                Contour contour = contours.get(contourIndex); 
                //4个数据一组，(x,y,z,regionID)，最少需要三个顶点
                if( contour.verts.Length < 3 * 4 ) 
                {
                    Logger.LogError("[PolyMeshFieldBuilder][build]Bad Contour|{0}", contour.regionID);
                    continue; 
                }

                //初始情况下，顶点索引对应的就是对应的顶点
                /*
                 *   Indices : 0 1 2 3 4 5 6  ....
                 *             0 1 2 3 4 5 6  ....
                 */
                workingIndices.Clear(); 
                for(int i = 0; i < contour.vertCount; ++i)  //这个顶点数是按四个一组的 
                {
                    workingIndices.Add(i);  
                } // for contour.vert

                //三角剖分
                int triangleCount = triangulate(contour.verts, ref workingIndices,ref  workingTriangles); 
                if( triangleCount <= 0 )
                {
                    Logger.LogError("[PolyMeshField][build]Triangulate Contour|{0}", contour.regionID);
                    continue;  
                }


                for(int iContourVert = 0; iContourVert < contour.vertCount ; ++iContourVert )
                {
                    int pContourVert = iContourVert * 4;
                    int vertHash = getHashCode(contour.verts[pContourVert],
                        contour.verts[pContourVert + 1],
                        contour.verts[pContourVert + 2]);

                    //vertIndices 里面储存的是根据顶点xyz hash出来的key，对应的在全局顶点表中的索引
                    //全局顶点表 以 xyz 3个为一组，储存轮廓的顶点
                    int iGlobalVert = 0;
                    if ( !vertIndices.TryGetValue(vertHash,out iGlobalVert) )
                    {
                        iGlobalVert = globalVertCount;
                        globalVertCount++;
                        vertIndices.Add(vertHash, iGlobalVert);

                        int newVertsBase = iGlobalVert * 3; 
                        globalVerts[newVertsBase] = contour.verts[pContourVert];
                        globalVerts[newVertsBase + 1] = contour.verts[pContourVert + 1];
                        globalVerts[newVertsBase + 2] = contour.verts[pContourVert + 2]; 
                    }

                    //这个东东是临时用的，就是记录 轮廓中某个顶点的索引，对应的是在全局顶点表中索引
                    //Contour Vertex index  -> global vertex index 
                    contourToGlobalIndicesMap[iContourVert] = iGlobalVert; 
                } // for iContourVert


                for(int i = 0; i < workingPolys.Length; ++i)
                {
                    workingPolys[i] = PolyMeshField.NULL_INDEX; 
                } // for workingPolys

                //只有这个阶段是三角形
                workingPolyCount = 0; 
                for(int i = 0; i < triangleCount; ++i)
                {
                    /*
                     *  workingTraingles 储存的是上一步三角剖分的三角形的顶点索引
                     *                     ||
                     *                      V 
                     *  contourToGlobalIndicesMap 储存的是轮廓顶点索引，对应在全局顶点表的索引关系
                     *                     ||
                     *                     V
                     *  workingPolys 中 储存的则是全局的顶点索引
                     *  
                     *  
                     */


                    int polyIdxBase = workingPolyCount * mMaxVertsPerPoly;
                    int triangleIdxBase = i * 3; 
                    workingPolys[polyIdxBase] =
                        contourToGlobalIndicesMap[workingTriangles[triangleIdxBase]];
                    workingPolys[polyIdxBase+1] =
                        contourToGlobalIndicesMap[workingTriangles[triangleIdxBase+1]];
                    workingPolys[polyIdxBase+2] =
                        contourToGlobalIndicesMap[workingTriangles[triangleIdxBase]+2];

                    workingPolyCount++;  

                }  //triangleCount 

                //合并三角形
                if( mMaxVertsPerPoly > 3 )
                {
                    while (true)
                    {
                        int longestMergeEdge = -1;
                        int pBestPolyA = -1;
                        int iPolyAVert = -1;
                        int pBestPolyB = -1;
                        int iPolyBVert = -1;

                        
                        for(int iPolyA = 0; iPolyA < workingPolyCount - 1;++iPolyA)
                        {
                            for(int iPolyB = iPolyA+1; iPolyB < workingPolyCount; ++iPolyB)
                            {
                                //因为一个多边形最多有mMaxVertsPerPoly的点，所以
                                //多边形的真正起点为就是用多边形的索引乘以顶点数
                                //Can polyB merge with polyA?
                                getPolyMergeInfo(iPolyA * mMaxVertsPerPoly,
                                    iPolyB * mMaxVertsPerPoly,
                                    workingPolys,
                                    globalVerts,
                                    result.maxVertsPerPoly(),
                                    out mergeInfo); 

                                if( mergeInfo[0] > longestMergeEdge )
                                {
                                    longestMergeEdge = mergeInfo[0];
                                    pBestPolyA = iPolyA * mMaxVertsPerPoly;
                                    iPolyAVert = mergeInfo[1];
                                    pBestPolyB = iPolyB * mMaxVertsPerPoly;
                                    iPolyBVert = mergeInfo[2]; 
                                }
                            } // for iPolyB
                        } // for iPolyA

                        if( longestMergeEdge <= 0 )
                        {
                            break; 
                        }

                        for(int i = 0; i < mergedPoly.Length; ++i)
                        {
                            mergedPoly[i] = PolyMeshField.NULL_INDEX; 
                        }

                        int vertCountA = PolyMeshField.getPolyVertCount(pBestPolyA, workingPolys, result.maxVertsPerPoly());
                        int vertCountB = PolyMeshField.getPolyVertCount(pBestPolyB, workingPolys, result.maxVertsPerPoly());
                        int position = 0; 


                        for(int i = 0; i < vertCountA - 1; ++i)
                        {
                            //pBestPolyA 为多边形对应的基索引
                            //iPolyAVert 为共享的一个端点
                            //+1 指的是共享点下一个顶点开始
                            int polyIdx = pBestPolyA + ((iPolyAVert + 1 + i) % vertCountA);
                            mergedPoly[position++] = workingPolys[polyIdx];  
                        } // for vertCountA

                        for(int i = 0;i < vertCountB - 1; ++i)
                        {
                            int polyIdx = pBestPolyB + ((iPolyBVert + 1 + i) % vertCountB);
                            mergedPoly[position++] = workingPolys[polyIdx];
                        }

                        //将合并之后的顶点拷到A指定的多边形
                        Array.Copy(mergedPoly, 0, workingPolys, pBestPolyA, mMaxVertsPerPoly);
                        //将多边形B删除
                        Array.Copy(workingPolys, pBestPolyB + mMaxVertsPerPoly, workingPolys, pBestPolyB, workingPolys.Length - pBestPolyB - mMaxVertsPerPoly);

                        workingPolyCount--; 
                    } // while true
                } // if MaxVertsPerPoly 

                for(int i = 0; i < workingPolyCount; ++i)
                {
                    Array.Copy(workingPolys, i * mMaxVertsPerPoly,
                        globalPolys, globalPolyCount * mMaxVertsPerPoly, mMaxVertsPerPoly);
                    globalRegions[globalPolyCount] = contour.regionID;
                    globalPolyCount++; 
                }

            } // for contours

            //xyz为一组
            result.verts = new int[globalVertCount * 3];
            Array.Copy(globalVerts, 0, result.verts, 0, globalVertCount * 3);

            result.polys = new int[globalPolyCount * mMaxVertsPerPoly * 2];
            for (int iPoly = 0; iPoly < globalPolyCount; ++iPoly)
            {
                int pPoly = iPoly * mMaxVertsPerPoly;  //第几个Poly的索引
                for (int offset = 0; offset < mMaxVertsPerPoly; ++offset)
                {
                    //result里的多边形是以 2 * mMaxVertsPerPoly为一组的
                    //第一组 mMaxVertsPerPoly 是多边形自身的数据 
                    //第二组 mMaxVertsPertPol 是邻接多边形的数据
                    //而globalPolys就是以一组 mMaxVertsPerPoly
                    result.polys[pPoly * 2 + offset] = globalPolys[pPoly + offset];
                    result.polys[pPoly * 2 + mMaxVertsPerPoly + offset] = PolyMeshField.NULL_INDEX;
                }
            } // for 

            result.polyRegions = new int[globalPolyCount];
            Array.Copy(globalRegions, 0, result.polyRegions, 0, globalPolyCount);

            buildAdjacencyData(result); 

            return result;
        } // build 

        private static void buildAdjacencyData(PolyMeshField mesh)
        {
            int vertCount = mesh.verts.Length / 3;
            int polyCount = mesh.polyRegions.Length;
            int maxEdgeCount = polyCount * mesh.maxVertsPerPoly();  // 多边形的个数乘以顶点数？

            int[] edges = new int[maxEdgeCount * 6];
            int edgeCount = 0;

            int[] startEdge = new int[vertCount]; 
            for(int i = 0;i < startEdge.Length;++i )
            {
                startEdge[i] = PolyMeshField.NULL_INDEX; 
            }

            //数组链表
            int[] nextEdge = new int[maxEdgeCount]; 
            for(int iPoly = 0; iPoly < polyCount; ++iPoly)
            {
                int pPoly = iPoly * mesh.maxVertsPerPoly() * 2; //两组maxVertPerPoly为步长
                for (int vertOffset = 0; vertOffset < mesh.maxVertsPerPoly(); ++vertOffset) 
                {
                    int iVert = mesh.polys[pPoly + vertOffset];  //第一组maxVertsPerPoly的数据 
                    if( PolyMeshField.NULL_INDEX == iVert )
                    {
                        break; 
                    }

                    //如果 maxVertsPerPoly = 6 ，那当vertOffset = 5 的时候会发生什么？
                    //那iNextVert就会回到起点
                    int iNextVert; 
                    if( vertOffset + 1 >= mesh.maxVertsPerPoly() 
                        || mesh.polys[pPoly+vertOffset+1] == PolyMeshField.NULL_INDEX) 
                    {
                        iNextVert = mesh.polys[pPoly];  
                    }
                    else
                    {
                        iNextVert = mesh.polys[pPoly + vertOffset + 1]; 
                    }

                    if( iVert < iNextVert )
                    {
                        int edgeBaseIdx = edgeCount * 6; 
                        //一条边的两个端点
                        edges[edgeBaseIdx] = iVert;
                        edges[edgeBaseIdx + 1] = iNextVert;

                        //这条边在多边形里面的信息
                        edges[edgeBaseIdx + 2] = iPoly;
                        edges[edgeBaseIdx + 3] = vertOffset;

                        //默认是边界边
                        edges[edgeBaseIdx + 4] = PolyMeshField.NULL_INDEX;
                        edges[edgeBaseIdx + 5] = PolyMeshField.NULL_INDEX;


                        //倒插链表？ iVert顶点 -> edgeCount 边 -> 然后edgeCount 就是链表尾了
                        //TODO  源码这里也是这样，不知道为毛
                        nextEdge[edgeCount] = startEdge[iVert];  //???这个东西不是NULL_INDEX么？
                        startEdge[iVert] = edgeCount;

                        edgeCount++; 
                    }
                } // for vertOffset
            } // for Poly

            for(int iPoly = 0; iPoly < polyCount; ++iPoly)
            {
                int pPoly = iPoly * mesh.maxVertsPerPoly() * 2;
                for(int vertOffset = 0; vertOffset < mesh.maxVertsPerPoly(); ++vertOffset)
                {
                    int iVert = mesh.polys[pPoly + vertOffset];  
                    if( PolyMeshField.NULL_INDEX == iVert )
                    {
                        break; 
                    }

                    int iNextVert; 
                    if( vertOffset + 1 >= mesh.maxVertsPerPoly() 
                        || mesh.polys[pPoly+vertOffset] == PolyMeshField.NULL_INDEX)
                    {
                        iNextVert = mesh.polys[pPoly]; 
                    }
                    else
                    {
                        iNextVert = mesh.polys[pPoly + vertOffset];
                    }

                    //这里是反向查找共享边的信息
                    if( iVert > iNextVert )
                    {
                        for(int edgeIndex = startEdge[iNextVert]; edgeIndex != PolyMeshField.NULL_INDEX; edgeIndex = nextEdge[edgeIndex])
                        {
                            if( iVert == edges[edgeIndex*6+1] )
                            {
                                edges[edgeIndex * 6 + 4] = iPoly;
                                edges[edgeIndex * 6 + 5] = vertOffset;
                                break; 
                            }
                        } // for 
                    }  //if ivert > iNextVert 

                } // for vertOffset 
            } // for Poly

            for(int pEdge = 0; pEdge < edgeCount; pEdge+=6 )
            {
                //是否共享边 ？
                if( edges[pEdge+4] != PolyMeshField.NULL_INDEX )
                {
                    int pPolyA = edges[pEdge + 2] * mesh.maxVertsPerPoly() * 2;
                    int pPolyB = edges[pEdge + 4] * mesh.maxVertsPerPoly() * 2;
                    //参考 poly和edges数组的定义。描述的就是polyA的哪些边与哪个poly共享
                    mesh.polys[pPolyA + mesh.maxVertsPerPoly() + edges[pEdge + 3]] = edges[pEdge + 4];
                    mesh.polys[pPolyB + mesh.maxVertsPerPoly() + edges[pEdge + 5]] = edges[pEdge + 2]; 
                }
            }


        }// func

        private static void getPolyMergeInfo(int polyAPointer,int polyBPointer,
            int[] polys,int[] verts ,
            int maxVertsPerPoly ,out int[] outResult )
        {
            outResult = new int[3]; 

            outResult[0] = -1;
            outResult[1] = -1;
            outResult[2] = -1;

            int vertCountA = PolyMeshField.getPolyVertCount(polyAPointer, polys, maxVertsPerPoly);
            int vertCountB = PolyMeshField.getPolyVertCount(polyBPointer, polys, maxVertsPerPoly);

            //减2 是因为合并了一条边，少了两个顶点
            if( vertCountA + vertCountB - 2 > maxVertsPerPoly )
            {
                return; 
            }

            for(int iPolyVertA = 0; iPolyVertA < vertCountA; ++iPolyVertA)
            {
                int iVertA = polys[polyAPointer + iPolyVertA];
                int iVertANext = polys[polyAPointer + getNextIndex(iPolyVertA, vertCountA)]; 

                for(int iPolyVertB = 0; iPolyVertB < vertCountB; ++iPolyVertB)
                {
                    int iVertB = polys[polyBPointer + iPolyVertB];
                    int iVertBNext = polys[polyBPointer + getNextIndex(iPolyVertB, vertCountB)]; 

                    /*
                     *    同一种顶点顺序，两个点是重合的话， 就是共享边了。
                     *    A/B+1
                     *     \
                     *      \ 
                     *      A+1/B
                     */

                    if( iVertA == iVertBNext
                        && iVertANext == iVertB )
                    {
                        outResult[1] = iPolyVertA;
                        outResult[2] = iPolyVertB; 
                    }
                } //iPolyB
            }// iPolyA


            if( -1 == outResult[1] )
            {
                return; 
            }

            //对顺时针包含的多边形有效
            int pSharedVertMinus;
            int pSharedVert;
            int pSharedVertPlus;

            //为什么是3呢?是因为xyz为一组
            //A-1
            pSharedVertMinus = polys[polyAPointer + getPreviousIndex(outResult[1], vertCountA)] * 3;  
            //A
            pSharedVert = polys[polyAPointer + outResult[1]] * 3;
            //????TODO，用B的，还是B+2了卧槽。。。
            //我艹，B+2 对应的就是图上的A+1，这个并不是真是意义是A点的下一个点
            pSharedVertPlus = polys[polyBPointer + ((outResult[2] + 2) % vertCountB)] * 3; 

            if(!isLeft(verts[pSharedVert],verts[pSharedVert+2],
                verts[pSharedVertMinus],verts[pSharedVertMinus+2],
                verts[pSharedVertPlus],verts[pSharedVertPlus+2]
                ))
            {
                return; 
            }

            pSharedVertMinus = polys[polyBPointer + getPreviousIndex(outResult[2], vertCountB)] * 3; 
            pSharedVert = polys[polyBPointer + outResult[2]] * 3;
            pSharedVertPlus = polys[polyAPointer + ((outResult[1] + 2) % vertCountA)] * 3; 

            if( !isLeft(verts[pSharedVert],verts[pSharedVert+2],
                verts[pSharedVertMinus],verts[pSharedVertMinus+2],
                verts[pSharedVertPlus],verts[pSharedVertPlus+2])) 
            {
                return; 
            }

            //共享边
            pSharedVertMinus = polys[polyAPointer + outResult[1]] * 3;
            pSharedVert = polys[polyAPointer + getNextIndex(outResult[1], vertCountA)] * 3;

            int deltaX = verts[pSharedVertMinus + 0] - verts[pSharedVert + 0];
            int deltaZ = verts[pSharedVertMinus + 2] - verts[pSharedVert + 2];
            outResult[0] = deltaX * deltaX + deltaZ * deltaZ; 

        }

        private static int getHashCode(int x,int y,int z)
        {
            uint h1 = 0x8da6b343;
            uint h2 = 0xd8163841;
            uint h3 = 0xcb1ab31f;
            long n = h1 * x + h2 * y + h3 * z; 
            return  (int)(n & ((1<<12)-1)); 
        }

        private static int triangulate(int[] verts,
            ref List<int> inoutIndices ,
            ref List<int> outTriangles )
        {
            outTriangles.Clear();  
            for(int i = 0; i < inoutIndices.Count; ++i)
            {
                int iPlus1 = getNextIndex(i, inoutIndices.Count);
                int iPlus2 = getNextIndex(iPlus1, inoutIndices.Count); 
                if( isValidPartition(i,iPlus2,verts,inoutIndices) )
                {
                    //除了自己所在的位，最高会也被标记了，这个相当于记录自己是CenterVertex
                    inoutIndices[iPlus1] = inoutIndices[iPlus1] | FLAG; 
                }

            } // for inoutIndices


            while( inoutIndices.Count > 3 )
            {
                int minLengthSq = -1;
                int iMinLenghtSqVert = -1;

                for(int i = 0; i < inoutIndices.Count; ++i)
                {
                    int iPlus1 = getNextIndex(i, inoutIndices.Count); 
                    //将上面标记的顶点恢复出来
                    if( ( inoutIndices[iPlus1] & FLAG ) == FLAG  )
                    {
                        int vert = (inoutIndices[i] & DEFLAG) * 4;
                        int vertPlus2 = (inoutIndices[getNextIndex(iPlus1, inoutIndices.Count)] & DEFLAG) * 4;

                        int deltaX = verts[vertPlus2] - verts[vert];
                        int deltaZ = verts[vertPlus2 + 2] - verts[vert + 2];
                        int lengthSq = deltaX * deltaX + deltaZ * deltaZ;  

                        if( minLengthSq < 0 
                            || lengthSq < minLengthSq )
                        {
                            minLengthSq = lengthSq;
                            iMinLenghtSqVert = i;  
                        }

                    } // if a Center Vertex 
                } // for inoutIndices

                //三角化失败了，剩余的顶点不能再变成三角形
                if( iMinLenghtSqVert == -1 )
                {
                    return -(outTriangles.Count / 3); 
                }

                int j = iMinLenghtSqVert;
                int jPlus1 = getNextIndex(j, inoutIndices.Count);

                outTriangles.Add(inoutIndices[j] & DEFLAG);
                outTriangles.Add(inoutIndices[jPlus1] & DEFLAG);
                outTriangles.Add(inoutIndices[getNextIndex(jPlus1, inoutIndices.Count)] & DEFLAG);

                //注意哦，是移除索引的。因为这个顶点已经在新的多边形外边
                inoutIndices.RemoveAt(jPlus1); 


                //防止删除的是特殊的索引
                if( 0 == jPlus1 
                    || jPlus1 >= inoutIndices.Count )
                {
                    j = inoutIndices.Count - 1;
                    jPlus1 = 0;
                }

                //看一下被变动的那两个顶点，是否可以组成新的Partition
                if( isValidPartition( getPreviousIndex(j,inoutIndices.Count),
                    jPlus1,
                    verts,
                    inoutIndices))
                {
                    inoutIndices[j] = inoutIndices[j] | FLAG; 
                }
                else
                {
                    inoutIndices[j] = inoutIndices[j] | DEFLAG; 
                }

                if( isValidPartition(j,getNextIndex(jPlus1,inoutIndices.Count),verts,inoutIndices)) 
                {
                    inoutIndices[jPlus1] = inoutIndices[jPlus1] | FLAG; 
                }
                else
                {
                    inoutIndices[jPlus1] = inoutIndices[jPlus1] | DEFLAG; 
                }

            } // while innoutIndices

            //最后剩下的就是一组了
            outTriangles.Add(inoutIndices[0] & DEFLAG);
            outTriangles.Add(inoutIndices[1] & DEFLAG);
            outTriangles.Add(inoutIndices[2] & DEFLAG);

            return outTriangles.Count / 3;  
        } // triangulate

        private static bool isValidPartition(int indexA,int indexB,
            int[] verts,
            List<int> indices)
        {
            return liesWithinInternalAngle(indexA, indexB, verts, indices)
                && !hasIllegalEdgeIntersection(indexA, indexB, verts, indices); 
        }


        private static bool hasIllegalEdgeIntersection(int indexA,int indexB,int[] verts,List<int> indices)
        {
            int pVertA = (indices[indexA] & DEFLAG) * 4;
            int pVertB = (indices[indexB] & DEFLAG) * 4; 
            
            for(int iPolyEdgeBegin = 0; iPolyEdgeBegin < indices.Count; ++iPolyEdgeBegin )
            {
                int iPolyEdgeEnd = getNextIndex(iPolyEdgeBegin, indices.Count); 
                if( !(iPolyEdgeBegin == indexA
                    || iPolyEdgeBegin == indexB
                    || iPolyEdgeEnd == indexA
                    || iPolyEdgeEnd == indexB))
                {
                    int pEdgeVertBegin = (indices[iPolyEdgeBegin] & DEFLAG) * 4;
                    int pEdgeVertEnd = (indices[iPolyEdgeEnd] & DEFLAG) * 4;

                    bool isBeginPointSameWithA = verts[pEdgeVertBegin] == verts[pVertA]
                        && verts[pEdgeVertBegin + 2] == verts[pVertA + 2];
                    bool isBeginPointSameWithB = verts[pEdgeVertBegin] == verts[pVertB]
                        && verts[pEdgeVertBegin + 2] == verts[pVertB + 2];
                    bool isEndPointSameWithA = verts[pEdgeVertEnd] == verts[pVertA]
                        && verts[pEdgeVertEnd + 2] == verts[pVertA + 2];
                    bool isEndPointSameWithB = verts[pEdgeVertEnd] == verts[pVertB]
                        && verts[pEdgeVertEnd + 2] == verts[pVertB + 2];  
                    if ( isBeginPointSameWithA
                        || isBeginPointSameWithB
                        || isEndPointSameWithA
                        || isEndPointSameWithB )
                    {
                        continue;
                    }

                    if( Geometry.segmentsIntersect(verts[pVertA],
                        verts[pVertA+2],
                        verts[pVertB],
                        verts[pVertB+2],
                        verts[pEdgeVertBegin],
                        verts[pEdgeVertBegin+2],
                        verts[pEdgeVertEnd],
                        verts[pEdgeVertEnd+2] ))
                    {
                        return true; 
                    }
                }
            }

            return false;  
        }

        /*
         *  请按照图来对号入座
         * 
         *      A-1          A-1        A-1    
         *        \           \          \
         *         A           A          A
         *        /            |           \    
         *      A+1           A+1          A+1
         *      
         *      A-1          A-1        A-1
         *       |            |          |
         *       A            A          A 
         *      /             |           \
         *    A+1            A+1          A+1
         *      
         *      A-1          A-1        A-1
         *       /            /          |
         *      A            A           A
         *     /             |            \
         *   A+1            A+1           A+1
         * 
         */
        private static bool liesWithinInternalAngle(int indexA,
            int indexB,
            int[] verts ,
            List<int> indices)
        {
            //这个DEFLAG就是一个全集吧
            int pVertA = ((indices[indexA] & DEFLAG) * 4);  //因为是按四个一组的，所以需要 *4
            int pVertB = ((indices[indexB] & DEFLAG) * 4);

            int prevIndexA = getPreviousIndex(indexA, indices.Count);
            int nextIndexA = getNextIndex(indexA, indices.Count); 
            int pVertAMinus = ((indices[prevIndexA] & DEFLAG) * 4 );
            int pVertAPlus = ((indices[nextIndexA] & DEFLAG) * 4 );

            
            //参考网站第二张图。。。然后它是在isRight那个地方返回的
            //内角小于180度的情况 
            //点A在 AMinu->APlus的左边 
            if ( isLeftOrCollinear( verts[pVertA],verts[pVertA+2],
                verts[pVertAMinus],verts[pVertAMinus+2],
                verts[pVertAPlus],verts[pVertAPlus+2] ) )
            {
                // B 在 A->AMinus的左边 
                return isLeft( verts[pVertB],verts[pVertB+2],
                    verts[pVertA],verts[pVertA+2],
                    verts[pVertAMinus],verts[pVertAMinus+2] )
                // B 在 A->APlus的右边
                && isRight(verts[pVertB], verts[pVertB + 2],
                    verts[pVertA], verts[pVertA + 2],
                    verts[pVertAPlus], verts[pVertAPlus + 2]);
            } 

            //内角大于180的情况，如果是位于外角，反转便是了
            return !(
                //判断一下
                //点B在 A->APlus的左边
                isLeftOrCollinear(verts[pVertB], verts[pVertB + 2],
                verts[pVertA], verts[pVertA + 2],
                verts[pVertAPlus], verts[pVertAPlus + 2])

                //点B 在 A->AMinus的右边
                && isRightOrCollinear(verts[pVertB], verts[pVertB + 2],
                verts[pVertA], verts[pVertA + 2],
                verts[pVertAMinus], verts[pVertAMinus + 2])
                );
        }

        private static bool isRightOrCollinear(int px,int py,
            int ax,int ay,
            int bx,int by)
        {
            return getSignedAreaX2(ax, ay, px, py, bx, by) >= 0;  
        }

        private static bool isRight(int px,int py,
            int ax,int ay,
            int bx,int by)
        {
            return getSignedAreaX2(ax, ay, px, py, bx, by) > 0; 
        }

        private static bool isLeft(int px,int py,
            int ax,int ay,
            int bx,int by)
        {
            return getSignedAreaX2(ax, ay, px, py, bx, by) < 0;  
        }

        private static bool isLeftOrCollinear(int px,int py,
            int ax,int ay,
            int bx,int by)
        {
            return getSignedAreaX2(ax, ay, px, py, bx, by) <= 0;    
        }

        /// <summary>
        /// http://www.mathopenref.com/coordpolygonarea.html
        /// http://mathworld.wolfram.com/TriangleArea.html
        /// http://www.everyinch.net/index.php/computergeometry1/  在现代三角形面积计算方法那里
        /// https://qiita.com/fujii-kotaro/items/a411f2a45627ed2f156e  割耳法
        /// </summary>
        /// <param name="ax"></param>
        /// <param name="ay"></param>
        /// <param name="bx"></param>
        /// <param name="by"></param>
        /// <param name="cx"></param>
        /// <param name="cy"></param>
        /// <returns></returns>
        private static int getSignedAreaX2(int ax,int ay,
            int bx,int by,
            int cx,int cy)
        {
            /*  
             *              v2
             *             / \ 
             *            /   \
             *           /     \
             *        v0 - - -- v1
             *            
             *  
             *      A = 1/2 | V x W|
             *        = 1/2 | (v1-v0) x (v2-v0)|
             * 
             * =>  2A = (x1-x0)(y2-y0) - (x2-x0)(y1-y0)
             * 
             *   v0 = a
             *   v1 = b
             *   v2 = c
             * 
             *   用右手定则，如果 v1-v0 x  v2-v0  小于 < 0 则，法向量指向下，证明v1 在 v2-v0左边
             */


            return (bx - ax) * (cy - ay) - (cx - ax) * (by - ay); 
        }


        private static int getPreviousIndex(int i,int n)
        {
            return i - 1 >= 0 ? i - 1 : n - 1; 
        }

        //为什么超出之后不是环绕呢？
        private static int getNextIndex(int i,int n)
        {
            return i + 1 < n ? i + 1 : 0; 
        }

    } // class
}
