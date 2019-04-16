using System;
using Utility.Logger;

namespace NMGen
{
    public class NMGenUtility
    {
        public static int ClockwiseRotateDir(int dir)
        {
            return (dir + 1) & 0x3; 
        }

        public static int CClockwiseRotateDir(int dir)
        {
            return (dir + 3) & 0x3; 
        }

        public static int AntiDir( int dir )
        {
            return (dir + 2) & 0x3;  
        }

    }
}
