namespace NMGen
{
    public interface IHeightFieldIterInterface
    {
        int depthIndex();
        int widthIndex();

        //重置为反向迭代器
        void ReverseReset(); 

    }
}

