#ifndef __VS_VIZ3D_H__
#define __VS_VIZ3D_H__
#include <opencv2/opencv.hpp>
#include <boost/thread.hpp>
#include <map>

class Viz3dThread
{
public:
    Viz3dThread():thread_ptr(new boost::thread(boost::bind(&Viz3dThread::threadFoo, this)))
    {
        viz.setBackgroundColor();
    }

    void updateWidget(const std::string& id, const cv::viz::Widget& w)
    {
        widget_table[id] = w;
    }

private:
    cv::viz::Viz3d viz;
    std::map<std::string,cv::viz::Widget> widget_table;
    std::shared_ptr<boost::thread> thread_ptr;

    void threadFoo()
    {
        while(!viz.wasStopped())
        {
            if(!widget_table.empty())
            {
                for(const auto& m:widget_table)
                {
                    viz.showWidget(m.first,m.second);
                }
                viz.spinOnce();
            }
            usleep(100000);
        }
    }
};

#endif