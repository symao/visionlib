#ifndef __VS_GRIDMAP_H__
#define __VS_GRIDMAP_H__

#include <vector>
#include <iostream>

/**
    \brief grid map buffer
    x - horizontal direction  y - verticle direction
    the coodinate of data(i,j) :  x = (i+0.5)*resolution+origin_x   y = (j+0.5)*resolution+origin_y
  */
class GridMap2D
{
public:
    GridMap2D():rows(0),cols(0),resolution(0),origin_x(0),origin_y(0){}
    GridMap2D(int rows_,int cols_,double ox=0,double oy=0,double res = 0.05,double val = 0)
        : rows(rows_)
        , cols(cols_)
        , data(std::vector<std::vector<double> >(rows_,std::vector<double>(cols_,val)))
        , origin_x(ox)
        , origin_y(oy)
        , resolution(res)
    {}

    GridMap2D(double xmax,double xmin,double ymax,double ymin,double res=0.05,double val=0)
        :rows((ymax-ymin)/res+1)
        ,cols((xmax-xmin)/res+1)
        ,origin_x(xmin)
        ,origin_y(ymin)
        ,resolution(res)
    {
        if(rows*cols>1e9){
            printf("[ERROR] GridMap build failed. Map size(%dx%d) out of memory.\n", cols, rows);
        }
        resize(rows,cols);
    }

    /** \brief Is a point (x,y) in this map */
    bool inMap(double x,double y) const{
        int cIdx = (int)((x-origin_x)/resolution);
        int rIdx = (int)((y-origin_y)/resolution);
        return (cIdx>=0 && rIdx>=0 && cIdx<cols && rIdx<rows);
    }

    /** \brief Is a indexed grid (cIdx,rIdx) in this map */
    bool inMap(int rIdx,int cIdx) const{
        return (cIdx>=0 && rIdx>=0 && cIdx<cols && rIdx<rows);

    }

    /** \brief Get the value in grid which point (x,y) located into. */
    const double& at(double x,double y) const
    {
        int cIdx = (int)((x-origin_x)/resolution);
        int rIdx = (int)((y-origin_y)/resolution);
        return data[rIdx][cIdx];
    }

    /** \brief Get the value in grid which point (x,y) located into. */
    double& at(double x,double y){
        int cIdx = (int)((x-origin_x)/resolution);
        int rIdx = (int)((y-origin_y)/resolution);
        return data[rIdx][cIdx];
    }

    /** \brief Get the value in grid (ridx,cidx) */
    double getValue(int r,int c) const
    {
        return data[r][c];
    }

    /** \brief Get the value in grid (ridx,cidx) */
    double & getValue(int r,int c)
    {
        return data[r][c];
    }

    /** \brief Convert grid idx to point cordinate.
        Use the center of the specific grid. 
        x = (c+0.5)*resolution+origin_x   y = (r+0.5)*resolution+origin_y
    */
    void idx2xy(int c,int r,double& x,double &y)const
    {
        x = origin_x + (c+0.5)*resolution;
        y = origin_y + (r+0.5)*resolution;
    }

    /** \brief Convert grid idx which point located.
        c = (int)((x-origin_x)/resolution) r = (int)((y-origin_y)/resolution)
    */
    void xy2idx(double x,double y,int &c,int &r)const
    {
        c = (x-origin_x)/resolution;
        r = (y-origin_y)/resolution;
    }

    /** \brief Resize the grid map */
    void resize(int r,int c)
    {
        rows = r;
        cols = c;
        data = std::vector<std::vector<double> >(r,std::vector<double>(c,0));
    }

    /** \brief Set all grid value to \val */
    void setTo(double val)
    {
        for(int i=0;i<rows;i++){
            for(int j=0;j<cols;j++){
                data[i][j] = val;
            }
        }
    }

    /** \brief Set the specific grid[r][c] value to \val */
    void setTo(int r,int c,double val){
        if(r>=0 && r<rows && c>=0 && c<cols){
            data[r][c] = val;
        }
    }

    /** \brief All grid value in map add \a */
    GridMap2D operator+ (double a)
    {
        GridMap2D m(*this);
        for(int i=0;i<m.rows;i++)
            for(int j=0;j<m.cols;j++){
                m.data[i][j]+=a;
            }
        return m;
    }

    /** \brief All grid value in map add \a */
    GridMap2D &operator += (double a)
    {
        for(int i=0;i<rows;i++)
            for(int j=0;j<cols;j++){
                data[i][j]+=a;
            }
        return (*this);
    }

    /** \brief All grid value in map multi \a */
    GridMap2D operator* (double a)
    {
        GridMap2D m(*this);
        for(int i=0;i<m.rows;i++)
            for(int j=0;j<m.cols;j++){
                m.data[i][j]*=a;
            }
        return m;
    }
    
    /** \brief All grid value in map multi \a */
    GridMap2D &operator *= (double a)
    {
        for(int i=0;i<rows;i++)
            for(int j=0;j<cols;j++){
                data[i][j]*=a;
            }
            return (*this);
    }

    std::vector<std::vector <double> > data;  // -1 means unknown; 0~1:prob of occupied
    double origin_x,origin_y;  //the (x,y) coodinate of data(0,0)
    int cols,rows;
    double resolution; //the length of each grid edge
};


#endif
